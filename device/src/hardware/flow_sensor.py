import threading
import time
from typing import Optional

from infra.config import FlowSensorConfig

try:
    import librpiplc.rpiplc as plc  # Industrial Shields I/O layer
except Exception:  # pragma: no cover
    plc = None


class FlowSensor:
    """
    PLC-stack flow meter:
    - reads a PLC input (e.g. "I1.0") via librpiplc
    - counts edges in software
    - computes flow each second from pulse count
    """

    def __init__(self, config: FlowSensorConfig) -> None:
        self.config = config

        self._running = False
        self._stop_worker = False
        self._lock = threading.Lock()

        self._last_error: Optional[str] = None
        self._pulse_count = 0
        self._flow_rate_lpm = 0.0
        self._total_liters = 0.0

        self._worker: Optional[threading.Thread] = None

        # Edge detect state
        self._last_level = 0

        # IMPORTANT:
        # Use PLC input name (NOT BCM). Typical: "I1.0" for INT1.0 on many units.
        # Put this in config as e.g. config.plc_input = "I1.0"
        if not hasattr(self.config, "plc_input"):
            # fallback for your current config shape; set manually below if you prefer
            self.config.plc_input = "I1.0"

        # choose which edge counts a pulse:
        # - For NPN open-collector sinking into a pull-up, pulses are often LOW-going (falling)
        self._count_edge = "falling"  # "rising" / "falling"

    def _read_level(self) -> int:
        if plc is None:
            raise RuntimeError("librpiplc.rpiplc not available")

        v = plc.digital_read(self.config.plc_input)
        return 1 if v else 0

    def _loop(self) -> None:
        poll_interval_s = 0.0005  # 0.5 ms; relax to 0.001–0.005 if CPU is high
        last_calc = time.monotonic()
        pulses_since_last = 0

        # Initialize last_level
        try:
            self._last_level = self._read_level()
        except Exception as e:
            with self._lock:
                self._last_error = f"read init failed: {e}"
                self._running = False
            return

        while not self._stop_worker:
            with self._lock:
                running = self._running

            if not running:
                with self._lock:
                    self._flow_rate_lpm = 0.0
                time.sleep(0.05)
                last_calc = time.monotonic()
                continue

            try:
                level = self._read_level()
            except Exception as e:
                with self._lock:
                    self._last_error = f"read failed: {e}"
                time.sleep(0.05)
                continue

            # Edge detect
            if level != self._last_level:
                # rising: 0->1 ; falling: 1->0
                if self._count_edge == "rising":
                    if self._last_level == 0 and level == 1:
                        pulses_since_last += 1
                else:  # falling
                    if self._last_level == 1 and level == 0:
                        pulses_since_last += 1
                self._last_level = level

            time.sleep(poll_interval_s)

            now = time.monotonic()
            if (now - last_calc) >= 1.0:
                with self._lock:
                    pulses = self._pulse_count + pulses_since_last
                    self._pulse_count = 0
                pulses_since_last = 0

                with self._lock:
                    ppl = float(self.config.pulses_per_liter)
                    self._flow_rate_lpm = (pulses / ppl) * 60.0
                    self._total_liters += pulses / ppl
                    # clear error if we’re making progress
                    self._last_error = None

                last_calc = now

    def start(self) -> None:
        if plc is None:
            with self._lock:
                self._last_error = "librpiplc.rpiplc not available"
                self._running = False
            return

        with self._lock:
            if self._running:
                return
            self._running = True
            self._last_error = None

        needs_worker = self._worker is None or not self._worker.is_alive()
        if needs_worker:
            self._stop_worker = False
            self._worker = threading.Thread(target=self._loop, daemon=True)
            self._worker.start()

    def stop(self) -> None:
        with self._lock:
            self._running = False

    def reset_totals(self) -> None:
        with self._lock:
            self._flow_rate_lpm = 0.0
            self._total_liters = 0.0

    def read(self) -> dict:
        with self._lock:
            flow_ml_min = float(self._flow_rate_lpm) * 1000.0
            total_ml = float(self._total_liters) * 1000.0
            return {
                "flow_ml_min": flow_ml_min,
                "total_ml": total_ml,
                "total_l": float(self._total_liters),
            }

    def is_running(self) -> bool:
        with self._lock:
            return bool(self._running)

    def get_last_error(self) -> Optional[str]:
        with self._lock:
            return self._last_error

    def close(self) -> None:
        with self._lock:
            self._running = False
            self._stop_worker = True