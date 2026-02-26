import threading
import time
from typing import Optional

from hardware.plc_utils import ensure_plc_init, plc, safe_plc_call
from infra.config import FlowSensorConfig

class FlowSensor:
    """
    MainGUI_v5-style pulse flow meter.
    Converts pulses to mL/min and accumulates total mL.
    """

    def __init__(self, config: FlowSensorConfig) -> None:
        self.config = config
        self._running = False
        self._stop_worker = False
        self._plc_ready = False
        self._lock = threading.Lock()
        self._last_error: Optional[str] = None
        self._flow_rate_lpm = 0.0
        self._total_liters = 0.0
        self._worker: Optional[threading.Thread] = None
        self._plc_input_pin = self._resolve_plc_input_pin(config.gpio_bcm)
        self._last_level: Optional[int] = None

    @staticmethod
    def _resolve_plc_input_pin(gpio_bcm: int) -> str:
        # Map known MainGUI/Industrial Shields interrupt aliases to PLC input names.
        mapping = {
            27: "I1.0",
            17: "I1.1",
        }
        return mapping.get(int(gpio_bcm), "I1.0")

    def _loop(self):
        # Keep MainGUI_v5 behavior: compute flow every 1s continuously once started.
        poll_interval_s = 0.0005
        pulses_since_last = 0
        last_calc = time.monotonic()
        while not self._stop_worker:
            with self._lock:
                if not self._running:
                    self._flow_rate_lpm = 0.0
                time.sleep(0.05)
                last_calc = time.monotonic()
                continue

            raw = safe_plc_call("digital_read", plc.digital_read, self._plc_input_pin) if plc else None
            if raw is not None:
                try:
                    level = 1 if bool(int(raw)) else 0
                except Exception:
                    level = 1 if bool(raw) else 0

                with self._lock:
                    prev = self._last_level
                    self._last_level = level

                # Count falling edge by default (high -> low).
                if prev is not None and prev == 1 and level == 0:
                    pulses_since_last += 1

            now = time.monotonic()
            if (now - last_calc) >= 1.0:
                pulses = pulses_since_last
                pulses_since_last = 0
                with self._lock:
                    self._flow_rate_lpm = (pulses / float(self.config.pulses_per_liter)) * 60.0
                    self._total_liters += pulses / float(self.config.pulses_per_liter)
                last_calc = now

            time.sleep(poll_interval_s)

    def start(self) -> None:
        if plc is None:
            with self._lock:
                self._last_error = "librpiplc not available"
                self._running = False
            return

        try:
            ensure_plc_init()
            with self._lock:
                already_running = self._running
                plc_ready = self._plc_ready
            if already_running:
                return

            if not plc_ready:
                probe = safe_plc_call("digital_read", plc.digital_read, self._plc_input_pin)
                if probe is None:
                    raise RuntimeError(f"Cannot read PLC input {self._plc_input_pin}")
                with self._lock:
                    try:
                        self._last_level = 1 if bool(int(probe)) else 0
                    except Exception:
                        self._last_level = 1 if bool(probe) else 0

            with self._lock:
                self._running = True
                self._plc_ready = True
                self._last_error = None
                needs_worker = self._worker is None or not self._worker.is_alive()
            if needs_worker:
                self._worker = threading.Thread(target=self._loop, daemon=True)
                self._worker.start()
        except Exception as exc:
            with self._lock:
                self._running = False
                self._last_error = str(exc)

    def stop(self) -> None:
        with self._lock:
            self._running = False

    def reset_totals(self) -> None:
        # Match MainGUI_v5 reset: clear flow and totals only.
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
