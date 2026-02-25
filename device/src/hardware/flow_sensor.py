import threading
import time
from typing import Optional

from infra.config import FlowSensorConfig

try:
    import pigpio  # type: ignore
except Exception:  # pragma: no cover
    pigpio = None


class FlowSensor:
    """
    MainGUI_v5-style pulse flow meter.
    Converts pulses to mL/min and accumulates total mL.
    """

    def __init__(self, config: FlowSensorConfig) -> None:
        self.config = config
        self._running = False
        self._stop_worker = False
        self._gpio_ready = False
        self._lock = threading.Lock()
        self._last_error: Optional[str] = None
        self._pulse_count = 0
        self._flow_rate_lpm = 0.0
        self._total_liters = 0.0
        self._worker: Optional[threading.Thread] = None
        self._pi = None
        self._cb = None

    def _pulse_callback(self, gpio: int, level: int, tick: int) -> None:
        # Open-collector flow sensor pulses are counted on falling edges.
        if level != 0:
            return
        with self._lock:
            self._pulse_count += 1

    def _loop(self):
        # Keep MainGUI_v5 behavior: compute flow every 1s continuously once started.
        while not self._stop_worker:
            time.sleep(1.0)
            with self._lock:
                if not self._running:
                    self._flow_rate_lpm = 0.0
                    continue
                pulses = self._pulse_count
                self._pulse_count = 0
                self._flow_rate_lpm = (pulses / float(self.config.pulses_per_liter)) * 60.0
                self._total_liters += pulses / float(self.config.pulses_per_liter)

    def start(self) -> None:
        if pigpio is None:
            with self._lock:
                self._last_error = "pigpio not available"
                self._running = False
            return

        try:
            with self._lock:
                already_running = self._running
                gpio_ready = self._gpio_ready
            if already_running:
                return

            if not gpio_ready:
                pi = pigpio.pi()
                if not pi.connected:
                    raise RuntimeError("pigpio daemon not running")
                pi.set_mode(self.config.gpio_bcm, pigpio.INPUT)
                pi.set_pull_up_down(self.config.gpio_bcm, pigpio.PUD_UP)
                cb = pi.callback(self.config.gpio_bcm, pigpio.EITHER_EDGE, self._pulse_callback)
                self._pi = pi
                self._cb = cb

            with self._lock:
                self._running = True
                self._gpio_ready = True
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
        cb = None
        pi = None
        with self._lock:
            self._running = False
            self._stop_worker = True
            cb = self._cb
            pi = self._pi
            self._cb = None
            self._pi = None
        try:
            if cb is not None:
                cb.cancel()
        except Exception:
            pass
        try:
            if pi is not None:
                pi.stop()
        except Exception:
            pass
