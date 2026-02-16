import signal
import threading
import time
from typing import Optional

from infra.config import FlowSensorConfig

try:
    import RPi.GPIO as GPIO  # type: ignore
except Exception:  # pragma: no cover
    GPIO = None


class FlowSensor:
    """
    MainGUI_v5-style pulse flow meter.
    Converts pulses to mL/min and accumulates total mL.
    """

    def __init__(self, config: FlowSensorConfig) -> None:
        self.config = config
        self._running = False
        self._lock = threading.Lock()
        self._last_error: Optional[str] = None
        self._pulse_count = 0
        self._flow_ml_min = 0.0
        self._total_ml = 0.0
        self._worker: Optional[threading.Thread] = None

    def _pulse_callback(self, channel):
        with self._lock:
            self._pulse_count += 1

    def _loop(self):
        while self._running:
            time.sleep(1.0)
            with self._lock:
                pulses = self._pulse_count
                self._pulse_count = 0
                liters_per_min = (pulses / float(self.config.pulses_per_liter)) * 60.0
                self._flow_ml_min = liters_per_min * 1000.0
                self._total_ml += (pulses / float(self.config.pulses_per_liter)) * 1000.0

    def start(self) -> None:
        if GPIO is None:
            self._last_error = "RPi.GPIO not available"
            self._running = False
            return

        with self._lock:
            if self._running:
                return

        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.config.gpio_bcm, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
            GPIO.add_event_detect(
                self.config.gpio_bcm,
                GPIO.RISING,
                callback=self._pulse_callback,
                bouncetime=1,
            )
            signal.signal(signal.SIGINT, lambda *_: GPIO.cleanup())
            with self._lock:
                self._running = True
                self._last_error = None
            self._worker = threading.Thread(target=self._loop, daemon=True)
            self._worker.start()
        except Exception as exc:
            with self._lock:
                self._running = False
                self._last_error = str(exc)

    def stop(self) -> None:
        with self._lock:
            self._running = False
        if GPIO is not None:
            try:
                GPIO.remove_event_detect(self.config.gpio_bcm)
            except Exception:
                pass

    def reset_totals(self) -> None:
        with self._lock:
            self._flow_ml_min = 0.0
            self._total_ml = 0.0
            self._pulse_count = 0

    def read(self) -> dict:
        with self._lock:
            return {
                "flow_ml_min": float(self._flow_ml_min),
                "total_ml": float(self._total_ml),
                "total_l": float(self._total_ml) / 1000.0,
            }

    def is_running(self) -> bool:
        with self._lock:
            return bool(self._running)

    def get_last_error(self) -> Optional[str]:
        with self._lock:
            return self._last_error

    def close(self) -> None:
        self.stop()
        if GPIO is not None:
            try:
                GPIO.cleanup()
            except Exception:
                pass
