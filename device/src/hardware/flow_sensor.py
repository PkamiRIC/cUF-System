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
        self._stop_worker = False
        self._gpio_ready = False
        self._lock = threading.Lock()
        self._last_error: Optional[str] = None
        self._pulse_count = 0
        self._flow_rate_lpm = 0.0
        self._total_liters = 0.0
        self._worker: Optional[threading.Thread] = None
        self._use_polling = False
        self._last_level = 1

    def _loop(self):
        # MainGUI_v5-compatible: compute flow each second from pulse count.
        poll_interval_s = 0.0005
        pulses_since_last = 0
        last_calc = time.monotonic()
        while not self._stop_worker:
            with self._lock:
                running = self._running
                use_polling = self._use_polling

            if not running:
                with self._lock:
                    self._flow_rate_lpm = 0.0
                time.sleep(0.05)
                last_calc = time.monotonic()
                continue

            if use_polling and GPIO is not None:
                try:
                    level = int(GPIO.input(self.config.gpio_bcm))
                    with self._lock:
                        if self._last_level == 1 and level == 0:
                            pulses_since_last += 1
                        self._last_level = level
                except Exception:
                    pass
                time.sleep(poll_interval_s)
            else:
                time.sleep(0.01)

            now = time.monotonic()
            if (now - last_calc) >= 1.0:
                with self._lock:
                    pulses = self._pulse_count + pulses_since_last
                    self._pulse_count = 0
                pulses_since_last = 0
                with self._lock:
                    self._flow_rate_lpm = (pulses / float(self.config.pulses_per_liter)) * 60.0
                    self._total_liters += pulses / float(self.config.pulses_per_liter)
                last_calc = now

    def _pulse_callback(self, channel) -> None:
        with self._lock:
            self._pulse_count += 1

    def start(self) -> None:
        if GPIO is None:
            with self._lock:
                self._last_error = "RPi.GPIO not available"
                self._running = False
            return

        try:
            with self._lock:
                already_running = self._running
                gpio_ready = self._gpio_ready
            if already_running:
                return

            if not gpio_ready:
                try:
                    GPIO.remove_event_detect(self.config.gpio_bcm)
                except Exception:
                    pass
                try:
                    GPIO.cleanup(self.config.gpio_bcm)
                except Exception:
                    pass
                GPIO.setmode(GPIO.BCM)
                # Sensor pulses are low-going in this setup: idle HIGH, pulse LOW.
                GPIO.setup(self.config.gpio_bcm, GPIO.IN, pull_up_down=GPIO.PUD_UP)
                try:
                    GPIO.add_event_detect(
                        self.config.gpio_bcm,
                        GPIO.FALLING,
                        callback=self._pulse_callback,
                        bouncetime=1,
                    )
                    with self._lock:
                        self._use_polling = False
                except Exception:
                    # Fallback when edge IRQ registration is unavailable.
                    with self._lock:
                        self._use_polling = True
                        try:
                            self._last_level = int(GPIO.input(self.config.gpio_bcm))
                        except Exception:
                            self._last_level = 1

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
        with self._lock:
            self._running = False
            self._stop_worker = True
        if GPIO is not None:
            try:
                if self._gpio_ready:
                    GPIO.remove_event_detect(self.config.gpio_bcm)
            except Exception:
                pass
            try:
                GPIO.cleanup()
            except Exception:
                pass
