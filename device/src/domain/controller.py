import asyncio
import json
import threading
import time
from dataclasses import asdict, dataclass, field
from typing import Optional

from hardware.flow_sensor import FlowSensor
from hardware.peristaltic_pump import PeristalticPump
from hardware.pid_valve import PidValveController
from hardware.plc_utils import plc, safe_plc_call, ensure_plc_init
from hardware.relay_board import RelayBoard
from infra.config import DeviceConfig
from domain.deaeration import run_deaeration
from domain.concentration import run_concentration
from domain.elution import run_elution
from domain.clean1 import run_clean1
from domain.clean2 import run_clean2


@dataclass
class DeviceState:
    state: str = "IDLE"  # IDLE, RUNNING, ERROR
    current_sequence: Optional[str] = None
    sequence_step: Optional[str] = None
    last_error: Optional[str] = None
    stop_requested: bool = False
    relay_states: dict = field(default_factory=dict)
    logs: list = field(default_factory=list)

    peristaltic_enabled: bool = False
    peristaltic_direction_cw: bool = True
    peristaltic_low_speed: bool = False

    pid_enabled: bool = False
    pid_setpoint: float = 80.0
    pid_hall: Optional[int] = None

    flow_ml_min: float = 0.0
    total_ml: float = 0.0
    flow_running: bool = False
    flow_error: Optional[str] = None

    pressure_in_mbar: Optional[float] = None
    pressure_out_mbar: Optional[float] = None
    filter_pressure_mbar: Optional[float] = None
    tmp_mbar: Optional[float] = None

    level_states: dict = field(default_factory=dict)
    level_sensors_disabled: bool = False
    target_volume_ml: Optional[float] = None


class DeviceController:
    def __init__(self, config: DeviceConfig):
        self.config = config
        self.relays = RelayBoard(config.relay)
        self.peristaltic = PeristalticPump(config.peristaltic)
        self.flow_sensor = FlowSensor(config.flow_sensor)
        self.pid_valve = PidValveController(config.pid_valve, self._read_tmp_for_pid)

        self.state = DeviceState()
        self.state.peristaltic_enabled = self.peristaltic.state.enabled
        self.state.peristaltic_direction_cw = self.peristaltic.state.direction_forward
        self.state.peristaltic_low_speed = self.peristaltic.state.low_speed
        self.state.pid_enabled = self.pid_valve.state.enabled
        self.state.pid_setpoint = self.pid_valve.state.setpoint

        self._stop_event = threading.Event()
        self._sequence_thread: Optional[threading.Thread] = None

        self._state_lock = threading.Lock()
        self._log_lock = threading.Lock()
        self._log_buffer: list[str] = []
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._sse_subscribers: list[asyncio.Queue] = []

        self._io_retry_attempts = 3
        self._io_retry_delay_s = 0.12
        self.relay_states = {ch: False for ch in range(1, 9)}

        self._ema_alpha = 0.5
        self._smoothed_pressures = [0.0, 0.0, 0.0]
        self._sensor_thread = threading.Thread(target=self._sensor_loop, daemon=True)
        self._sensor_thread.start()
        self.flow_sensor.start()

        self._log("Control console ready.")

    # ---------------------------------------------------
    # STATUS
    # ---------------------------------------------------
    def get_status(self) -> dict:
        with self._state_lock:
            self.state.peristaltic_enabled = self.peristaltic.state.enabled
            self.state.peristaltic_direction_cw = self.peristaltic.state.direction_forward
            self.state.peristaltic_low_speed = self.peristaltic.state.low_speed
            self.state.pid_enabled = self.pid_valve.state.enabled
            self.state.pid_setpoint = self.pid_valve.state.setpoint
            self.state.pid_hall = self.pid_valve.state.hall_state
            self.state.relay_states = dict(self.relay_states)
            self.state.logs = list(self._log_buffer)

            flow = self.flow_sensor.read()
            self.state.flow_ml_min = float(flow.get("flow_ml_min", 0.0))
            self.state.total_ml = float(flow.get("total_ml", 0.0))
            self.state.flow_running = self.flow_sensor.is_running()
            self.state.flow_error = self.flow_sensor.get_last_error()

            snapshot = self._snapshot_unlocked()
        return snapshot

    def attach_event_loop(self, loop: asyncio.AbstractEventLoop) -> None:
        self._loop = loop

    # ---------------------------------------------------
    # COMMANDS
    # ---------------------------------------------------
    def start_sequence(
        self,
        sequence_name: str,
        target_volume_ml: Optional[float] = None,
        temp_target_c: Optional[float] = None,
    ) -> None:
        if self._sequence_thread and self._sequence_thread.is_alive():
            raise RuntimeError("A sequence is already running")

        ok, messages = self._check_start_conditions()
        if not ok:
            raise RuntimeError("; ".join(messages))

        with self._state_lock:
            self.state.state = "RUNNING"
            self.state.current_sequence = sequence_name
            self.state.sequence_step = "Starting"
            self.state.last_error = None
            self.state.stop_requested = False
            self.state.target_volume_ml = target_volume_ml
        self._broadcast_status()

        self._stop_event.clear()
        self._sequence_thread = threading.Thread(
            target=self._run_sequence, args=(sequence_name,), daemon=True
        )
        self._sequence_thread.start()

    def stop_sequence(self) -> None:
        self._stop_event.set()
        try:
            self.peristaltic.set_enabled(False)
        except Exception:
            pass
        try:
            self.pid_valve.set_enabled(False)
        except Exception:
            pass
        try:
            self._force_close_elute_valve()
        except Exception:
            pass
        with self._state_lock:
            self.state.stop_requested = True
            self.state.state = "ERROR"
            self.state.last_error = "Operation stopped"
            self.state.current_sequence = None
            self.state.sequence_step = None
        self._broadcast_status()

    def emergency_stop(self) -> None:
        self.stop_sequence()
        self._log("[E-STOP] Emergency stop triggered")
        try:
            self.peristaltic.force_stop()
        except Exception:
            pass
        try:
            self.pid_valve.set_enabled(False)
        except Exception:
            pass
        try:
            self.relays.all_off()
            for ch in range(1, 9):
                self.relay_states[ch] = False
        except Exception:
            pass
        with self._state_lock:
            self.state.state = "ERROR"
            self.state.current_sequence = None
            self.state.last_error = "Emergency stop activated"
        self._broadcast_status()

    def home_all(self) -> None:
        self.pid_home()

    def set_relay(self, channel: int, enabled: bool) -> bool:
        self._ensure_manual_allowed()
        self._clear_last_error()
        self._log(f"[Relay] R{channel} {'ON' if enabled else 'OFF'}")
        return self._retry_bool(
            f"Relay R{channel} {'ON' if enabled else 'OFF'}",
            lambda: self._set_relay(channel, enabled),
        )

    def set_all_relays(self, enabled: bool) -> bool:
        self._ensure_manual_allowed()
        self._clear_last_error()
        ok = self._retry_bool(
            f"Relays ALL {'ON' if enabled else 'OFF'}",
            lambda: self.relays.all_on() if enabled else self.relays.all_off(),
        )
        if ok:
            for ch in range(1, 9):
                self.relay_states[ch] = enabled
            with self._state_lock:
                self.state.relay_states = dict(self.relay_states)
            self._broadcast_status()
        return ok

    def set_peristaltic_enabled(self, enabled: bool) -> None:
        self._ensure_manual_allowed()
        self._clear_last_error()
        self.peristaltic.set_enabled(enabled)
        with self._state_lock:
            self.state.peristaltic_enabled = self.peristaltic.state.enabled
        self._broadcast_status()

    def set_peristaltic_direction(self, forward: bool) -> None:
        self._ensure_manual_allowed()
        self._clear_last_error()
        self.peristaltic.set_direction(forward)
        with self._state_lock:
            self.state.peristaltic_direction_cw = self.peristaltic.state.direction_forward
        self._broadcast_status()

    def set_peristaltic_speed(self, low_speed: bool) -> None:
        self._ensure_manual_allowed()
        self._clear_last_error()
        self.peristaltic.set_speed_checked(low_speed)
        with self._state_lock:
            self.state.peristaltic_low_speed = self.peristaltic.state.low_speed
        self._broadcast_status()

    def set_pid_enabled(self, enabled: bool) -> None:
        self._ensure_manual_allowed()
        self._clear_last_error()
        self._log(f"[PID] {'Enabled' if enabled else 'Disabled'}")
        self._retry_void(
            f"PID {'enable' if enabled else 'disable'}",
            lambda: self.pid_valve.set_enabled(enabled),
        )
        with self._state_lock:
            self.state.pid_enabled = self.pid_valve.state.enabled
        self._broadcast_status()

    def set_pid_setpoint(self, value: float) -> None:
        self._ensure_manual_allowed()
        self._clear_last_error()
        self._log(f"[PID] Setpoint {value}")
        self._retry_void(
            f"PID setpoint {value}",
            lambda: self.pid_valve.set_setpoint(value),
        )
        with self._state_lock:
            self.state.pid_setpoint = self.pid_valve.state.setpoint
        self._broadcast_status()

    def pid_home(self) -> None:
        self._ensure_manual_allowed()
        self._clear_last_error()
        self._log("[PID] Home")
        self._retry_void("PID home", self.pid_valve.homing_routine)
        self._broadcast_status()

    def pid_close(self) -> None:
        self._ensure_manual_allowed()
        self._clear_last_error()
        self._log("[PID] Close")
        self._retry_void("PID close", self.pid_valve.force_close)
        self._broadcast_status()

    def flow_start(self) -> None:
        self._ensure_manual_allowed()
        self._clear_last_error()
        self.flow_sensor.start()
        with self._state_lock:
            self.state.flow_running = self.flow_sensor.is_running()
            self.state.flow_error = self.flow_sensor.get_last_error()
        self._broadcast_status()

    def flow_stop(self) -> None:
        self._ensure_manual_allowed()
        self._clear_last_error()
        self.flow_sensor.stop()
        with self._state_lock:
            self.state.flow_running = self.flow_sensor.is_running()
            self.state.flow_error = self.flow_sensor.get_last_error()
        self._broadcast_status()

    def flow_reset(self) -> None:
        self._ensure_manual_allowed()
        self._clear_last_error()
        self.flow_sensor.reset_totals()
        with self._state_lock:
            self.state.flow_error = self.flow_sensor.get_last_error()
        self._broadcast_status()

    def set_level_sensors_disabled(self, disabled: bool) -> None:
        with self._state_lock:
            self.state.level_sensors_disabled = bool(disabled)
        self._log(
            "Level sensors check disabled."
            if disabled
            else "Level sensors check enabled."
        )
        self._broadcast_status()

    # ---------------------------------------------------
    # NOT USED BY MainGUI_v5 (kept for API compatibility)
    # ---------------------------------------------------
    def move_syringe(self, volume_ml: float, flow_ml_min: float) -> None:
        raise RuntimeError("Syringe is not used in MainGUI_v5 backend")

    def stop_syringe(self) -> None:
        raise RuntimeError("Syringe is not used in MainGUI_v5 backend")

    def home_syringe(self) -> None:
        raise RuntimeError("Syringe is not used in MainGUI_v5 backend")

    def move_axis(self, axis: str, position_mm: float, rpm: float) -> None:
        raise RuntimeError("Axes are not used in MainGUI_v5 backend")

    def home_axis(self, axis: str) -> None:
        raise RuntimeError("Axes are not used in MainGUI_v5 backend")

    def set_temp_enabled(self, enabled: bool) -> None:
        raise RuntimeError("Temperature controller is not used in MainGUI_v5 backend")

    def set_temp_target(self, target_c: float) -> None:
        raise RuntimeError("Temperature controller is not used in MainGUI_v5 backend")

    # ---------------------------------------------------
    # Internals
    # ---------------------------------------------------
    def clear_logs(self) -> None:
        with self._log_lock:
            self._log_buffer = []
        with self._state_lock:
            self.state.logs = []
        self._broadcast_status()

    def _sensor_loop(self) -> None:
        ensure_plc_init()
        pressure_pins = ["I0.9", "I0.10", "I0.11"]
        levels = [
            ("H2O", "I1.5"),
            ("NaOH", "I1.4"),
            ("Drain Sample", "I1.3"),
            ("Drain Cleaning", "I1.2"),
        ]

        while True:
            try:
                pressures = []
                if plc:
                    for i, pin in enumerate(pressure_pins):
                        raw = safe_plc_call("analog_read", plc.analog_read, pin)
                        try:
                            raw_f = float(raw)
                        except Exception:
                            raw_f = 0.0
                        voltage = (raw_f / 4095.0) * 10.0
                        mbar = voltage * 250.0
                        self._smoothed_pressures[i] = (
                            self._ema_alpha * mbar +
                            (1.0 - self._ema_alpha) * self._smoothed_pressures[i]
                        )
                        pressures.append(self._smoothed_pressures[i])

                with self._state_lock:
                    if len(pressures) == 3:
                        self.state.pressure_in_mbar = pressures[0]
                        self.state.pressure_out_mbar = pressures[1]
                        self.state.filter_pressure_mbar = pressures[2]
                        self.state.tmp_mbar = ((pressures[0] + pressures[1]) / 2.0) - pressures[2]

                    level_states = {}
                    for name, pin in levels:
                        raw = safe_plc_call("digital_read", plc.digital_read, pin) if plc else 0
                        state = bool(raw)
                        active = (not state) if name in ("Drain Sample", "Drain Cleaning") else state
                        level_states[name] = bool(active)
                    self.state.level_states = level_states
            except Exception:
                pass

            self._broadcast_status()
            time.sleep(1.0)

    def _read_tmp_for_pid(self) -> float:
        with self._state_lock:
            return float(self.state.tmp_mbar or 0.0)

    def _run_sequence(self, sequence_name: str) -> None:
        seq = sequence_name.lower()
        try:
            self._force_close_elute_valve()

            if seq in {"full sequence", "full_sequence", "full", "sequence_full"}:
                self._run_full_sequence()
            elif seq in {"deaeration"}:
                self._run_deaeration()
            elif seq in {"concentration", "sequence1", "seq1", "maf", "maf1"}:
                self._run_concentration()
            elif seq in {"elution", "sequence2", "seq2", "maf2"}:
                self._run_elution()
            elif seq in {"clean 1", "clean1"}:
                self._run_clean1()
            elif seq in {"clean 2", "clean2", "cleaning", "clean", "cleaning_sequence"}:
                self._run_clean2()
            else:
                raise ValueError(f"Unknown sequence '{sequence_name}'")

            with self._state_lock:
                self.state.state = "IDLE"
                self.state.last_error = None
        except Exception as exc:
            with self._state_lock:
                self.state.state = "ERROR"
                self.state.last_error = str(exc)
        finally:
            with self._state_lock:
                self.state.current_sequence = None
                self.state.sequence_step = None
                self.state.stop_requested = False
                self.state.target_volume_ml = None
            self.peristaltic.set_enabled(False)
            self.pid_valve.set_enabled(False)
            self._force_close_elute_valve()
            self._stop_event.clear()
            self._broadcast_status()

    def _run_full_sequence(self) -> None:
        for step in ("deaeration", "concentration", "elution"):
            self._check_stop()
            self._set_sequence_step(f"{step.title()} start")
            if step == "deaeration":
                self._run_deaeration()
            elif step == "concentration":
                self._run_concentration()
            else:
                self._run_elution()
            self._set_sequence_step(f"{step.title()} complete")

    def _run_deaeration(self) -> None:
        self._set_sequence_step("Deaeration")
        run_deaeration(
            stop_flag=lambda: self._stop_event.is_set(),
            log=self._append_log,
            reset_flow_totals=self.flow_sensor.reset_totals,
            set_relays=self._set_relays,
            set_pump_direction=self.peristaltic.set_direction,
            set_pump_low_speed=self.peristaltic.set_speed_checked,
            set_pump_enabled=self.peristaltic.set_enabled,
            duration_s=25.0,
        )

    def _run_concentration(self) -> None:
        self._set_sequence_step("Concentration")
        run_concentration(
            stop_flag=lambda: self._stop_event.is_set(),
            log=self._append_log,
            reset_flow_totals=self.flow_sensor.reset_totals,
            get_total_volume_ml=lambda: float(self.flow_sensor.read().get("total_ml", 0.0)),
            target_volume_ml=float(self.state.target_volume_ml or 1000.0),
            set_relays=self._set_relays,
            set_pump_direction=self.peristaltic.set_direction,
            set_pump_low_speed=self.peristaltic.set_speed_checked,
            set_pump_enabled=self.peristaltic.set_enabled,
            set_pid_enabled=self.pid_valve.set_enabled,
            home_pid=self.pid_valve.homing_routine,
        )

    def _run_elution(self) -> None:
        self._set_sequence_step("Elution")
        run_elution(
            stop_flag=lambda: self._stop_event.is_set(),
            log=self._append_log,
            set_relay=self._set_relay,
            set_relays=self._set_relays,
            set_pump_direction=self.peristaltic.set_direction,
            set_pump_low_speed=self.peristaltic.set_speed_checked,
            set_pump_enabled=self.peristaltic.set_enabled,
            duration_s=20.0,
        )

    def _run_clean1(self) -> None:
        self._set_sequence_step("Clean 1")
        run_clean1(
            stop_flag=lambda: self._stop_event.is_set(),
            log=self._append_log,
            set_relays=self._set_relays,
            set_pump_direction=self.peristaltic.set_direction,
            set_pump_low_speed=self.peristaltic.set_speed_checked,
            set_pump_enabled=self.peristaltic.set_enabled,
            duration_s=20.0,
        )

    def _run_clean2(self) -> None:
        self._set_sequence_step("Clean 2")
        run_clean2(
            stop_flag=lambda: self._stop_event.is_set(),
            log=self._append_log,
            set_relays=self._set_relays,
            set_pump_direction=self.peristaltic.set_direction,
            set_pump_low_speed=self.peristaltic.set_speed_checked,
            set_pump_enabled=self.peristaltic.set_enabled,
            duration_s=20.0,
        )

    def _run_pump_phase(self, open_relays: list[int], seconds: float, forward: bool, low_speed: bool) -> None:
        self._set_relays(open_relays, True)
        self.peristaltic.set_direction(forward)
        self.peristaltic.set_speed_checked(low_speed)
        self.peristaltic.set_enabled(True)
        try:
            self._wait_interruptible(seconds)
        finally:
            self.peristaltic.set_enabled(False)
            self._set_relays(open_relays, False)

    def _force_close_elute_valve(self) -> None:
        # MainGUI_v5 reset behavior: relay 7 set to default (non-collect) state.
        self._set_relay(7, True)

    def _set_relays(self, channels: list[int], enabled: bool) -> None:
        for ch in channels:
            self._set_relay(ch, enabled)

    def _wait_interruptible(self, seconds: float) -> None:
        end = time.time() + max(0.0, seconds)
        while time.time() < end:
            self._check_stop()
            time.sleep(0.1)

    def _check_stop(self) -> None:
        if self._stop_event.is_set():
            raise RuntimeError("Operation stopped")

    def _set_sequence_step(self, step: str) -> None:
        with self._state_lock:
            self.state.sequence_step = step
        self._append_log(step)
        self._broadcast_status()

    def _check_start_conditions(self):
        if bool(self.state.level_sensors_disabled):
            return (True, [])
        states = dict(self.state.level_states or {})
        messages = []
        if not states.get("H2O", False):
            messages.append("Fill H2O")
        if not states.get("NaOH", False):
            messages.append("Fill NaOH")
        if not states.get("Drain Sample", False):
            messages.append("Empty Drain Sample")
        if not states.get("Drain Cleaning", False):
            messages.append("Empty Drain Cleaning")
        return (len(messages) == 0, messages)

    def _set_relay(self, channel: int, enabled: bool) -> bool:
        ok = self.relays.on(channel) if enabled else self.relays.off(channel)
        if ok:
            self.relay_states[channel] = enabled
            with self._state_lock:
                self.state.relay_states = dict(self.relay_states)
            self._broadcast_status()
        return ok

    def _ensure_manual_allowed(self) -> None:
        with self._state_lock:
            running = self.state.state == "RUNNING"
            current = self.state.current_sequence
        if running:
            op = current or "operation"
            raise RuntimeError(f"Manual command blocked while '{op}' is running")

    def _clear_last_error(self) -> None:
        with self._state_lock:
            if self.state.last_error is not None:
                self.state.last_error = None
                if self.state.state == "ERROR":
                    self.state.state = "IDLE"
        self._broadcast_status()

    def _retry_bool(self, label: str, fn):
        last_ok = False
        for attempt in range(1, self._io_retry_attempts + 1):
            try:
                last_ok = bool(fn())
            except Exception as exc:
                last_ok = False
                self._log(f"[Retry] {label} failed ({exc})")
            if last_ok:
                return True
            if attempt < self._io_retry_attempts:
                self._log(f"[Retry] {label} attempt {attempt + 1}/{self._io_retry_attempts}")
                time.sleep(self._io_retry_delay_s)
        return False

    def _retry_void(self, label: str, fn) -> None:
        for attempt in range(1, self._io_retry_attempts + 1):
            try:
                fn()
                return
            except Exception as exc:
                self._log(f"[Retry] {label} failed ({exc})")
                if attempt < self._io_retry_attempts:
                    self._log(f"[Retry] {label} attempt {attempt + 1}/{self._io_retry_attempts}")
                    time.sleep(self._io_retry_delay_s)
        raise RuntimeError(f"{label} failed after {self._io_retry_attempts} attempts")

    def _log(self, message: str) -> None:
        self._append_log(message)
        self._broadcast_status()

    def _append_log(self, message: str) -> None:
        with self._log_lock:
            self._log_buffer.append(message)
            self._log_buffer = self._log_buffer[-500:]

    def _broadcast_status(self) -> None:
        if not self._loop or not self._sse_subscribers:
            return
        with self._state_lock:
            snapshot = self._snapshot_unlocked()
        payload = json.dumps(snapshot)
        for q in list(self._sse_subscribers):
            try:
                self._loop.call_soon_threadsafe(q.put_nowait, payload)
            except Exception:
                continue

    def _snapshot_unlocked(self) -> dict:
        return {"device_id": self.config.device_id, **asdict(self.state)}
