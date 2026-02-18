import time
from typing import Callable, Optional


def run_concentration(
    *,
    stop_flag: Callable[[], bool],
    log: Optional[Callable[[str], None]] = None,
    reset_flow_totals: Callable[[], None],
    get_total_volume_ml: Callable[[], float],
    target_volume_ml: float,
    set_relays: Callable[[list[int], bool], None],
    set_pump_direction: Callable[[bool], None],
    set_pump_low_speed: Callable[[bool], None],
    set_pump_enabled: Callable[[bool], None],
    set_pid_enabled: Callable[[bool], None],
    home_pid: Callable[[], None],
) -> None:
    """Legacy-style Concentration sequence from MainGUI_v5 flow."""

    def _log(msg: str) -> None:
        if log:
            log(msg)

    _log(f"Concentration: target volume {target_volume_ml:.1f} mL")
    reset_flow_totals()
    set_pid_enabled(True)
    set_pump_direction(True)
    set_pump_low_speed(True)
    set_pump_enabled(True)
    set_relays([1, 4, 5], True)
    try:
        while True:
            if stop_flag():
                raise RuntimeError("Operation stopped")
            total_ml = float(get_total_volume_ml() or 0.0)
            if total_ml >= float(target_volume_ml):
                _log(f"Concentration: target reached ({total_ml:.2f} mL)")
                break
            time.sleep(0.2)
    finally:
        _log("Concentration: disable pump/PID, close relays, home PID")
        set_pump_enabled(False)
        set_pid_enabled(False)
        set_relays([1, 4, 5], False)
        home_pid()

