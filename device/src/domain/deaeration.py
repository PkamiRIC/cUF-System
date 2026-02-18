import time
from typing import Callable, Optional


def run_deaeration(
    *,
    stop_flag: Callable[[], bool],
    log: Optional[Callable[[str], None]] = None,
    reset_flow_totals: Callable[[], None],
    set_relays: Callable[[list[int], bool], None],
    set_pump_direction: Callable[[bool], None],
    set_pump_low_speed: Callable[[bool], None],
    set_pump_enabled: Callable[[bool], None],
    duration_s: float = 25.0,
) -> None:
    """Legacy-style Deaeration sequence from MainGUI_v5 flow."""

    def _log(msg: str) -> None:
        if log:
            log(msg)

    _log("Deaeration: reset flow totals")
    reset_flow_totals()
    _log("Deaeration: open relays R1,R2,R3")
    set_relays([1, 2, 3], True)
    _log("Deaeration: set pump forward/high and enable")
    set_pump_direction(True)
    set_pump_low_speed(False)
    set_pump_enabled(True)
    try:
        end = time.time() + max(0.0, duration_s)
        while time.time() < end:
            if stop_flag():
                raise RuntimeError("Operation stopped")
            time.sleep(0.1)
    finally:
        _log("Deaeration: disable pump and close relays")
        set_pump_enabled(False)
        set_relays([1, 2, 3], False)

