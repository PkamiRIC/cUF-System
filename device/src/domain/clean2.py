import time
from typing import Callable, Optional


def run_clean2(
    *,
    stop_flag: Callable[[], bool],
    log: Optional[Callable[[str], None]] = None,
    set_relays: Callable[[list[int], bool], None],
    set_pump_direction: Callable[[bool], None],
    set_pump_low_speed: Callable[[bool], None],
    set_pump_enabled: Callable[[bool], None],
    duration_s: float = 20.0,
) -> None:
    """Legacy-style Clean 2 sequence from MainGUI_v5 flow."""

    def _log(msg: str) -> None:
        if log:
            log(msg)

    _log("Clean 2: open relays R3,R6")
    set_relays([3, 6], True)
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
        _log("Clean 2: disable pump and close relays")
        set_pump_enabled(False)
        set_relays([3, 6], False)

