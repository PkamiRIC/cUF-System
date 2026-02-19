import time
from typing import Callable, Optional


def run_concentration(
    *,
    stop_flag: Callable[[], bool],
    log: Optional[Callable[[str], None]] = None,
    reset_flow_totals: Callable[[], None],
    get_total_volume_ml: Callable[[], float],
    target_volume_ml: float,
    set_relay: Callable[[int, bool], bool],
    set_pump_direction: Callable[[bool], None],
    set_pump_enabled: Callable[[bool], None],
) -> None:
    """Port of Old_Codes/concentration.py using mL totals from current flow API."""

    def _log(msg: str) -> None:
        if log:
            log(msg)

    def open_v(channel: int) -> None:
        set_relay(channel, True)

    def close_v(channel: int) -> None:
        set_relay(channel, False)

    # V1,V2,V3,V8,V9,V10 -> R1,R2,R3,R4,R5,R6
    V1, V2, V3, V8, V9, V10 = 1, 2, 3, 4, 5, 6
    target_liters = max(0.0, float(target_volume_ml or 0.0)) / 1000.0

    def abort_cleanup() -> None:
        set_pump_enabled(False)
        open_v(V1)
        open_v(V2)
        open_v(V3)
        open_v(V8)
        open_v(V9)
        open_v(V10)

    def check_stop() -> bool:
        if stop_flag():
            _log("[INFO] Concentration sequence aborted.")
            abort_cleanup()
            return True
        return False

    _log("Starting Concentration sequence")
    reset_flow_totals()
    _log(f"[INFO] Target volume: {target_liters:.3f} L")

    if check_stop():
        raise RuntimeError("Operation stopped")

    close_v(V1)
    open_v(V2)
    open_v(V3)
    close_v(V8)
    open_v(V9)
    open_v(V10)

    set_pump_direction(True)
    set_pump_enabled(True)

    try:
        while True:
            if check_stop():
                raise RuntimeError("Operation stopped")
            total_liters = max(0.0, float(get_total_volume_ml() or 0.0)) / 1000.0
            _log(f"[DEBUG] Volume: {total_liters:.3f} L")
            if total_liters >= target_liters:
                _log("[INFO] Target volume reached.")
                break
            time.sleep(0.5)
    finally:
        set_pump_enabled(False)
        open_v(V1)
        open_v(V8)
        _log("Concentration sequence complete")
