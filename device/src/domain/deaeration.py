from typing import Callable, Optional

from domain.sleeper import InterruptibleSleeper


def run_deaeration(
    *,
    stop_flag: Callable[[], bool],
    log: Optional[Callable[[str], None]] = None,
    reset_flow_totals: Callable[[], None],
    set_relay: Callable[[int, bool], bool],
    set_pump_direction: Callable[[bool], None],
    set_pump_low_speed: Callable[[bool], None],
    set_pump_enabled: Callable[[bool], None],
) -> None:
    """Port of Old_Codes/deaeration.py using relay channels instead of Valve objects."""

    def _log(msg: str) -> None:
        if log:
            log(msg)

    def open_v(channel: int) -> None:
        set_relay(channel, True)

    def close_v(channel: int) -> None:
        set_relay(channel, False)

    # V1,V2,V3,V8,V9,V10 -> R1,R2,R3,R4,R5,R6
    V1, V2, V3, V8, V9, V10 = 1, 2, 3, 4, 5, 6
    sleeper = InterruptibleSleeper(stop_flag)

    def abort_cleanup() -> None:
        set_pump_enabled(False)
        set_pump_direction(False)
        set_pump_low_speed(False)
        open_v(V1)
        open_v(V2)
        open_v(V3)
        open_v(V8)
        open_v(V9)
        open_v(V10)

    def check_stop() -> bool:
        if stop_flag():
            _log("[INFO] Deaeration sequence aborted.")
            abort_cleanup()
            return True
        return False

    _log("Starting Deaeration sequence")
    reset_flow_totals()

    if check_stop():
        raise RuntimeError("Operation stopped")

    # Step 1
    open_v(V1)
    close_v(V2)
    open_v(V3)
    open_v(V8)
    close_v(V9)
    open_v(V10)
    set_pump_enabled(True)
    set_pump_direction(True)
    set_pump_low_speed(False)
    try:
        sleeper.sleep(12)
    except InterruptedError:
        check_stop()
        raise RuntimeError("Operation stopped")

    # Step 2
    close_v(V1)
    open_v(V9)
    close_v(V8)
    try:
        sleeper.sleep(6)
    except InterruptedError:
        check_stop()
        raise RuntimeError("Operation stopped")

    # Step 3
    open_v(V1)
    open_v(V8)
    close_v(V9)
    try:
        sleeper.sleep(10)
    except InterruptedError:
        check_stop()
        raise RuntimeError("Operation stopped")

    # Step 4
    open_v(V2)
    try:
        sleeper.sleep(1)
    except InterruptedError:
        check_stop()
        raise RuntimeError("Operation stopped")

    # Step 5
    close_v(V2)
    try:
        sleeper.sleep(10)
    except InterruptedError:
        check_stop()
        raise RuntimeError("Operation stopped")

    # Step 6
    close_v(V1)
    open_v(V2)
    try:
        sleeper.sleep(7)
    except InterruptedError:
        check_stop()
        raise RuntimeError("Operation stopped")

    # Step 7
    open_v(V1)
    set_pump_enabled(False)
    set_pump_direction(False)
    set_pump_low_speed(False)
    try:
        sleeper.sleep(1)
    except InterruptedError:
        check_stop()
        raise RuntimeError("Operation stopped")

    _log("Deaeration complete")
