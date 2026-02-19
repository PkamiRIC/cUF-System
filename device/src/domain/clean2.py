from typing import Callable, Optional

from domain.sleeper import InterruptibleSleeper


def run_clean2(
    *,
    stop_flag: Callable[[], bool],
    log: Optional[Callable[[str], None]] = None,
    reset_flow_totals: Callable[[], None],
    set_relay: Callable[[int, bool], bool],
    set_pump_direction: Callable[[bool], None],
    set_pump_low_speed: Callable[[bool], None],
    set_pump_enabled: Callable[[bool], None],
) -> None:
    """Port of Old_Codes/clean2.py using relay channels."""

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
            _log("[INFO] Clean 2 sequence aborted.")
            abort_cleanup()
            return True
        return False

    _log("Starting Clean 2 sequence")
    reset_flow_totals()

    try:
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
        set_pump_low_speed(True)
        sleeper.sleep(15)

        # Step 2
        if check_stop():
            raise RuntimeError("Operation stopped")
        close_v(V1)
        sleeper.sleep(6)

        # Step 3
        if check_stop():
            raise RuntimeError("Operation stopped")
        open_v(V1)
        sleeper.sleep(10)

        # Step 4
        if check_stop():
            raise RuntimeError("Operation stopped")
        open_v(V2)
        close_v(V3)
        sleeper.sleep(30)

        # Step 5
        if check_stop():
            raise RuntimeError("Operation stopped")
        close_v(V2)
        open_v(V3)
        sleeper.sleep(10)

        # Step 6
        if check_stop():
            raise RuntimeError("Operation stopped")
        close_v(V1)
        open_v(V2)
        sleeper.sleep(7)

        # Step 7
        if check_stop():
            raise RuntimeError("Operation stopped")
        open_v(V1)
        open_v(V2)
        close_v(V9)
        open_v(V10)
        set_pump_enabled(False)
        sleeper.sleep(1)

        _log("Clean 2 sequence complete")

    except InterruptedError:
        _log("[INFO] Sequence stopped during sleep.")
        check_stop()
        raise RuntimeError("Operation stopped")
