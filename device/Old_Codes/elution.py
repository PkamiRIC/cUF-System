import time
from sleeper import InterruptibleSleeper

def run_elution(valves, motor_pump, stop_flag, flow_meter, reset_flow_and_timer):
    print("Starting Elution sequence")

    # Reset flow and timer at start
    reset_flow_and_timer()
    flow_meter.reset()

    sleeper = InterruptibleSleeper(stop_flag)

    V1, V2, V3, V8, V9, V10 = valves[0], valves[1], valves[2], valves[3], valves[4], valves[5]

    def check_stop():
        if stop_flag():
            print("[INFO] Elution sequence aborted.")
            motor_pump.set_enabled(False)
            motor_pump.set_direction(False)
            motor_pump.set_speed_checked(False)
            V1.open(); V2.open(); V3.open(); V8.open(); V9.open(); V10.open()
            return True
        return False

    # Step 1
    if check_stop(): return
    V1.close(); V2.open(); V3.open(); V8.open(); V9.open(); V10.open()
    motor_pump.set_enabled(True)
    motor_pump.set_direction(True)
    motor_pump.set_speed_checked(False)
    try:
        sleeper.sleep(60)
    except InterruptedError:
        check_stop()
        return

    # Step 2
    motor_pump.set_enabled(False)
    motor_pump.set_direction(False)
    try:
        sleeper.sleep(1)
    except InterruptedError:
        check_stop()
        return

    # Step 3
    motor_pump.set_enabled(True)
    try:
        sleeper.sleep(60)
    except InterruptedError:
        check_stop()
        return

    # Step 4
    motor_pump.set_enabled(False)
    try:
        sleeper.sleep(1)
    except InterruptedError:
        check_stop()
        return

    # Step 5
    V1.open(); V3.close(); V8.close()
    motor_pump.set_enabled(True)
    motor_pump.set_direction(True)
    motor_pump.set_speed_checked(True)
    try:
        sleeper.sleep(13)
    except InterruptedError:
        check_stop()
        return

    # Step 6
    V3.open(); V8.open()
    motor_pump.set_enabled(False)
    motor_pump.set_direction(False)
    try:
        sleeper.sleep(1)
    except InterruptedError:
        check_stop()
        return

    # Step 7
    motor_pump.set_enabled(False)
    motor_pump.set_direction(False)
    try:
        sleeper.sleep(1)
    except InterruptedError:
        check_stop()
        return

    print("Elution sequence complete")
