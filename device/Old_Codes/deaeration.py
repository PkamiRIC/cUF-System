import time
from sleeper import InterruptibleSleeper

def run_deaeration(valves, motor_pump, stop_flag, flow_meter, reset_flow_and_timer):
    print("Starting Deaeration sequence")

    # Reset flow and timer
    reset_flow_and_timer()
    flow_meter.reset()

    sleeper = InterruptibleSleeper(stop_flag)
    V1, V2, V3, V8, V9, V10 = valves[0], valves[1], valves[2], valves[3], valves[4], valves[5]

    def check_stop():
        if stop_flag():
            print("[INFO] Deaeration sequence aborted.")
            motor_pump.set_enabled(False)
            motor_pump.set_direction(False)
            motor_pump.set_speed_checked(False)
            V1.open(); V2.open(); V3.open(); V8.open(); V9.open(); V10.open()
            return True
        return False

    # Step 1: Initial valve state
    if check_stop(): return
    V1.open(); V2.close(); V3.open(); V8.open(); V9.close(); V10.open()

    print("Enabling motor...")
    motor_pump.set_enabled(True)

    print("Setting direction to CW...")
    motor_pump.set_direction(True)

    print("Setting speed (JOG2 equivalent)...")
    motor_pump.set_speed_checked(False)

    print("Pump should now be ON")
    try:
        sleeper.sleep(12)
    except InterruptedError:
        check_stop()
        return

    # Step 2
    V1.close(), V9.open(), V8.close();  
    try:
        sleeper.sleep(6)
    except InterruptedError:
        check_stop()
        return

    # Step 3
    V1.open(), V8.open(), V9.close(); 
    try:
        sleeper.sleep(10)
    except InterruptedError:
        check_stop()
        return

    # Step 4
    V2.open()
    try:
        sleeper.sleep(1)
    except InterruptedError:
        check_stop()
        return

    # Step 5
    V2.close()
    try:
        sleeper.sleep(10)
    except InterruptedError:
        check_stop()
        return

    # Step 6
    V1.close(); V2.open()
    try:
        sleeper.sleep(7)
    except InterruptedError:
        check_stop()
        return

    # Step 7: Stop motor
    V1.open()
    motor_pump.set_enabled(False)
    motor_pump.set_direction(False)
    motor_pump.set_speed_checked(False)

    try:
        sleeper.sleep(1)
    except InterruptedError:
        check_stop()
        return

    print("Deaeration complete")
