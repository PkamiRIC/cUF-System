import time
from sleeper import InterruptibleSleeper

def run_clean1(valves, motor_pump, get_idle_time_minutes, stop_flag, flow_meter, reset_flow_and_timer):
    print("Starting Clean 1 sequence")

    # Reset flow and timer at start
    reset_flow_and_timer()
    flow_meter.reset()
    
    sleeper = InterruptibleSleeper(stop_flag)
    V1, V2, V3, V8, V9, V10 = valves[0], valves[1], valves[2], valves[3], valves[4], valves[5]

    def check_stop():
        if stop_flag():
            print("[INFO] Clean 1 sequence aborted.")
            motor_pump.set_enabled(False)
            V1.open(); V2.open(); V3.open(); V8.close(); V9.close(); V10.open()
            return True
        return False

    try:
        # Step 1
        if check_stop(): return
        V1.open(); V2.close(); V3.open(); V8.open(); V9.open(); V10.close()
        motor_pump.set_enabled(True)
        motor_pump.set_direction(True)
        motor_pump.set_speed_checked(True)
        sleeper.sleep(12)

        # Step 2
        if check_stop(): return
        V1.close()
        sleeper.sleep(6)

        # Step 3
        if check_stop(): return
        V1.open()
        sleeper.sleep(10)

        # Step 4
        if check_stop(): return
        V2.open(); V3.close()
        sleeper.sleep(23)

        # Step 5
        if check_stop(): return
        V2.close(); V3.open()
        sleeper.sleep(10)

        # Step 6
        if check_stop(): return
        V1.close(); V2.open()
        sleeper.sleep(7)

        # Step 7
        if check_stop(): return
        V1.open(); V2.open(); V10.open()
        motor_pump.set_enabled(False)
        sleeper.sleep(1)

        print("Clean 1 sequence complete")

        # Idle time
        idle_minutes = get_idle_time_minutes()
        print(f"[INFO] Idling for {idle_minutes} minutes...")
        sleeper.sleep(idle_minutes * 60)

        print("Idle period complete")

    except InterruptedError:
        print("[INFO] Sequence stopped during timed operation.")
        check_stop()
        return
