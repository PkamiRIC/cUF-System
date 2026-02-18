import time
from sleeper import InterruptibleSleeper

def run_clean2(valves, motor_pump, stop_flag, flow_meter, reset_flow_and_timer):
    print("Starting Clean 2 sequence")

   # Reset volume + timer
    reset_flow_and_timer()
    flow_meter.reset()

    sleeper = InterruptibleSleeper(stop_flag)
    V1, V2, V3, V8, V9, V10 = valves[0], valves[1], valves[2], valves[3], valves[4], valves[5]

    def check_stop():
        if stop_flag():
            print("[INFO] Clean 2 sequence aborted.")
            motor_pump.set_enabled(False)
            motor_pump.set_direction(False)
            motor_pump.set_speed_checked(False)
            V1.open(); V2.open(); V3.open(); V8.open(); V9.open(); V10.open()
            return True
        return False

    try:
        # Step 1
        if check_stop(): return
        V1.open(); V2.close(); V3.open(); V8.open(); V9.close(); V10.open()
        motor_pump.set_enabled(True)
        motor_pump.set_direction(True)
        motor_pump.set_speed_checked(True)
        sleeper.sleep(15)

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
        sleeper.sleep(30)

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
        V1.open(); V2.open(); V9.close(); V10.open()
        motor_pump.set_enabled(False)
        sleeper.sleep(1)
        

        print("Clean 2 sequence complete")

    except InterruptedError:
        print("[INFO] Sequence stopped during sleep.")
        check_stop()
        return
