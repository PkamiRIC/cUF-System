import time

def run_concentration(valves, motor_pump, flow_meter, get_target_liters, stop_flag, reset_flow_and_timer):
    print("Starting Concentration sequence")

    reset_flow_and_timer()
    flow_meter.reset()
    target_volume = get_target_liters()
    print(f"[INFO] Target volume: {target_volume} L")

    V1, V2, V3, V8, V9, V10 = valves[0], valves[1], valves[2], valves[3], valves[4], valves[5]

    def check_stop():
        if stop_flag():
            print("[INFO] Concentration sequence aborted.")
            motor_pump.set_enabled(False)
            V1.open(); V2.open(); V3.open(); V8.open(); V9.open(); V10.open()
            return True
        return False

    if check_stop(): return

    V1.close(); V2.open(); V3.open(); V8.close(); V9.open(); V10.open()

    motor_pump.set_direction(True)
    motor_pump.set_enabled(True)

    try:
        while True:
            if check_stop(): return
            _, total_volume = flow_meter.get_flow_data()
            print(f"[DEBUG] Volume: {total_volume:.3f} L")
            if total_volume >= target_volume:
                print("[INFO] Target volume reached.")
                break
            time.sleep(0.5)
    finally:
        motor_pump.set_enabled(False)
        V1.open(); V8.open()
        print("Concentration sequence complete")
