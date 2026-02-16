import sys
import threading
import time
import signal
import RPi.GPIO as GPIO
from simple_pid import PID
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout, QLabel, QHBoxLayout, QLineEdit
from PyQt5.QtGui import QDoubleValidator
from PyQt5.QtCore import Qt, QTimer
from pyqtgraph import PlotWidget
from librpiplc import rpiplc as plc
from sequences.deaeration import run_deaeration
from sequences.concentration import run_concentration
from sequences.elution import run_elution
from sequences.clean1 import run_clean1
from sequences.clean2 import run_clean2
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout, QLabel, QHBoxLayout, QLineEdit, QSizePolicy, QSpacerItem

GPIO.cleanup()
PULSES_PER_LITER = 6800
FLOW_GPIO = 27  # I1.0 (INT)

class FlowMeter:
    def __init__(self):
        self.pulse_count = 0
        self.total_liters = 0.0
        self.flow_rate_lpm = 0.0
        self.lock = threading.Lock()

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(FLOW_GPIO, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.add_event_detect(FLOW_GPIO, GPIO.RISING,
                              callback=self._pulse_callback, bouncetime=1)
        signal.signal(signal.SIGINT, self._signal_handler)

        threading.Thread(target=self._flow_processing_loop, daemon=True).start()

    def _pulse_callback(self, channel):
        with self.lock:
            self.pulse_count += 1

    def _flow_processing_loop(self):
        while True:
            time.sleep(1.0)
            with self.lock:
                pulses = self.pulse_count
                self.pulse_count = 0
                self.flow_rate_lpm = (pulses / PULSES_PER_LITER) * 60.0
                self.total_liters += pulses / PULSES_PER_LITER

    def get_flow_data(self):
        with self.lock:
            return self.flow_rate_lpm, self.total_liters

    def reset(self):
        with self.lock:
            self.flow_rate_lpm = 0.0
            self.total_liters = 0.0

    def _signal_handler(self, signum, frame):
        GPIO.cleanup()
        sys.exit(0)

class PIDValveController:
    def __init__(self, get_tmp_func):
        self.get_tmp = get_tmp_func
        self.enabled = False
        self.setpoint = 80
        self.pid = PID(1.0, 0.1, 0.05, setpoint=self.setpoint)
        self.pid.output_limits = (-1.0, 1.0)

        self.STEP = "Q0.5"
        self.DIR = "Q0.4"
        self.EN = "Q0.3"
        self.HALL = "I0.12"

        plc.pin_mode(self.STEP, plc.OUTPUT)
        plc.pin_mode(self.DIR, plc.OUTPUT)
        plc.pin_mode(self.EN, plc.OUTPUT)
        plc.pin_mode(self.HALL, plc.INPUT)

        self.hall_led_callback = None
        self._start_hall_monitor()  # ✅ This now works properly
        threading.Thread(target=self._control_loop, daemon=True).start()

    def set_enabled(self, enabled):
        self.enabled = enabled
        plc.digital_write(self.EN, plc.LOW if enabled else plc.HIGH)  # LOW = enabled

    def set_setpoint(self, value):
        self.setpoint = value
        self.pid.setpoint = value

    def _step_valve(self, direction, steps=20):
        plc.digital_write(self.DIR, plc.HIGH if direction else plc.LOW)
        time.sleep(0.001)
        for _ in range(steps):
            plc.digital_write(self.STEP, plc.HIGH)
            time.sleep(0.00002)
            plc.digital_write(self.STEP, plc.LOW)
            time.sleep(0.00002)

    def _control_loop(self):
        while True:
            if self.enabled:
                tmp = self.get_tmp()
                output = self.pid(tmp)
                steps = int(abs(output) * 10) # Change number of steps for smoother operation of the PID
                if steps > 0:
                    self._step_valve(direction=(output > 0), steps=steps)
            time.sleep(0.1)

    def _start_hall_monitor(self):
            def monitor():
                while True:
                    val = plc.digital_read(self.HALL)
                    if self.hall_led_callback:
                        self.hall_led_callback(val)
                    time.sleep(1)
            threading.Thread(target=monitor, daemon=True).start()

    def homing_routine(self):
        EN_PIN = "Q0.3"
        DIR_PIN = "Q0.4"
        STEP_PIN = "Q0.5"
        HALL_PIN = "I0.12"

        print("Starting homing...")
        plc.digital_write(EN_PIN, plc.LOW)
        plc.digital_write(DIR_PIN, plc.LOW)

        initial_hall = plc.digital_read(HALL_PIN)
        print(f"Initial Hall sensor state: {initial_hall} (0 = LOW, 1 = HIGH)")
        steps = 0
        hall_state = plc.digital_read(HALL_PIN)
        while hall_state == 1 :
            plc.digital_write(STEP_PIN, plc.HIGH)
            time.sleep(0.001)
            plc.digital_write(STEP_PIN, plc.LOW)
            time.sleep(0.001)
            steps += 1

            hall_state = plc.digital_read(HALL_PIN)
            print(f"Step {steps}, Hall={hall_state}")


        print("Hall sensor triggered. Homing complete.")
        plc.digital_write(EN_PIN, plc.HIGH)
        print("Valve homed and drive disabled.")

# --- Valve Class ---
class Valve:
    def __init__(self, name):
        self.name = name
        self.state = False
        plc.pin_mode(self.name, plc.OUTPUT)

    def open(self):
        plc.digital_write(self.name, plc.HIGH)
        plc.delay(10)
        self.state = True

    def close(self):
        plc.digital_write(self.name, plc.LOW)
        plc.delay(10)
        self.state = False

    def toggle(self):
        if self.state:
            self.close()
        else:
            self.open()

class ValveButton(QPushButton):
    def __init__(self, valve: Valve, label: str = None, size: int = 100,
                 font_px: int = 14, on_color: str = "red", off_color: str = "green"):
        super().__init__(label or self.label_for(valve.name))
        self.valve = valve
        self.on_color = on_color
        self.off_color = off_color
        self.font_px = font_px

        self.setFixedSize(size, size)
        self.setCheckable(True)

        # Kill the dashed focus rectangle
        self.setFocusPolicy(Qt.NoFocus)
        self.setAutoDefault(False)
        try:
            self.setDefault(False)
        except Exception:
            pass

        self.setStyleSheet(self.style_for_state(False))
        self.clicked.connect(self.toggle_valve)
   
    def refresh_visual(self):
        self.setChecked(self.valve.state)
        self.setStyleSheet(self.style_for_state(self.valve.state))
   
    def style_for_state(self, state):
        color = self.on_color if state else self.off_color
        return (
            f"border-radius: {int(self.width()/2)}px;"
            f"background-color: {color};"
            f"font-size: {self.font_px}px;"
        )

    def label_for(self, pin_name):
        mapping = {
            "R1.1": "V1",
            "R1.2": "V2",
            "R1.3": "V3",
            "R1.4": "V8",
            "R1.5": "V9",
            "R1.6": "V10",
            "R1.7": "Elute"
        }
        return mapping.get(pin_name, pin_name)

    def toggle_valve(self):
        self.valve.toggle()
        self.setChecked(self.valve.state)
        self.setStyleSheet(self.style_for_state(self.valve.state))


class MotorPumpControl(QWidget):
    def __init__(self):
        super().__init__()
        layout = QHBoxLayout()  # horizontal layout for pump buttons
        layout.setContentsMargins(0, 0, 0, 0)   
        layout.setSpacing(12)                   
        
        self.enable_pin = "Q0.0"
        self.dir_plus_pin = "Q0.1"
        self.dir_minus_pin = "Q0.2"
        self.speed_toggle_pin = "Q0.7"

        for pin in [self.enable_pin, self.dir_plus_pin, self.dir_minus_pin, self.speed_toggle_pin]:
            plc.pin_mode(pin, plc.OUTPUT)
            plc.digital_write(pin, plc.LOW)

        self.enable_button = QPushButton("Enable")
        self.enable_button.setCheckable(True)
        self.enable_button.setFixedSize(120, 50)
        self.enable_button.clicked.connect(self.toggle_enable)

        self.direction_button = QPushButton("Dir +")
        self.direction_button.setCheckable(True)
        self.direction_button.setFixedSize(120, 50)
        self.direction_button.clicked.connect(self.toggle_direction)

        self.speed_button = QPushButton("Speed")
        self.speed_button.setCheckable(True)
        self.speed_button.setFixedSize(120, 50)
        self.speed_button.clicked.connect(self.toggle_speed)

        pump_button_style = """
        QPushButton {
            background-color: lightgray;
            color: black;
            font-weight: bold;
            border: 2px solid #555;
            border-radius: 10px;
            padding: 6px;
            outline: none;
        }
        QPushButton:checked {
            background-color: green;
            color: white;
            font-weight: bold;
            border: 2px solid #555;
            border-radius: 10px;
            padding: 6px;
            outline: none;
        }
        QPushButton:focus {
        outline: none; 
        }
        QPushButton:pressed {
        background-color: #228B22;   
        color: white;
        padding: 6px;                
        }
        """

        self.enable_button.setStyleSheet(pump_button_style)
        self.direction_button.setStyleSheet(pump_button_style)
        self.speed_button.setStyleSheet(pump_button_style)

        layout.addWidget(self.enable_button)
        layout.addWidget(self.direction_button)
        layout.addWidget(self.speed_button)

        self.setLayout(layout)


    def toggle_enable(self):
        state = self.enable_button.isChecked()
        print(f"[DEBUG] toggle_enable() called. state={state}")
        plc.digital_write(self.enable_pin, plc.LOW if state else plc.HIGH)
        self.enable_button.setText("Pump ON" if state else "Pump OFF")

    def toggle_direction(self):
        forward = self.direction_button.isChecked()
        print(f"[DEBUG] toggle_direction() called. forward={forward}")
        plc.digital_write(self.dir_plus_pin, plc.HIGH if forward else plc.LOW)
        plc.digital_write(self.dir_minus_pin, plc.LOW if forward else plc.HIGH)
        self.direction_button.setText("Dir: CW" if forward else "Dir CCW")

    def toggle_speed(self):
        state = self.speed_button.isChecked()
        print(f"[DEBUG] toggle_speed() called. state={state}")
        plc.digital_write(self.speed_toggle_pin, plc.HIGH if state else plc.LOW)
        self.speed_button.setText("Low Speed" if state else "High Speed")

    # ---- Thread-safe control methods for background sequences ----
    def _sync_enable_ui(self, state: bool):
        try:
            self.enable_button.setChecked(state)
            self.enable_button.setText("Pump ON" if state else "Pump OFF")
        except Exception as e:
            print(f"[WARN] _sync_enable_ui failed: {e}")

    def set_enabled(self, state: bool):
        # Do the actual IO immediately (works from any thread)
        try:
            # Active-low: LOW = enabled, HIGH = disabled
            plc.digital_write(self.enable_pin, plc.LOW if state else plc.HIGH)
        except Exception as e:
            print(f"[WARN] set_enabled IO failed: {e}")
        # Then reflect the change on the GUI thread
        QTimer.singleShot(0, lambda: self._sync_enable_ui(state))

    def _sync_direction_ui(self, forward: bool):
        try:
            self.direction_button.setChecked(forward)
            self.direction_button.setText("Dir: CW" if forward else "Dir CCW")
        except Exception as e:
            print(f"[WARN] _sync_direction_ui failed: {e}")

    def set_direction(self, forward: bool):
        # Immediate IO so motion happens even during long sequences
        try:
            plc.digital_write(self.dir_plus_pin, plc.HIGH if forward else plc.LOW)
            plc.digital_write(self.dir_minus_pin, plc.LOW if forward else plc.HIGH)
        except Exception as e:
            print(f"[WARN] set_direction IO failed: {e}")
        # Then update button state/text on the GUI thread
        QTimer.singleShot(0, lambda: self._sync_direction_ui(forward))

    def _sync_speed_ui(self, checked: bool):
        try:
            self.speed_button.setChecked(checked)
            self.speed_button.setText("Low Speed" if checked else "High Speed")
        except Exception as e:
            print(f"[WARN] _sync_speed_ui failed: {e}")

    # Accept and ignore extra args to remain backward-compatible with any callers
    def set_speed_checked(self, checked: bool, *_, **__):
        # Immediate IO
        try:
            plc.digital_write(self.speed_toggle_pin, plc.HIGH if checked else plc.LOW)
        except Exception as e:
            print(f"[WARN] set_speed_checked IO failed: {e}")
        # GUI update later
        QTimer.singleShot(0, lambda: self._sync_speed_ui(checked))


class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Valve and Pump Control Panel")
        self.setGeometry(100, 100, 1000, 400)

        main_layout = QVBoxLayout()
        valve_layout = QHBoxLayout()
        plc.init("RPIPLC_V6", "RPIPLC_38AR")

        self._homing_running = False
        pump_pid_button_style = """
        QPushButton {
            background-color: lightgray;
            color: black;
            font-weight: bold;
            border: 2px solid #555;
            border-radius: 10px;
            padding: 6px;
            outline: none;
        }
        QPushButton:checked {
            background-color: green;
            color: white;
            font-weight: bold;
            border: 2px solid #555;
            border-radius: 10px;
            padding: 6px;
            outline: none;
        }
        QPushButton:focus { outline: none; }
        QPushButton:pressed {
            background-color: #228B22;
            color: white;
            padding: 6px;
        }
        """
        self.stop_requested = False
        self.valve_buttons = []   # <-- create the list before appending to it

        self.valves = [Valve(f"R1.{i}") for i in [1, 2, 3, 4, 5, 6]]
        for valve in self.valves:
            vb = ValveButton(valve)
            valve_layout.addWidget(vb)
            self.valve_buttons.append(vb)
                
        self.elute_valve = Valve("R1.7")
        self.elute_valve.open()  # ensure it's CLOSED at startup
        self.elute_button = ValveButton(self.elute_valve, label="Collect\nElution", size=64, font_px=12, on_color="red", off_color="#FFA500")
        valve_layout.addWidget(self.elute_button)
        self.valve_buttons.append(self.elute_button)

        main_layout.addLayout(valve_layout)

        self.motor_pump = MotorPumpControl()
        for b in [self.motor_pump.enable_button, self.motor_pump.direction_button, self.motor_pump.speed_button]:
            b.setFixedSize(120, 50)
            b.setStyleSheet(pump_pid_button_style)
            b.setFocusPolicy(Qt.NoFocus)


        self.pressure_labels = []
        pressure_names = ["Pressure IN", "Pressure OUT", "Filter"]
        self.pressure_pins = ["I0.9", "I0.10", "I0.11"]
        pressure_layout = QHBoxLayout()

        for name in pressure_names:
            label = QLabel(f"{name}: -- mBar")
            self.pressure_labels.append(label)
            pressure_layout.addWidget(label)

        self.hall_led = QLabel()
        self.hall_led.setFixedSize(20, 20)
        self.hall_led.setStyleSheet("background-color: grey; border-radius: 10px;")

        self.tmp_label = QLabel("TMP: -- mBar")
        self.hall_label = QLabel("Hall Sensor: --")
        self.hall_led = QLabel()
        self.hall_led.setFixedSize(20, 20)
        self.hall_led.setStyleSheet("background-color: grey; border-radius: 10px;")
        pressure_layout.addWidget(self.tmp_label)
        pressure_layout.addWidget(self.hall_label)
        pressure_layout.addWidget(self.hall_led)
        main_layout.addLayout(pressure_layout)

        self.tmp_plot = PlotWidget()
        self.tmp_plot.setTitle("TMP Over Time")
        self.tmp_plot.setYRange(0, 200)
        self.tmp_curve = self.tmp_plot.plot(pen='g')

        # Initialize TMP value + time histories
        self.tmp_history = []
        self.tmp_time_history = []
        self.elapsed_tmp_time = 0  # in seconds

        self.tmp_plot.setMinimumWidth(200)
        self.tmp_plot.setMinimumHeight(200)
        main_layout.addWidget(self.tmp_plot)


        self.flow_meter = FlowMeter()
        self.flow_rate_label = QLabel("Flow Rate: -- L/min")
        self.total_volume_label = QLabel("Total Volume: 00.00 L")
        self.total_volume_label.setFixedWidth(QLabel("Total Volume: 10.00 L").sizeHint().width())
        self.timer_label = QLabel("Timer: 00:00")        # mm:ss        
        self.timer_label.setFixedWidth(self.timer_label.sizeHint().width())
        self.reset_button = QPushButton("Reset")
        self.reset_button.setFocusPolicy(Qt.NoFocus)  # no dashed focus
        self._reset_btn_style_normal = (
        "background-color: lightgray; color: black; font-weight: bold; "
        "border: 2px solid #555; border-radius: 10px; padding: 6px;"
        )
        self._reset_btn_style_flash = (
            "background-color: #2E7D32; color: white; font-weight: bold; "
            "border: 2px solid #1B5E20; border-radius: 10px; padding: 6px;"
        )

        self.reset_button.setStyleSheet(self._reset_btn_style_normal)

        # Lock size to current hint so it won't change during flash
        hint = self.reset_button.sizeHint()
        self.reset_button.setFixedSize(hint.width(), hint.height())
        self.reset_button.clicked.connect(self._on_reset_clicked)
        self.target_liters_input = QLineEdit("1.0")  # default 1.0 liter
        self.target_liters_input.setValidator(QDoubleValidator(0.1, 100.0, 2))
        self.target_liters_input.setFixedWidth(80)

        flow_layout = QHBoxLayout()
        flow_layout.addWidget(self.flow_rate_label)
        flow_layout.addWidget(self.total_volume_label)
        timer_reset = QHBoxLayout()
        timer_reset.setContentsMargins(0, 0, 0, 0)
        timer_reset.setSpacing(7)   # tight gap between Timer and Reset
        timer_reset.addWidget(self.timer_label)
        timer_reset.addWidget(self.reset_button)

        flow_layout.addLayout(timer_reset)

        # Preserve the original gap to the next group
        flow_layout.addItem(QSpacerItem(100, 0, QSizePolicy.Fixed, QSizePolicy.Minimum))

        # Continue
        flow_layout.addWidget(QLabel("Target Volume (L):"))        
        self.target_liters_input = QLineEdit("1.0")
        self.target_liters_input.setValidator(QDoubleValidator(0.1, 100.0, 2))
        self.target_liters_input.setFixedWidth(80)
        flow_layout.addWidget(self.target_liters_input)

        main_layout.addLayout(flow_layout)

        self.idle_time_input = QLineEdit("2.0")  # default 2 minutes
        self.idle_time_input.setValidator(QDoubleValidator(0.1, 60.0, 2))
        self.idle_time_input.setFixedWidth(80)
        flow_layout.addWidget(QLabel("Clean Time (min):"))
        flow_layout.addWidget(self.idle_time_input)

        self.pid_button = QPushButton("Enable PID")
        self.pid_button.setCheckable(True)
        self.pid_button.setChecked(False)  # or True, depending on desired default
        self.pid_button.clicked.connect(self.toggle_pid)
        self.pid_button.setAutoFillBackground(False)
        self.pid_button.setStyle(QApplication.style())  # resets native system override

        self.setpoint_input = QLineEdit("80.0")
        self.setpoint_input.setValidator(QDoubleValidator(0.0, 10000.0, 1))
        self.setpoint_input.textChanged.connect(self.set_new_pid_setpoint)

        self.pid_button.setStyleSheet("""
        QPushButton {
            background-color: lightgray;
            color: black;
            font-weight: bold;
            border: 2px solid #555;
            border-radius: 10px;
            padding: 8px;
        }

        QPushButton:checked {
            background-color: green;
            color: white;
            font-weight: bold;
            border: 2px solid #555;
            border-radius: 10px;
            padding: 8px;
        }
        """)

        self.home_button = QPushButton("Home Valve")
        self.home_button.setCheckable(True)
        self.home_button.setFixedSize(120, 50)
        self.home_button.setStyleSheet(pump_pid_button_style)
        self.home_button.setFocusPolicy(Qt.NoFocus)
        self.home_button.clicked.connect(self.start_homing)

                # ========= New single-row control strip =========
        controls_row = QHBoxLayout()
        controls_row.setContentsMargins(0, 0, 0, 0)
        controls_row.setSpacing(12)

        # Left group: motor pump (buttons already horizontal)
        controls_row.addWidget(self.motor_pump, alignment=Qt.AlignVCenter)

        # Spacer pushes right group to the right
        controls_row.addStretch(1)

        # Right group: PID enable, Home, Setpoint
        right_group = QHBoxLayout()
        right_group.setContentsMargins(0, 0, 0, 0)
        right_group.setSpacing(12)

        # 1) PID button
        self.pid_button.setFixedSize(120, 50)
        self.pid_button.setStyleSheet(pump_pid_button_style)
        self.pid_button.setFocusPolicy(Qt.NoFocus)

        # 2) Home button (make sure it is CREATED before these lines)
        self.home_button.setFixedSize(120, 50)
        self.home_button.setStyleSheet(pump_pid_button_style)
        self.home_button.setFocusPolicy(Qt.NoFocus)

        # 3) Setpoint "box"
        self.setpoint_input.setFixedSize(120, 50)
        self.setpoint_input.setAlignment(Qt.AlignCenter)
        self.setpoint_input.setPlaceholderText("Setpoint")
        self.setpoint_input.setStyleSheet("""
        QLineEdit {
            background-color: lightgray;
            color: black;
            border: 2px solid #555;
            border-radius: 0px;
            padding: 0px;
        }
        QLineEdit:focus { border: 2px solid #555; }
        """)

        right_group.addWidget(self.pid_button)
        right_group.addWidget(self.home_button)
        right_group.addWidget(self.setpoint_input)

        controls_row.addLayout(right_group)

        # Add the row to the main layout
        main_layout.addLayout(controls_row)


        self.pid_valve = PIDValveController(lambda: self.tmp_value)
        self.pid_valve.hall_led_callback = lambda val: self.hall_led.setStyleSheet(
    "background-color: green; border-radius: 10px;" if val else "background-color: grey; border-radius: 10px;"
)
        self.sequence_buttons = {}
        sequence_names = ["Deaeration", "Concentration", "Elution", "Clean 1", "Clean 2"]

        # Left side: the five sequence buttons (100x100)
        seq_buttons = QHBoxLayout()
        seq_buttons.setContentsMargins(0, 0, 0, 0)
        seq_buttons.setSpacing(15)

        for name in sequence_names:
            btn = QPushButton(name)
            btn.setFixedSize(100, 100)
            btn.setStyleSheet("background-color: #87CEFA; font-size: 14px; font-weight: bold; border-radius: 10px;")
            btn.clicked.connect(lambda _, n=name: threading.Thread(target=self.run_sequence, args=(n,), daemon=True).start())
            seq_buttons.addWidget(btn)
            self.sequence_buttons[name] = btn

        # Whole row: sequence buttons on left, STOP on right
        seq_row = QHBoxLayout()
        seq_row.setContentsMargins(0, 0, 0, 0)
        seq_row.setSpacing(12)
        seq_row.addLayout(seq_buttons)
        seq_row.addStretch(1)

        # Small rectangular STOP on the right
        self.stop_button = QPushButton("STOP")
        self.stop_button.setFixedSize(120, 50)
        # Keep both styles handy
        self._stop_btn_style_normal = (
            "background-color: red; color: white; font-weight: bold; "
            "border: 2px solid #555; border-radius: 10px; padding: 6px;"
        )
        self._stop_btn_style_flash = (
            "background-color: #8B0000; color: white; font-weight: bold; "  # deeper red
            "border: 2px solid #444; border-radius: 10px; padding: 6px;"
        )

        self.stop_button.setStyleSheet(self._stop_btn_style_normal)
        self.stop_button.setFocusPolicy(Qt.NoFocus)
        self.stop_button.clicked.connect(self._on_stop_clicked)   # <-- use handler

        seq_row.addWidget(self.stop_button, alignment=Qt.AlignRight)

        # Add the row
        main_layout.addLayout(seq_row)


        self.smoothed_pressures = [0.0, 0.0, 0.0]  # IN, OUT, FILTER
        self.ema_alpha = 0.5  # Tune this (lower = smoother)

        self.setLayout(main_layout)
        self.startTimer(1000)
        self.elapsed_seconds = 0

        self.valve_sync_timer = QTimer(self)
        self.valve_sync_timer.timeout.connect(self._sync_valve_buttons)
        self.valve_sync_timer.start(200)   # 5x per second; adjust if you like

    def force_close_elute_valve(self):
            """Force-close the elution collection valve and reset its button UI (thread-safe)."""
            try:
                self.elute_valve.open()
            except Exception as e:
                print(f"[WARN] elute_valve.open failed: {e}")
            def _ui():
                try:
                    self.elute_button.setChecked(False)
                    self.elute_button.setStyleSheet(self.elute_button.style_for_state(False) + " font-size:12px;")
                except Exception as e:
                    print(f"[WARN] force_close_elute_valve UI update failed: {e}")
            QTimer.singleShot(0, _ui)

            self.elute_button.setStyleSheet(
            self.elute_button.style_for_state(False) + " font-size:12px;"
            )

    def stop_current_sequence(self):
        print("[INFO] Stop button pressed.")
        self.stop_requested = True
        self.force_close_elute_valve()
        self._sync_valve_buttons()


    def reset_flow_and_timer(self):
        self.flow_meter.reset()
        self.elapsed_seconds = 0
        self.tmp_history.clear()
        self.tmp_time_history.clear()
        self.elapsed_tmp_time = 0
        self.tmp_curve.setData([], [])
    
    def get_target_liters(self):
        try:
            return float(self.target_liters_input.text())
        except ValueError:
            return 1.0  # fallback default
    
    def toggle_pid(self):
        enabled = self.pid_button.isChecked()
        self.pid_valve.set_enabled(enabled)

        self.pid_button.setStyleSheet(f"""
            QPushButton {{
                background-color: {'green' if enabled else 'lightgray'};
                color: {'white' if enabled else 'black'};
                font-weight: bold;
                border: 2px solid #555;
                border-radius: 10px;
                padding: 8px;
            }}
        """)


    def set_new_pid_setpoint(self, text):
        try:
            self.pid_valve.set_setpoint(float(text))
        except ValueError:
            pass

    def start_homing(self):
        if getattr(self, "_homing_running", False):
            return
        self._homing_running = True
        self.home_button.setChecked(True)   # stays green (uses :checked style)
        self.home_button.setEnabled(False)
        threading.Thread(target=self._run_homing, daemon=True).start()

    def _run_homing(self):
        try:
            self.pid_valve.homing_routine()
        finally:
            QTimer.singleShot(0, self._end_homing)  # back to Qt main thread

    def _end_homing(self):
        self.home_button.setChecked(False)  # back to gray
        self.home_button.setEnabled(True)
        self._homing_running = False

    def run_sequence(self, name):
        self.stop_requested = False

        self.force_close_elute_valve()   # Always close elution valve before (re)starting any sequence
        self._run_homing()

        if name == "Deaeration":
            run_deaeration(self.valves, self.motor_pump, lambda: self.stop_requested, self.flow_meter, self.reset_flow_and_timer)
        elif name == "Concentration":
            # Enable PID before running (use True/False, not 1/0)
            try:
                self.pid_valve.set_enabled(True)
                # reflect in UI button without blocking the worker thread
                QTimer.singleShot(0, lambda: self.pid_button.setChecked(True))
            except Exception as e:
                print(f"[WARN] could not enable PID automatically: {e}")

            try:
                run_concentration(self.valves, self.motor_pump, self.flow_meter,
                                  self.get_target_liters,
                                  lambda: self.stop_requested,
                                  self.reset_flow_and_timer)
            finally:
                # Always disable PID after concentration ends or is stopped
                try:
                    self.pid_valve.set_enabled(False)
                    QTimer.singleShot(0, lambda: self.pid_button.setChecked(False))
                    self._run_homing()
                except Exception as e:
                    print(f"[WARN] could not disable PID automatically: {e}")

        elif name == "Elution":
            run_elution(self.valves, self.motor_pump, lambda: self.stop_requested, self.flow_meter, self.reset_flow_and_timer)
        elif name == "Clean 1":
            run_clean1(self.valves, self.motor_pump, self.get_idle_time_minutes, lambda: self.stop_requested, self.flow_meter, self.reset_flow_and_timer)
        elif name == "Clean 2":
            run_clean2(self.valves, self.motor_pump, lambda: self.stop_requested, self.flow_meter, self.reset_flow_and_timer,)

    def _on_stop_clicked(self):
        # Flash the button immediately
        self._flash_stop_button(ms=1000)
        # Perform the stop action right away
        self.stop_current_sequence()
        self.pid_valve.set_enabled(False)  # Disable PID on stop
        self._run_homing()

    def _flash_stop_button(self, ms=1000):
        # set deep red, temporarily disable to avoid repeat clicks
        self.stop_button.setStyleSheet(self._stop_btn_style_flash)
        self.stop_button.setEnabled(False)
        QTimer.singleShot(ms, self._restore_stop_button)

    def _restore_stop_button(self):
        self.stop_button.setStyleSheet(self._stop_btn_style_normal)
        self.stop_button.setEnabled(True)

    def _on_reset_clicked(self):
        # do the actual reset
        self.reset_flow_and_timer()
        # flash green without changing size/text
        self.reset_button.setEnabled(False)
        self.reset_button.setStyleSheet(self._reset_btn_style_flash)
        QTimer.singleShot(1000, self._restore_reset_button)

    def _flash_reset_button(self, ms=1000):
        # remember current look/text to restore later
        self._reset_prev_style = self.reset_button.styleSheet()
        self._reset_prev_text = self.reset_button.text()
        # show a green flash + prevent double-clicks
        self.reset_button.setText("Reset ✓")
        self.reset_button.setStyleSheet(
            "background-color: #2E7D32; color: white; font-weight: bold; "
            "border: 2px solid #1B5E20; border-radius: 10px; padding: 6px;"
        )
        self.reset_button.setEnabled(False)
        QTimer.singleShot(ms, self._restore_reset_button)

    def _restore_reset_button(self):
        self.reset_button.setStyleSheet(self._reset_btn_style_normal)
        self.reset_button.setEnabled(True)

    def get_idle_time_minutes(self):
        try:
            return float(self.idle_time_input.text())
        except ValueError:
            return 2.0  # default fallback

    def _sync_valve_buttons(self):
        for btn in getattr(self, "valve_buttons", []):
            btn.refresh_visual()

    def timerEvent(self, event):
        self.elapsed_seconds += 1
        mins = self.elapsed_seconds // 60
        secs = self.elapsed_seconds % 60
        self.timer_label.setText(f"Timer: {mins:02d}:{secs:02d}")

        pressures = []
        for i, pin in enumerate(self.pressure_pins):
            raw = plc.analog_read(pin)
            voltage = (raw / 4095.0) * 10.0
            mbar = voltage * 250

            # Apply EMA
            self.smoothed_pressures[i] = (
                self.ema_alpha * mbar +
                (1 - self.ema_alpha) * self.smoothed_pressures[i]
            )
            pressures.append(self.smoothed_pressures[i])

            self.pressure_labels[i].setText(
                f"{['Pressure IN', 'Pressure OUT', 'Filter Pressure'][i]}: {self.smoothed_pressures[i]:.1f} mBar"
            )


        if len(pressures) == 3:
            tmp = ((pressures[0] + pressures[1]) / 2.0) - pressures[2]
            self.tmp_label.setText(f"TMP: {tmp:.1f} mBar")
            self.tmp_value = tmp

            # Update TMP time-series
            self.elapsed_tmp_time += 1  # assumes timerEvent runs every 1 sec
            self.tmp_history.append(tmp)
            self.tmp_time_history.append(self.elapsed_tmp_time)

            if len(self.tmp_history) > 100:
                self.tmp_history.pop(0)
                self.tmp_time_history.pop(0)

            # Plot with X = time
            self.tmp_curve.setData(self.tmp_time_history, self.tmp_history)

            # Auto-scroll last 30s
            x_max = self.elapsed_tmp_time
            x_min = max(0, x_max - 30)
            self.tmp_plot.setXRange(x_min, x_max)

        flow_rate, total_liters = self.flow_meter.get_flow_data()
        self.flow_rate_label.setText(f"Flow Rate: {flow_rate:.2f} L/min")
        self.total_volume_label.setText(f"Total Volume: {total_liters:05.2f} L")
        hall = plc.digital_read("I0.12")
        self.hall_label.setText(f"Hall Sensor: {'HIGH' if hall else 'LOW'}")

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    try:
        sys.exit(app.exec_())
    finally:
        GPIO.cleanup()