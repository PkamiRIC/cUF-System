import sys
import threading
import time
import signal
import RPi.GPIO as GPIO
from simple_pid import PID
from PyQt5.QtWidgets import (
    QApplication,
    QWidget,
    QPushButton,
    QVBoxLayout,
    QLabel,
    QHBoxLayout,
    QLineEdit,
    QSizePolicy,
    QFrame,
    QGridLayout,
    QPlainTextEdit,
)
from PyQt5.QtGui import QDoubleValidator
from PyQt5.QtCore import Qt, QTimer
from pyqtgraph import PlotWidget
from librpiplc import rpiplc as plc
from sequences.deaeration import run_deaeration
from sequences.concentration import run_concentration
from sequences.elution import run_elution
from sequences.clean1 import run_clean1
from sequences.clean2 import run_clean2
GPIO.cleanup()
PULSES_PER_LITER = 6800
FLOW_GPIO = 27  # I1.0 (INT)

VALVE_LABELS = {
    "R1.1": "V1",
    "R1.2": "V2",
    "R1.3": "V3",
    "R1.4": "V8",
    "R1.5": "V9",
    "R1.6": "V10",
    "R1.7": "Elute",
}

_ui_log_callback = None


def register_ui_logger(callback):
    global _ui_log_callback
    _ui_log_callback = callback


def emit_ui_log(message: str):
    if _ui_log_callback:
        try:
            _ui_log_callback(message)
        except Exception as exc:
            print(f"[WARN] UI log callback failed: {exc}")


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
        self.display_name = VALVE_LABELS.get(self.name, self.name)
        plc.pin_mode(self.name, plc.OUTPUT)

    def open(self):
        plc.digital_write(self.name, plc.HIGH)
        plc.delay(10)
        self.state = True
        emit_ui_log(f"Valve {self.display_name} OPEN")

    def close(self):
        plc.digital_write(self.name, plc.LOW)
        plc.delay(10)
        self.state = False
        emit_ui_log(f"Valve {self.display_name} CLOSED")

    def toggle(self):
        if self.state:
            self.close()
        else:
            self.open()

class ValveButton(QPushButton):
    def __init__(
        self,
        valve: Valve,
        label: str = None,
        size: int = 70,
        font_px: int = 12,
        on_color: str = "#f97316",
        off_color: str = "#3b82f6",
    ):
        super().__init__(label or self.label_for(valve.name))
        self.valve = valve
        self.on_color = on_color
        self.off_color = off_color
        self.font_px = font_px

        self.setFixedSize(size, size)
        self.setCheckable(True)
        self.setCursor(Qt.PointingHandCursor)

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
            f"border-radius: 18px;"
            f"background-color: {color};"
            "color: #f8fafc;"
            "font-weight: 600;"
            f"font-size: {self.font_px}px;"
            "border: none;"
            "padding: 6px;"
        )

    def label_for(self, pin_name):
        return VALVE_LABELS.get(pin_name, pin_name)

    def toggle_valve(self):
        self.valve.toggle()
        self.setChecked(self.valve.state)
        self.setStyleSheet(self.style_for_state(self.valve.state))
        emit_ui_log(
            f"Manual toggle -> {self.valve.display_name} {'OPEN' if self.valve.state else 'CLOSED'}"
        )


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
        self.enable_button.setMinimumWidth(112)
        self.enable_button.setFixedHeight(48)
        self.enable_button.setCursor(Qt.PointingHandCursor)
        self.enable_button.clicked.connect(self.toggle_enable)

        self.direction_button = QPushButton("Dir +")
        self.direction_button.setCheckable(True)
        self.direction_button.setMinimumWidth(112)
        self.direction_button.setFixedHeight(48)
        self.direction_button.setCursor(Qt.PointingHandCursor)
        self.direction_button.clicked.connect(self.toggle_direction)

        self.speed_button = QPushButton("Speed")
        self.speed_button.setCheckable(True)
        self.speed_button.setMinimumWidth(112)
        self.speed_button.setFixedHeight(48)
        self.speed_button.setCursor(Qt.PointingHandCursor)
        self.speed_button.clicked.connect(self.toggle_speed)

        pump_button_style = """
        QPushButton {
            background-color: #1d4ed8;
            color: #f8fafc;
            font-weight: 600;
            border: none;
            border-radius: 12px;
            padding: 12px 18px;
            outline: none;
        }
        QPushButton:checked {
            background-color: #22c55e;
            color: #0f172a;
            outline: none;
        }
        QPushButton:hover {
            background-color: #2563eb;
        }
        QPushButton:pressed {
            background-color: #1e40af;
            color: #f8fafc;
        }
        """

        self.enable_button.setStyleSheet(pump_button_style)
        self.direction_button.setStyleSheet(pump_button_style)
        self.speed_button.setStyleSheet(pump_button_style)

        for button in (
            self.enable_button,
            self.direction_button,
            self.speed_button,
        ):
            button.setFocusPolicy(Qt.NoFocus)

        layout.addWidget(self.enable_button)
        layout.addWidget(self.direction_button)
        layout.addWidget(self.speed_button)

        self.setLayout(layout)


    def toggle_enable(self):
        state = self.enable_button.isChecked()
        print(f"[DEBUG] toggle_enable() called. state={state}")
        plc.digital_write(self.enable_pin, plc.LOW if state else plc.HIGH)
        self.enable_button.setText("Pump ON" if state else "Pump OFF")
        emit_ui_log(f"Pump {'ENABLED' if state else 'DISABLED'}")

    def toggle_direction(self):
        forward = self.direction_button.isChecked()
        print(f"[DEBUG] toggle_direction() called. forward={forward}")
        plc.digital_write(self.dir_plus_pin, plc.HIGH if forward else plc.LOW)
        plc.digital_write(self.dir_minus_pin, plc.LOW if forward else plc.HIGH)
        self.direction_button.setText("Dir: CW" if forward else "Dir CCW")
        emit_ui_log(f"Pump direction set to {'CW' if forward else 'CCW'}")

    def toggle_speed(self):
        state = self.speed_button.isChecked()
        print(f"[DEBUG] toggle_speed() called. state={state}")
        plc.digital_write(self.speed_toggle_pin, plc.HIGH if state else plc.LOW)
        self.speed_button.setText("Low Speed" if state else "High Speed")
        emit_ui_log(f"Pump speed set to {'LOW' if state else 'HIGH'}")

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
        emit_ui_log(f"Pump {'ENABLED' if state else 'DISABLED'} (auto)")

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
        emit_ui_log(f"Pump direction set to {'CW' if forward else 'CCW'} (auto)")

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
        emit_ui_log(f"Pump speed set to {'LOW' if checked else 'HIGH'} (auto)")


class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Filtration Control Console")
        self.setMinimumSize(1024, 620)
        self.resize(1080, 640)
        self.move(80, 70)

        plc.init("RPIPLC_V6", "RPIPLC_38AR")

        self.setStyleSheet(
            """
            QWidget {
                background-color: #0f172a;
                color: #e2e8f0;
                font-family: 'Segoe UI', 'Helvetica Neue', Arial, sans-serif;
                font-size: 14px;
            }
            QLabel#appTitle {
                font-size: 24px;
                font-weight: 600;
                color: #f8fafc;
            }
            QLabel#panelTitle {
                font-size: 16px;
                font-weight: 600;
                color: #f8fafc;
            }
            QLabel#timerBadge {
                background-color: #1f2937;
                color: #38bdf8;
                border-radius: 12px;
                padding: 8px 16px;
                font-weight: 600;
            }
            QFrame#panel {
                background-color: #1e293b;
                border-radius: 16px;
            }
            QLineEdit {
                background-color: #0f172a;
                border: 1px solid #334155;
                border-radius: 10px;
                padding: 6px 12px;
                color: #f8fafc;
                font-weight: 500;
            }
            QLineEdit:focus {
                border: 1px solid #38bdf8;
            }
            """
        )

        self._homing_running = False
        self.stop_requested = False
        self._full_sequence_running = False
        self._full_sequence_thread = None
        self._log_context = "System"
        self.valve_buttons = []

        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(12, 8, 12, 12)
        main_layout.setSpacing(10)

        content_row = QHBoxLayout()
        content_row.setContentsMargins(0, 0, 0, 0)
        content_row.setSpacing(14)

        self.valves = [Valve(f"R1.{i}") for i in [1, 2, 3, 4, 5, 6]]
        valve_panel, valve_panel_layout = self._build_panel("Valves")
        valve_panel.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)
        valve_grid = QGridLayout()
        valve_grid.setContentsMargins(0, 0, 0, 0)
        valve_grid.setHorizontalSpacing(10)
        valve_grid.setVerticalSpacing(10)

        for idx, valve in enumerate(self.valves):
            vb = ValveButton(valve, size=66, font_px=12)
            row, col = divmod(idx, 3)
            valve_grid.addWidget(vb, row, col)
            self.valve_buttons.append(vb)

        self.elute_valve = Valve("R1.7")
        self.elute_valve.open()
        self.elute_button = ValveButton(
            self.elute_valve,
            label="Collect\nElution",
            size=66,
            font_px=11,
            on_color="#f97316",
            off_color="#6366f1",
        )
        row, col = divmod(len(self.valves), 3)
        valve_grid.addWidget(self.elute_button, row, col)
        self.valve_buttons.append(self.elute_button)

        valve_panel_layout.addLayout(valve_grid)
        content_row.addWidget(valve_panel, 1)

        self.pressure_pins = ["I0.9", "I0.10", "I0.11"]
        self.pressure_labels = []
        metrics_panel, metrics_layout = self._build_panel("Process Metrics")
        metrics_panel.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        metrics_panel.setMinimumHeight(320)

        pressure_grid = QGridLayout()
        pressure_grid.setContentsMargins(0, 0, 0, 0)
        pressure_grid.setHorizontalSpacing(18)
        pressure_grid.setVerticalSpacing(6)

        for idx, name in enumerate(["Pressure IN", "Pressure OUT", "Filter Pressure"]):
            label = QLabel(f"{name}: -- mBar")
            row, col = divmod(idx, 2)
            pressure_grid.addWidget(label, row, col)
            self.pressure_labels.append(label)

        self.tmp_label = QLabel("TMP: -- mBar")
        pressure_grid.addWidget(self.tmp_label, 1, 1)
        pressure_grid.setColumnStretch(0, 1)
        pressure_grid.setColumnStretch(1, 1)
        pressure_grid.setRowStretch(0, 0)
        pressure_grid.setRowStretch(1, 0)
        pressure_grid.setRowStretch(2, 0)

        hall_row = QHBoxLayout()
        hall_row.setContentsMargins(0, 0, 0, 0)
        hall_row.setSpacing(10)
        self.hall_label = QLabel("Hall Sensor: --")
        self.hall_led = QLabel()
        self.hall_led.setFixedSize(16, 16)
        self.hall_led.setStyleSheet("background-color: #475569; border-radius: 8px;")
        hall_row.addWidget(self.hall_label)
        hall_row.addWidget(self.hall_led)
        hall_row.addStretch()
        pressure_grid.addLayout(hall_row, 2, 0, 1, 2)

        metrics_layout.addLayout(pressure_grid)

        flow_row = QHBoxLayout()
        flow_row.setContentsMargins(0, 0, 0, 0)
        flow_row.setSpacing(14)
        self.flow_rate_label = QLabel("Flow Rate: -- L/min")
        self.total_volume_label = QLabel("Total Volume: 00.00 L")
        self.flow_rate_label.setStyleSheet("font-size: 12px;")
        self.total_volume_label.setStyleSheet("font-size: 12px;")
        self.total_volume_label.setMinimumWidth(170)
        flow_row.addWidget(self.flow_rate_label)
        flow_row.addWidget(self.total_volume_label)
        flow_row.addStretch()
        metrics_layout.addLayout(flow_row)

        self.tmp_plot = PlotWidget()
        self.tmp_plot.setBackground("#0f172a")
        self.tmp_plot.showGrid(x=True, y=True, alpha=0.15)
        self.tmp_plot.setLabel("left", "TMP (mBar)")
        self.tmp_plot.setLabel("bottom", "Time (s)")
        self.tmp_plot.setYRange(0, 200)
        self.tmp_plot.setMinimumHeight(220)
        self.tmp_plot.setMaximumHeight(360)
        self.tmp_plot.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.tmp_curve = self.tmp_plot.plot(pen={"color": "#38bdf8", "width": 2})
        metrics_layout.addWidget(self.tmp_plot, stretch=1)
        metrics_layout.setStretch(0, 0)
        metrics_layout.setStretch(1, 0)
        metrics_layout.setStretch(2, 1)

        content_row.addWidget(metrics_panel, 2)

        controls_panel, controls_layout = self._build_panel("Controls")
        controls_layout.setSpacing(14)

        self.motor_pump = MotorPumpControl()
        self.motor_pump.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        controls_layout.addWidget(self.motor_pump)

        control_button_style = """
        QPushButton {
            background-color: #3b82f6;
            color: #f8fafc;
            font-weight: 600;
            border: none;
            border-radius: 12px;
            padding: 12px 18px;
        }
        QPushButton:hover {
            background-color: #2563eb;
        }
        QPushButton:pressed {
            background-color: #1d4ed8;
        }
        QPushButton:checked {
            background-color: #22c55e;
            color: #0f172a;
        }
        """

        self.pid_button = QPushButton("Enable PID")
        self.pid_button.setCheckable(True)
        self.pid_button.setChecked(False)
        self.pid_button.clicked.connect(self.toggle_pid)
        self.pid_button.setFocusPolicy(Qt.NoFocus)
        self.pid_button.setCursor(Qt.PointingHandCursor)
        self._pid_btn_style_disabled = (
            "background-color: #1e293b; color: #f8fafc; font-weight: 600; "
            "border: 1px solid #334155; border-radius: 12px; padding: 12px 18px;"
        )
        self._pid_btn_style_enabled = (
            "background-color: #22c55e; color: #0f172a; font-weight: 600; "
            "border: none; border-radius: 12px; padding: 12px 18px;"
        )
        self.pid_button.setStyleSheet(self._pid_btn_style_disabled)
        self.pid_button.setFixedHeight(48)

        self.home_button = QPushButton("Home Valve")
        self.home_button.setCheckable(True)
        self.home_button.setFixedHeight(48)
        self.home_button.setStyleSheet(control_button_style)
        self.home_button.setFocusPolicy(Qt.NoFocus)
        self.home_button.setCursor(Qt.PointingHandCursor)
        self.home_button.clicked.connect(self.start_homing)

        pid_row = QHBoxLayout()
        pid_row.setContentsMargins(0, 0, 0, 0)
        pid_row.setSpacing(12)
        pid_row.addWidget(self.pid_button)
        pid_row.addWidget(self.home_button)
        controls_layout.addLayout(pid_row)

        self.setpoint_input = QLineEdit("80.0")
        self.setpoint_input.setValidator(QDoubleValidator(0.0, 10000.0, 1))
        self.setpoint_input.textChanged.connect(self.set_new_pid_setpoint)
        self.setpoint_input.setFixedHeight(26)
        self.setpoint_input.setAlignment(Qt.AlignCenter)
        self.setpoint_input.setPlaceholderText("PID Setpoint")

        self.target_liters_input = QLineEdit("1.0")
        self.target_liters_input.setValidator(QDoubleValidator(0.1, 100.0, 2))
        self.target_liters_input.setFixedHeight(26)
        self.target_liters_input.setAlignment(Qt.AlignCenter)
        self.target_liters_input.setPlaceholderText("e.g. 1.0")

        self.idle_time_input = QLineEdit("2.0")
        self.idle_time_input.setValidator(QDoubleValidator(0.1, 60.0, 2))
        self.idle_time_input.setFixedHeight(26)
        self.idle_time_input.setAlignment(Qt.AlignCenter)
        self.idle_time_input.setPlaceholderText("e.g. 2.0")

        form_layout = QGridLayout()
        form_layout.setContentsMargins(0, 0, 0, 0)
        form_layout.setHorizontalSpacing(10)
        form_layout.setVerticalSpacing(6)
        form_layout.addWidget(QLabel("Setpoint (mBar)"), 0, 0)
        form_layout.addWidget(self.setpoint_input, 0, 1)
        form_layout.addWidget(QLabel("Target Volume (L)"), 1, 0)
        form_layout.addWidget(self.target_liters_input, 1, 1)
        form_layout.addWidget(QLabel("Clean Time (min)"), 2, 0)
        form_layout.addWidget(self.idle_time_input, 2, 1)
        controls_layout.addLayout(form_layout)

        content_row.addWidget(controls_panel, 1)

        main_layout.addLayout(content_row)

        self._reset_btn_style_normal = (
            "background-color: #38bdf8; color: #0f172a; font-weight: 600; "
            "border: none; border-radius: 12px; padding: 10px 16px;"
        )
        self._reset_btn_style_flash = (
            "background-color: #22c55e; color: #0f172a; font-weight: 600; "
            "border: none; border-radius: 12px; padding: 10px 16px;"
        )
        self.reset_button = QPushButton("Reset Metrics")
        self.reset_button.setFocusPolicy(Qt.NoFocus)
        self.reset_button.setCursor(Qt.PointingHandCursor)
        self.reset_button.setStyleSheet(self._reset_btn_style_normal)
        self.reset_button.setFixedHeight(38)
        self.reset_button.clicked.connect(self._on_reset_clicked)

        sequences_panel, sequences_layout = self._build_panel("Sequences")
        sequences_panel.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        sequences_layout.setSpacing(12)

        self.sequence_buttons = {}
        sequence_names = [
            "Full Sequence",
            "Deaeration",
            "Concentration",
            "Elution",
            "Clean 1",
            "Clean 2",
        ]
        sequence_button_style = """
        QPushButton {
            background-color: #3b82f6;
            color: #f8fafc;
            font-size: 14px;
            font-weight: 600;
            border: none;
            border-radius: 12px;
            padding: 12px 16px;
        }
        QPushButton:hover {
            background-color: #2563eb;
        }
        QPushButton:pressed {
            background-color: #1d4ed8;
        }
        """

        seq_grid = QGridLayout()
        seq_grid.setContentsMargins(0, 0, 0, 0)
        seq_grid.setHorizontalSpacing(10)
        seq_grid.setVerticalSpacing(10)

        columns = 3
        special_handlers = {"Full Sequence": self._on_full_sequence_clicked}
        for idx, name in enumerate(sequence_names):
            btn = QPushButton(name)
            btn.setMinimumSize(130, 48)
            btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            btn.setStyleSheet(sequence_button_style)
            btn.setFocusPolicy(Qt.NoFocus)
            btn.setCursor(Qt.PointingHandCursor)
            handler = special_handlers.get(name)
            if handler:
                btn.clicked.connect(handler)
            else:
                btn.clicked.connect(
                    lambda _, n=name: threading.Thread(
                        target=self.run_sequence, args=(n,), daemon=True
                    ).start()
                )
            row, col = divmod(idx, columns)
            seq_grid.addWidget(btn, row, col)
            self.sequence_buttons[name] = btn

        for col in range(columns):
            seq_grid.setColumnStretch(col, 1)

        sequences_layout.addLayout(seq_grid)

        controls_row = QHBoxLayout()
        controls_row.setContentsMargins(0, 0, 0, 0)
        controls_row.setSpacing(10)
        controls_row.addStretch()
        self.timer_label = QLabel("Timer: 00:00")
        self.timer_label.setObjectName("timerBadge")
        controls_row.addWidget(self.timer_label)
        controls_row.addSpacing(6)
        controls_row.addWidget(self.reset_button)
        self.stop_button = QPushButton("STOP")
        self.stop_button.setFixedHeight(42)
        self.stop_button.setMinimumWidth(150)
        self._stop_btn_style_normal = (
            "background-color: #dc2626; color: #f8fafc; font-weight: 700; "
            "border: none; border-radius: 14px; padding: 8px 20px; font-size: 17px;"
        )
        self._stop_btn_style_flash = (
            "background-color: #991b1b; color: #f8fafc; font-weight: 700; "
            "border: none; border-radius: 14px; padding: 8px 20px; font-size: 17px;"
        )
        self.stop_button.setStyleSheet(self._stop_btn_style_normal)
        self.stop_button.setFocusPolicy(Qt.NoFocus)
        self.stop_button.setCursor(Qt.PointingHandCursor)
        self.stop_button.clicked.connect(self._on_stop_clicked)
        controls_row.addWidget(self.stop_button)
        sequences_layout.addLayout(controls_row)

        main_layout.addWidget(sequences_panel)

        log_panel, log_layout = self._build_panel("Event Log")
        log_panel.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        log_panel.setMinimumHeight(130)
        log_panel.setMaximumHeight(160)
        self.event_log = QPlainTextEdit()
        self.event_log.setReadOnly(True)
        self.event_log.setMaximumBlockCount(500)
        self.event_log.setStyleSheet(
            "background-color: #0f172a; color: #e2e8f0; "
            "border: 1px solid #1f2937; border-radius: 10px; font-family: 'Consolas', 'Courier New', monospace;"
        )
        self.event_log.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        log_layout.addWidget(self.event_log)
        main_layout.addWidget(log_panel)
        main_layout.setStretch(0, 4)
        main_layout.setStretch(1, 1)
        main_layout.setStretch(2, 1)

        register_ui_logger(self.log_event)

        self.flow_meter = FlowMeter()
        self.pid_valve = PIDValveController(lambda: self.tmp_value)
        self.pid_valve.hall_led_callback = lambda val: self.hall_led.setStyleSheet(
            "background-color: #22c55e; border-radius: 8px;"
            if val
            else "background-color: #475569; border-radius: 8px;"
        )

        self.tmp_history = []
        self.tmp_time_history = []
        self.elapsed_tmp_time = 0
        self.tmp_value = 0.0
        self.smoothed_pressures = [0.0, 0.0, 0.0]  # IN, OUT, FILTER
        self.ema_alpha = 0.5  # Tune this (lower = smoother)

        self.startTimer(1000)
        self.elapsed_seconds = 0

        self.valve_sync_timer = QTimer(self)
        self.valve_sync_timer.timeout.connect(self._sync_valve_buttons)
        self.valve_sync_timer.start(200)   # 5x per second; adjust if you like

        self.log_event("Control console ready.")

    def _build_panel(self, title: str):
        frame = QFrame()
        frame.setObjectName("panel")
        layout = QVBoxLayout(frame)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(12)
        heading = QLabel(title)
        heading.setObjectName("panelTitle")
        layout.addWidget(heading)
        return frame, layout

    def log_event(self, message: str, context: str = None):
        timestamp = time.strftime("%H:%M:%S")
        ctx = context if context is not None else (self._log_context or "System")
        entry = f"[{timestamp}] ({ctx}) {message}"
        QTimer.singleShot(0, lambda e=entry: self._append_log(e))

    def _append_log(self, entry: str):
        if not hasattr(self, "event_log") or self.event_log is None:
            return
        self.event_log.appendPlainText(entry)
        scrollbar = self.event_log.verticalScrollBar()
        if scrollbar:
            scrollbar.setValue(scrollbar.maximum())

    def _on_full_sequence_clicked(self):
        if self._full_sequence_running:
            self.log_event("Full Sequence already running.", context="Full Sequence")
            return

        self.stop_requested = False
        self._full_sequence_running = True
        self.log_event("Full Sequence started.", context="Full Sequence")
        emit_ui_log("Full Sequence started.")

        fs_button = self.sequence_buttons.get("Full Sequence")
        if fs_button:
            QTimer.singleShot(0, lambda: fs_button.setEnabled(False))

        def worker():
            prev_context = self._log_context
            self._log_context = "Full Sequence"
            sequence_order = ["Deaeration", "Concentration", "Elution"]
            aborted = False
            error_msg = None
            try:
                for step in sequence_order:
                    if self.stop_requested:
                        aborted = True
                        break
                    self.log_event(f"{step} sequence started (Full Sequence).", context="Full Sequence")
                    self.run_sequence(step, announce=False)
                    if self.stop_requested:
                        aborted = True
                        break
                    self.log_event(f"{step} sequence completed (Full Sequence).", context="Full Sequence")
            except Exception as exc:
                aborted = True
                error_msg = str(exc)
                self.log_event(f"Full Sequence encountered an error: {exc}", context="Full Sequence")
            finally:
                self._full_sequence_running = False
                self._full_sequence_thread = None
                if fs_button:
                    QTimer.singleShot(0, self._ensure_full_sequence_button_ready)
                if self.stop_requested:
                    self.log_event("Full Sequence stopped by user.", context="Full Sequence")
                    emit_ui_log("Full Sequence stopped by user.")
                elif error_msg:
                    emit_ui_log("Full Sequence encountered an error.")
                elif not aborted:
                    self.log_event("Full Sequence completed.", context="Full Sequence")
                    emit_ui_log("Full Sequence completed.")
                self._log_context = prev_context

        self._full_sequence_thread = threading.Thread(target=worker, daemon=True)
        self._full_sequence_thread.start()

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
        self.log_event("Elution valve reset to default state.")

    def stop_current_sequence(self):
        print("[INFO] Stop button pressed.")
        self.stop_requested = True
        self._full_sequence_running = False
        self.log_event("Stop requested for active sequence.")
        self.force_close_elute_valve()
        self._sync_valve_buttons()
        self._ensure_full_sequence_button_ready()

    def _ensure_full_sequence_button_ready(self):
        fs_button = self.sequence_buttons.get("Full Sequence")
        if not fs_button:
            return
        thread = getattr(self, "_full_sequence_thread", None)
        if thread and thread.is_alive():
            QTimer.singleShot(200, self._ensure_full_sequence_button_ready)
            return
        fs_button.setEnabled(True)


    def reset_flow_and_timer(self):
        self.flow_meter.reset()
        self.elapsed_seconds = 0
        self.tmp_history.clear()
        self.tmp_time_history.clear()
        self.elapsed_tmp_time = 0
        self.tmp_curve.setData([], [])
        self.log_event("Flow meter and timer cleared.")
    
    def get_target_liters(self):
        try:
            return float(self.target_liters_input.text())
        except ValueError:
            return 1.0  # fallback default
    
    def toggle_pid(self):
        enabled = self.pid_button.isChecked()
        self.pid_valve.set_enabled(enabled)

        self.pid_button.setText("PID Enabled" if enabled else "Enable PID")
        self.pid_button.setStyleSheet(
            self._pid_btn_style_enabled if enabled else self._pid_btn_style_disabled
        )
        self.log_event(f"PID {'enabled' if enabled else 'disabled'}.")


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
        self.log_event("PID homing requested.", context="PID")
        threading.Thread(target=self._run_homing, daemon=True).start()

    def _run_homing(self):
        try:
            self.log_event("PID homing started.", context="PID")
            self.pid_valve.homing_routine()
            self.log_event("PID homing completed.", context="PID")
        finally:
            QTimer.singleShot(0, self._end_homing)  # back to Qt main thread

    def _end_homing(self):
        self.home_button.setChecked(False)  # back to gray
        self.home_button.setEnabled(True)
        self._homing_running = False

    def run_sequence(self, name, announce=True):
        self.stop_requested = False
        prev_context = self._log_context
        self._log_context = name
        if announce:
            self.log_event(f"{name} sequence started.", context=name)

        self.force_close_elute_valve()   # Always close elution valve before (re)starting any sequence
        self._run_homing()

        errored = False
        try:
            if name == "Deaeration":
                run_deaeration(self.valves, self.motor_pump, lambda: self.stop_requested, self.flow_meter, self.reset_flow_and_timer)
            elif name == "Concentration":
                # Enable PID before running (use True/False, not 1/0)
                try:
                    # Grab current GUI setpoint; fall back to existing setpoint if invalid
                    setpoint_text = self.setpoint_input.text().strip()
                    try:
                        setpoint_value = float(setpoint_text)
                    except ValueError:
                        setpoint_value = self.pid_valve.setpoint
                    QTimer.singleShot(
                        0, lambda v=setpoint_value: self.setpoint_input.setText(f"{v:.1f}")
                    )
                    self.pid_valve.set_setpoint(setpoint_value)
                    self.log_event(f"PID setpoint applied: {setpoint_value:.1f} mBar", context=name)

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
                        self.log_event("PID disabled and valve homing triggered.", context=name)
                    except Exception as e:
                        print(f"[WARN] could not disable PID automatically: {e}")

            elif name == "Elution":
                run_elution(self.valves, self.motor_pump, lambda: self.stop_requested, self.flow_meter, self.reset_flow_and_timer)
            elif name == "Clean 1":
                run_clean1(self.valves, self.motor_pump, self.get_idle_time_minutes, lambda: self.stop_requested, self.flow_meter, self.reset_flow_and_timer)
            elif name == "Clean 2":
                run_clean2(self.valves, self.motor_pump, lambda: self.stop_requested, self.flow_meter, self.reset_flow_and_timer,)
            else:
                self.log_event(f"No handler defined for {name}.", context=name)
                errored = True
        except Exception as exc:
            errored = True
            self.log_event(f"{name} sequence error: {exc}", context=name)
            raise
        finally:
            if self.stop_requested:
                self.log_event(f"{name} sequence stopped by user.", context=name)
            elif not errored and announce:
                self.log_event(f"{name} sequence completed.", context=name)
            self._log_context = prev_context

    def _on_stop_clicked(self):
        # Flash the button immediately
        self._flash_stop_button(ms=1000)
        self.log_event("Stop button pressed.")
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
        # flash confirmation
        self._flash_reset_button()
        self.log_event("Metrics reset requested.")

    def _flash_reset_button(self, ms=1000):
        # remember current look/text to restore later
        self._reset_prev_style = self.reset_button.styleSheet()
        self._reset_prev_text = self.reset_button.text()
        # show a green flash + prevent double-clicks
        self.reset_button.setText("Reset ✓")
        self.reset_button.setStyleSheet(self._reset_btn_style_flash)
        self.reset_button.setEnabled(False)
        QTimer.singleShot(ms, self._restore_reset_button)

    def _restore_reset_button(self):
        if hasattr(self, "_reset_prev_text"):
            self.reset_button.setText(self._reset_prev_text)
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
