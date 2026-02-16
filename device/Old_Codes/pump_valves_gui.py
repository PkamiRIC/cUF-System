import sys
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout, QLabel, QHBoxLayout
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QColor, QPalette
from librpiplc import rpiplc as plc

# Real Valve class using Raspberry PLC relay output
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
    def __init__(self, valve: Valve):
        super().__init__(self.label_for(valve.name))
        self.valve = valve
        self.setFixedSize(100, 100)
        self.setCheckable(True)
        self.setStyleSheet(self.style_for_state(False))
        self.clicked.connect(self.toggle_valve)

    def style_for_state(self, state):
        color = "green" if state else "red"
        return f"border-radius: 50px; background-color: {color}; font-size: 16px;"

    def label_for(self, pin_name):
        mapping = {
            "R1.1": "V1",
            "R1.2": "V2",
            "R1.3": "V3",
            "R1.4": "V8",
            "R1.5": "V9",
            "R1.6": "V10"
        }
        return mapping.get(pin_name, pin_name)

    def toggle_valve(self):
        self.valve.toggle()
        self.setChecked(self.valve.state)
        self.setStyleSheet(self.style_for_state(self.valve.state))

class MotorPumpControl(QWidget):
    def __init__(self):
        super().__init__()
        layout = QVBoxLayout()

        self.enable_pin = "Q0.0"
        self.dir_plus_pin = "Q0.1"
        self.dir_minus_pin = "Q0.2"
        self.speed_toggle_pin = "Q0.7"

        for pin in [self.enable_pin, self.dir_plus_pin, self.dir_minus_pin, self.speed_toggle_pin]:
            plc.pin_mode(pin, plc.OUTPUT)
            plc.digital_write(pin, plc.LOW)

        self.enable_button = QPushButton("Enable Pump")
        self.enable_button.setCheckable(True)
        self.enable_button.clicked.connect(self.toggle_enable)

        self.direction_button = QPushButton("Direction: +")
        self.direction_button.setCheckable(True)
        self.direction_button.clicked.connect(self.toggle_direction)

        self.speed_button = QPushButton("Speed Toggle")
        self.speed_button.setCheckable(True)
        self.speed_button.clicked.connect(self.toggle_speed)

        layout.addWidget(self.enable_button)
        layout.addWidget(self.direction_button)
        layout.addWidget(self.speed_button)
        self.setLayout(layout)

    def toggle_enable(self):
        state = self.enable_button.isChecked()
        # Logic LOW = Enabled
        plc.digital_write(self.enable_pin, plc.LOW if state else plc.HIGH)
        self.enable_button.setText("Pump ON (LOW)" if state else "Pump OFF (HIGH)")

    def toggle_direction(self):
        forward = self.direction_button.isChecked()
        plc.digital_write(self.dir_plus_pin, plc.HIGH if forward else plc.LOW)
        plc.digital_write(self.dir_minus_pin, plc.LOW if forward else plc.HIGH)
        self.direction_button.setText("Direction: +" if forward else "Direction: -")

    def toggle_speed(self):
        state = self.speed_button.isChecked()
        plc.digital_write(self.speed_toggle_pin, plc.HIGH if state else plc.LOW)

class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Valve and Pump Control Panel")
        self.setGeometry(100, 100, 1000, 300)

        main_layout = QVBoxLayout()
        valve_layout = QHBoxLayout()

        # Initialize PLC
        plc.init("RPIPLC_V6", "RPIPLC_38AR")

        # Create 6 valves on relays and buttons
        self.valves = [
            Valve("R1.1"),  # V1
            Valve("R1.2"),  # V2
            Valve("R1.3"),  # V3
            Valve("R1.4"),  # V8
            Valve("R1.5"),  # V9
            Valve("R1.6")   # V10
        ]
        for valve in self.valves:
            valve_layout.addWidget(ValveButton(valve))

        main_layout.addLayout(valve_layout)
        main_layout.addWidget(MotorPumpControl())
        self.setLayout(main_layout)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
