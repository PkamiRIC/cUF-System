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

class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Valve Control Panel")
        self.setGeometry(100, 100, 800, 200)

        layout = QHBoxLayout()

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
            layout.addWidget(ValveButton(valve))

        self.setLayout(layout)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
