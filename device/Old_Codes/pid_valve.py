import sys
import threading
import time
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QLabel,
    QPushButton, QLineEdit, QHBoxLayout
)
from PyQt5.QtCore import QTimer
from librpiplc import rpiplc as plc

class ValveControl(QWidget):
    def __init__(self):
        plc.init("RPIPLC_V6", "RPIPLC_38AR")
        super().__init__()
        self.setWindowTitle("Manual Valve Control")
        self.setGeometry(200, 200, 400, 200)

        self.enabled = False
        self.direction = True

        # Define pins
        self.STEP = "Q0.5"
        self.DIR = "Q0.4"
        self.EN = "Q0.3"
        self.HALL = "I0.12"

        # Set pin modes
        plc.pin_mode(self.STEP, plc.OUTPUT)
        plc.pin_mode(self.DIR, plc.OUTPUT)
        plc.pin_mode(self.EN, plc.OUTPUT)
        plc.pin_mode(self.HALL, plc.INPUT)

        # GUI layout
        layout = QVBoxLayout()

        self.enable_button = QPushButton("Run 1500 Steps")
        self.enable_button.clicked.connect(self.run_valve)

        self.dir_button = QPushButton("Direction: Forward")
        self.dir_button.setCheckable(True)
        self.dir_button.clicked.connect(self.toggle_direction)

        self.led = QLabel()
        self.led.setFixedSize(20, 20)
        self.led.setStyleSheet("background-color: grey; border-radius: 10px;")

        layout.addWidget(self.enable_button)
        layout.addWidget(self.dir_button)
        layout.addWidget(QLabel("Hall Sensor:"))
        layout.addWidget(self.led)
        self.setLayout(layout)

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_hall_led)
        self.timer.start(100)

    def toggle_direction(self):
        self.direction = self.dir_button.isChecked()
        plc.digital_write(self.DIR, plc.HIGH if self.direction else plc.LOW)
        print(f"DIR = {'HIGH' if self.direction else 'LOW'} ({self.DIR})")
        self.dir_button.setText("Direction: Forward" if self.direction else "Direction: Reverse")

    def run_valve(self):
        threading.Thread(target=self._step_1500, daemon=True).start()

    def _step_1500(self):
        step_count = 1500
        plc.digital_write(self.DIR, plc.HIGH if self.direction else plc.LOW)
        plc.digital_write(self.EN, plc.LOW)  # LOW = ENABLE
        time.sleep(0.01)

        print(f"Starting {step_count} steps...")
        for i in range(step_count):
            plc.digital_write(self.STEP, plc.HIGH)
            time.sleep(0.00002)
            plc.digital_write(self.STEP, plc.LOW)
            time.sleep(0.00002)

        print("Done.")
        plc.digital_write(self.EN, plc.HIGH)  # Disable

    def update_hall_led(self):
        val = plc.digital_read(self.HALL)
        color = "green" if val else "grey"
        self.led.setStyleSheet(f"background-color: %s; border-radius: 10px;" % color)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = ValveControl()
    window.show()
    sys.exit(app.exec_())
