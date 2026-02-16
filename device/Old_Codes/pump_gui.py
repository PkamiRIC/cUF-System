import sys
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout
from librpiplc import rpiplc as plc

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
        plc.digital_write(self.enable_pin, plc.LOW if state else plc.HIGH)  # LOW = ENABLED
        self.enable_button.setText("Pump ON (LOW)" if state else "Pump OFF (HIGH)")

    def toggle_direction(self):
        forward = self.direction_button.isChecked()
        plc.digital_write(self.dir_plus_pin, plc.HIGH if forward else plc.LOW)
        plc.digital_write(self.dir_minus_pin, plc.LOW if forward else plc.HIGH)
        self.direction_button.setText("Direction: +" if forward else "Direction: -")

    def toggle_speed(self):
        state = self.speed_button.isChecked()
        plc.digital_write(self.speed_toggle_pin, plc.HIGH if state else plc.LOW)

if __name__ == '__main__':
    plc.init("RPIPLC_V6", "RPIPLC_38AR")
    app = QApplication(sys.argv)
    window = MotorPumpControl()
    window.setWindowTitle("Pump Control")
    window.show()
    sys.exit(app.exec_())
