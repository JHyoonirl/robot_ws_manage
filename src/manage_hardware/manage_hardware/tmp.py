from PyQt5 import QtWidgets, QtCore
from FT_SENSOR import FTSensor
import sys

class SensorVisual(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.sensor = FTSensor(port='COM10')
        self.initUI()

    def initUI(self):
        # Initialize sensor and check its status
        if not self.sensor.ft_sensor_init():
            raise Exception("센서 초기화 실패")

        # Set up the GUI layout
        self.grid = QtWidgets.QGridLayout()
        self.labels = [QtWidgets.QLabel('0') for _ in range(6)]
        for i, label in enumerate(self.labels):
            self.grid.addWidget(label, 0, i)

        # Add a button to close the application
        self.quitButton = QtWidgets.QPushButton('Quit', self)
        self.quitButton.clicked.connect(self.close_application)
        self.grid.addWidget(self.quitButton, 1, 0, 1, 6)  # Span the button across all columns

        self.setLayout(self.grid)
        self.setWindowTitle('Sensor Data Display')
        self.setGeometry(300, 300, 350, 150)
        self.show()

        # Timer to update sensor data regularly
        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(1)  # 100 ms
        self.timer.timeout.connect(self.update_labels)
        self.timer.start()

    def update_labels(self):
        # Read sensor data
        data = self.sensor.data_read_and_process()
        if self.sensor.new_data_available:
            for i, value in enumerate(data):
                self.labels[i].setText(str(value))
            self.sensor.new_data_available = False

    def close_application(self):
        # Safely close the application
        self.sensor.stop()  # Ensure the sensor thread is stopped properly
        self.close()  # Close the GUI

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    ex = SensorVisual()
    sys.exit(app.exec_())
