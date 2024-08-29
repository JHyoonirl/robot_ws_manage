from FT_SENSOR import FTSensor
import sys

class SensorVisual():
    def __init__(self):
        super().__init__()
        self.sensor = FTSensor(port='/dev/ttyUSB0')
        self.initUI()

    def initUI(self):
        # Initialize sensor and check its status
        if not self.sensor.ft_sensor_init():
            raise Exception("센서 초기화 실패")

    def update_labels(self):
        # Read sensor data
        data = self.sensor.data_read_and_process()
        if self.sensor.new_data_available:
            # for i, value in enumerate(data):
                # self.labels[i].setText(str(value))
            print(data)
            self.sensor.new_data_available = False

    def close_application(self):

        self.sensor.stop()  # Ensure the sensor thread is stopped properly
        self.close()  # Close the GUI

if __name__ == "__main__":
    sensor = SensorVisual()
    while True:
        sensor.update_labels()
