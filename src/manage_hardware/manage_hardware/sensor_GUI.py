from PyQt5.QtWidgets import QPushButton, QLabel, QApplication, QMainWindow
from PyQt5 import uic
from PyQt5.QtCore import QTimer
from FT_SENSOR_jh import FTSensor
import sys
import os
import threading
from pyqtgraph import PlotWidget  # 그래프를 위한 라이브러리
print(os.getcwd())
import argparse


class SensorApp(QMainWindow):
    def __init__(self, sensor):
        QMainWindow.__init__(self)
        self.sensor = sensor
        self.threadhold = 200

        self.ui = uic.loadUi('UI/sensor.ui', self)
        self.init_ui()
        
        self.force_data = [[], [], []]  # 각 축의 힘 데이터를 저장하는 리스트
        self.torque_data = [[], [], []]  # 각 축의 토크 데이터를 저장하는 리스트
        self.show()

    def init_ui(self):

        # plot 위젯 찾기
        self.plot_force = self.findChild(PlotWidget, 'plot_force')
        self.plot_torque = self.findChild(PlotWidget, 'plot_torque')

        
        self.plot_force_x = self.plot_force.plot(pen='r', name='Force_x')
        self.plot_force_y = self.plot_force.plot(pen='g', name='Force_y')
        self.plot_force_z = self.plot_force.plot(pen='b', name='Force_z')
        self.plot_forces = [self.plot_force_x, self.plot_force_y, self.plot_force_z]
        self.plot_force.setTitle("Force Readings")
        self.plot_force.setBackground("w")
        self.plot_force.setYRange(-30, 30)
        self.plot_force.addLegend(offset=(10, 30))

        self.plot_torque_x = self.plot_torque.plot(pen='r', name='Torque_x')
        self.plot_torque_y = self.plot_torque.plot(pen='g', name='Torque_y')
        self.plot_torque_z = self.plot_torque.plot(pen='b', name='Torque_z')
        self.plot_torques = [self.plot_torque_x, self.plot_torque_y, self.plot_torque_z]
        self.plot_torque.setTitle("Torque Readings")
        self.plot_torque.setYRange(-3, 3)
        self.plot_torque.setBackground("w")
        self.plot_torque.addLegend(offset=(10, 30))

        ### 타이머 설정 ###
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_data)
        self.timer.start(5)  # 100ms 간격으로 업데이트
        
        # 버튼 위젯 찾기
        self.btn_bias = self.findChild(QPushButton, 'Setbias')
        self.btn_start = self.findChild(QPushButton, 'Setstart')
        self.btn_stop = self.findChild(QPushButton, 'Setstop')
        self.btn_quit = self.findChild(QPushButton, 'Quit')

        # 라벨 위젯 찾기
        self.label_force_x = self.findChild(QLabel, 'ForceX')
        self.label_torque_x = self.findChild(QLabel, 'TorqueX')
        self.label_force_y = self.findChild(QLabel, 'ForceY')
        self.label_torque_y = self.findChild(QLabel, 'TorqueY')
        self.label_force_z = self.findChild(QLabel, 'ForceZ')
        self.label_torque_z = self.findChild(QLabel, 'TorqueZ')

        self.force_labels = [
            self.findChild(QLabel, 'force_x_data'),
            self.findChild(QLabel, 'force_y_data'),
            self.findChild(QLabel, 'force_z_data')
        ]

        self.torque_labels = [
            self.findChild(QLabel, 'torque_x_data'),
            self.findChild(QLabel, 'torque_y_data'),
            self.findChild(QLabel, 'torque_z_data')
        ]

        # 버튼 클릭 이벤트 연결
        self.btn_bias.clicked.connect(self.set_bias)
        self.btn_start.clicked.connect(self.turn_on)
        self.btn_stop.clicked.connect(self.turn_off)
        self.btn_quit.clicked.connect(self.close)

    def update_data(self):
        force = self.sensor.decoded_force
        torque = self.sensor.decoded_torque
        # self.label_force_x_data.setText(f'Force: {force[0]}, {force[1]}, {force[2]}')
        # self.label_force_y_data.setText(f'Torque: {torque[0]}, {torque[1]}, {torque[2]}')

        for i, label in enumerate(self.force_labels):
            label.setText(f'{force[i]}')
        
        for i, label in enumerate(self.torque_labels):
            label.setText(f'{torque[i]}')


        ### 데이터 저장 및 그래프 업데이트 ###
        for i in range(3):
            if len(self.force_data[i]) >= self.threadhold:  # 최대 threadhold개 데이터 유지
                self.force_data[i].pop(0)
            if len(self.torque_data[i]) >= self.threadhold:
                self.torque_data[i].pop(0)
            self.force_data[i].append(force[i])
            self.torque_data[i].append(torque[i])
            self.plot_forces[i].setData(self.force_data[i])
            self.plot_torques[i].setData(self.torque_data[i])

        # self.plot_force.setData(self.force_data[0])  # 평탄화하여 데이터 설정
        # self.plot_torque.setData(self.torque_data[0])

    def set_bias(self):
        if self.sensor.init_status:
            self.sensor.sensor.ft_sensor_bias_set()

    def turn_on(self):
        if self.sensor.sensor.ft_sensor_continuous_data():
            self.sensor.sensor_state = True

        if self.sensor.sensor_state:
            self.btn_bias.setDisabled(True)

    def turn_off(self):
        if self.sensor.sensor.ft_sensor_stop_data():
            self.sensor.sensor_state = False

        if not self.sensor.sensor_state:
            self.btn_bias.setEnabled(True)
    
    def close(self):
        sys.exit()

class Sensor:
    def __init__(self, port='COM10'):
        self.sensor = FTSensor(port=port)
        self.init_status = self.sensor.ft_sensor_init()
        self.decoded_force = [0, 0, 0]
        self.decoded_torque = [0, 0, 0]
        self.sensor_state = False

    def data_process(self):
        while True:
            if self.sensor_state:
                try:
                    self.decoded_force, self.decoded_torque = self.sensor.ft_sensor_continuous_data_read()
                except Exception as e:
                    print(f"Error: {e}")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Control FT Sensor via CLI")
    parser.add_argument('--port', type=str, default='COM8', help='Serial port to connect to')
    args = parser.parse_args()
    app = QApplication(sys.argv)
    sensor = Sensor(args.port)
    sensor_thread = threading.Thread(target=sensor.data_process, daemon=True)
    sensor_thread.start()
    ex = SensorApp(sensor)
    
    sys.exit(app.exec_())
