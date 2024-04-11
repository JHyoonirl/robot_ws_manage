import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QPushButton
from PyQt5.QtCore import QCoreApplication
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib import animation
import numpy as np
import random
import serial
import time



class MyMplCanvas(FigureCanvas):
    '''
    그래프 생성을 위한 초기화 부분
    '''
    def __init__(self, parent=None, width=5, height=3, dpi=200, data_num=100):
        fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = fig.add_subplot(211, xlim=(0, data_num), ylim=(-100, 100))
        self.axes.set_title('Force data')
        # self.axes.set_xlabel('data number')
        self.axes.set_ylabel('force (N)')
        self.axes2 = fig.add_subplot(212, xlim=(0, data_num), ylim=(-5, 5))
        self.axes2.set_title('Torque data')
        # self.axes2.set_xlabel('data number')
        self.axes2.set_ylabel('Torque (N*m)')

        self.compute_initial_figure()
        FigureCanvas.__init__(self, fig)
        self.setParent(parent)
    
    def compute_initial_figure(self):
        pass


class AnimationWidget(QWidget):
    def __init__(self):
        QMainWindow.__init__(self)
        
        self.ser = serial.Serial(
            port='COM8',\
            baudrate=115200,\
            parity=serial.PARITY_NONE,\
            stopbits=serial.STOPBITS_ONE,\
            bytesize=serial.EIGHTBITS,\
                timeout=0)
        if self.ser != False:
            self.ft_sensor_init()
        self.data_read = [0x0A, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        self.data_num = 100

        vbox = QVBoxLayout()
        self.canvas = MyMplCanvas(self, width=10, height=8, dpi=100, data_num=self.data_num)
        vbox.addWidget(self.canvas)
        
        hbox = QHBoxLayout()
        self.start_button = QPushButton("start", self)
        self.stop_button = QPushButton("stop", self)
        self.quit_button = QPushButton("quit", self)
        self.start_button.clicked.connect(self.on_start)
        self.stop_button.clicked.connect(self.on_stop)
        self.quit_button.clicked.connect(QCoreApplication.instance().quit)
        hbox.addWidget(self.start_button)
        hbox.addWidget(self.stop_button)
        hbox.addWidget(self.quit_button)        
        vbox.addLayout(hbox)
        self.setLayout(vbox)
        

        self.x_ = np.arange(self.data_num)
        self.force_x = np.ones(self.data_num)*np.nan
        self.force_y = np.ones(self.data_num)*np.nan
        self.force_z = np.ones(self.data_num)*np.nan
        self.line1_x, = self.canvas.axes.plot(self.x_, self.force_x, animated=True, lw=2, label='force_x')
        self.line1_y, = self.canvas.axes.plot(self.x_, self.force_y, animated=True, lw=2, label='force_y')
        self.line1_z, = self.canvas.axes.plot(self.x_, self.force_z, animated=True, lw=2, label='force_z')

        self.torque_x = np.ones(self.data_num)*np.nan
        self.torque_y = np.ones(self.data_num)*np.nan
        self.torque_z = np.ones(self.data_num)*np.nan
        self.line2_x, = self.canvas.axes2.plot(self.x_, self.torque_x, animated=True, lw=2, label='torque_x')
        self.line2_y, = self.canvas.axes2.plot(self.x_, self.torque_y, animated=True, lw=2, label='torque_y')
        self.line2_z, = self.canvas.axes2.plot(self.x_, self.torque_z, animated=True, lw=2, label='torque_z')

        self.line = [self.line1_x, self.line1_y, self.line1_z, self.line2_x, self.line2_y, self.line2_z]

        # 범례 추가
        self.canvas.axes.legend()
        self.canvas.axes2.legend()
        self.canvas.figure.suptitle('FT Sensor data')

    def update_line(self, i):
        self.send_data_without_read(self.data_read)
        read_msg = self.ser.read(19)
        decoded_data = self.decode_received_data(read_msg)
        if decoded_data != False:
            for data, line in zip(decoded_data, self.line):
                old_y = line.get_ydata()
                new_y = np.r_[old_y[-self.data_num + 1:], data]
                self.line[self.line.index(line)].set_ydata(new_y)

        return self.line

    def on_start(self):
        self.ani = animation.FuncAnimation(self.canvas.figure, self.update_line, blit=True, interval=10)
        # self.ani2 = animation.FuncAnimation(self.canvas.figure, self.update_line2, blit=True, interval=10)
    
    def on_stop(self):
        self.ani._stop()
        # self.ani2._stop()

    def calculate_checksum(self, data):
        """
        데이터의 체크섬을 계산합니다.
        """
        checksum = sum(data) % 256
        return checksum

    def decode_received_data(self, data):
        """
        수신된 데이터에서 Start of Packet (SOP)와 End of Packet (EOP)을 찾아 데이터를 출력합니다.
        """
        sop_index = data.find(bytes([0x55]))  # SOP (Start of Packet)의 인덱스 찾기
        eop_index = data.find(bytes([0xAA]))  # EOP (End of Packet)의 인덱스 찾기
        force_raw = []
        torque_raw = []
        force = []
        torque = []
        try:
            if sop_index != -1 and eop_index != -1:  # SOP와 EOP 모두 존재하는 경우
                received_packet = data[sop_index:eop_index+1]  # SOP부터 EOP까지의 패킷 추출
                # print(received_packet)
                # print(received_packet[0])
                force_raw.append( received_packet[2] << 8 | received_packet[3])
                force_raw.append( received_packet[4] << 8 | received_packet[5])
                force_raw.append( received_packet[6] << 8 | received_packet[7])
                torque_raw.append( received_packet[8] << 8 | received_packet[9])
                torque_raw.append( received_packet[10] << 8 | received_packet[11])
                torque_raw.append( received_packet[12] << 8 | received_packet[13])
                for i in range(0, 3):
                    force_temp  = force_raw[i].to_bytes(2, 'big')
                    torque_temp  = torque_raw[i].to_bytes(2, 'big')
                    # [Divider] Force: 50, Torque: 2000
                    # Note: Resolution of RFT76-HA01 is same with RFT40-SA01
                    force.append(int.from_bytes(force_temp, "big", signed=True) / 50)
                    torque.append(int.from_bytes(torque_temp, "big", signed=True) / 2000)
                # 여기서 데이터 처리를 수행하거나 필요한 작업을 수행할 수 있습니다.
                # ///print(force)
            # else:
                # print("Incomplete packet received")
                data = force + torque
            return data
        except:
            pass
            return False
        
    def send_data_with_read(self, data):
        """
        데이터를 주어진 구조에 맞게 시리얼 포트로 전송합니다.
        """
        sop = bytes([0x55])  # SOP (Start of Packet)
        eop = bytes([0xAA])  # EOP (End of Packet)

        # 데이터와 체크섬 조합
        checksum = self.calculate_checksum(data)
        packet = sop + bytes(data) + bytes([checksum]) + eop

        # 시리얼 포트로 전송
        self.ser.write(packet)
        # print("send packet: {0}".format(packet.hex()))
        sent_data = self.ser.read(19)
        return sent_data

    def send_data_without_read(self, data):
        """
        데이터를 주어진 구조에 맞게 시리얼 포트로 전송합니다.
        """
        sop = bytes([0x55])  # SOP (Start of Packet)
        eop = bytes([0xAA])  # EOP (End of Packet)

        # 데이터와 체크섬 조합
        checksum = self.calculate_checksum(data)
        packet = sop + bytes(data) + bytes([checksum]) + eop

        # 시리얼 포트로 전송
        self.ser.write(packet)
    
    def ft_sensor_init(self):
        '''
        ft_sensor의 초기화 함수
        '''
        
        data_bias = [0x11, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        data_baudrate = [0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        data_filter = [0x08, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00]
        data_name = [0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        try:
            self.send_data_without_read(data_bias)
            self.send_data_without_read(data_baudrate)
            # print(data_bias)
            time.sleep(0.1)
            self.send_data_without_read(data_filter)
            time.sleep(0.1)
            print('FT sensor 초기화 성공')
        except:
            print('FT sensor 초기화 실패')
        
        

if __name__ == "__main__":
    qApp = QApplication(sys.argv)
    aw = AnimationWidget()
    aw.show()
    sys.exit(qApp.exec_())