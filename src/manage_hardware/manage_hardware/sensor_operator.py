from multiprocessing import Process, Queue
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.executors import MultiThreadedExecutor
# from FT_SENSOR import FTSensor
from custom_module.FT_SENSOR import FTSensor
import time
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QPushButton, QVBoxLayout, QSpinBox, QLabel
from PyQt5.QtCore import QTimer
from PyQt5 import uic
from threading import Thread
import threading
from std_msgs.msg import Bool, Float64
from geometry_msgs.msg import Vector3
from custominterface.srv import Status
from builtin_interfaces.msg import Time
from pyqtgraph import PlotWidget
import sys
import signal


class Sensor(Node):
    def __init__(self):
        super().__init__('sensor_operator')
        self.declare_parameter('usb_port', '/dev/ttyUSB0')
        usb_port = self.get_parameter('usb_port').get_parameter_value().string_value
        self.qos_profile = QoSProfile(depth=10)
        # self.qos_profile = QoSProfile(history = QoSHistoryPolicy.KEEP_LAST, depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.sensor_state = False

        self.sensor_data_read_thread = Thread(target=self.data_process, daemon=True)
        self.sensor = FTSensor(port=usb_port)
        self.init_status = self.sensor.ft_sensor_init()
        self.decoded_force = [0, 0, 0]
        self.decoded_torque = [0, 0, 0]
        self.sensor_state = False

        # 데이터 퍼블리싱을 위한 퍼블리셔 설정
        self.force_publisher = self.create_publisher(Vector3, 'force_data', self.qos_profile)
        self.torque_publisher = self.create_publisher(Vector3, 'torque_data', self.qos_profile)
        
        self.test_switch = False
        self.sensor_test_msg = 3

        self.data_on = False
        self.test_bias = False
        self.data_buffer = bytearray()
        self.lock = threading.Lock()
        self.packet_count = 0
        
        if self.init_status:
            # self.read_thread = Thread(target=self.sensor.start_reading, daemon=True)
            # self.process_thread = Thread(target=self.sensor.process_data, daemon=True)
            self.sensor_data_read_thread.start()
            self.timer = self.create_timer(0.005, self.publish_sensor)
        

    def publish_sensor(self):
        force_msg = Vector3()
        torque_msg = Vector3()

        # 모든 메시지에 동일한 시간 스탬프 적용
        # if self.data_on and self.decoded_force != None and self.decoded_torque != None:
            # 데이터 할당
        force_msg.x =float(self.decoded_force[0])
        force_msg.y =float(self.decoded_force[1])
        force_msg.z =float(self.decoded_force[2])
        torque_msg.x =float(self.decoded_torque[0])
        torque_msg.y =float(self.decoded_torque[1])
        torque_msg.z =float(self.decoded_torque[2])
        
        # 데이터 발행
        self.force_publisher.publish(force_msg)
        self.torque_publisher.publish(torque_msg)
    
    def data_process(self):
        while True:
            if self.sensor_state:
                try:
                    self.decoded_force, self.decoded_torque = self.sensor.ft_sensor_continuous_data_read()
                except Exception as e:
                    print(f"Error: {e}")

    def start_reading(self):
        # next_reading_time = time.time()
        while True:
            self.next_process_time = time.time()
            data = self.sensor.sensor.read(50)  # 16바이트 데이터 읽기
            # print(data)
            with self.lock:
                self.data_buffer.extend(data)
             # 다음 데이터 처리 시간을 기다립니다.
            self.next_process_time += 0.005
            sleep_time = self.next_process_time - time.time()
            # if sleep_time > 0:
                # time.sleep(sleep_time)
                
    def process_data(self):
        while True:
            with self.lock:
                if len(self.data_buffer) >= 16:
                    if self.data_buffer[0] == 0x0B:
                        packet = self.data_buffer[:16]
                        self.decoded_force, self.decoded_torque = self.sensor.decode_received_data(packet[1:13])
                        self.packet_count += 1
                        self.data_buffer = self.data_buffer[16:]
                        # now = self.get_clock().now()  # 현재 시간을 한 번만 계산
                            
                        # 모든 메시지 생성
                        # time_msg = Float64()
                        force_msg = Vector3()
                        torque_msg = Vector3()

                        # 모든 메시지에 동일한 시간 스탬프 적용
                        # force_msg.header.stamp = now.to_msg()
                        # torque_msg.header.stamp = now.to_msg()
                        # time_msg.data = float(now.nanoseconds) / 1e9  # 나노초를 초로 변환
                        if self.data_on and self.decoded_force != None and self.decoded_torque != None:
                            # 데이터 할당
                            force_msg.x, force_msg.y, force_msg.z = self.decoded_force[:]
                            torque_msg.x, torque_msg.y, torque_msg.z = self.decoded_torque[:]
                            
                            # 데이터 발행
                            self.force_publisher.publish(force_msg)
                            self.torque_publisher.publish(torque_msg)
                            # self.sensor_time.publish(time_msg)
                        time.sleep(0.0005)
                    else:
                        self.data_buffer.pop(0)
            

class SensorApp(QMainWindow):
    def __init__(self, sensor: Sensor):
        QMainWindow.__init__(self)
        self.sensor = sensor
        self.threadhold = 200

        self.ui = uic.loadUi('UI/sensor.ui', self)
        self.init_ui()
        
        self.force_data = [[], [], []]  # 각 축의 힘 데이터를 저장하는 리스트
        self.torque_data = [[], [], []]  # 각 축의 토크 데이터를 저장하는 리스트
        self.move(1400, 0)
        self.show()

    def init_ui(self):

        # 버튼 위젯 찾기
        self.btn_bias = self.findChild(QPushButton, 'Setbias')
        self.btn_start = self.findChild(QPushButton, 'Setstart')
        self.btn_stop = self.findChild(QPushButton, 'Setstop')
        self.btn_quit = self.findChild(QPushButton, 'Quit')

        # # 버튼 클릭 이벤트 연결
        self.btn_bias.clicked.connect(self.set_bias)
        self.btn_start.clicked.connect(self.turn_on)
        self.btn_stop.clicked.connect(self.turn_off)
        self.btn_quit.clicked.connect(self.close)


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

class Ros2Thread(Thread):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.executor = MultiThreadedExecutor()

    def run(self):
        self.executor.add_node(self.node)
        try:
            self.executor.spin()
        finally:
            self.executor.shutdown()

    def stop(self):
        self.executor.remove_node(self.node)
        self.node.destroy_node()

def main(args=None):
    def signal_handler(sig, frame):
        print("Shutting down...")
        app.quit()  # QApplication을 종료합니다.
        sys.exit(0)  # 강제로 종료합니다.

    signal.signal(signal.SIGINT, signal_handler)  # SIGINT 신호를 처리하기 위해 핸들러를 등록합니다.

    rclpy.init(args=args)
    sensor = Sensor()
    ros2_thread = Ros2Thread(sensor)
    ros2_thread.start()

    app = QApplication(sys.argv)
    ex = SensorApp(sensor)

    try:
        sys.exit(app.exec_())
    finally:
        ros2_thread.stop()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
