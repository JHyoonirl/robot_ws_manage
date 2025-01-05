from multiprocessing import Process, Queue
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
# from FT_SENSOR import FTSensor
from sensor_GUI import Sensor, SensorApp
import time
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout, QSpinBox, QLabel
from threading import Thread
import threading
from std_msgs.msg import Bool, Float64
from geometry_msgs.msg import Vector3Stamped
from custominterface.srv import Status
from datetime import datetime
from builtin_interfaces.msg import Time
import sys


class FTSENSOR(Node):
    def __init__(self):
        super().__init__('FT_data')
        self.declare_parameter('usb_port', '/dev/ttyUSB0')
        usb_port = self.get_parameter('usb_port').get_parameter_value().string_value
        self.qos_profile = QoSProfile(depth=10)
        # self.qos_profile = QoSProfile(history = QoSHistoryPolicy.KEEP_LAST, depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)


        # self.sensor = Sensor(port=usb_port)

        # self.serial_port = self.sensor.sensor
        self.init_status = self.init_sensor()
        self.decoded_force = None
        self.decoded_torque = None

        # 데이터 퍼블리싱을 위한 퍼블리셔 설정
        self.force_publisher = self.create_publisher(Vector3Stamped, 'force_data', self.qos_profile)
        self.torque_publisher = self.create_publisher(Vector3Stamped, 'torque_data', self.qos_profile)
        # self.sensor_time = self.create_publisher(Float64, 'sensor_time', self.qos_profile)
        # self.pwm_sub = self.create_subscription(
        #     Float64,
        #     'pwm_signal',
        #     self.pwm_subscriber,
        #     self.qos_profile)
        # self.sensor_sub = self.create_subscription(Float64, 'sensor_test', self.sensor_test_fcn, self.qos_profile)
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
            
            self.timer = self.create_timer(0.005, self.publish_sensor)
        
        
    def init_sensor(self):
        if not self.sensor.ft_sensor_init():
            # self.get_logger().error('Failed to initialize sensor')
            self.get_logger().info('Failed to initialize sensor')
            return False
        else:
            self.sensor.ft_sensor_continuous_data()
            return True
            
    def publish_sensor(self):
        pass
        
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
                        now = self.get_clock().now()  # 현재 시간을 한 번만 계산
                            
                        # 모든 메시지 생성
                        time_msg = Float64()
                        force_msg = Vector3Stamped()
                        torque_msg = Vector3Stamped()

                        # 모든 메시지에 동일한 시간 스탬프 적용
                        force_msg.header.stamp = now.to_msg()
                        torque_msg.header.stamp = now.to_msg()
                        time_msg.data = float(now.nanoseconds) / 1e9  # 나노초를 초로 변환
                        if self.data_on and self.decoded_force != None and self.decoded_torque != None:
                            # 데이터 할당
                            force_msg.vector.x, force_msg.vector.y, force_msg.vector.z = self.decoded_force[:]
                            torque_msg.vector.x, torque_msg.vector.y, torque_msg.vector.z = self.decoded_torque[:]
                            force_msg.header.frame_id = 'force_frame'
                            torque_msg.header.frame_id = 'torque_frame'

                            # 데이터 발행
                            self.force_publisher.publish(force_msg)
                            self.torque_publisher.publish(torque_msg)
                            # self.sensor_time.publish(time_msg)
                        time.sleep(0.0005)
                    else:
                        self.data_buffer.pop(0)
            

class SensorApp(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.init_ui()
    
    def init_ui(self):
        layout = QVBoxLayout()
        self.btn_bias = QPushButton('Set Bias', self)
        self.btn_on = QPushButton('Turn On Data', self)
        self.btn_off = QPushButton('Turn Off Data', self)
        self.btn_bias.clicked.connect(self.set_bias)
        self.btn_on.clicked.connect(self.turn_on)
        self.btn_off.clicked.connect(self.turn_off)

        layout.addWidget(self.btn_bias)
        layout.addWidget(self.btn_on)
        layout.addWidget(self.btn_off)

        self.setLayout(layout)
        self.setWindowTitle('Sensor UI')
        
        self.show()
        QApplication.processEvents()
        self.move(1000, 500)

    def set_bias(self):
        if self.node.init_status or self.node.test_bias == True:
            bias_result = self.node.sensor.ft_sensor_bias_set()

    def turn_on(self):
        if self.node.sensor.ft_sensor_continuous_data():
            self.btn_bias.setDisabled(True)
            self.node.data_on = True

    def turn_off(self):
        if self.node.sensor.ft_sensor_stop_data():
            self.btn_bias.setEnabled(True)
            self.node.data_on = False

def main(args=None):
    rclpy.init(args=args)
    sensor = Sensor()
    thread = Thread(target=rclpy.spin, args=(sensor,), daemon=True)
    sensor_data_read_thread = Thread(target=sensor.data_process, daemon=True)
    # process_thread = Thread(target=node.process_data, daemon=True)
    thread.start()
    time.sleep(1)
    if sensor.init_status:
        sensor_data_read_thread.start()
        # process_thread.start()

    app = QApplication(sys.argv)
    ex = SensorApp(sensor)
    app.exec_()
    
    sensor.destroy_node()
    rclpy.shutdown()
    thread.join()

if __name__ == '__main__':
    main()
