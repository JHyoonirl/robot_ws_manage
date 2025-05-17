import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Vector3
import serial
import time
import math
from scipy.signal import savgol_filter
import numpy as np
import re


# sudo dmesg | grep tty :: usb 포트를 확인하는 코드
'''
시리얼 데이터를 읽는 부분
'''
class ESP32Board(Node):
    ##### Publisher와 Subscriber를 정의, Serial Port 정보를 정의
    def __init__(self):
        super().__init__('IMU_node')
        qos_profile = QoSProfile(depth=10)
        
        self.declare_parameter('usb_port_imu', '/dev/ttyUSB0')  # 기본값을 제공

        usb_port = self.get_parameter('usb_port_imu').get_parameter_value().string_value
        
        ser = serial.Serial(
            port = usb_port,
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=0.1)
        self.ser = ser
        self.status = False
        self.currnet_time = time.time()
        self.current_angle = 0.0
        self.previous_angle = 0.0
        self.knee_velocity = 0.0
        self.previous_velocity = 0.0
        self.smoothed_velocity = 0.0
        self.smoothed_acceleration = 0.0
        self.previous_time = 0.0

        # Savitzky-Golay filter parameters
        self.filter_window = 5  # Window size for Savitzky-Golay filter (must be odd)
        self.poly_order = 2     # Polynomial order for Savitzky-Golay filter

        # Buffers for Savitzky-Golay filter
        self.angle_buffer = []
        self.velocity_buffer = []
        self.acceleration_buffer = []


        self.imu_shank = self.create_publisher(
            Vector3,
            'imu_data_shank',
            qos_profile)
        
        self.imu_velocity = self.create_publisher(
            Float64,
            'imu_data_velocity',
            qos_profile)
        
        self.imu_acceleration = self.create_publisher(
            Float64,
            'imu_data_acceleration',
            qos_profile)
        
        # self.imu_data_list = [] # imu number / roll / pitch / yaw
        
        self.esp_serial()
        if self.status:
            self.create_timer(0.0125, self.publish_imu_data)
            # self.create_timer(0.005, self.publish_etc_data)


    #  --------------   Publisher def 정의 -------------

    def publish_imu_data(self):
        self.ser.write(b'get\n')
        line = self.ser.readline().decode(errors='ignore').strip()

        if not line:
            return

        data = self.parse_imu_data(line)
        if not data or not all(k in data for k in ('r', 'p', 'y')):
            self.get_logger().warn(f"Invalid data: {line}")
            return

        imu_msg = Vector3()
        '''
        deg
        '''
        imu_msg.x = (-data['r'] + 90) / 16.00
        self.get_logger().info(f"IMU data: {imu_msg.x}")
        imu_msg.y = data['p'] / 16.00
        imu_msg.z = data['y'] / 16.00

        # Apply smoothing to imu_msg using a low-pass filter
        alpha = 0.5  # Low-pass filter coefficient (adjustable for smoothing level)

        if not hasattr(self, 'smoothed_imu_msg'):
            self.smoothed_imu_msg = Vector3()

        # Smooth each component of imu_msg
        self.smoothed_imu_msg.x = alpha * imu_msg.x + (1 - alpha) * self.smoothed_imu_msg.x
        self.smoothed_imu_msg.y = alpha * imu_msg.y + (1 - alpha) * self.smoothed_imu_msg.y
        self.smoothed_imu_msg.z = alpha * imu_msg.z + (1 - alpha) * self.smoothed_imu_msg.z

        # Publish the smoothed imu_msg
        self.imu_shank.publish(self.smoothed_imu_msg)

        self.current_angle = imu_msg.x
        now = time.time()
        dt = now - self.previous_time

        vel_msg = Float64()
        '''
        deg/s
        '''
        acc_msg = Float64()
        '''
        deg/s^2
        '''

        if dt > 0:
            velocity = (self.current_angle - self.previous_angle) / dt
            acceleration = (velocity - self.previous_velocity) / dt

            vel_msg.data = velocity * 1000000  # 단위 스케일 조정
            self.smoothed_acceleration = 0.8 * self.smoothed_acceleration + 0.2 * acceleration
            acc_msg.data = self.smoothed_acceleration

            self.imu_velocity.publish(vel_msg)
            self.imu_acceleration.publish(acc_msg)

        self.imu_shank.publish(imu_msg)

        self.previous_angle = self.current_angle
        self.previous_velocity = velocity
        self.previous_time = now

            
        

        
        
    # -------------  공통 사용 함수 정의 -----------
    
    def parse_imu_data(self,line: str):
        try:
            # ex) "i:1,r:123,p:456,y:789" -> {'i': 1, 'r': 123, 'p': 456, 'y': 789}
            pattern = r'([a-z]):([-+]?[0-9]*\.?[0-9]+)'
            matches = re.findall(pattern, line)
            return {key: float(val) for key, val in matches}
        except Exception as e:
            return None
    
    def esp_serial(self):
        if self.ser.readable():
            self.status = True
        else:
            self.status = False

def main(args=None):
    rclpy.init(args=args)
    node_read = ESP32Board()

    try:
        rclpy.spin(node_read)
    except KeyboardInterrupt:
        node_read.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node_read.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
