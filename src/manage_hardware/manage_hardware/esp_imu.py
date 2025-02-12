import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Vector3
import serial
import time


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
            timeout=0)
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


        self.i2c_write = self.create_publisher(
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
        
        self.imu_data_list = [0, 0, 0, 0] # imu number / roll / pitch / yaw
        
        self.esp_serial()
        if self.status:
            self.create_timer(0.005, self.publish_Imu)
            self.create_timer(0.005, self.publish_etc_data)


    #  --------------   Publisher def 정의 -------------

    def publish_Imu(self):
        imu_data = Vector3()
        EncodeData = ""
        EncodedData_indexes = []
        raw_data = []
        # raw_data_index = []
        data_list = []
        EncodeData = self.ser.readline().decode()[0:-1]
        find_index_list = ["imu", "r", "p", "y"]    

        for i, find_index in enumerate(find_index_list): # index number(start with 0) / string to find out( ex: imu)
            EncodedData_indexes.append(EncodeData.find(find_index))

        for i, EncodedData_index in enumerate(EncodedData_indexes): # EncodData_index = [0, 4, 8, 10]
            if i == 3:
                raw_data = EncodeData[EncodedData_index:-1]
            else:
                raw_data = EncodeData[EncodedData_index:EncodedData_indexes[i+1]]
            raw_data_index = raw_data.find(":")
            data_list.append(raw_data[raw_data_index+1:])
        for i, data in enumerate(data_list):
            # status = False
            try:
                data = float(data)
                self.imu_data_list[i] = data
                # status = True
            except:
                pass

        for i, data in enumerate(self.imu_data_list):
            if i == 1:
                imu_data.x = float(data)
            elif i == 2:
                imu_data.y = float(data)
            elif i == 3:
                imu_data.z = float(data)
                self.current_angle = float( - imu_data.x) + 90
        
        

        velocity = Float64()
        acceleration = Float64()
        self.currnet_time = time.time()

        time_delta = self.currnet_time - self.previous_time
        alpha = 0.1  # Smoothing factor

        if time_delta > 0:
            # 속도 계산
            raw_velocity = (self.current_angle - self.previous_angle) / time_delta
            # Smoothing velocity
            self.smoothed_velocity = alpha * raw_velocity + (1 - alpha) * self.smoothed_velocity
            velocity.data = self.smoothed_velocity

            # 가속도 계산
            raw_acceleration = (self.smoothed_velocity - self.previous_velocity) / time_delta
            # Smoothing acceleration
            self.smoothed_acceleration = alpha * raw_acceleration + (1 - alpha) * self.smoothed_acceleration
            acceleration.data = self.smoothed_acceleration
            
            self.i2c_write.publish(imu_data)
            self.imu_velocity.publish(velocity)
            self.imu_acceleration.publish(acceleration)

        # 현재 각도와 시간, 속도를 이전 값으로 저장
        self.previous_angle = self.current_angle
        self.previous_velocity = self.smoothed_velocity
        self.previous_time = self.currnet_time
        

        
        
    # -------------  공통 사용 함수 정의 -----------
        
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
