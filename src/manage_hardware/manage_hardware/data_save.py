import sys
import rclpy
import pandas as pd
from rclpy.node import Node
from std_msgs.msg import Float64
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool
from PyQt5.QtWidgets import QApplication, QVBoxLayout, QWidget, QLabel, QLineEdit, QGroupBox, QPushButton, QHBoxLayout
from PyQt5.QtCore import QTimer, QDateTime
from threading import Thread, Lock
from collections import deque
import datetime
import time

class DataSaver(Node):
    def __init__(self):
        super().__init__('pwm_botton')
        # self.qos_profile = QoSProfile(history = QoSHistoryPolicy.KEEP_LAST, depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.qos_profile = QoSProfile(depth=10)
        self.qos_profile_ = QoSProfile(depth=100)
        self.pwm = 50.0
        self.pwm_timestamp = 0.0

        self.force_x = self.force_y = self.force_z = 0.0
        self.torque_x = self.torque_y = self.torque_z = 0.0
        
        # self.prev_pwm = None
        self.prev_force = (None, None, None)
        self.prev_torque = (None, None, None)
        self.sensor_time = 0
        self.force_packet = 0
        self.torque_packet = 0
        self.force_timestamp = 0.0
        self.torque_timestamp = 0.0
        self.saving_status = False
        self.last_pwm_save_time = time.time()
        self.last_force_save_time = time.time()
        self.last_torque_save_time = time.time()

        self.data_sheet_pwm = deque()
        self.data_sheet_force = deque()
        self.data_sheet_torque = deque()

        self.force_ = ()
        self.torque_ = ()

        
        self.pwm_pub = self.create_publisher(Float64, 'pwm_signal', self.qos_profile)
        
        self.pwm_sub = self.create_subscription(
            Float64,
            'pwm_signal',
            self.pwm_subscriber,
            self.qos_profile)
        
        # self.pwm_time_sub = self.create_subscription(
        #     Float64,
        #     'pwm_time',
        #     self.pwm_time_subsciber,
        #     self.qos_profile)
        
        self.force_sub = self.create_subscription(
            Vector3,
            'force_data',
            self.force_subscriber,
            self.qos_profile_)
        
        self.torque_sub = self.create_subscription(
            Vector3,
            'torque_data',
            self.torque_subscriber,
            self.qos_profile_)
        
        # self.sensor_time_sub = self.create_subscription(
        #     Float64,
        #     'sensor_time',
        #     self.sensor_time_subsciber,
        #     self.qos_profile)

        # Lock for thread safety when accessing node data
        self.data_lock = Lock()
        self.timer_1 = self.create_timer(0.005, self.data_saver)
        self.timer_2 = self.create_timer(0.005, self.pwm_publisher)

    def pwm_publisher(self):
        msg = Float64()

        msg.data = self.pwm
        self.pwm_pub.publish(msg)

    def pwm_subscriber(self, msg):
        # with self.data_lock:
        self.pwm = msg.data
        self.pwm_timestamp = time.time()
        if self.saving_status:
            dt_object = datetime.datetime.fromtimestamp(self.pwm_timestamp)
            formatted_time = dt_object.strftime('%H:%M:%S.%f')[:-3]  # .%f는 마이크로세컨드까지 포함하므로, 마지막 3자리를 잘라 밀리세컨드로 사용

            data_entry = [formatted_time, self.pwm]
            self.data_sheet_pwm.append(data_entry)
            self.last_pwm_save_time = time.time()

    def force_subscriber(self, msg):
        # with self.data_lock:
        self.force_x, self.force_y, self.force_z = msg.x, msg.y, msg.z
        self.force_timestamp = time.time()
        if self.saving_status:
            dt_object = datetime.datetime.fromtimestamp(self.force_timestamp)
            formatted_time = dt_object.strftime('%H:%M:%S.%f')[:-3]
            
            data_entry = [formatted_time, self.force_x, self.force_y, self.force_z]
            self.data_sheet_force.append(data_entry)
            self.last_force_save_time = time.time()
                

    def torque_subscriber(self, msg):
        # with self.data_lock:
        self.torque_x, self.torque_y, self.torque_z = msg.x, msg.y, msg.z
        self.torque_timestamp = time.time()
        if self.saving_status:
            dt_object = datetime.datetime.fromtimestamp(self.torque_timestamp)
            formatted_time = dt_object.strftime('%H:%M:%S.%f')[:-3]
            
            # 시간, 분, 초, 밀리세컨드를 포함하는 문자열 형식으로 반환합니다.
            # formatted_time = now.strftime("%H:%M:%S.%f")[:-3]  # .%f는 마이크로세컨드까지 포함하므로, 마지막 3자리를 잘라 밀리세컨드로 사용

            data_entry = [formatted_time, self.torque_x, self.torque_y, self.torque_z]
            self.data_sheet_torque.append(data_entry)
            self.last_force_save_time = time.time()

    # def sensor_time_subsciber(self, msg):
    #     # with self.data_lock:
    #     self.sensor_time = msg.data

    # def pwm_time_subsciber(self, msg):
    #     # with self.data_lock:
    #     self.pwm_time = msg.data

    def data_saver(self):
        # time_delay = self.sensor_time - self.pwm_time
        if self.saving_status:
            current_time = time.time()

class Form(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.data = deque()  # Using deque for more efficient append operations
        self.saving = False
        self.pwm = 0.0

        # QTimer to periodically update the data
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_data)
        # self.timer.start(5)  # Update every 10ms

        # Main layout setup
        main_layout = QVBoxLayout()

        # Group boxes for PWM, Force, and Torque values
        self.label_pwm = QLabel('0.0', self)
        self.label_force_x = QLabel('0.0', self)
        self.label_force_y = QLabel('0.0', self)
        self.label_force_z = QLabel('0.0', self)
        self.label_torque_x = QLabel('0.0', self)
        self.label_torque_y = QLabel('0.0', self)
        self.label_torque_z = QLabel('0.0', self)

        # Group box layout
        horizontal_layout = QHBoxLayout()
        horizontal_layout.addWidget(self.create_group_box('PWM', self.label_pwm))
        horizontal_layout.addWidget(self.create_group_box('Force X', self.label_force_x))
        horizontal_layout.addWidget(self.create_group_box('Force Y', self.label_force_y))
        horizontal_layout.addWidget(self.create_group_box('Force Z', self.label_force_z))
        horizontal_layout.addWidget(self.create_group_box('Torque X', self.label_torque_x))
        horizontal_layout.addWidget(self.create_group_box('Torque Y', self.label_torque_y))
        horizontal_layout.addWidget(self.create_group_box('Torque Z', self.label_torque_z))
        main_layout.addLayout(horizontal_layout)

        # Buttons
        button_layout = QHBoxLayout()

        self.status_label = QLabel("Login", self)
        
        # self.button_test_auto = QPushButton('test_auto', self)
        # self.button_test_auto.clicked.connect(self.test_auto)
        self.button_reset = QPushButton('Data Reset', self)
        self.button_reset.clicked.connect(self.data_reset)
        self.button_load = QPushButton('Data Load', self)
        self.button_load.clicked.connect(self.data_load)
        self.button_stop = QPushButton('Data Stop', self)
        self.button_stop.clicked.connect(self.data_stop)
        self.button_save = QPushButton('Data Save', self)
        self.button_save.clicked.connect(self.data_save)
        self.button_auto_save = QPushButton('Data Auto Saver', self)
        self.button_auto_save.clicked.connect(self.button_auto_saver)
        self.file_name_input = QLineEdit(self)
        self.file_name_input.setPlaceholderText('Enter file name')
        # self.file_name_test = QLineEdit(self)
        # self.file_name_test.setPlaceholderText('Enter test iteration')

        button_layout.addWidget(self.status_label)
        # button_layout.addWidget(self.button_test_auto)
        button_layout.addWidget(self.button_reset)
        button_layout.addWidget(self.button_load)
        button_layout.addWidget(self.button_stop)
        button_layout.addWidget(self.button_save)
        button_layout.addWidget(self.button_auto_save)
        button_layout.addWidget(self.file_name_input)
        # button_layout.addWidget(self.file_name_test)
        main_layout.addLayout(button_layout)

        self.setLayout(main_layout)
        self.setWindowTitle('Data Save')
        self.resize(800, 200)
        
        self.show()
        QApplication.processEvents()
        self.move(0, 0)

    def create_group_box(self, title, label):
        group_box = QGroupBox(title)
        layout = QVBoxLayout()
        layout.addWidget(label)
        group_box.setLayout(layout)
        return group_box

    def update_data(self):
        # with self.node.data_lock:
        self.label_pwm.setText(f'{self.node.pwm:.2f}')
        self.label_force_x.setText(f'{self.node.force_x:.2f}')
        self.label_force_y.setText(f'{self.node.force_y:.2f}')
        self.label_force_z.setText(f'{self.node.force_z:.2f}')
        self.label_torque_x.setText(f'{self.node.torque_x:.2f}')
        self.label_torque_y.setText(f'{self.node.torque_y:.2f}')
        self.label_torque_z.setText(f'{self.node.torque_z:.2f}')

    def data_reset(self):
        # self.saving = True
        self.node.data_sheet = deque()
        self.status_label.setText("data_reset")
    
    def data_load(self):
        self.node.saving_status = True
        self.status_label.setText("data_load")
        self.button_reset.setEnabled(False)
        self.button_save.setEnabled(False)

    def data_stop(self):
        self.node.saving_status  = False
        self.status_label.setText("data_stop")
        self.button_reset.setEnabled(True)
        self.button_save.setEnabled(True)
    
    def data_save(self):
        self.status_label.setText("Data save start")
        # self.file_name = self.file_name_input.text()
        self.file_name = str('{0}_{1}'.format(int(self.pwm), int(self.file_name_input.text())))
        if self.file_name:
            try:
                # PWM, Force, Torque 데이터 프레임 생성
                df_pwm = pd.DataFrame(self.node.data_sheet_pwm, columns=['Time', 'PWM'])
                df_force = pd.DataFrame(self.node.data_sheet_force, columns=['Time', 'Force_X', 'Force_Y', 'Force_Z'])
                df_torque = pd.DataFrame(self.node.data_sheet_torque, columns=['Time', 'Torque_X', 'Torque_Y', 'Torque_Z'])
                

                df_pwm.to_excel(f'{self.file_name}_pwm.xlsx', index=False)
                df_force.to_excel(f'{self.file_name}_force.xlsx', index=False)
                df_torque.to_excel(f'{self.file_name}_torque.xlsx', index=False)
                # 세 데이터 프레임을 시간 열을 기준으로 합치기
                # df_merged = pd.merge(pd.merge(df_pwm, df_force, on='Time', how='outer'), df_torque, on='Time', how='outer')
                
                # 누락된 데이터가 있는 행 제거
                # df_cleaned = df_merged.dropna()

                # Excel 파일로 저장
                # df_merged.to_excel(f'{self.file_name}.xlsx', index=False)
                print(f'Data saved to {self.file_name}.xlsx')
                self.status_label.setText("Data save complete")
            except Exception as e:
                print(f"Error saving data: {e}")
                self.status_label.setText("Data save failed")
        else:
            print("Please enter a file name.")
            self.status_label.setText("Data save failed")

        # 데이터 시트 초기화
        self.node.data_sheet_pwm.clear()
        self.node.data_sheet_force.clear()
        self.node.data_sheet_torque.clear()
    def button_auto_saver(self):
        for i in range(0, 101, 5):
            self.data_reset()
            self.node.get_logger().info('1')
            time.sleep(0.1)
            self.data_load()
            self.node.get_logger().info('2')
            time.sleep(0.1)
            self.node.pwm = float(i)
            self.pwm = float(i)
            self.node.get_logger().info('3')
            time.sleep(7)
            self.data_stop()
            self.node.get_logger().info('4')
            time.sleep(0.1)
            self.data_save()
            self.node.get_logger().info('6')
            time.sleep(3)
            self.node.pwm = float(50)
            self.node.get_logger().info('7')
            time.sleep(3)


def run_node(node):
    rclpy.spin(node)

def main(args=None):
    try:
        rclpy.init(args=args)
        node = DataSaver()
        node_thread = Thread(target=run_node, args=(node,))
        node_thread.start()

        app = QApplication(sys.argv)
        ex = Form(node)
        app.exec_()

        node.destroy_node()
        rclpy.shutdown()
        node_thread.join()

    except KeyboardInterrupt:
        print("Interrupt received! Shutting down.")

    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if node_thread.is_alive():
            node_thread.join()
        print("Cleanup complete.")

if __name__ == '__main__':
    main()
