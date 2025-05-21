import sys
import rclpy
import pandas as pd
from rclpy.node import Node
from std_msgs.msg import Float64
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool, Float64MultiArray
from PyQt5.QtWidgets import QApplication, QVBoxLayout, QWidget, QLabel, QLineEdit, QGroupBox, QPushButton, QHBoxLayout, QMainWindow, QCheckBox, QTextBrowser, QTextEdit
from PyQt5.QtCore import QTimer, QDateTime, Qt, QThread
from threading import Thread, Lock
from collections import deque
import datetime
import time
from PyQt5 import uic
import signal
import openpyxl
import traceback  # 파일 상단에 추가

class DataSaver(Node):
    def __init__(self):
        super().__init__('data_saver')
        # self.qos_profile = QoSProfile(history = QoSHistoryPolicy.KEEP_LAST, depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.qos_profile = QoSProfile(depth=10)
        self.qos_profile_ = QoSProfile(depth=100)

        
        self.thruster_moment_timestamp = 0.0

        self.force_x = self.force_y = self.force_z = 0.0
        self.torque_x = self.torque_y = self.torque_z = 0.0

        self.thruster_moment, self.added_mass, self.drag = 0.0, 0.0, 0.0

        self.desired_position, self.knee_angle, self.imu_velocity, self.imu_acceleration = 0.0, 0.0, 0.0, 0.0
        
        self.trajectory_state = self.desired_velocity = 0
        # self.prev_pwm = None
        # self.prev_force = (None, None, None)
        # self.prev_torque = (None, None, None)
        self.sensor_time = 0
        self.force_packet = 0
        self.torque_packet = 0
        self.force_timestamp = 0.0
        self.torque_timestamp = 0.0
        self.saving_status = False


        self.thruster_status = True
        self.sensor_status = True
        self.motor_status = True

        self.data_sheet_thruster = deque()
        self.data_sheet_force = deque()
        self.data_sheet_torque = deque()
        self.data_sheet_motor = deque()
        self.data_sheet_trajectory = deque()

        self.force_ = ()
        self.torque_ = ()
        self.motor_ = ()

        self.past_time_motor = time.time()

        #### 이제 분석 완료하여 주석 처리, 삭제하지는 말 것 #####
        # self.Thruster_publisher = self.create_publisher(Float64, 'thruster_signal', self.qos_profile) # 실험을 위해 주석 처리
        
        self.thruster_sub = self.create_subscription(
            Float64,
            'thruster_signal',
            self.thruster_subscriber,
            self.qos_profile)
        
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
        
        self.motor_sub = self.create_subscription(
            Float64MultiArray,
            'Motor_info',
            self.motor_subscriber,
            self.qos_profile_)
        
        self.thruster_sub = self.create_subscription(
            Float64,
            'thruster_moment',
            self.thruster_subscriber,
            self.qos_profile)
        
        self.added_mass_sub = self.create_subscription(
            Float64,
            'added_mass',
            self.added_mass_subscriber,
            self.qos_profile)
        
        self.drag_sub = self.create_subscription(
            Float64,
            'drag',
            self.drag_subscriber,
            self.qos_profile)
        
        self.exercise_desired_trajectory_position_subscriber = self.create_subscription(
            Float64,
            'desired_trajectory_position',
            self.exercise_desired_trajectory_position_callback,
            self.qos_profile)
        
        self.exercise_desired_trajectory_velocity_subscriber = self.create_subscription(
            Float64,
            'desired_trajectory_velocity',
            self.exercise_desired_trajectory_velocity_callback,
            self.qos_profile)
        
        self.exercise_trajectory_state_subscriber = self.create_subscription(
            Float64,
            'trajectory_state',
            self.exercise_desired_trajectory_state_callback,
            self.qos_profile)
        
        self.imu_knee_angle_sub = self.create_subscription(
            Vector3,
            'imu_data_shank',
            self.imu_knee_angle_subscriber,
            self.qos_profile)
        
        self.imu_velocity_subscriber = self.create_subscription(
            Float64,
            'imu_data_velocity',
            self.imu_velocity_callback,
            self.qos_profile)
        
        self.imu_acceleration_subscriber = self.create_subscription(
            Float64,
            'imu_data_acceleration',
            self.imu_acceleration_callback,
            self.qos_profile)



        # Updated motor_info structure
        self.voltage = 0.0
        self.torque_current = 0.0
        self.motor_velocity = 0.0
        self.motor_knee_angle = 0.0


        self.data_lock = Lock()


    def thruster_subscriber(self, msg: Float64):
        # with self.data_lock:
        self.thruster_moment = msg.data
        self.thruster_moment_timestamp = time.time()
        if self.saving_status and self.thruster_status == True:
            dt_object = datetime.datetime.fromtimestamp(self.thruster_moment_timestamp)
            formatted_time = dt_object.strftime('%H:%M:%S.%f')[:-3]  # .%f는 마이크로세컨드까지 포함하므로, 마지막 3자리를 잘라 밀리세컨드로 사용

            data_entry = [formatted_time, self.thruster_moment, self.added_mass, self.drag]
            self.data_sheet_thruster.append(data_entry)

    def added_mass_subscriber(self, msg: Float64):
        self.added_mass = msg.data

    def drag_subscriber(self, msg: Float64):
        self.drag = msg.data
        
    def imu_knee_angle_subscriber(self, msg: Vector3):
        # with self.data_lock:
        self.knee_angle = msg.x
        self.knee_angle_timestamp = time.time()
        if self.saving_status and self.sensor_status == True:
            dt_object = datetime.datetime.fromtimestamp(self.knee_angle_timestamp)
            formatted_time = dt_object.strftime('%H:%M:%S.%f')[:-3]
            
            data_entry = [formatted_time, self.desired_position, self.desired_velocity, self.trajectory_state, self.knee_angle, self.imu_velocity, self.imu_acceleration]
            self.data_sheet_trajectory.append(data_entry)


    def imu_velocity_callback(self, msg: Float64):
        self.imu_velocity = msg.data

    def imu_acceleration_callback(self, msg: Float64):
        self.imu_acceleration = msg.data
        
    def exercise_desired_trajectory_position_callback(self, msg: Float64):
        self.desired_position = msg.data

    def exercise_desired_trajectory_velocity_callback(self, msg: Float64):
        self.desired_velocity = msg.data

    def exercise_desired_trajectory_state_callback(self, msg: Float64):
        self.trajectory_state = msg.data

    def force_subscriber(self, msg):
        # with self.data_lock:
        self.force_x, self.force_y, self.force_z = msg.x, msg.y, msg.z
        self.force_timestamp = time.time()
        if self.saving_status and self.sensor_status == True:
            dt_object = datetime.datetime.fromtimestamp(self.force_timestamp)
            formatted_time = dt_object.strftime('%H:%M:%S.%f')[:-3]
            
            data_entry = [formatted_time, self.force_x, self.force_y, self.force_z]
            self.data_sheet_force.append(data_entry)

                

    def torque_subscriber(self, msg):
        # with self.data_lock:
        self.torque_x, self.torque_y, self.torque_z = msg.x, msg.y, msg.z
        self.torque_timestamp = time.time()
        if self.saving_status and self.sensor_status == True:
            dt_object = datetime.datetime.fromtimestamp(self.torque_timestamp)
            formatted_time = dt_object.strftime('%H:%M:%S.%f')[:-3]

            data_entry = [formatted_time, self.torque_x, self.torque_y, self.torque_z]
            self.data_sheet_torque.append(data_entry)
            # self.get_logger().info("{0}".format(data_entry))
            # self.last_force_save_time = time.time()

    def motor_subscriber(self, msg: Float64MultiArray):
        _, _, self.voltage, self.torque_current, self.motor_velocity, _, self.motor_knee_angle = msg.data
        self.motor_timestamp = time.time()
        if self.saving_status and self.motor_status == True:
            dt_object = datetime.datetime.fromtimestamp(self.motor_timestamp)
            formatted_time = dt_object.strftime('%H:%M:%S.%f')[:-3]
            'Time', 'voltage', 'torque_current', 'angle', 'velocity'
            data_entry = [
                formatted_time, self.voltage, self.torque_current,
                self.motor_knee_angle, self.motor_velocity 
            ]
            self.data_sheet_motor.append(data_entry)
            # self.get_logger().info("{0}".format(data_entry))

class DataSaveApp(QMainWindow):
    def __init__(self, node: DataSaver):
        super().__init__()
        self.node = node
        self.data = deque()  # Using deque for more efficient append operations
        self.saving = False
        self.thruster_signal = 0.0
        self.ui = uic.loadUi('UI/data_save.ui', self)
        self.init_ui()
        self.move(850,1920)
        
        self.show()

    def init_ui(self):
        # QTimer to periodically update the data
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_data)
        self.timer.start(50)  # Update every 5ms --> 수정 필요함!!!!

        # Main layout setup
        # main_layout = QVBoxLayout()

        # check boxes
        self.Thruster_data_checkbox = self.findChild(QCheckBox, 'thruster_check')
        self.Sensor_data_checkbox = self.findChild(QCheckBox, 'sensor_check')
        self.Motor_data_checkbox = self.findChild(QCheckBox, 'motor_check')

        self.Thruster_data_checkbox.setChecked(True)
        self.Sensor_data_checkbox.setChecked(True)
        self.Motor_data_checkbox.setChecked(True)

        self.Thruster_data_checkbox.stateChanged.connect(self.thruster_checked)
        self.Sensor_data_checkbox.stateChanged.connect(self.sensor_checked)
        self.Motor_data_checkbox.stateChanged.connect(self.motor_checked)

        # buttons
        self.data_reset_btn = self.findChild(QPushButton, 'data_reset')
        self.data_load_btn = self.findChild(QPushButton, 'data_load')
        self.data_stop_btn = self.findChild(QPushButton, 'data_stop')
        self.data_save_btn = self.findChild(QPushButton, 'data_save')

        self.data_reset_btn.clicked.connect(self.data_reset_fcn)
        self.data_load_btn.clicked.connect(self.data_load_fcn)
        self.data_stop_btn.clicked.connect(self.data_stop_fcn)
        self.data_save_btn.clicked.connect(self.data_save_fcn)

        # text browsers
        self.Thruster_data_text = self.findChild(QTextBrowser, 'thruster_data')
        self.Sensor_data_text = self.findChild(QTextBrowser, 'sensor_data')
        self.Motor_data_text = self.findChild(QTextBrowser, 'motor_data')
        self.Save_status_text = self.findChild(QTextBrowser, 'save_status')

        # text edit
        self.file_name_text = self.findChild(QTextEdit, 'file_name')


    def update_data(self):
        # with self.node.data_lock:
        if self.node.thruster_status == True:
            self.Thruster_data_text.setPlainText(f'{self.node.thruster_moment:.2f}')
        else:
            self.Thruster_data_text.clear()

        if self.node.sensor_status == True:
            self.Sensor_data_text.setPlainText(f'force: {self.node.force_x:.2f}, {self.node.force_y:.2f}, {self.node.force_z:.2f}')
            self.Sensor_data_text.append(f'torque: {self.node.torque_x:.2f}, {self.node.torque_y:.2f}, {self.node.torque_z:.2f}')
        else:
            self.Sensor_data_text.clear()

        if self.node.motor_status == True:
            # self.votage = self.angle = self.velocity
            self.Motor_data_text.setPlainText(f'motor: {self.node.voltage:.2f}, {self.node.torque_current:.2f},{self.node.motor_knee_angle:.2f}, {self.node.motor_velocity:.2f}')
        else:
            self.Motor_data_text.clear()
    
    def thruster_checked(self, state):
        if state == Qt.Checked:
            self.thruster_status = True
            self.node.thruster_status = True
        else:
            self.thruster_status = False
            self.node.thruster_status = False

    def sensor_checked(self, state):
        if state == Qt.Checked:
            self.sensor_status = True
            self.node.sensor_status = True
        else:
            self.sensor_status = False
            self.node.sensor_status = False

    def motor_checked(self, state):
        if state == Qt.Checked:
            self.motor_status = True
            self.node.motor_status = True
        else:
            self.motor_status = False
            self.node.motor_status = False

    def data_reset_fcn(self):
        # self.saving = True
        self.node.data_sheet_thruster.clear()
        self.node.data_sheet_trajectory.clear()
        self.node.data_sheet_force.clear()
        self.node.data_sheet_torque.clear()
        self.node.data_sheet_motor.clear()
        self.Save_status_text.setText("data_reset")
    
    def data_load_fcn(self):
        self.node.saving_status = True
        self.Save_status_text.setText("data_load")
        self.data_reset_btn.setEnabled(False)
        self.data_save_btn.setEnabled(False)

    def data_stop_fcn(self):
        self.node.saving_status  = False
        self.Save_status_text.setText("data_stop")
        self.data_reset_btn.setEnabled(True)
        self.data_save_btn.setEnabled(True)
    
    def data_save_fcn(self):
        self.Save_status_text.setText("Data save start")
        self.file_name = self.file_name_text.toPlainText()
        # self.file_name = str('{0}_{1}'.format(int(self.thruster_signal), int(self.file_name_input.text())))
        if self.file_name:
            try:
                # 파일 이름에 확장자 추가
                full_filename = f"{self.file_name}.xlsx"
                with pd.ExcelWriter(full_filename, engine='openpyxl') as writer:
                    # 각 데이터 저장 여부에 따라 DataFrame 생성 및 시트로 저장
                    if self.node.thruster_status and self.node.data_sheet_thruster:
                        df_thruster = pd.DataFrame(self.node.data_sheet_thruster, columns=['Time', 'thruster_moment', 'added_mass', 'drag'])
                        df_thruster.to_excel(writer, sheet_name='Thruster', index=False)
                    if self.node.data_sheet_trajectory:
                        df_trajectory = pd.DataFrame(self.node.data_sheet_trajectory, columns=['Time', 'desired_position', 'desired_velocity', 'state','angle', 'velocity', 'acceleration'])
                        df_trajectory.to_excel(writer, sheet_name='Trajectory', index=False)
                    if self.node.sensor_status and self.node.data_sheet_force:
                        df_force = pd.DataFrame(self.node.data_sheet_force, columns=['Time', 'Force_X', 'Force_Y', 'Force_Z'])
                        df_force.to_excel(writer, sheet_name='Force', index=False)
                    if self.node.sensor_status and self.node.data_sheet_torque:
                        df_torque = pd.DataFrame(self.node.data_sheet_torque, columns=['Time', 'Torque_X', 'Torque_Y', 'Torque_Z'])
                        df_torque.to_excel(writer, sheet_name='Torque', index=False)
                    if self.node.motor_status and self.node.data_sheet_motor:
                        df_motor = pd.DataFrame(self.node.data_sheet_motor, columns=['Time', 'voltage', 'torque_current', 'angle', 'velocity'])
                        df_motor.to_excel(writer, sheet_name='Motor', index=False)

                print(f"Data saved to {full_filename}")
                self.Save_status_text.setText("Data save complete")
            except Exception as e:
                traceback.print_exc()  # 터미널에 출력
                self.node.get_logger().error("Data save error:\n" + traceback.format_exc())
                self.Save_status_text.setText("Data save failed")
        else:
            print("Please enter a file name.")
            self.Save_status_text.setText("Data save failed")

        # 데이터 시트 초기화
        self.node.data_sheet_thruster.clear()
        self.node.data_sheet_trajectory.clear()
        self.node.data_sheet_force.clear()
        self.node.data_sheet_torque.clear()
        self.node.data_sheet_motor.clear()
        
    def button_auto_saver(self):
        for i in range(0, 101, 5):
            self.data_reset()
            self.node.get_logger().info('1')
            time.sleep(0.1)
            self.data_load()
            self.node.get_logger().info('2')
            time.sleep(0.1)
            self.node.thruster_signal = float(i)
            self.thruster_signal = float(i)
            self.node.get_logger().info('3')
            time.sleep(7)
            self.data_stop()
            self.node.get_logger().info('4')
            time.sleep(0.1)
            self.data_save()
            self.node.get_logger().info('6')
            time.sleep(3)
            self.node.thruster_signal = float(50)
            self.node.get_logger().info('7')
            time.sleep(3)


def run_node(node):
    rclpy.spin(node)

def main(args=None):
    def signal_handler(sig, frame):
        print("Shutting down...")
        QApplication.quit()  # QApplication을 종료합니다.

    signal.signal(signal.SIGINT, signal_handler)  # SIGINT 신호를 처리하기 위해 핸들러를 등록합니다.
    
    
    try:
        rclpy.init(args=args)
        node = DataSaver()
        node_thread = Thread(target=run_node, args=(node,))
        node_thread.start()

        app = QApplication(sys.argv)
        ex = DataSaveApp(node)
        sys.exit(app.exec_())

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
        if node_thread.isRunning():
            node_thread.quit()
            node_thread.wait()
        print("Cleanup complete.")

if __name__ == '__main__':
    main()
