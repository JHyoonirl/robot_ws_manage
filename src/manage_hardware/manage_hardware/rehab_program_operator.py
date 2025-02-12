import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray
from geometry_msgs.msg import Vector3
from custominterface.srv import Status
from rclpy.qos import QoSProfile
import math
from thruster_torque_converter import thruster_converter
from passive_program import Passive_mode
from RMD_custom import RMD# 가정한 모듈과 클래스 이름
# from FT_SENSOR_jh import FTSensor
from Muscle import Muscle

import time
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QPushButton, QVBoxLayout, QSpinBox, QLabel, QDoubleSpinBox, QCheckBox
from PyQt5.QtCore import QTimer, Qt
from PyQt5 import uic
from pyqtgraph import PlotWidget, TextItem
from threading import Thread
import threading
import sys
import signal
import json


# class Rehab(Node):
class Rehab(Node):
    def __init__(self):
        super().__init__('rehab_program_operator')

        self.muscle_passive = Muscle()
        
        self.load_from_json()

        self.qos_profile = QoSProfile(depth=10)

        self.desired_angle = 0.0
        self.current_angle = 0.0
        self.desired_velocity = 0
        
        self.dt = 0.005
        self.past_time = 0.0
        self.thruster_torque = 0
        '''
        Thruster에 입력하게 되는 무릎 기준 토크
        '''
        self.motor_torque = 0
        '''
        RMD motor에 입력하게 되는 모터 토크
        '''

        self.input_torque_passive= 0
        self.integral_error_passive = 0
        self.error_passive = 0
        self.smoothed_torque_passive = 0

        self.input_torque_resistance= 0
        self.integral_error_resistance = 0
        self.error_resistance = 0

        self.input_torque_assistance= 0
        self.integral_error_assistance = 0
        self.error_assistance = 0

        '''
        muscle passive component
        '''

        self.muscle_passive_component = 0

        '''
        운동을 진행하는 프로토콜과 관련된 method와 정보를 가지고 있는 객체
        '''
        self.control_check_status = False
        self.passive_control_check_status = False # 작동 Main Switch
        self.resistance_control_check_status = False # 작동 Main Switch
        self.assistance_control_check_status = False # 작동 Main Switch
        self.muscle_componenet_control_check_status = False # 작동 Main Switch
        self.passive_control_activate_status = False # step 작동 on/off Switch
        self.resistance_control_activate_status = False # step 작동 on/off Switch
        self.assistance_control_activate_status = False # step 작동 on/off Switch
        # self.muscle_componenet_control_activate_status = False # step 작동 on/off Switch

        self.passive_mode = Passive_mode(self)
        self.active_resist_mode = Active_Resistance_mode(self)
        self.active_assist_mode = Active_Assistance_mode(self)

        self.controller_thread = threading.Thread(target=self.controller, daemon=True)

        self.controller_thread.start()

        

        '''
        IMU 값 가져오기
        '''
        self.knee_data_time = time.time()
        self.knee_angle = 0
        self.knee_velocity = 0.0
        self.knee_acceleration = 0.0
        self.read_imu()
        

        '''
        RMD 모터 제어하고 값 Publish
        '''
        self.setting_motor()

        '''
        Thruster 제어하고 값 Publish

        '''
        self.setting_thruster()

        '''
        torque sensor 값 Publish

        '''
        # self.setting_torque_sensor()

    def controller(self):
        while True:
            
            if self.control_check_status == True:
                    
                if self.passive_control_check_status == True and self.passive_control_activate_status == True:
                    self.desired_angle, self.desired_velocity = self.passive_mode.Passive_exercise()
                    self.input_torque_passive, self.integral_error_passive, self.error_passive = self.PID_Controller(self.desired_angle * math.pi / 180, self.current_angle* math.pi / 180, self.integral_error_passive, 
                                                                                                                    self.error_passive, self.Kp_passive, self.Ki_passive, self.Kd_passive, self.past_time)

                    self.input_torque_passive = self.input_torque_passive

                    self.thruster_torque = self.input_torque_passive
                    self.get_logger().info('passive_control : {0}'.format(self.thruster_torque))
                    # self.publish_thruster(self.input_torque_passive)
                elif self.resistance_control_check_status == True and self.resistance_control_activate_status == True:
                    self.get_logger().info('resistance_control : {0}'.format(self.thruster_torque))
                elif self.assistance_control_check_status == True and self.assistance_control_activate_status == True:
                    self.get_logger().info('assistance_control : {0}'.format(self.thruster_torque))
                else:
                    self.thruster_torque = 0
            else:
                self.thruster_torque = 0
                # pass
            self.past_time = time.time() # 따로 loop에서 진행해야 할 수 도?
            time.sleep(self.dt)
    def load_from_json(self):

        '''
        재활 운동에 사용되는 기본적인 안전 정보를 가지고 있는 객체
        '''

        with open("custom_json/motor_info.json", "r") as fr:
            data = json.load(fr)
        # print(data)

        self.Kp_motor = float(data["kp"])
        self.Ki_motor = float(data["ki"])
        self.Kd_motor = float(data["kd"])
        self.Neutral_angle = float(data["neutral_angle"])

        with open("custom_json/rehab.json", "r") as fr:
            data = json.load(fr)

        
        self.ThetaMin = float(data["theta_min"])
        self.ThetaMax = float(data["theta_max"])
        self.dThetaMax = float(data["dtheta_max"])
        self.ddThetaMax = float(data["ddtheta_max"])
        self.acc_time = float(data["acc_time"])
        self.repeatationnumber = float(data["repeatation_number"])
        self.thrusteracc = float(data["thruster_acc"])
        self.holdtime = float(data["hold_time"])
        self.Kp_passive = float(data["kp_passive"])
        self.Ki_passive = float(data["ki_passive"])
        self.Kd_passive = float(data["kd_passive"])
        self.M_resistance = float(data["m_resistance"])
        self.Kp_assistance = float(data["kp_assistance"])
        self.Ki_assistance = float(data["ki_assistance"])
        self.Kd_assistance = float(data["kd_assistance"])
    def controller_reset(self):
        self.input_torque_passive= 0
        self.integral_error_passive = 0
        self.error_passive = 0

        self.input_torque_resistance= 0
        self.integral_error_resistance = 0
        self.error_resistance = 0

        self.input_torque_assistance= 0
        self.integral_error_assistance = 0
        self.error_assistance = 0
    def read_imu(self):
        self.imu_shank = self.create_subscription(
            Vector3,
            'imu_data_shank',
            self.imu_to_knee_angle,
            self.qos_profile
        )
        
    def imu_to_knee_angle(self, msg):
        self.current_angle = float( - msg.x) + 90
        # self.knee_velocity = 0.0
        '''
        수정해야 함.
        '''

    def setting_motor(self):
        self.declare_parameter('usb_port_motor', '/dev/ttyACM0')
        usb_port = self.get_parameter('usb_port_motor').get_parameter_value().string_value
        
        self.RMD = RMD(0x141)
        self.RMD.setup('slcan', usb_port)

        self.motor_contol_thread = threading.Thread(target=self.control_motor_loop, daemon=True)

        self.Motor_info_publisher = self.create_publisher(Float64MultiArray, 'Motor_info', self.qos_profile)
        # self.Motor_velocity_publisher = self.create_publisher(Float64, 'Motor_velocity', self.qos_profile)
        
        ## 모터 반시계 회전(-)은 Extension
        ## 모터 시계 회전(+)은 flexion

        self.MOTOR_ID = 1
        # self.Desired_angle = 0.0
        # self.neutral_torque = 0 # 중립 위치에서 발생하는 토크는?
        # self.position_error = 0

        self.motor_control_status = False
        self.motor_control_activate_status = False
        self.motor_neutral_status = False
        self.motor_muscle_component_status = False

        with open("custom_json/motor_info.json", "r") as fr:
            data = json.load(fr)
        # print(data)

        self.Kp = float(data["kp"])
        self.Kp_tmp = 30
        self.Ki = float(data["ki"])
        self.Kd = float(data["kd"])
        self.Neutral_angle = float(data["neutral_angle"]) # 무릎의 0도에 해당하는 motor encoder의 각도
        self.knee_angle = 0 # 무릎 각도 = encoder angle - self.Neutral_angle

        self.pos_error = 0
        self.pos_error_prev = 0
        self.pos_error_integral = 0
        self.dt_sleep = 0.001
        self.past_time = time.time()

        # self.amplitude = 0.0
        # self.period = 0.0
        # self.pos_offset = 0.0
        # self.RMD_timer_sinusoidal = 0.0

        # self.amplitude_ramp = 0.0 #  initial_condition
        # self.velocity_ramp = 0.0 # velocity
        # self.pos_offset_ramp = 0.0 # last_condition
        # self.RMD_timer_ramp = 0.0
        # self.state_ramp = 1 # 1: decrease 2: increase
        # self.waypoint_ramp = 0

        self.voltage = 0
        self.temperature = 0
        self.torque_current = 0
        self.velocity = 0
        self.angle = 0

        self.torque_threshold = 650

        self.torque_out = 0
        status_1 = self.init_motor()
        status_2 = self.init_acceleration()
        if status_1 == True and status_2 == True:
            self.motor_contol_thread.start()
    
    def init_motor(self):
        # 스케일링된 값들을 바이트 배열로 변환하여 전달
        response = self.RMD.read_pid()
        data = response.data
        self.kp_cur = data[2]
        self.ki_cur = data[3]
        self.kp_vel = data[4]
        self.ki_vel = data[5]
        self.kp_pos = data[6]
        self.ki_pos = data[7]

        print(self.kp_cur, self.ki_cur)
        print(self.kp_vel, self.ki_vel)
        print(self.kp_pos, self.ki_pos)
        data = [
            self.RMD.byteArray(self.kp_cur, 1) ,
            self.RMD.byteArray(self.ki_cur, 1),
            self.RMD.byteArray(self.kp_vel, 1),
            self.RMD.byteArray(self.ki_vel, 1),
            self.RMD.byteArray(self.kp_pos, 1),
            self.RMD.byteArray(self.ki_pos, 1)
        ]
        # 바이트 배열을 하나의 플랫 리스트로 변환
        flat_data = [item for sublist in data for item in sublist]
        self.RMD.write_pid_ram(flat_data)
        print('initialized Motor')
        return True

    def init_acceleration(self):
        for i in range(4):
            index = self.RMD.byteArray(i, 1)
            response = self.RMD.read_acceleration(index)
            data = response.data
            acc = int.from_bytes(data[4:8], byteorder='little', signed=True)
            input_acc = self.RMD.byteArray(20000, 4)
            self.RMD.write_acceleration(index, input_acc)
        print('initialized Motor acc')
        return True
    
    def motor_off(self):
        self.RMD.raw_motor_off()
        self.pos_error = 0
        self.pos_error_prev = 0
        self.pos_error_integral = 0
    
    def motor_on(self):
        self.RMD.raw_motor_run()

    def control_motor_loop(self):
        while True:
            if self.motor_control_status == True:
                if self.motor_control_activate_status == True:
                    try:
                        self.voltage, self.temperature, self.torque_current, self.velocity, self.angle, error = self.RMD.status_motor() #
                        self.knee_angle = self.angle - self.Neutral_angle # encoder angle - neutral angle
                        self.motor_info = Float64MultiArray()
                        self.motor_info.data = [self.voltage, self.torque_current, self.knee_angle, self.velocity]
                        self.Motor_info_publisher.publish(self.motor_info)
                        self.get_logger().info('motor_activate: {0}'.format(self.motor_info))
                    except Exception as e:
                        self.get_logger().info('e: {0}'.format(e))

                    if self.motor_neutral_status == True:
                        '''
                        PID controller를 이용해서 90도 맞추기
                        '''
                        self.get_logger().info('motor_neutral: {0}'.format(self.motor_info))

                    if self.motor_muscle_component_status == True:
                        '''
                        muscle component만을 motor controller에 input으로 적용
                        '''
                        self.passive_muscle_torque = self.muscle_passive.M_passive(self.knee_angle, self.knee_velocity)
                        self.get_logger().info('muscle_component: {0}'.format(self.motor_info))

            else:
                pass
            time.sleep(0.005)

    def setting_thruster(self):

        self.thruster_publisher = self.create_publisher(Float64, 'thruster_signal', self.qos_profile)
        self.thruster = 50.0  # 초기 thruster 값 설정
        self.torque = 0
        self.thruster_response = True  # 서비스 응답 상태 초기화
        self.control_mode = 0 # 0: thruster direct, 1: thruster controlled by torque, 2: neutral
        self.torque_converter = thruster_converter()
        # 실험을 위해 주석 처리

        self.timer = self.create_timer(0.005, self.publish_thruster)
        
    def send_request(self, thruster_switch):
        req = Status.Request()
        req.thruster_switch = thruster_switch
        # try:
        future = self.cli.call_async(req)
        future.add_done_callback(self.handle_service_response)
   
    def handle_service_response(self, future):
        try:
            response = future.result()
            # self.thruster_response = response.thruster_result
            self.thruster_response = True
            self.get_logger().info(f'Service call failed {self.thruster_response}')
        except Exception as e:
            self.get_logger().error(f'Service call failed {e}')

    def publish_thruster(self):
        msg = Float64()
        time_ = Float64()
        if self.thruster_response:
            
            msg.data = self.torque_converter.torque_to_percentage(self.thruster_torque)
            if msg.data > 100:
                msg.data = 100
            elif msg.data  < 0:
                msg.data = 0
        else:
            msg.data = 50.0  # thruster 값을 기본값으로 재설정

            self.get_logger().info('{0}'.format(msg.data))
        msg.data = float(msg.data)
        self.thruster_publisher.publish(msg)

    def PID_Controller(self, desired_angle, current_angle, integral_error, last_error, p, i, d, past_time):
        dt = time.time() - past_time
        error = desired_angle - current_angle
        integral_error += error * dt
        p_term = p * error
        i_term = i * integral_error
        d_term = d * (error - last_error) / dt
        input_force = p_term + i_term + d_term
        
        return input_force, integral_error, error
    
    def calculate_fluidic_resistance_moment(self):
        pass


    
class Active_Resistance_mode:
    def __init__(self, rehab):
        pass
    def reset(self):
        # self.Rehab = Rehab()
        pass
    def Active_resistance_exercise(self):
        pass

class Active_Assistance_mode:
    def __init__(self, rehab):
        pass

    def reset(self):
        pass


    def Active_assistance_exercise(self, theta_max, theta_min, hold_time, Dtheta_max, DDtheta_time, repeatation_number, Thruster_acc):
        pass

class RehabApp(QMainWindow):
    def __init__(self, rehab):
        QMainWindow.__init__(self)
        self.rehab = rehab
        self.threadhold = 200

        self.Desired_Angle_list = []
        self.Current_Angle_list = []
        self.Speed_list = []
        self.updata_period = 20

        self.ui = uic.loadUi('UI/rehab.ui', self)
        self.init_ui()
        
        self.force_data = [[], [], []]  # 각 축의 힘 데이터를 저장하는 리스트
        self.torque_data = [[], [], []]  # 각 축의 토크 데이터를 저장하는 리스트
        self.show()
    def init_ui(self):

        # plot 위젯 찾기
        self.plot_sensor = self.findChild(PlotWidget, 'sensorgraph')
        self.plot_torque = self.findChild(PlotWidget, 'torquegraph')
        self.plot_sensor.setBackground("w")
        self.plot_torque.setBackground("w")

        
        self.plot_current_angle = self.plot_sensor.plot(pen='r', name='current_angle')
        self.plot_desired_angle = self.plot_sensor.plot(pen='b', name='desired_angle')
        self.plot_desired_velocity = self.plot_sensor.plot(pen='g', name='desired_angle')
        self.plot_torque_z = self.plot_torque.plot(pen='g', name='torque_z')

        self.desired_angle_text = TextItem(text="Desired Angle: 0", anchor=(1, 0), color='r')
        self.plot_sensor.addItem(self.desired_angle_text, ignoreBounds=True)
        self.desired_angle_text.setPos(10, 0)  # 상단 오른쪽에 위치

        self.current_angle_text = TextItem(text="current Angle: 0", anchor=(1, 0), color='b')
        self.plot_sensor.addItem(self.current_angle_text, ignoreBounds=True)
        self.current_angle_text.setPos(10, -20)  # 상단 오른쪽에 위치

        self.plot_sensor.setYRange(-50, 150, padding=0)

        # spinbox 위젯 찾기

        self.ThetaMin = self.findChild(QDoubleSpinBox, 'ThetaMin')
        self.ThetaMax = self.findChild(QDoubleSpinBox, 'ThetaMax')
        self.dThetaMax = self.findChild(QDoubleSpinBox, 'dThetaMax')
        self.ddThetaMax = self.findChild(QDoubleSpinBox, 'ddThetaMax')
        self.acc_time = self.findChild(QDoubleSpinBox, 'acc_time')
        self.repeatationnumber = self.findChild(QDoubleSpinBox, 'repeatationnumber')
        self.thrusteracc = self.findChild(QDoubleSpinBox, 'thrusteracc')
        self.holdtime = self.findChild(QDoubleSpinBox, 'holdtime')
        self.Kp_passive = self.findChild(QDoubleSpinBox, 'Kp_passive')
        self.Ki_passive = self.findChild(QDoubleSpinBox, 'Ki_passive')
        self.Kd_passive = self.findChild(QDoubleSpinBox, 'Kd_passive')
        self.M_resistance = self.findChild(QDoubleSpinBox, 'M_resistance')
        self.Kp_assistance = self.findChild(QDoubleSpinBox, 'Kp_assistance')
        self.Ki_assistance = self.findChild(QDoubleSpinBox, 'Ki_assistance')
        self.Kd_assistance = self.findChild(QDoubleSpinBox, 'Kd_assistance')

        self.Kp_motor = self.findChild(QDoubleSpinBox, 'Kp_motor')
        self.Ki_motor = self.findChild(QDoubleSpinBox, 'Ki_motor')
        self.Kd_motor = self.findChild(QDoubleSpinBox, 'Kd_motor')
        self.Neutral_angle = self.findChild(QDoubleSpinBox, 'neutral_angle')



        self.set_default_values()
        # self.thruster_percentage_spinbox.setRange(0, 100)  # thruster 범위 설정
        # self.thruster_percentage_spinbox.setSingleStep(0.05)
        # self.thruster_percentage_spinbox.setValue(int(self.node.thruster))  # 초기값 설정
        # self.thruster_percentage_spinbox.valueChanged.connect(self.thruster_percentage_changed)

        # 체크 박스 정의
        self.control_checkbox = self.findChild(QCheckBox, 'control_checkbox')
        self.passive_checkbox = self.findChild(QCheckBox, 'passive_checkbox')
        self.resistance_checkbox = self.findChild(QCheckBox, 'resistance_checkbox')
        self.assistance_checkbox = self.findChild(QCheckBox, 'assistance_checkbox')
        self.motor_control_checkbox = self.findChild(QCheckBox, 'motor_control_checkbox')
        self.muscle_component_checkbox = self.findChild(QCheckBox, 'muscle_component_checkbox')
        self.motor_neutral_angle_checkbox = self.findChild(QCheckBox, 'motor_neutral_angle')

        
        self.passive_checkbox.setDisabled(True)
        self.resistance_checkbox.setDisabled(True)
        self.assistance_checkbox.setDisabled(True)
        # self.motor_control_checkbox.setDisabled(True)
        self.muscle_component_checkbox.setDisabled(True)
        self.motor_neutral_angle_checkbox.setDisabled(True)

        self.control_checkbox.stateChanged.connect(self.control_on_off_changed)
        self.passive_checkbox.stateChanged.connect(self.passive_control_checked)
        self.resistance_checkbox.stateChanged.connect(self.resistance_control_checked)
        self.assistance_checkbox.stateChanged.connect(self.assistance_control_checked)
        self.motor_control_checkbox.stateChanged.connect(self.motor_control_checkbox_checked)
        self.muscle_component_checkbox.stateChanged.connect(self.muscle_component_control_checked)
        self.motor_neutral_angle_checkbox.stateChanged.connect(self.motor_neutral_angle_checked)

        self.btn_off()

        # buttons 정의

        # self.system_quit_btn = self.findChild(QPushButton, 'system_quit_btn')

        self.setup_btn = self.findChild(QPushButton, 'setup_btn')

        self.passive_setup_btn = self.findChild(QPushButton, 'passive_setup_btn')
        self.passive_start_btn = self.findChild(QPushButton, 'passive_start_btn')
        self.passive_stop_btn = self.findChild(QPushButton, 'passive_stop_btn')

        self.resistance_setup_btn = self.findChild(QPushButton, 'resistance_setup_btn')
        self.resistance_start_btn = self.findChild(QPushButton, 'resistance_start_btn')
        self.resistance_stop_btn = self.findChild(QPushButton, 'resistance_stop_btn')

        self.assistance_setup_btn = self.findChild(QPushButton, 'assistance_setup_btn')
        self.assistance_start_btn = self.findChild(QPushButton, 'assistance_start_btn')
        self.assistance_stop_btn = self.findChild(QPushButton, 'assistance_stop_btn')

        # self.sensor_bias_btn = self.findChild(QPushButton, 'sensor_bias_btn')
        # self.sensor_on_btn = self.findChild(QPushButton, 'sensor_on_btn')
        # self.sensor_off_btn = self.findChild(QPushButton, 'sensor_off_btn')

        self.motor_setup_btn = self.findChild(QPushButton, 'motor_setup_btn')
        self.motor_on_btn = self.findChild(QPushButton, 'motor_on_btn')
        self.motor_off_btn = self.findChild(QPushButton, 'motor_off_btn')

        self.thruster_on_btn = self.findChild(QPushButton, 'thruster_on_btn')
        self.thruster_off_btn = self.findChild(QPushButton, 'thruster_off_btn')

        # self.system_quit_btn.clicked.connect(self.system_quit_btn_clicked)

        self.setup_btn.clicked.connect(self.setup_btn_clicked)
        
        self.passive_setup_btn.clicked.connect(self.passive_setup_btn_clicked)
        self.passive_start_btn.clicked.connect(self.passive_start_btn_clicked)
        self.passive_stop_btn.clicked.connect(self.passive_stop_btn_clicked)

        self.resistance_setup_btn.clicked.connect(self.resistance_setup_btn_clicked)
        self.resistance_start_btn.clicked.connect(self.resistance_start_btn_clicked)
        self.resistance_stop_btn.clicked.connect(self.resistance_stop_btn_clicked)

        self.assistance_setup_btn.clicked.connect(self.assistance_setup_btn_clicked)
        self.assistance_start_btn.clicked.connect(self.assistance_start_btn_clicked)
        self.assistance_stop_btn.clicked.connect(self.assistance_stop_btn_clicked)

        # self.sensor_bias_btn.clicked.connect(self.sensor_bias_btn_clicked)
        # self.sensor_on_btn.clicked.connect(self.sensor_on_btn_clicked)
        # self.sensor_off_btn.clicked.connect(self.sensor_off_btn_clicked)

        self.motor_setup_btn.clicked.connect(self.motor_setup_btn_clicked)
        self.motor_on_btn.clicked.connect(self.motor_on_btn_clicked)
        self.motor_off_btn.clicked.connect(self.motor_off_btn_clicked)

        self.thruster_on_btn.clicked.connect(self.thruster_on_btn_clicked)
        self.thruster_off_btn.clicked.connect(self.thruster_off_btn_clicked)

        ### 타이머 설정 ###
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_data)
        self.timer.start(self.updata_period)  # 100ms 간격으로 업데이트  

    def control_on_off_changed(self, state):
        if state == Qt.Checked:
            self.rehab.control_check_status = True
            self.passive_checkbox.setEnabled(True)
            self.resistance_checkbox.setEnabled(True)
            self.assistance_checkbox.setEnabled(True)
            self.rehab.motor_control_status = True
            self.rehab.motor_on()
            #
            # self.btn_on()
        else:
            self.rehab.control_check_status = True
            self.rehab.passive_control_check_status = False # 작동 Main Switch
            self.rehab.resistance_control_check_status = False # 작동 Main Switch
            self.rehab.assistance_control_check_status = False # 작동 Main Switch
            self.rehab.muscle_componenet_check_status = False # 작동 Main Switch
            self.rehab.motor_control_status = False

            self.passive_checkbox.setDisabled(True)
            self.resistance_checkbox.setDisabled(True)
            self.assistance_checkbox.setDisabled(True)

            self.rehab.motor_off()

    def passive_control_checked(self, state):
        # state == 0 : unchecked, state == 2 : checked
        if state == Qt.Checked:
            self.rehab.passive_control_check_status = True
            self.rehab.resistance_control_check_status = False
            self.rehab.assistance_control_check_status = False
            self.passive_btn_on()

            if self.rehab.resistance_control_check_status == False and self.resistance_checkbox.checkState() == 2:
                self.resistance_checkbox.toggle()

            if self.rehab.assistance_control_check_status == False and self.assistance_checkbox.checkState() == 2:
                self.assistance_checkbox.toggle()
                # print(self._RMD.position_control_check_status, self._RMD.sinusoidal_control_check_status)
                
        else:
            self.btn_off()

    def resistance_control_checked(self, state):
        if state == Qt.Checked:
            self.rehab.resistance_control_check_status = True
            self.rehab.passive_control_check_status = False
            self.rehab.assistance_control_check_status = False
            self.resistance_btn_on()
            if self.rehab.passive_control_check_status == False and self.passive_checkbox.checkState() == 2:
                self.passive_checkbox.toggle()
            
            if self.rehab.assistance_control_check_status == False and self.assistance_checkbox.checkState() == 2:
                self.assistance_checkbox.toggle()
        else:
            self.btn_off()

    def assistance_control_checked(self, state):
        if state == Qt.Checked:
            self.rehab.assistance_control_check_status = True
            self.rehab.passive_control_check_status = False
            self.rehab.resistance_control_check_status = False
            self.assistance_btn_on()
            if self.rehab.passive_control_check_status == False and self.passive_checkbox.checkState() == 2:
                self.passive_checkbox.toggle()
            
            if self.rehab.resistance_control_check_status == False and self.resistance_checkbox.checkState() == 2:
                self.resistance_checkbox.toggle()
        else:
            self.btn_off()

    def motor_control_checkbox_checked(self, state):
        if state == Qt.Checked:
            self.rehab.motor_control_status = True
            self.muscle_component_checkbox.setEnabled(True)
            self.motor_neutral_angle_checkbox.setEnabled(True)
            self.motor_on_btn.setEnabled(True)
            self.motor_off_btn.setEnabled(True)
            # self.resistance_stop_btn.setDisabled(True)
        else:
            self.rehab.motor_control_status = False
            self.muscle_component_checkbox.setDisabled(True)
            self.motor_neutral_angle_checkbox.setDisabled(True)
            self.motor_on_btn.setDisabled(True)
            self.motor_off_btn.setDisabled(True)

    def muscle_component_control_checked(self, state):
        if state == Qt.Checked:
            self.rehab.motor_muscle_component_status = True
            self.rehab.motor_neutral_status = False
            if self.rehab.motor_neutral_status == False and self.motor_neutral_angle_checkbox.checkState() == 2:
                self.motor_neutral_angle_checkbox.toggle()

        else:
            self.rehab.motor_muscle_component_status = False

    def motor_neutral_angle_checked(self, state):
        if state == Qt.Checked:
            self.rehab.motor_neutral_status = True
            self.rehab.motor_muscle_component_status = False
            if self.rehab.motor_muscle_component_status == False and self.muscle_component_checkbox.checkState() == 2:
                self.muscle_component_checkbox.toggle()
        else:
            self.rehab.motor_neutral_status = False

    def passive_btn_on(self):

        self.passive_setup_btn.setEnabled(True)
        self.passive_start_btn.setEnabled(True)
        self.passive_stop_btn.setEnabled(True)

        self.resistance_setup_btn.setDisabled(True)
        self.resistance_start_btn.setDisabled(True)
        self.resistance_stop_btn.setDisabled(True)

        self.assistance_setup_btn.setDisabled(True)
        self.assistance_start_btn.setDisabled(True)
        self.assistance_stop_btn.setDisabled(True)

    def resistance_btn_on(self):
        self.passive_setup_btn.setDisabled(True)
        self.passive_start_btn.setDisabled(True)
        self.passive_stop_btn.setDisabled(True)
        
        self.resistance_setup_btn.setEnabled(True)
        self.resistance_start_btn.setEnabled(True)
        self.resistance_stop_btn.setEnabled(True)
        
        self.assistance_setup_btn.setDisabled(True)
        self.assistance_start_btn.setDisabled(True)
        self.assistance_stop_btn.setDisabled(True)

    def assistance_btn_on(self):
        self.passive_setup_btn.setDisabled(True)
        self.passive_start_btn.setDisabled(True)
        self.passive_stop_btn.setDisabled(True)
        
        self.resistance_setup_btn.setDisabled(True)
        self.resistance_start_btn.setDisabled(True)
        self.resistance_stop_btn.setDisabled(True)
        
        self.assistance_setup_btn.setEnabled(True)
        self.assistance_start_btn.setEnabled(True)
        self.assistance_stop_btn.setEnabled(True)

    def btn_off(self):
        self.passive_setup_btn.setDisabled(True)
        self.passive_start_btn.setDisabled(True)
        self.passive_stop_btn.setDisabled(True)
        
        self.resistance_setup_btn.setDisabled(True)
        self.resistance_start_btn.setDisabled(True)
        self.resistance_stop_btn.setDisabled(True)
        
        self.assistance_setup_btn.setDisabled(True)
        self.assistance_start_btn.setDisabled(True)
        self.assistance_stop_btn.setDisabled(True)

    def set_default_values(self):
        self.ThetaMin.setValue(self.rehab.ThetaMin)
        self.ThetaMax.setValue(self.rehab.ThetaMax)
        self.dThetaMax.setValue(self.rehab.dThetaMax)
        self.ddThetaMax.setValue(self.rehab.ddThetaMax)
        self.acc_time.setValue(self.rehab.acc_time)
        self.repeatationnumber.setValue(self.rehab.repeatationnumber)
        self.thrusteracc.setValue(self.rehab.thrusteracc)
        self.holdtime.setValue(self.rehab.holdtime)
        self.Kp_passive.setValue(self.rehab.Kp_passive)
        self.Ki_passive.setValue(self.rehab.Ki_passive)
        self.Kd_passive.setValue(self.rehab.Kd_passive)
        self.M_resistance.setValue(self.rehab.M_resistance)
        self.Kp_assistance.setValue(self.rehab.Kp_assistance)
        self.Ki_assistance.setValue(self.rehab.Ki_assistance)
        self.Kd_assistance.setValue(self.rehab.Kd_assistance)
        self.Kp_motor.setValue(self.rehab.Kp_motor)
        self.Ki_motor.setValue(self.rehab.Ki_motor)
        self.Kd_motor.setValue(self.rehab.Kd_motor)
        self.Neutral_angle.setValue(self.rehab.Neutral_angle)

    def motor_setup_btn_clicked(self):
        Kp_motor = self.Kp_motor.value()
        Ki_motor = self.Ki_motor.value()
        Kd_motor = self.Kd_motor.value()
        Neutral_angle = self.Neutral_angle.value()
        try:
            self.rehab.Kp_motor = Kp_motor
            self.rehab.Ki_motor = Ki_motor
            self.rehab.Kd_motor = Kd_motor
            self.rehab.Neutral_angle = Neutral_angle

            data = {"kp": self.rehab.Kp_motor, "ki": self.rehab.Ki_motor, "kd": self.rehab.Kd_motor, 
                    "neutral_angle": self.rehab.Neutral_angle}
            
            with open("custom_json/motor_info.json", "w") as fr:
                json.dump(data, fr)
            print(f"motor setup successful!")
        except Exception as e:
            print(f"motor setup failed! {e}")

    def motor_on_btn_clicked(self):
        self.rehab.motor_control_activate_status = True
        self.rehab.motor_on()
        # time.sleep(0.001)

    def motor_off_btn_clicked(self):
        self.rehab.motor_control_activate_status = False
        self.rehab.motor_off()
        # time.sleep(0.001)

    def setup_btn_clicked(self):
        ThetaMin = self.ThetaMin.value()
        ThetaMax = self.ThetaMax.value()
        dThetaMax = self.dThetaMax.value()
        ddThetaMax = self.ddThetaMax.value()
        acc_time = self.acc_time.value()
        repeatationnumber = self.repeatationnumber.value()
        thrusteracc = self.thrusteracc.value()
        holdtime = self.holdtime.value()
        Kp_passive = self.Kp_passive.value()
        Ki_passive = self.Ki_passive.value()
        Kd_passive = self.Kd_passive.value()
        M_resistance = self.M_resistance.value()
        Kp_assistance = self.Kp_assistance.value()
        Ki_assistance = self.Ki_assistance.value()
        Kd_assistance = self.Kd_assistance.value()
        try:
            self.rehab.ThetaMin = ThetaMin
            self.rehab.ThetaMax = ThetaMax
            self.rehab.dThetaMax = dThetaMax
            self.rehab.ddThetaMax = ddThetaMax
            self.rehab.acc_time = acc_time
            self.rehab.repeatationnumber = repeatationnumber
            self.rehab.thrusteracc = thrusteracc
            self.rehab.holdtime = holdtime
            self.rehab.Kp_passive = Kp_passive
            self.rehab.Ki_passive = Ki_passive
            self.rehab.Kd_passive = Kd_passive
            self.rehab.M_resistance = M_resistance
            self.rehab.Kp_assistance = Kp_assistance
            self.rehab.Ki_assistance = Ki_assistance
            self.rehab.Kd_assistance = Kd_assistance
            data = {"theta_min": self.rehab.ThetaMin, "theta_max": self.rehab.ThetaMax, "dtheta_max": self.rehab.dThetaMax, 
                    "ddtheta_max": self.rehab.ddThetaMax, "acc_time": self.rehab.acc_time, "repeatation_number": self.rehab.repeatationnumber,
                    "thruster_acc": self.rehab.thrusteracc, "hold_time": self.rehab.holdtime, "kp_passive": self.rehab.Kp_passive, 
                    "ki_passive": self.rehab.Ki_passive, "kd_passive": self.rehab.Kd_passive, "m_resistance": self.rehab.M_resistance,
                    "kp_assistance": self.rehab.Kp_assistance, "ki_assistance": self.rehab.Ki_assistance, "kd_assistance": self.rehab.Kd_assistance}
            
            with open("custom_json/rehab.json", "w") as fr:
                json.dump(data, fr)
            print(f"Rehabilitation setup successful!")
        except Exception as e:
            print(f"Rehabilitation setup failed! {e}")

    def thruster_on_btn_clicked(self):
        self.rehab.thruster_response = True
    
    def thruster_off_btn_clicked(self):
        self.rehab.thruster_response = False

    def passive_setup_btn_clicked(self):
        pass

    def passive_start_btn_clicked(self):
        if self.rehab.passive_control_check_status != False:
            self.rehab.passive_control_activate_status = True

    def passive_stop_btn_clicked(self):
        if self.rehab.passive_control_check_status != False:
            self.rehab.passive_control_activate_status = False
            self.rehab.passive_mode.reset()
            self.rehab.controller_reset()

    def resistance_setup_btn_clicked(self):
        pass

    def resistance_start_btn_clicked(self):
        if self.rehab.resistance_control_check_status != False:
            self.rehab.resistance_control_activate_status = True
    
    def resistance_stop_btn_clicked(self):
        if self.rehab.resistance_control_check_status != False:
            self.rehab.resistance_control_activate_status = False
            self.rehab.resistance_control_reset()
            self.rehab.controller_reset()
    
    def assistance_setup_btn_clicked(self):
        pass

    def assistance_start_btn_clicked(self):
        if self.rehab.assistance_control_check_status != False:
            self.rehab.assistance_control_activate_status = True
    
    def assistance_stop_btn_clicked(self):
        if self.rehab.assistance_control_check_status != False:
            self.rehab.assistance_control_activate_status = False
            self.rehab.resistance_control_reset()
            self.rehab.controller_reset()

    def update_data(self):
        # Motor_angle = self._RMD.angle
        Current_angle = self.rehab.current_angle
        Desired_angle = self.rehab.desired_angle
        Desired_velocity = self.rehab.desired_velocity

        # velocity = self._RMD.velocity
        # self.Angle_text.setText(f"{Motor_angle:.2f}")
        
        # for i, label in enumerate(self.torque_labels):
        # self.Velocity_text.setText(f"{velocity:.2f}")


        ### 데이터 저장 및 그래프 업데이트 ###
        # for i in range(3):
        if len(self.Desired_Angle_list) >= self.threadhold:  # 최대 threadhold개 데이터 유지
            self.Desired_Angle_list.pop(0)
        if len(self.Speed_list) >= self.threadhold:  # 최대 threadhold개 데이터 유지
            self.Speed_list.pop(0)
        if len(self.Current_Angle_list) >= self.threadhold:
            self.Current_Angle_list.pop(0)
        self.Desired_Angle_list.append(Desired_angle)
        self.Current_Angle_list.append(Current_angle)
        self.Speed_list.append(Desired_velocity)
        self.plot_current_angle.setData(self.Desired_Angle_list)
        self.plot_desired_angle.setData(self.Current_Angle_list)
        self.plot_desired_velocity.setData(self.Speed_list)

            # TextItem 텍스트 업데이트
        self.desired_angle_text.setText(f"Desired Angle: {Desired_angle:.2f}")
        self.current_angle_text.setText(f"Current Angle: {Current_angle:.2f}")

        # TextItem 위치 조정 (필요한 경우)
        view_range = self.plot_sensor.viewRange()
        self.desired_angle_text.setPos(view_range[0][1], view_range[1][1])
        self.current_angle_text.setPos(view_range[0][1], view_range[1][1] -20)

    


def run_node(node):
    rclpy.spin(node)

def main(args=None):
    def signal_handler(sig, frame):
        print("Shutting down...")
        QApplication.quit()  # QApplication을 종료합니다.
    signal.signal(signal.SIGINT, signal_handler)
    rclpy.init(args=args)
    rehab_info = Rehab()

    node_thread = Thread(target=run_node, args=(rehab_info,))
    app = QApplication(sys.argv)
    
    ex = RehabApp(rehab_info)
    node_thread.start()

    

    sys.exit(app.exec_())

    rehab_info.destroy_node()
    rclpy.shutdown()
    node_thread.join()
    
    # sensor.destroy_node()
    # rclpy.shutdown()
    # thread.join()
    


if __name__ == '__main__':
    main()
