from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QLabel, QWidget, QDial, QPushButton, QTextBrowser, QCheckBox, QTextEdit
from PyQt5.QtCore import QTimer, Qt
from PyQt5 import uic
from pyqtgraph import PlotWidget  # 그래프를 위한 라이브러리
import sys
import rclpy
from std_msgs.msg import Float64, Float64MultiArray
from rclpy.node import Node
from rclpy.qos import QoSProfile
# print(sys.path)
# sys.path.append('C:/Users/IRL/Knee rehab')


import warnings

# warnings.filterwarnings("ignore", category=DeprecationWarning)

import argparse
from RMD_custom import RMD# 가정한 모듈과 클래스 이름
import threading
import time
import math
import traceback
import json
from Muscle import Muscle
import signal



class Motor(Node):
    def __init__(self):
        
        ### ROS2 진행을 위한 코드 ###
        super().__init__('RMD_Motor')
        self.declare_parameter('usb_port', '/dev/ttyACM0')
        usb_port = self.get_parameter('usb_port').get_parameter_value().string_value
        self.qos_profile = QoSProfile(depth=10)

        self.muscle = Muscle()

        self.muscle.neutral_torque = 0 #중성부력 측정 실험 이후에 세팅해야 되는 값. 
        '''
        muscle component
        '''

        self.RMD = RMD(0x141)
        self.RMD.setup('slcan', usb_port)

        self.motor_contol_thread = threading.Thread(target=self.controller, daemon=True)

        self.Motor_info_publisher = self.create_publisher(Float64MultiArray, 'Motor_info', self.qos_profile)
        # self.Motor_velocity_publisher = self.create_publisher(Float64, 'Motor_velocity', self.qos_profile)
        
        ## 모터 반시계 회전(-)은 Extension
        ## 모터 시계 회전(+)은 flexion

        self.MOTOR_ID = 1
        self.ANGLE_INIT = 10
        self.VELOCITY_LIMIT = 5000
        self.Desired_angle = 0.0
        self.control_check_status = False
        self.position_control_check_status = False # 작동 Main Switch
        self.sinusoidal_control_check_status = False # 작동 Main Switch
        self.ramp_control_check_status = False # 작동 Main Switch
        self.position_control_activate_status = False # step 작동 on/off Switch
        self.sinusoidal_control_activate_status = False # step 작동 on/off Switch
        self.ramp_control_activate_status = False # step 작동 on/off Switch
        # self.neutral_angle = 0
        self.neutral_torque = 0 # 중립 위치에서 발생하는 토크는?
        self.position_error = 0

        with open("RMD/motor_info.json", "r") as fr:
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

        self.amplitude = 0.0
        self.period = 0.0
        self.pos_offset = 0.0
        self.RMD_timer_sinusoidal = 0.0

        self.amplitude_ramp = 0.0 #  initial_condition
        self.velocity_ramp = 0.0 # velocity
        self.pos_offset_ramp = 0.0 # last_condition
        self.RMD_timer_ramp = 0.0
        self.state_ramp = 1 # 1: decrease 2: increase
        self.waypoint_ramp = 0

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

    def controller(self):
        while True:
            # print(self.control_check_status)
            try:
                self.voltage, self.temperature, self.torque_current, self.velocity, self.angle, error = self.RMD.status_motor() #
                # print(error)
                self.knee_angle = self.angle - self.Neutral_angle # encoder angle - neutral angle
                self.motor_info = Float64MultiArray()
                self.motor_info.data = [self.voltage, self.torque_current, self.knee_angle, self.velocity]
                self.Motor_info_publisher.publish(self.motor_info)
                # self.motor_info.data = float(self.knee_angle)
                # self.Motor_info_publisher.publish(self.motor_info)
                # time.sleep(0.001)
            except Exception as e:
                print(f'Error1: {e}')
            if self.control_check_status == False:
                self.motor_off()
            else:  
                if self.position_control_check_status == True and self.position_control_activate_status == True: # step 제어 가능
                    # if self.position_control_activate_status == True: #제어 활성화
                    self.motor_step()
                elif self.sinusoidal_control_check_status == True and self.sinusoidal_control_activate_status == True: # sine 제어 가능
                    self.motor_sine()
                elif self.ramp_control_check_status == True and self.ramp_control_activate_status == True: # sine 제어 가능
                    self.motor_ramp()
                else:
                    self.motor_off()

            
            # print(time.time() - self.past_time)
            self.past_time = time.time()

            # time.sleep(self.dt_sleep)


    def motor_off(self):
        self.RMD.raw_motor_off()
        self.pos_error = 0
        self.pos_error_prev = 0
        self.pos_error_integral = 0

    def motor_step(self):
        try:
            # _ = self.RMD.position_closed_loop(self.Desired_pos, self.VELOCITY_LIMIT)
            self.dt = time.time() - self.past_time
            self.pos_error = self.Desired_angle - self.knee_angle

            # Proportional term
            proportional = self.Kp * self.pos_error

            # Integral term
            self.pos_error_integral += self.pos_error * self.dt
            integral = self.Ki * self.pos_error_integral

            # Derivative term
            derivative = self.Kd * (self.pos_error - self.pos_error_prev) / self.dt

            # Update previous error
            self.pos_error_prev = self.pos_error

            # Calculate the control output
            output = proportional + integral + derivative

            if abs(output) > self.torque_threshold:
                if output > 0:
                    output = self.torque_threshold
                elif output < 0:
                    output = - self.torque_threshold

            temperature, torque, speed, angle = self.RMD.torque_closed_loop(int(output))
            # self.get_logger().info('rmd torque, speed, angle: {0}'.format(torque, speed, angle))

        except Exception as e:
            print(f'Error2: {e}')

    def motor_sine(self):
        try:
            self.dt = time.time() - self.past_time
            sine_time = time.time() - self.RMD_timer_sinusoidal
            self.Desired_angle = self.amplitude * math.sin(self.period * sine_time) + self.pos_offset

            self.pos_error = self.Desired_angle - self.knee_angle
            print(self.pos_error)

            # Proportional term
            proportional = self.Kp * self.pos_error

            # Integral term
            self.pos_error_integral += self.pos_error * self.dt
            integral = self.Ki * self.pos_error_integral

            # Derivative term
            derivative = self.Kd * (self.pos_error - self.pos_error_prev) / self.dt

            # Update previous error
            self.pos_error_prev = self.pos_error

            # Calculate the control output
            output = proportional + integral + derivative
            if abs(output) > self.torque_threshold:
                if output > 0:
                    output = self.torque_threshold
                elif output < 0:
                    output = - self.torque_threshold
            self.torque_out = output
            temperature, torque, speed, angle = self.RMD.torque_closed_loop(int(output))
            # self.get_logger().info('rmd torque, speed, angle: {0}'.format(torque, speed, angle))
                        

        except Exception as e:
            print(f'Error3: {e}')

    def motor_ramp(self):
        try:
            self.dt = time.time() - self.past_time
            
            # self.Desired_angle = self.amplitude * math.sin(self.period * sine_time) + self.pos_offset
            
            # if self.state_ramp == 1 and abs(self.Desired_angle - (self.pos_offset_ramp - self.amplitude_ramp)) < 1:
            #     self.state_ramp = 2
            #     self.waypoint_ramp = self.Desired_angle
            #     self.RMD_timer_ramp = time.time()
            # elif self.state_ramp == 2 and abs(self.Desired_angle - (self.pos_offset_ramp + self.amplitude_ramp)) < 1:
            #     self.state_ramp = 1
            #     self.waypoint_ramp = self.Desired_angle
            #     self.RMD_timer_ramp = time.time()
            
            # if self.state_ramp == 1:
            #     sign = -1
            # elif self.state_ramp == 2:
            #     sign = 1
            ramp_time = time.time() - self.RMD_timer_ramp
            if ramp_time < 3:
                ramp_time = 0
            else:
                ramp_time = ramp_time - 3
            sign = -1

            self.Desired_angle = self.amplitude_ramp + sign * self.velocity_ramp * (ramp_time)

            if self.Desired_angle < self.pos_offset_ramp:
                self.Desired_angle = self.pos_offset_ramp

            
            self.get_logger().info('rself.state_ramp: {0}'.format(self.state_ramp))

            self.pos_error = self.Desired_angle - self.knee_angle
            # print(self.pos_error)

            if ramp_time < 3:
                proportional = self.Kp_tmp * self.pos_error
            else:
                proportional = self.Kp * self.pos_error
            

            # Integral term
            self.pos_error_integral += self.pos_error * self.dt
            integral = self.Ki * self.pos_error_integral

            # Derivative term
            derivative = self.Kd * (self.pos_error - self.pos_error_prev) / self.dt

            # Update previous error
            self.pos_error_prev = self.pos_error

            # Calculate the control output
            output = proportional + integral + derivative
            self.torque_out = output
            if abs(output) > self.torque_threshold:
                if output > 0:
                    output = self.torque_threshold
                elif output < 0:
                    output = - self.torque_threshold
 
            temperature, torque, speed, angle = self.RMD.torque_closed_loop(int(output))
            # self.get_logger().info('rmd torque, speed, angle: {0}'.format(torque, speed, angle))
                        

        except Exception as e:
            print(f'Error3: {e}')

    # def muscle_passive(self):
    #     theta = self.angle
    #     dtheta = self.velocity


    #     pass


class MotorWindow(QMainWindow):
    def __init__(self, RMD=None):
        QMainWindow.__init__(self)
        # self.rmd = RMD(port='COM3')  # 포트는 환경에 따라 변경
        self._RMD = RMD
        self.Desired_Angle_list = []
        self.Current_Angle_list = []
        self.error_Angle_list = []
        self.Speed_list = []
        self.threadhold = 500
        self.ui = uic.loadUi('UI/motor.ui', self)

        self.initUI()
        # print("initialize rmd motor")
        # self.initTimer()
        self.show()

    def initUI(self):
        # plot 위젯 찾기
        self.Plot_Angle = self.findChild(PlotWidget, 'Plot_Angle')
        self.Plot_Velocity = self.findChild(PlotWidget, 'Plot_Velocity')
        
        self.Plot_Angle_desired_data = self.Plot_Angle.plot(pen='r', name='Angle_desired')
        self.Plot_Angle_current_data = self.Plot_Angle.plot(pen='b', name='Angle_current')
        # self.Plot_Angle_error_data = self.Plot_Angle.plot(pen='g', name='Angle_current')
        # self.Plot_Angle.setTitle("Angle Readings")
        self.Plot_Angle.setBackground("w")
        # self.Plot_Angle.setYRange(-30, 30)
        # self.Plot_Angle.addLegend(offset=(10, 30))

        self.Plot_Velocity_data = self.Plot_Velocity.plot(pen='r', name='Velocity')
        # self.Plot_Velocity.setTitle("Velocity Readings")
        # self.plot_torque.setYRange(-3, 3)
        self.Plot_Velocity.setBackground("w")
        # self.plot_torque.addLegend(offset=(10, 30))

        # textbrowser 위젲
        self.Angle_text = self.findChild(QTextBrowser, 'angle_data')
        self.Velocity_text = self.findChild(QTextBrowser, 'velocity_data')

        ### 타이머 설정 ###
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_data)
        self.timer.start(100)  # 100ms 간격으로 업데이트
        
        # 체크 박스 정의
        self.control_on_off = self.findChild(QCheckBox, 'Control_on_off_check')
        self.position_control_check = self.findChild(QCheckBox, 'position_control_check')
        self.sinusoidal_control_check = self.findChild(QCheckBox, 'sinusoidal_control_check')
        self.ramp_control_check = self.findChild(QCheckBox, 'ramp_control_check')

        self.control_on_off.stateChanged.connect(self.control_on_off_changed)
        self.position_control_check.setDisabled(True)
        self.sinusoidal_control_check.setDisabled(True)
        self.ramp_control_check.setDisabled(True)

        self.position_control_check.stateChanged.connect(self.position_control_checked)
        self.sinusoidal_control_check.stateChanged.connect(self.sinusoidal_control_checked)
        self.ramp_control_check.stateChanged.connect(self.ramp_control_checked)

        #textedit 정의
        self.Desired_pos = self.findChild(QTextEdit, 'desired_pos')
        self.Kp = self.findChild(QTextEdit, 'Pos_kp')
        self.Ki = self.findChild(QTextEdit, 'Pos_ki')
        self.Kd = self.findChild(QTextEdit, 'Pos_kd')
        self.Neutral_angle = self.findChild(QTextEdit, 'Neutral_angle')
        # self.vel_ki = self.findChild(QTextEdit, 'vel_ki')
        # self.cur_kp = self.findChild(QTextEdit, 'cur_kp')
        # self.cur_ki = self.findChild(QTextEdit, 'cur_ki')

        self.Kp.setPlainText(f"{self._RMD.Kp}")
        self.Ki.setPlainText(f"{self._RMD.Ki}")
        self.Kd.setPlainText(f"{self._RMD.Kd}")
        self.Neutral_angle.setPlainText(f"{self._RMD.Neutral_angle}")
        # self.vel_ki.setPlainText(f"{self._RMD.ki_vel}")
        # self.cur_kp.setPlainText(f"{self._RMD.kp_cur}")
        # self.cur_ki.setPlainText(f"{self._RMD.ki_cur}")


        self.amplitude = self.findChild(QTextEdit, 'amplitude')
        self.period = self.findChild(QTextEdit, 'period')
        self.pos_offset = self.findChild(QTextEdit, 'pos_offset')

        self.amplitude_ramp = self.findChild(QTextEdit, 'amplitude_ramp')
        self.velocity_ramp = self.findChild(QTextEdit, 'velocity_ramp')
        self.pos_offset_ramp = self.findChild(QTextEdit, 'pos_offset_ramp')

        # buttons 정의

        self.system_quit_btn = self.findChild(QPushButton, 'system_quit_btn')

        self.parameter_setting_btn = self.findChild(QPushButton, 'parameter_setting_btn')

        self.pos_setting_btn = self.findChild(QPushButton, 'pos_setting_btn')
        self.pos_start_btn = self.findChild(QPushButton, 'pos_start_btn')
        self.pos_stop_btn = self.findChild(QPushButton, 'pos_stop_btn')

        self.sinusoidal_setting_btn = self.findChild(QPushButton, 'sinusoidal_setting_btn')
        self.sinusoidal_start_btn = self.findChild(QPushButton, 'sinusoidal_start_btn')
        self.sinusoidal_stop_btn = self.findChild(QPushButton, 'sinusoidal_stop_btn')

        self.ramp_setting_btn = self.findChild(QPushButton, 'ramp_setting_btn')
        self.ramp_start_btn = self.findChild(QPushButton, 'ramp_start_btn')
        self.ramp_stop_btn = self.findChild(QPushButton, 'ramp_stop_btn')

        self.system_quit_btn.clicked.connect(self.system_quit_btn_clicked)

        self.parameter_setting_btn.clicked.connect(self.parameter_setting_btn_clicked)
        
        self.pos_setting_btn.clicked.connect(self.pos_setting_btn_clicked)
        self.pos_start_btn.clicked.connect(self.pos_start_btn_clicked)
        self.pos_stop_btn.clicked.connect(self.pos_stop_btn_clicked)

        self.sinusoidal_setting_btn.clicked.connect(self.sinusoidal_setting_btn_clicked)
        self.sinusoidal_start_btn.clicked.connect(self.sinusoidal_start_btn_clicked)
        self.sinusoidal_stop_btn.clicked.connect(self.sinusoidal_stop_btn_clicked)

        self.ramp_setting_btn.clicked.connect(self.ramp_setting_btn_clicked)
        self.ramp_start_btn.clicked.connect(self.ramp_start_btn_clicked)
        self.ramp_stop_btn.clicked.connect(self.ramp_stop_btn_clicked)

        self.btn_off()

    def system_quit_btn_clicked(self):
        self._RMD.RMD.raw_motor_off()
        sys.exit()
            # print(self._RMD.position_control_check_status, self._RMD.position_control_activate_status)

    def parameter_setting_btn_clicked(self):
        try:        
                Kp = self.Kp.toPlainText()
                Ki = self.Ki.toPlainText()
                Kd = self.Kd.toPlainText()
                neutral_angle = self.Neutral_angle.toPlainText()
            
                # ki_vel = self.vel_ki.toPlainText()
                # kp_cur = self.cur_kp.toPlainText()
                # ki_cur = self.cur_ki.toPlainText()
                time.sleep(0.001)
                self._RMD.Kp = float(Kp)
                self._RMD.Ki = float(Ki)
                self._RMD.Kd = float(Kd)
                self._RMD.Neutral_angle = float(neutral_angle)

                data = {"kp": self._RMD.Kp, "ki": self._RMD.Ki, "kd": self._RMD.Kd, "neutral_angle": self._RMD.Neutral_angle}
                with open("RMD/motor_info.json", "w") as fr:
                    json.dump(data, fr)
                # print(data)
                # self._RMD.ki_vel = int(ki_vel)
                # self._RMD.kp_cur = int(kp_cur)
                # self._RMD.ki_cur = int(ki_cur)
                # time.sleep(0.005)
                # data = [
                #     self._RMD.RMD.byteArray(self._RMD.kp_cur, 1),
                #     self._RMD.RMD.byteArray(self._RMD.ki_cur, 1),
                #     self._RMD.RMD.byteArray(self._RMD.kp_vel, 1),
                #     self._RMD.RMD.byteArray(self._RMD.ki_vel, 1),
                #     self._RMD.RMD.byteArray(self._RMD.kp_pos, 1),
                #     self._RMD.RMD.byteArray(self._RMD.ki_pos, 1)
                # ]
                # # 바이트 배열을 하나의 플랫 리스트로 변환
                # flat_data = [item for sublist in data for item in sublist]
                # time.sleep(0.005)
                # self._RMD.RMD.write_pid_ram(flat_data)
                # print('setting complete')
        except Exception as e:
            traceback_message = traceback.format_exc()
            print('error' + traceback_message)

    def pos_setting_btn_clicked(self):
        if self._RMD.position_control_check_status != False:
            try:
                pos = self.Desired_pos.toPlainText()
                self._RMD.Desired_angle = float(pos)
                print(f"position: {pos}")
            except Exception as e:
                    print(f"Error: {e}")
            time.sleep(0.005)

    def pos_start_btn_clicked(self):
        if self._RMD.position_control_check_status != False:
            self._RMD.position_control_activate_status = True
            
            # print(self._RMD.position_control_check_status, self._RMD.position_control_activate_status)

    def pos_stop_btn_clicked(self):
        if self._RMD.position_control_check_status != False:
            self._RMD.position_control_activate_status = False

    def sinusoidal_setting_btn_clicked(self):
        if self._RMD.sinusoidal_control_check_status != False:
            

            amplitude = self.amplitude.toPlainText()
            period = self.period.toPlainText()
            pos_offset = self.pos_offset.toPlainText()
            try:
                self._RMD.amplitude = float(amplitude)
                self._RMD.period = float(period)
                self._RMD.pos_offset = float(pos_offset)
            except Exception as e:
                print(f"Error: {e}")
            except Exception as e:
                print(f"Error: {e}")


    def sinusoidal_start_btn_clicked(self):
        if self._RMD.sinusoidal_control_check_status != False:
            self._RMD.sinusoidal_control_activate_status = True
            self._RMD.RMD_timer_sinusoidal = time.time()

    def sinusoidal_stop_btn_clicked(self):
        if self._RMD.sinusoidal_control_check_status != False:
            self._RMD.sinusoidal_control_activate_status = False

    def ramp_setting_btn_clicked(self):
        if self._RMD.ramp_control_check_status != False:
            amplitude_ramp = self.amplitude_ramp.toPlainText()
            velocity_ramp = self.velocity_ramp.toPlainText()
            pos_offset_ramp = self.pos_offset_ramp.toPlainText()
            try:
                self._RMD.amplitude_ramp = float(amplitude_ramp)
                self._RMD.velocity_ramp = float(velocity_ramp)
                self._RMD.pos_offset_ramp = float(pos_offset_ramp)
                self._RMD.waypoint_ramp = float(pos_offset_ramp)
            except Exception as e:
                print(f"Error: {e}")


    def ramp_start_btn_clicked(self):
        if self._RMD.ramp_control_check_status != False:
            self._RMD.ramp_control_activate_status = True
            self._RMD.RMD_timer_ramp = time.time()
            self._RMD.state_ramp = 1

    def ramp_stop_btn_clicked(self):
        if self._RMD.ramp_control_check_status != False:
            self._RMD.ramp_control_activate_status = False
            self._RMD.state_ramp = 1

    
    
    def control_on_off_changed(self, state):
        if state == Qt.Checked:
            self._RMD.control_check_status = True
            self.position_control_check.setEnabled(True)
            self.sinusoidal_control_check.setEnabled(True)
            self.ramp_control_check.setEnabled(True)
            
            # self.btn_on()
        else:
            self._RMD.control_check_status = False
            self._RMD.position_control_check_status = False
            self._RMD.sinusoidal_control_check_status = False
            self._RMD.ramp_control_check_status = False
            self.position_control_check.setDisabled(True)
            self.sinusoidal_control_check.setDisabled(True)
            self.ramp_control_check.setDisabled(True)
            # self.btn_off()

    def btn_on(self):
        self.pos_setting_btn.setEnabled(True)
        self.pos_start_btn.setEnabled(True)
        self.pos_stop_btn.setEnabled(True)
    
        self.sinusoidal_setting_btn.setEnabled(True)
        self.sinusoidal_start_btn.setEnabled(True)
        self.sinusoidal_stop_btn.setEnabled(True)

        self.ramp_setting_btn.setEnabled(True)
        self.ramp_start_btn.setEnabled(True)
        self.ramp_stop_btn.setEnabled(True)

    def pos_btn_on(self):
        self.pos_setting_btn.setEnabled(True)
        self.pos_start_btn.setEnabled(True)
        self.pos_stop_btn.setEnabled(True)
    
        self.sinusoidal_setting_btn.setDisabled(True)
        self.sinusoidal_start_btn.setDisabled(True)
        self.sinusoidal_stop_btn.setDisabled(True)

        self.ramp_setting_btn.setDisabled(True)
        self.ramp_start_btn.setDisabled(True)
        self.ramp_stop_btn.setDisabled(True)

    def sinusoidal_btn_on(self):
        self.pos_setting_btn.setDisabled(True)
        self.pos_start_btn.setDisabled(True)
        self.pos_stop_btn.setDisabled(True)
    
        self.sinusoidal_setting_btn.setEnabled(True)
        self.sinusoidal_start_btn.setEnabled(True)
        self.sinusoidal_stop_btn.setEnabled(True)

        self.ramp_setting_btn.setDisabled(True)
        self.ramp_start_btn.setDisabled(True)
        self.ramp_stop_btn.setDisabled(True)

    def ramp_btn_on(self):
        self.pos_setting_btn.setDisabled(True)
        self.pos_start_btn.setDisabled(True)
        self.pos_stop_btn.setDisabled(True)
    
        self.sinusoidal_setting_btn.setDisabled(True)
        self.sinusoidal_start_btn.setDisabled(True)
        self.sinusoidal_stop_btn.setDisabled(True)

        self.ramp_setting_btn.setEnabled(True)
        self.ramp_start_btn.setEnabled(True)
        self.ramp_stop_btn.setEnabled(True)
    
    def btn_off(self):
        self.pos_setting_btn.setDisabled(True)
        self.pos_start_btn.setDisabled(True)
        self.pos_stop_btn.setDisabled(True)
    
        self.sinusoidal_setting_btn.setDisabled(True)
        self.sinusoidal_start_btn.setDisabled(True)
        self.sinusoidal_stop_btn.setDisabled(True)

        self.ramp_setting_btn.setDisabled(True)
        self.ramp_start_btn.setDisabled(True)
        self.ramp_stop_btn.setDisabled(True)

    def position_control_checked(self, state):
        # state == 0 : unchecked, state == 2 : checked
        if state == Qt.Checked:
            self._RMD.position_control_check_status = True
            self._RMD.sinusoidal_control_check_status = False
            self._RMD.ramp_control_check_status = False
            self.pos_btn_on()

            if self._RMD.sinusoidal_control_check_status == False and self.sinusoidal_control_check.checkState() == 2:
                self.sinusoidal_control_check.toggle()
                # print(self._RMD.position_control_check_status, self._RMD.sinusoidal_control_check_status)

            if self._RMD.ramp_control_check_status == False and self.ramp_control_check.checkState() == 2:
                self.ramp_control_check.toggle()
                
        else:
            self.position_control_check_status = False
    
    def sinusoidal_control_checked(self, state):
        if state == Qt.Checked:
            self._RMD.sinusoidal_control_check_status = True
            self._RMD.position_control_check_status = False
            self._RMD.ramp_control_check_status = False
            self.sinusoidal_btn_on()
            if self._RMD.position_control_check_status == False and self.position_control_check.checkState() == 2:
                self.position_control_check.toggle()
                # print(self._RMD.position_control_check_status, self._RMD.sinusoidal_control_check_status)
            if self._RMD.ramp_control_check_status == False and self.ramp_control_check.checkState() == 2:
                self.ramp_control_check.toggle()
        else:
            self.sinusoidal_control_check_status = False

    def ramp_control_checked(self, state):
        if state == Qt.Checked:
            self._RMD.ramp_control_check_status = True
            self._RMD.position_control_check_status = False
            self._RMD.sinusoidal_control_check_status = False
            self.ramp_btn_on()
            if self._RMD.position_control_check_status == False and self.position_control_check.checkState() == 2:
                self.position_control_check.toggle()
                # print(self._RMD.position_control_check_status, self._RMD.sinusoidal_control_check_status)
            if self._RMD.sinusoidal_control_check_status == False and self.sinusoidal_control_check.checkState() == 2:
                self.sinusoidal_control_check.toggle()
        else:
            self.ramp_control_check_status = False

    def update_data(self):
        Motor_angle = self._RMD.angle
        Current_angle = self._RMD.knee_angle
        Desired_angle = self._RMD.Desired_angle
        torque = self._RMD.torque_out

        velocity = self._RMD.velocity
        self.Angle_text.setText(f"{Motor_angle:.2f}")
        
        # for i, label in enumerate(self.torque_labels):
        self.Velocity_text.setText(f"{velocity:.2f}")


        ### 데이터 저장 및 그래프 업데이트 ###
        # for i in range(3):
        if len(self.Desired_Angle_list) >= self.threadhold:  # 최대 threadhold개 데이터 유지
            self.Desired_Angle_list.pop(0)
        if len(self.Speed_list) >= self.threadhold:  # 최대 threadhold개 데이터 유지
            self.Speed_list.pop(0)
        if len(self.Current_Angle_list) >= self.threadhold:
            self.Current_Angle_list.pop(0)
        if len(self.error_Angle_list) >= self.threadhold:
            self.error_Angle_list.pop(0)
        self.Desired_Angle_list.append(Desired_angle)
        self.Current_Angle_list.append(Current_angle)
        self.error_Angle_list.append(10*(Desired_angle - Current_angle))
        self.Speed_list.append(torque)
        self.Plot_Angle_desired_data.setData(self.Desired_Angle_list)
        self.Plot_Angle_current_data.setData(self.Current_Angle_list)
        # self.Plot_Angle_error_data.setData(self.error_Angle_list)
        self.Plot_Velocity_data.setData(self.Speed_list)

def run_node(node):
    rclpy.spin(node)


def main(args=None):
    # rclpy.init(args=args)
    # motor = Motor()
    # thread = threading.Thread(target=run_node, args=(motor, ), daemon=True)
    # # motor_thread = threading.Thread(target=motor.controller, daemon=True)
    # thread.start()
    # time.sleep(0.5)
    # app = QApplication(sys.argv)
    # main_window = MotorWindow(motor)
    # # main_window.show()
    # sys.exit(app.exec_())




    def signal_handler(sig, frame):
        print("Shutting down...")
        QApplication.quit()  # QApplication을 종료합니다.

    signal.signal(signal.SIGINT, signal_handler)  # SIGINT 신호를 처리하기 위해 핸들러를 등록합니다.
    
    rclpy.init(args=args)
    motor = Motor()
    thread = threading.Thread(target=run_node, args=(motor, ), daemon=True)
    thread.start()
    time.sleep(0.5)
    
    app = QApplication(sys.argv)
    main_window = MotorWindow(motor)
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
    
