import numpy as np
# from rclpy.node import Node
# from rclpy.qos import QoSProfile
import math
import time
import threading
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QPushButton, QVBoxLayout, QSpinBox, QLabel, QDoubleSpinBox, QCheckBox
from PyQt5.QtCore import QTimer, Qt
from PyQt5 import uic
from pyqtgraph import PlotWidget, TextItem
from threading import Thread
import sys
import signal
import json


# class Rehab(Node):
class Rehab():
    def __init__(self):
        '''
        재활 운동에 사용되는 기본적인 안전 정보를 가지고 있는 객체
        '''

        with open("assets/motor_info.json", "r") as fr:
            data = json.load(fr)
        # print(data)

        self.Kp_motor = float(data["kp"])
        self.Ki_motor = float(data["ki"])
        self.Kd_motor = float(data["kd"])
        self.Neutral_angle = float(data["neutral_angle"])

        with open("assets/rehab.json", "r") as fr:
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

        self.desired_angle = 0.0
        self.current_angle = 0.0
        self.desired_velocity = 0
        
        self.dt = 0.005
        self.past_time = 0.0

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
        self.muscle_componenet_control_activate_status = False # step 작동 on/off Switch

        self.passive_mode = Passive_mode(self)
        self.active_resist_mode = Active_Resistance_mode(self)
        self.active_assist_mode = Active_Assistance_mode(self)

        self.controller_thread = threading.Thread(target=self.controller, daemon=True)

        self.controller_thread.start()

        

        '''
        IMU 값 가져오기
        '''

        '''
        RMD 모터 제어하고 값 Publish
        '''

        '''
        Thruster 제어하고 값 Publish
        '''
    def controller(self):
        while True:
            
            if self.control_check_status == True:
                if self.muscle_componenet_control_activate_status == True:
                    pass
                if self.passive_control_check_status == True and self.passive_control_activate_status == True:
                    self.desired_angle, self.desired_velocity = self.passive_mode.Passive_exercise()
                    # print('Passive_exercise')
                    # print(self.desired_angle)
                if self.resistance_control_check_status == True and self.resistance_control_activate_status == True:
                    pass
                if self.assistance_control_check_status == True and self.assistance_control_activate_status == True:
                    pass
            else:
                pass
            self.past_time = time.time() # 따로 loop에서 진행해야 할 수 도?
            time.sleep(self.dt)

    def PID_Controller(self, desired_angle, current_angle, integral_error, last_error, p, i, d):
        error = desired_angle - current_angle
        integral_error += error * self.dt
        p_term = p * error
        i_term = i * integral_error
        d_term = d * (error - last_error) / self.dt
        input_force = p_term + i_term + d_term
        
        return input_force, integral_error, error
    
    def calculate_fluidic_resistance_moment(self):
        pass



class Passive_mode:
    def __init__(self, rehab):
        
        self.rehab = rehab
        self.reset()

    def reset(self):
        self.time_stamp = 0

        self.desired_position_end = 0
        self.desired_position_start = 0
        self.dtheta_desired_current = 0

        self.desired_angle = 0
        self.current_position = 90
        '''
        나중에 센서값을 변경해야 함!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        
        '''
        self.current_velocity = 0

        self.repeatation_memory = 0 
        '''
        짝수: Extension
        홀수: Flexion
        '''
        self.protocol_start_time = 0

        self.signal = 0
        '''
        signal: 0(현재 state 유지), 1(Generate desired trajectory), 2(Hold position)
        '''
        
        self.state = 0
        '''
        ## state: 0(최초 움직임), 1(Desired trajectory를 따라 움직임), 2(Holding), 3(운동 종료)
        '''
        
        self.integer_init_signal = 0 # 굳이 필요한가? 추후 확인 필요

    def control_generator(self):
        '''
        Passive mode에서 시간에 따른 signal 생성
        '''

        if self.state == 0:
            self.signal = 1
            self.state = 1
            self.time_stamp = time.time()
            self.desired_angle = self.current_position
            
        elif self.state == 1 and abs(self.desired_position_end - self.current_position) < 2 and abs(self.current_velocity) < 0.05:
            self.state = 2
            self.time_stamp = time.time()
        elif self.state == 2 and time.time() > self.time_stamp + self.rehab.holdtime:
            self.state = 1
            self.signal = 1
            self.time_stamp = time.time()
            self.repeatation_memory += 1
        if self.repeatation_memory >= self.rehab.repeatationnumber:
            self.state = 3
            self.signal = 0
        print(self.state, self.repeatation_memory)
        

    def desired_position_generator(self):
        if self.signal == 1:
            if self.repeatation_memory % 2 == 0:
                self.desired_position_start = self.current_position
                self.desired_position_end = self.rehab.ThetaMin
            else:
                self.desired_position_start = self.current_position
                self.desired_position_end = self.rehab.ThetaMax

            # desired position을 설정하고 나서 signal 다시 0으로 초기화
            self.signal = 0


    def desired_velocity_profile_generator(self):
        '''
        Passive mode에서 시간에 따른 프로파일 생성
        '''
        time_now = time.time() - self.time_stamp
        self.dt = time.time() - self.rehab.past_time
        acc_calculated = 0
        distance = abs(self.desired_position_end - self.desired_position_start)
        # constant_vel_time = distance/self.rehab.dThetaMax - self.rehab.acc_time
        constant_vel_time = distance/self.rehab.dThetaMax - self.rehab.dThetaMax/self.rehab.ddThetaMax
        acc_vel_time = self.rehab.dThetaMax/self.rehab.ddThetaMax
        # Acc = self.rehab.dThetaMax
        if self.state == 1:
                
            if self.desired_position_end < self.desired_position_start:
                if time_now < acc_vel_time:
                    self.dtheta_desired_current = - time_now*self.rehab.ddThetaMax

                elif time_now < constant_vel_time + acc_vel_time:
                    self.dtheta_desired_current =  - self.rehab.dThetaMax

                elif time_now < constant_vel_time + 2*acc_vel_time:
                    self.dtheta_desired_current = - self.rehab.dThetaMax + (time_now - (constant_vel_time + acc_vel_time))*self.rehab.ddThetaMax

                else:
                    self.dtheta_desired_current = 0
            else:
                if time_now < self.rehab.dThetaMax/self.rehab.ddThetaMax:
                    self.dtheta_desired_current = time_now*self.rehab.ddThetaMax

                elif time_now < constant_vel_time +self.rehab.dThetaMax/self.rehab.ddThetaMax:
                    self.dtheta_desired_current = self.rehab.dThetaMax

                elif time_now < constant_vel_time + 2*self.rehab.dThetaMax/self.rehab.ddThetaMax:
                    self.dtheta_desired_current = self.rehab.dThetaMax - (time_now - (constant_vel_time + acc_vel_time))*self.rehab.ddThetaMax

                else:
                    self.dtheta_desired_current = 0

        elif self.state == 2:
            self.dtheta_desired_current = 0
        

    
    def desired_position_trajectory_generator(self):
        '''
        Passive mode에서 desired_velocity에 의한 위치 프로파일 생성
        오일러 적분으로 진행
        '''

        self.desired_angle += self.dtheta_desired_current * self.dt
        self.current_position = self.desired_angle
        

    def Passive_exercise(self):
        '''
        Passive mode 진행 프로토콜을 Main loop로 실행되는 부분
        '''
        self.control_generator()
        self.desired_position_generator()
        self.desired_velocity_profile_generator()
        self.desired_position_trajectory_generator()
        # print(self.desired_position_start, self.desired_position_end)
        return self.desired_angle, self.dtheta_desired_current
    

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

        self.desired_angle_text = TextItem(text="Desired Angle: 0", anchor=(1, 0), color='b')
        self.plot_sensor.addItem(self.desired_angle_text, ignoreBounds=True)
        self.desired_angle_text.setPos(10, 0)  # 상단 오른쪽에 위치

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
        self.muscle_component_checkbox = self.findChild(QCheckBox, 'muscle_component_checkbox')

        
        self.passive_checkbox.setDisabled(True)
        self.resistance_checkbox.setDisabled(True)
        self.assistance_checkbox.setDisabled(True)
        self.muscle_component_checkbox.setDisabled(True)

        self.control_checkbox.stateChanged.connect(self.control_on_off_changed)
        self.passive_checkbox.stateChanged.connect(self.passive_control_checked)
        self.resistance_checkbox.stateChanged.connect(self.resistance_control_checked)
        self.assistance_checkbox.stateChanged.connect(self.assistance_control_checked)
        self.muscle_component_checkbox.stateChanged.connect(self.muscle_component_control_checked)

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

        self.sensor_bias_btn = self.findChild(QPushButton, 'sensor_bias_btn')
        self.sensor_on_btn = self.findChild(QPushButton, 'sensor_on_btn')
        self.sensor_off_btn = self.findChild(QPushButton, 'sensor_off_btn')

        self.motor_setup_btn = self.findChild(QPushButton, 'motor_setup_btn')

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

        self.sensor_bias_btn.clicked.connect(self.sensor_bias_btn_clicked)
        self.sensor_on_btn.clicked.connect(self.sensor_on_btn_clicked)
        self.sensor_off_btn.clicked.connect(self.sensor_off_btn_clicked)

        self.motor_setup_btn.clicked.connect(self.motor_setup_btn_clicked)

        self.thruster_on_btn.clicked.connect(self.thruster_on_btn_clicked)
        self.thruster_off_btn.clicked.connect(self.thruster_off_btn_clicked)

        ### 타이머 설정 ###
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_data)
        self.timer.start(5)  # 100ms 간격으로 업데이트
        
        
        # 버튼 위젯 찾기
        # self.btn_bias = self.findChild(QPushButton, 'Setbias')
        # self.btn_start = self.findChild(QPushButton, 'Setstart')
        # self.btn_stop = self.findChild(QPushButton, 'Setstop')
        # self.btn_quit = self.findChild(QPushButton, 'Quit')

        # 라벨 위젯 찾기
        # self.label_force_x = self.findChild(QLabel, 'ForceX')
        # self.label_torque_x = self.findChild(QLabel, 'TorqueX')
        # self.label_force_y = self.findChild(QLabel, 'ForceY')
        # self.label_torque_y = self.findChild(QLabel, 'TorqueY')
        # self.label_force_z = self.findChild(QLabel, 'ForceZ')
        # self.label_torque_z = self.findChild(QLabel, 'TorqueZ')

        # self.force_labels = [
        #     self.findChild(QLabel, 'force_x_data'),
        #     self.findChild(QLabel, 'force_y_data'),
        #     self.findChild(QLabel, 'force_z_data')
        # ]

        # self.torque_labels = [
        #     self.findChild(QLabel, 'torque_x_data'),
        #     self.findChild(QLabel, 'torque_y_data'),
        #     self.findChild(QLabel, 'torque_z_data')
        # ]

        # 버튼 클릭 이벤트 연결
        # self.btn_bias.clicked.connect(self.set_bias)
        # self.btn_start.clicked.connect(self.turn_on)
        # self.btn_stop.clicked.connect(self.turn_off)
        # self.btn_quit.clicked.connect(self.close)

    def control_on_off_changed(self, state):
        if state == Qt.Checked:
            self.rehab.control_check_status = True
            self.passive_checkbox.setEnabled(True)
            self.resistance_checkbox.setEnabled(True)
            self.assistance_checkbox.setEnabled(True)
            self.muscle_component_checkbox.setEnabled(True)
            # self.btn_on()
        else:
            self.rehab.control_check_status = True
            self.rehab.passive_control_check_status = False # 작동 Main Switch
            self.rehab.resistance_control_check_status = False # 작동 Main Switch
            self.rehab.assistance_control_check_status = False # 작동 Main Switch
            self.rehab.muscle_componenet_check_status = False # 작동 Main Switch
            self.passive_checkbox.setDisabled(True)
            self.resistance_checkbox.setDisabled(True)
            self.assistance_checkbox.setDisabled(True)
            self.muscle_component_checkbox.setDisabled(True)

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

    def muscle_component_control_checked(self, state):
        if state == Qt.Checked:
            self.rehab.muscle_componenet_check_status = True
        else:
            self.rehab.muscle_componenet_check_status = False

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
            
            with open("assets/motor_info.json", "w") as fr:
                json.dump(data, fr)
            print(f"motor setup successful!")
        except Exception as e:
            print(f"motor setup failed! {e}")

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
            
            with open("assets/rehab.json", "w") as fr:
                json.dump(data, fr)
            print(f"Rehabilitation setup successful!")
        except Exception as e:
            print(f"Rehabilitation setup failed! {e}")

    def sensor_bias_btn_clicked(self):
        pass

    def sensor_on_btn_clicked(self):
        pass
    
    def sensor_off_btn_clicked(self):
        pass

    def thruster_on_btn_clicked(self):
        pass
    
    def thruster_off_btn_clicked(self):
        pass

    def passive_setup_btn_clicked(self):
        pass

    def passive_start_btn_clicked(self):
        if self.rehab.passive_control_check_status != False:
            self.rehab.passive_control_activate_status = True

    def passive_stop_btn_clicked(self):
        if self.rehab.passive_control_check_status != False:
            self.rehab.passive_control_activate_status = False
            self.rehab.passive_mode.reset()

    def resistance_setup_btn_clicked(self):
        pass

    def resistance_start_btn_clicked(self):
        if self.rehab.resistance_control_check_status != False:
            self.rehab.resistance_control_activate_status = True
    
    def resistance_stop_btn_clicked(self):
        if self.rehab.resistance_control_check_status != False:
            self.rehab.resistance_control_activate_status = False
            self.rehab.resistance_control_reset()
    
    def assistance_setup_btn_clicked(self):
        pass

    def assistance_start_btn_clicked(self):
        if self.rehab.assistance_control_check_status != False:
            self.rehab.assistance_control_activate_status = True
    
    def assistance_stop_btn_clicked(self):
        if self.rehab.assistance_control_check_status != False:
            self.rehab.assistance_control_activate_status = False
            self.rehab.resistance_control_reset()

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

        # TextItem 위치 조정 (필요한 경우)
        view_range = self.plot_sensor.viewRange()
        self.desired_angle_text.setPos(view_range[0][1], view_range[1][1])

    


# def run_node(node):
#     rclpy.spin(node)

def main(args=None):
    def signal_handler(sig, frame):
        print("Shutting down...")
        QApplication.quit()  # QApplication을 종료합니다.
    signal.signal(signal.SIGINT, signal_handler)

    rehab_info = Rehab()
    app = QApplication(sys.argv)
    
    ex = RehabApp(rehab_info)
    

    # rclpy.init(args=args)
    # sensor = Sensor()
    # thread = Thread(target=run_node, args=(sensor,), daemon=True)

    # thread.start()
    # time.sleep(0.5)

    sys.exit(app.exec_())
    
    # sensor.destroy_node()
    # rclpy.shutdown()
    # thread.join()
    


if __name__ == '__main__':
    main()
