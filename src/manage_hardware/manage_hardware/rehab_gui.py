from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QLabel, QWidget, QDial, QPushButton, QTextBrowser, QCheckBox, QTextEdit
from PyQt5.QtCore import QTimer, Qt, QThread
from PyQt5 import uic
from pyqtgraph import PlotWidget  # 그래프를 위한 라이브러리
import sys
import rclpy
from std_msgs.msg import Float64, Float64MultiArray
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.executors import MultiThreadedExecutor

import time
import json
import signal



class Rehab_program(Node):
    def __init__(self):
        
        '''
        코드 구성
        1) 전체 구성 자체는 ROS2 노드로 구성되어 있음.
        2) RMD motor 동작을 위해서 사용하는 torque/speed control 제어 입력은 외부 ROS2 topic을 통해서 subscribe함.
        --> torque/speed control 제어는 동시에 켜질 수 없도록 함.
        3) RMD motor의 상태 정보는 ROS2 topic을 통해서 publish함.
        4) 
        
        '''


        ### ROS2 진행을 위한 코드 ###
        super().__init__('RMD_Motor_GUI')
        self.qos_profile = QoSProfile(depth=10)


        with open("custom_json/rehab.json", "r") as fr:
            rehab_data = json. load(fr)

        exercise_parameter = rehab_data['exercise_parameter']
        passive_parameter = rehab_data['passive_parameter']
        assistance_parameter = rehab_data['assistance_parameter']
        resistance_parameter = rehab_data['resistance_parameter']
        motor_setting_parameter = rehab_data['motor_setting_parameter']
        hydrodynamic_parameter = rehab_data['hydrodynamic_parameter']

        ## ROS2 topic publisher/subscriber 설정

        self.exercise_info_publisher = self.create_publisher(
            Float64MultiArray, 'Exercise_info', self.qos_profile)
        
        self.exercise_parameter_publisher = self.create_publisher(
            Float64MultiArray, 'Exercise_parameter', self.qos_profile)
        
        self.passive_parameter_publisher = self.create_publisher(
            Float64MultiArray, 'Passive_parameter', self.qos_profile)
        
        self.assistance_parameter_publisher = self.create_publisher(
            Float64MultiArray, 'Assistance_parameter', self.qos_profile)
        
        self.resistance_parameter_publisher = self.create_publisher(
            Float64MultiArray, 'Resistance_parameter', self.qos_profile)
        
        self.motor_setting_parameter_publisher = self.create_publisher(
            Float64MultiArray, 'Motor_setting_parameter', self.qos_profile)

        self.motor_hydrodynamic_control_publisher = self.create_publisher(
            Float64MultiArray, 'Motor_hydrodynamic_control_info', self.qos_profile)
        
        self.imu_knee_angle_subscriber = self.create_subscription(
            Float64,
            'imu_knee_angle',
            self.imu_knee_angle_callback,
            self.qos_profile)
        
        self.thruster_info_subscriber = self.create_subscription(
            Float64MultiArray,
            'Thruster_info',
            self.thruster_info_callback,
            self.qos_profile)
        
        self.motor_info_subscriber = self.create_subscription(
            Float64MultiArray,
            'Motor_info',
            self.motor_info_callback,
            self.qos_profile)
        
        ###########################
        ##  exercise information ##
        ###########################

        self.desired_angle = 0.0
        '''
        desired exercise knee angle [deg/s]
        '''
        self.exercise_state = 0
        '''
        0: stop
        1: start
        2: hold
        '''
        self.repeatation_number = exercise_parameter['repeatation_number']
        self.hold_time = exercise_parameter['hold_time']
        self.power_enabled = 0
        '''
        0. off
        1. on
        '''

        self.knee_angle = 0.0
        self.motor_velocity = 0.0
        self.power_enabled = 0
        '''
        0. off
        1. on
        '''
        
        self.control_active = 0
        '''
        0. stop
        1. start
        '''       

        self.control_mode = 0
        '''
        1. extension constant velocity
        2. extension constant acceleration
        3. passive exercise
        4. resistance exercise
        5. assistance exercise
        6. angle move
        '''
        
        self.muscle_passive_component_switch = 0
        '''
        0. stop
        1. start
        '''

        ############################
        ####### imu variable #######
        ############################

        self.imu_knee_angle = 0.0
        '''
        knee angle from IMU [deg]
        '''

        #################################
        ####### thruster variable #######
        #################################
        self.generated_thruster_moment = 0.0
        self.thruster_power_enabled = 0
        '''
        0. off
        1. on
        '''
        self.thruster_control_active = 0

        ##############################
        ####### motor variable #######
        ##############################
        self.motor_power_enabled = 0
        self.motor_control_active = 0
        self.voltage = 0.0
        self.torque_current = 0.0
        self.motor_velocity = 0.0
        self.motor_angle = 0.0
        self.motor_knee_angle = 0.0


        ##### 실험 진행을 위해서 계속 기억해야 하는 것을 dict로 저장 #####

        ### 아래 dict는 publish 할 때 필요한 정보 ###
        self.exercise_info_dict = {'desired_angle': self.desired_angle,
                                      'exercise_state': self.exercise_state,
                                        'repeatation_number': exercise_parameter['repeatation_number'],
                                        'hold_time': exercise_parameter['hold_time'],
                                        'power_enabled': self.power_enabled,
                                        'control_active': self.control_active,
                                        'control_mode': self.control_mode,
                                        'muscle_passive_component_switch': self.muscle_passive_component_switch,
                                        }
        
        self.exercise_para_dict = {'desired_velocity': exercise_parameter['desired_velocity'],
                                   'desired_acceleration': exercise_parameter['desired_acceleration'],
                                   'repeatation_number': exercise_parameter['repeatation_number'],
                                    'hold_time': exercise_parameter['hold_time'],
                                    'rom_upper': exercise_parameter['rom_upper'],
                                    'rom_lower': exercise_parameter['rom_lower'],
                                   }
        
        self.passive_para_dict = {'passive_p': passive_parameter['passive_p'],
                                    'passive_i': passive_parameter['passive_i'],
                                   'passive_d': passive_parameter['passive_d'],
                                   }
        
        self.assistance_para_dict = {'assistance_gain_k': assistance_parameter['assistance_gain_k'],
                                   'assistance_p': assistance_parameter['assistance_p'],
                                   'assistance_i': assistance_parameter['assistance_i'],
                                   'assistance_d': assistance_parameter['assistance_d'],
                                   'assistance_muscle_p': assistance_parameter['assistance_muscle_p'],
                                   'assistance_muscle_i': assistance_parameter['assistance_muscle_i'],
                                   'assistance_muscle_d': assistance_parameter['assistance_muscle_d'],
                                   }
        
        self.resistance_para_dict = {'resistance_moment': resistance_parameter['resistance_moment'],
                                   'resistance_muscle_p': resistance_parameter['resistance_muscle_p'],
                                   'resistance_muscle_i': resistance_parameter['resistance_muscle_i'],
                                    'resistance_muscle_d': resistance_parameter['resistance_muscle_d'],
                                   }

        self.motor_setting_dict = {'rom_safe_upper': motor_setting_parameter['rom_safe_upper'],
                    'rom_safe_lower': motor_setting_parameter['rom_safe_lower'],
                    'perpendicular_angle': motor_setting_parameter['perpendicular_angle']}
        
        self.hydro_dict = {'test_velocity_input': hydrodynamic_parameter['test_velocity_input'],
                    'test_acceleration_input': hydrodynamic_parameter['test_acceleration_input'], 
                    'test_p_input': hydrodynamic_parameter['test_p_input'],
                    'test_i_input': hydrodynamic_parameter['test_i_input'],
                    'test_d_input': hydrodynamic_parameter['test_d_input'],
                    'test_rom_upper_input': hydrodynamic_parameter['test_rom_upper_input'],
                    'test_rom_lower_input': hydrodynamic_parameter['test_rom_lower_input'],
                    'test_hold_time_input': hydrodynamic_parameter['test_hold_time_input'],}
        ####### test hold time 은 hold time과 동일하게 설정 ######

        self.timer = self.create_timer(0.01, self.rehab_control_pub)

    def rehab_control_pub(self):
        self.exercise_info_pub()
        self.exercise_parameter_pub()
        self.passive_parameter_pub()
        self.assistance_parameter_pub()
        self.resistance_parameter_pub()
        self.motor_setting_parameter_pub()
        self.motor_hydrodynamic_pub()

    def exercise_info_pub(self):
        msg = Float64MultiArray()
        msg.data = [self.exercise_info_dict['desired_angle'],
                    self.exercise_info_dict['exercise_state'],
                    self.exercise_info_dict['repeatation_number'],
                    self.exercise_info_dict['hold_time'],
                    self.exercise_info_dict['power_enabled'],
                    self.exercise_info_dict['control_active'],
                    self.exercise_info_dict['control_mode'],
                    self.exercise_info_dict['muscle_passive_component_switch']]

        self.exercise_info_publisher.publish(msg)    
    
    def exercise_parameter_pub(self):
        msg = Float64MultiArray()

        msg.data = [self.exercise_para_dict['desired_velocity'],
                    self.exercise_para_dict['desired_acceleration'],
                    self.exercise_para_dict['repeatation_number'],
                    self.exercise_para_dict['hold_time'],
                    self.exercise_para_dict['rom_upper'],
                    self.exercise_para_dict['rom_lower'],]
        self.exercise_parameter_publisher.publish(msg)
    
    def passive_parameter_pub(self):
        msg = Float64MultiArray()
        msg.data = [self.passive_para_dict['passive_p'],
                    self.passive_para_dict['passive_i'],
                    self.passive_para_dict['passive_d']]
        self.passive_parameter_publisher.publish(msg)
    
    def assistance_parameter_pub(self):
        msg = Float64MultiArray()
        msg.data = [self.assistance_para_dict['assistance_gain_k'],
                    self.assistance_para_dict['assistance_p'],
                    self.assistance_para_dict['assistance_i'],
                    self.assistance_para_dict['assistance_d'],
                    self.assistance_para_dict['assistance_muscle_p'],
                    self.assistance_para_dict['assistance_muscle_i'],
                    self.assistance_para_dict['assistance_muscle_d']]
        self.passive_parameter_publisher.publish(msg)
    
    def resistance_parameter_pub(self):
        msg = Float64MultiArray()
        msg.data = [self.resistance_para_dict['resistance_moment'],
                    self.resistance_para_dict['resistance_muscle_p'],
                    self.resistance_para_dict['resistance_muscle_i'],
                    self.resistance_para_dict['resistance_muscle_d']]
        self.passive_parameter_publisher.publish(msg)

    def motor_setting_parameter_pub(self):
        msg = Float64MultiArray()
        msg.data = [self.motor_setting_dict['rom_safe_upper'],
                    self.motor_setting_dict['rom_safe_lower'],
                    self.motor_setting_dict['perpendicular_angle']]
        self.motor_setting_parameter_publisher.publish(msg)


    def motor_hydrodynamic_pub(self):
        msg = Float64MultiArray()
        msg.data = [self.hydro_dict['test_velocity_input'],
                    self.hydro_dict['test_acceleration_input'],
                    self.hydro_dict['test_p_input'],
                    self.hydro_dict['test_i_input'],
                    self.hydro_dict['test_d_input'],
                    self.hydro_dict['test_rom_upper_input'],
                    self.hydro_dict['test_rom_lower_input'],
                    self.hydro_dict['test_hold_time_input']]
        self.motor_hydrodynamic_control_publisher.publish(msg)


    ###### subscriber callback ######
    def imu_knee_angle_callback(self, msg):
        self.knee_angle = msg.data
        # print(f"IMU Knee Angle: {self.knee_angle:.2f}")
    def thruster_info_callback(self, msg):
        pass
    
    def motor_info_callback(self, msg):
        self.motor_power_enabled, self.motor_control_active, self.voltage, self.torque_current, self.motor_velocity, self.motor_angle, self.motor_knee_angle = msg.data
    


class RehabWindow(QMainWindow):
    def __init__(self, rehab=Rehab_program):
        QMainWindow.__init__(self)
        self.rehab = rehab

        self.ui = uic.loadUi('UI/rehab_gui.ui', self)

        self.move(0,0)
        self.initUI()
        self.show()

    def initUI(self):
        
        # textbrowser 위젲
        self.angle_text = self.findChild(QTextBrowser, 'angle_data')
        self.velocity_text = self.findChild(QTextBrowser, 'velocity_data')
        self.power_enabled_status_text = self.findChild(QTextBrowser, 'power_enabled_status')
        self.control_active_text = self.findChild(QTextBrowser, 'control_active')

        ### 타이머 설정 ###
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_data)
        self.timer.start(100)  # 100ms 간격으로 업데이트
        
        # 체크 박스 정의
        self.control_on_off_check = self.findChild(QCheckBox, 'control_on_off_check')

        self.control_on_off_check.stateChanged.connect(self.control_on_off_changed)

        # 체크박스 이름과 연결 함수 딕셔너리 정의
        self.checkboxes = {
            'hydrodynamic_constant_velocity_check': {
                'widget': self.findChild(QCheckBox, 'hydrodynamic_constant_velocity_check'),
                'handler': self.hydrodynamic_constant_velocity_checked,
                'status': False
            },
            'hydrodynamic_constant_acceleration_check': {
                'widget': self.findChild(QCheckBox, 'hydrodynamic_constant_acceleration_check'),
                'handler': self.hydrodynamic_constant_acceleration_checked,
                'status': False
            },
            'const_angle_move_check': {
                'widget': self.findChild(QCheckBox, 'const_angle_move_check'),
                'handler': self.const_angle_move_checked,
                'status': False
            },
            'muscle_passive_component_check': {
                'widget': self.findChild(QCheckBox, 'muscle_passive_component_check'),
                'handler': self.muscle_passive_component_checked,
                'status': False
            },
            'exercise_passive_check': {
                'widget': self.findChild(QCheckBox, 'exercise_passive_check'),
                'handler': self.exercise_passive_checked,
                'status': False
            },
            'exercise_assistance_check': {
                'widget': self.findChild(QCheckBox, 'exercise_assistance_check'),
                'handler': self.exercise_assistance_checked,
                'status': False
            },
            'exercise_resistance_check': {
                'widget': self.findChild(QCheckBox, 'exercise_resistance_check'),
                'handler': self.exercise_resistance_checked,
                'status': False
            },
        }

        # connect는 여기서만 한 번!
        for info in self.checkboxes.values():
            if info['widget'] is not None:
                info['widget'].stateChanged.connect(info['handler'])

        self.control_off()

        # buttons 정의

        self.system_quit_btn = self.findChild(QPushButton, 'system_quit_btn')
        self.power_on_off_btn = self.findChild(QPushButton, 'power_on_off_btn')
        self.parameter_setting_btn = self.findChild(QPushButton, 'parameter_setting_btn')
        self.parameter_save_btn = self.findChild(QPushButton, 'parameter_save_btn')

        self.system_quit_btn.clicked.connect(self.system_quit_btn_clicked)
        self.power_on_off_btn.clicked.connect(self.power_on_off_btn_clicked)
        self.parameter_setting_btn.clicked.connect(self.parameter_setting_btn_clicked)
        self.parameter_save_btn.clicked.connect(self.parameter_save_btn_clicked)

        # 버튼 그룹별 정의
        self.button_groups = {
            'hydro': {
                'hydrodynamic_test_setting_btn': self.hydrodynamic_test_setting_btn_clicked,
                'hydrodynamic_test_start_btn': self.hydrodynamic_test_start_btn_clicked,
                'hydrodynamic_test_stop_btn': self.hydrodynamic_test_stop_btn_clicked,
            },
            'const_angle': {
                'const_angle_setting_btn': self.const_angle_setting_btn_clicked,
                'const_angle_start_btn': self.const_angle_start_btn_clicked,
                'const_angle_stop_btn': self.const_angle_stop_btn_clicked,
            },
            'muscle': {
                'muscle_start_btn': self.muscle_start_btn_clicked,
                'muscle_stop_btn': self.muscle_stop_btn_clicked,
            },
            'exercise': {
                'exercise_setting_btn': self.exercise_setting_btn_clicked,
                'exercise_start_btn': self.exercise_start_btn_clicked,
                'exercise_stop_btn': self.exercise_stop_btn_clicked,
            }
        }

        # 위젯 객체 저장
        self.button_widgets = {}

        # findChild + connect
        for group_name, btn_dict in self.button_groups.items():
            for btn_name, handler in btn_dict.items():
                btn = self.findChild(QPushButton, btn_name)
                if btn:
                    btn.clicked.connect(handler)
                    self.button_widgets[btn_name] = btn

        self.btn_off()

        # QTextEdit 그룹별 정의
        self.textedit_groups = {
            'motor_setting': {
                'rom_safe_upper': None,
                'rom_safe_lower': None,
                'perpendicular_angle': None,
            },
            'hydro': {
                'test_velocity_input': None,
                'test_acceleration_input': None,
                'test_p_input': None,
                'test_i_input': None,
                'test_d_input': None,
                'test_rom_upper_input': None,
                'test_rom_lower_input': None,
                'test_hold_time_input': None,
            },
            'const_angle': {
                'const_angle_input': None,
            },
            'exercise': {
                'exercise_desired_velocity': None,
                'exercise_desired_acceleration': None,
                'exercise_repeatation_number': None,
                'exercise_hold_time': None,
                'exercise_rom_upper': None,
                'exercise_rom_lower': None,
            },
            'passive': {
                'passive_p_input': None,
                'passive_i_input': None,
                'passive_d_input': None,
            },
            'assistance': {
                'assistance_gain_k_input': None,
                'assistance_p_input': None,
                'assistance_i_input': None,
                'assistance_d_input': None,
                'assistance_muscle_p_input': None,
                'assistance_muscle_i_input': None,
                'assistance_muscle_d_input': None,
            },
            'resistance': {
                'resistance_moment_input': None,
                'resistance_muscle_p_input': None,
                'resistance_muscle_i_input': None,
                'resistance_muscle_d_input': None,
            },
        }

        # 각 QTextEdit 위젯 findChild로 바인딩
        for group in self.textedit_groups.values():
            for name in group:
                widget = self.findChild(QTextEdit, name)
                group[name] = widget

        self.QtextEdit_init()
    
    def QtextEdit_init(self):
        # 각 QTextEdit 위젯에 초기값 설정
        self.textedit_groups['motor_setting']['rom_safe_upper'].setPlainText(str(self.rehab.motor_setting_dict['rom_safe_upper']))
        self.textedit_groups['motor_setting']['rom_safe_lower'].setPlainText(str(self.rehab.motor_setting_dict['rom_safe_lower']))
        self.textedit_groups['motor_setting']['perpendicular_angle'].setPlainText(str(self.rehab.motor_setting_dict['perpendicular_angle']))

        self.textedit_groups['hydro']['test_velocity_input'].setPlainText(str(self.rehab.hydro_dict['test_velocity_input']))
        self.textedit_groups['hydro']['test_acceleration_input'].setPlainText(str(self.rehab.hydro_dict['test_acceleration_input']))
        self.textedit_groups['hydro']['test_p_input'].setPlainText(str(self.rehab.hydro_dict['test_p_input']))
        self.textedit_groups['hydro']['test_i_input'].setPlainText(str(self.rehab.hydro_dict['test_i_input']))
        self.textedit_groups['hydro']['test_d_input'].setPlainText(str(self.rehab.hydro_dict['test_d_input']))
        self.textedit_groups['hydro']['test_rom_upper_input'].setPlainText(str(self.rehab.hydro_dict['test_rom_upper_input']))
        self.textedit_groups['hydro']['test_rom_lower_input'].setPlainText(str(self.rehab.hydro_dict['test_rom_lower_input']))
        self.textedit_groups['hydro']['test_hold_time_input'].setPlainText(str(self.rehab.hydro_dict['test_hold_time_input']))
        
        self.textedit_groups['exercise']['exercise_desired_velocity'].setPlainText(str(self.rehab.exercise_para_dict['desired_velocity']))
        self.textedit_groups['exercise']['exercise_desired_acceleration'].setPlainText(str(self.rehab.exercise_para_dict['desired_acceleration']))
        self.textedit_groups['exercise']['exercise_repeatation_number'].setPlainText(str(self.rehab.exercise_para_dict['repeatation_number']))
        self.textedit_groups['exercise']['exercise_hold_time'].setPlainText(str(self.rehab.exercise_para_dict['hold_time']))
        self.textedit_groups['exercise']['exercise_rom_upper'].setPlainText(str(self.rehab.exercise_para_dict['rom_upper']))
        self.textedit_groups['exercise']['exercise_rom_lower'].setPlainText(str(self.rehab.exercise_para_dict['rom_lower']))

        self.textedit_groups['passive']['passive_p_input'].setPlainText(str(self.rehab.passive_para_dict['passive_p']))
        self.textedit_groups['passive']['passive_i_input'].setPlainText(str(self.rehab.passive_para_dict['passive_i']))
        self.textedit_groups['passive']['passive_d_input'].setPlainText(str(self.rehab.passive_para_dict['passive_d']))

        self.textedit_groups['assistance']['assistance_gain_k_input'].setPlainText(str(self.rehab.assistance_para_dict['assistance_gain_k']))
        self.textedit_groups['assistance']['assistance_p_input'].setPlainText(str(self.rehab.assistance_para_dict['assistance_p']))
        self.textedit_groups['assistance']['assistance_i_input'].setPlainText(str(self.rehab.assistance_para_dict['assistance_i']))
        self.textedit_groups['assistance']['assistance_d_input'].setPlainText(str(self.rehab.assistance_para_dict['assistance_d']))
        self.textedit_groups['assistance']['assistance_muscle_p_input'].setPlainText(str(self.rehab.assistance_para_dict['assistance_muscle_p']))
        self.textedit_groups['assistance']['assistance_muscle_i_input'].setPlainText(str(self.rehab.assistance_para_dict['assistance_muscle_i']))
        self.textedit_groups['assistance']['assistance_muscle_d_input'].setPlainText(str(self.rehab.assistance_para_dict['assistance_muscle_d']))

        self.textedit_groups['resistance']['resistance_moment_input'].setPlainText(str(self.rehab.resistance_para_dict['resistance_moment']))
        self.textedit_groups['resistance']['resistance_muscle_p_input'].setPlainText(str(self.rehab.resistance_para_dict['resistance_muscle_p']))
        self.textedit_groups['resistance']['resistance_muscle_i_input'].setPlainText(str(self.rehab.resistance_para_dict['resistance_muscle_i']))
        self.textedit_groups['resistance']['resistance_muscle_d_input'].setPlainText(str(self.rehab.resistance_para_dict['resistance_muscle_d']))

    def control_on_off_changed(self, state):
        if state == Qt.Checked:
            for info in self.checkboxes.values():
                info['widget'].setEnabled(True)
            self.control_on()
        else:
            self.control_off()

    def control_on(self):
        for info in self.checkboxes.values():
            if info['widget'] is not None:
                info['widget'].setEnabled(True)

    def control_off(self):
        for info in self.checkboxes.values():
            if info['widget'] is not None:
                info['widget'].setDisabled(True)

    def hydrodynamic_constant_velocity_checked(self, state):
        # state == 0 : unchecked, state == 2 : checked
        if state == Qt.Checked:
            
            self.enable_btn_group('hydro')
            self.disable_btn_group('const_angle')
            self.rehab.control_mode = 1

            for name, info in self.checkboxes.items():                
                if name == 'hydrodynamic_constant_velocity_check':
                    info['status'] = True
                else:
                    if info['widget'].checkState() == 2:
                        info['widget'].toggle()
                    info['status'] = False
                
        else:
            for name, info in self.checkboxes.items():
                if name == 'hydrodynamic_constant_velocity_check':
                    info['status'] = False
            self.disable_btn_group('hydro')
    
    def hydrodynamic_constant_acceleration_checked(self, state):
        if state == Qt.Checked:
            
            self.enable_btn_group('hydro')
            self.disable_btn_group('const_angle')
            self.rehab.control_mode = 2

            for name, info in self.checkboxes.items():                
                if name == 'hydrodynamic_constant_acceleration_check':
                    info['status'] = True
                else:
                    if info['widget'].checkState() == 2:
                        info['widget'].toggle()
                    info['status'] = False
                
        else:
            for name, info in self.checkboxes.items():
                if name == 'hydrodynamic_constant_acceleration_check':
                    info['status'] = False
            self.disable_btn_group('hydro')

    def const_angle_move_checked(self, state):
        if state == Qt.Checked:
            
            self.disable_btn_group('hydro')
            self.enable_btn_group('const_angle')
            self.rehab.control_mode = 6

            for name, info in self.checkboxes.items():                
                if name == 'const_angle_move_check':
                    info['status'] = True
                else:
                    if info['widget'].checkState() == 2:
                        info['widget'].toggle()
                    info['status'] = False
                
        else:
            for name, info in self.checkboxes.items():
                if name == 'const_angle_move_check':
                    info['status'] = False
            self.disable_btn_group('const_angle')

    def exercise_passive_checked(self, state):
        if state == Qt.Checked:
            self.enable_btn_group('exercise')
            self.disable_btn_group('hydro')
            self.disable_btn_group('const_angle')

            self.rehab.control_mode = 3
            for name, info in self.checkboxes.items():                
                if name == 'exercise_passive_check':
                    info['status'] = True
                else:
                    if info['widget'].checkState() == 2:
                        info['widget'].toggle()
                    info['status'] = False

        else:
            for name, info in self.checkboxes.items():
                if name == 'exercise_passive_check':
                    info['status'] = False
            self.disable_btn_group('exercise')

    def exercise_assistance_checked(self, state):
        if state == Qt.Checked:
            self.enable_btn_group('exercise')
            self.disable_btn_group('hydro')
            self.disable_btn_group('const_angle')

            self.rehab.control_mode = 4
            for name, info in self.checkboxes.items():                
                if name == 'exercise_assistance_check':
                    info['status'] = True
                else:
                    if info['widget'].checkState() == 2:
                        info['widget'].toggle()
                    info['status'] = False

        else:
            for name, info in self.checkboxes.items():
                if name == 'exercise_assistance_check':
                    info['status'] = False
            self.disable_btn_group('exercise')
    
    def exercise_resistance_checked(self, state):
        if state == Qt.Checked:
            self.enable_btn_group('exercise')
            self.disable_btn_group('hydro')
            self.disable_btn_group('const_angle')

            self.rehab.control_mode = 5
            for name, info in self.checkboxes.items():                
                if name == 'exercise_resistance_check':
                    info['status'] = True
                else:
                    if info['widget'].checkState() == 2:
                        info['widget'].toggle()
                    info['status'] = False

        else:
            for name, info in self.checkboxes.items():
                if name == 'exercise_resistance_check':
                    info['status'] = False
            self.disable_btn_group('exercise')

    def muscle_passive_component_checked(self, state):
        if state == Qt.Checked:
            self.rehab.muscle_passive_component_switch = 1
            self.enable_btn_group('muscle')
            for name, info in self.checkboxes.items():                
                if name == 'muscle_passive_component_check':
                    info['status'] = True

        else:
            self.rehab.muscle_passive_component_switch = 0
            self.disable_btn_group('muscle')
            for name, info in self.checkboxes.items():                
                if name == 'muscle_passive_component_check':
                    info['status'] = False

    def system_quit_btn_clicked(self):
        self.rehab.power_enabled = 0
        time.sleep(0.5)
        sys.exit()
             
    def power_on_off_btn_clicked(self):
        if self.rehab.power_enabled == 1:
            self.rehab.power_enabled = 0
        else:
            self.rehab.power_enabled = 1

    def parameter_setting_btn_clicked(self):
        motor_setting_dict = self.textedit_groups['motor_setting']

        try:
            self.rehab.motor_setting_dict['rom_safe_upper'] = float(motor_setting_dict['rom_safe_upper'].toPlainText())
            self.rehab.motor_setting_dict['rom_safe_lower'] = float(motor_setting_dict['rom_safe_lower'].toPlainText())
            self.rehab.motor_setting_dict['perpendicular_angle'] = float(motor_setting_dict['perpendicular_angle'].toPlainText())

        except Exception as e:
            print(f"[RMD 설정 실패] {e}")

    def parameter_save_btn_clicked(self):
        

        data = {
                'exercise_parameter': {
                    'desired_velocity': self.rehab.exercise_para_dict['desired_velocity'],
                    'desired_acceleration': self.rehab.exercise_para_dict['desired_acceleration'],
                    'repeatation_number': self.rehab.exercise_para_dict['repeatation_number'],
                    'hold_time': self.rehab.exercise_para_dict['hold_time'],
                    'rom_upper': self.rehab.exercise_para_dict['rom_upper'],
                    'rom_lower': self.rehab.exercise_para_dict['rom_lower'],
                },
                'passive_parameter': {
                    'passive_p': self.rehab.passive_para_dict['passive_p'],
                    'passive_i': self.rehab.passive_para_dict['passive_i'],
                    'passive_d': self.rehab.passive_para_dict['passive_d'],
                },
                'assistance_parameter': {
                    'assistance_gain_k': self.rehab.assistance_para_dict['assistance_gain_k'],
                    'assistance_p': self.rehab.assistance_para_dict['assistance_p'],
                    'assistance_i': self.rehab.assistance_para_dict['assistance_i'],
                    'assistance_d': self.rehab.assistance_para_dict['assistance_d'],
                    'assistance_muscle_p': self.rehab.assistance_para_dict['assistance_muscle_p'],
                    'assistance_muscle_i': self.rehab.assistance_para_dict['assistance_muscle_i'],
                    'assistance_muscle_d': self.rehab.assistance_para_dict['assistance_muscle_d'],
                },
                'resistance_parameter': {
                    'resistance_moment': self.rehab.resistance_para_dict['resistance_moment'],
                    'resistance_muscle_p': self.rehab.resistance_para_dict['resistance_muscle_p'],
                    'resistance_muscle_i': self.rehab.resistance_para_dict['resistance_muscle_i'],
                    'resistance_muscle_d': self.rehab.resistance_para_dict['resistance_muscle_d'],
                },
                'motor_setting_parameter': {
                    'rom_safe_upper': self.rehab.motor_setting_dict['rom_safe_upper'],
                    'rom_safe_lower': self.rehab.motor_setting_dict['rom_safe_lower'],
                    'perpendicular_angle': self.rehab.motor_setting_dict['perpendicular_angle'],
                },
                'hydrodynamic_parameter': {
                    'test_velocity_input': self.rehab.hydro_dict['test_velocity_input'],
                    'test_acceleration_input': self.rehab.hydro_dict['test_acceleration_input'],
                    'test_p_input': self.rehab.hydro_dict['test_p_input'],
                    'test_i_input': self.rehab.hydro_dict['test_i_input'],
                    'test_d_input': self.rehab.hydro_dict['test_d_input'],
                    'test_rom_upper_input': self.rehab.hydro_dict['test_rom_upper_input'],
                    'test_rom_lower_input': self.rehab.hydro_dict['test_rom_lower_input'],
                    'test_hold_time_input': self.rehab.hydro_dict['test_hold_time_input'],
                }
            }


        with open("custom_json/rehab.json", "w") as fw:
            json.dump(data, fw, indent=4)


    def hydrodynamic_test_setting_btn_clicked(self):
        hydro_dict = self.textedit_groups['hydro']
        try:
            self.rehab.hydro_dict['test_velocity_input'] = float(hydro_dict['test_velocity_input'].toPlainText())
            self.rehab.hydro_dict['test_acceleration_input'] = float(hydro_dict['test_acceleration_input'].toPlainText())
            self.rehab.hydro_dict['test_p_input'] = float(hydro_dict['test_p_input'].toPlainText())
            self.rehab.hydro_dict['test_i_input'] = float(hydro_dict['test_i_input'].toPlainText())
            self.rehab.hydro_dict['test_d_input'] = float(hydro_dict['test_d_input'].toPlainText())
            self.rehab.hydro_dict['test_rom_upper_input'] = float(hydro_dict['test_rom_upper_input'].toPlainText())
            self.rehab.hydro_dict['test_rom_lower_input'] = float(hydro_dict['test_rom_lower_input'].toPlainText())
            self.rehab.hydro_dict['test_hold_time_input'] = float(hydro_dict['test_hold_time_input'].toPlainText())
        except Exception as e:
            print(f"[RMD 설정 실패] {e}")   

    def hydrodynamic_test_start_btn_clicked(self):
        if self.checkboxes['hydrodynamic_constant_velocity_check']['status'] == True or self.checkboxes['hydrodynamic_constant_acceleration_check']['status'] == True:
            self.rehab.control_active = 1

    def hydrodynamic_test_stop_btn_clicked(self):
        if self.checkboxes['hydrodynamic_constant_velocity_check']['status'] == True or self.checkboxes['hydrodynamic_constant_acceleration_check']['status'] == True:
            self.rehab.control_active = 0

    def const_angle_setting_btn_clicked(self):
        angle_dict = self.textedit_groups['const_angle']
        try:
            self.rehab.const_angle_dict['const_angle_input'] = float(angle_dict['const_angle_input'].toPlainText())
        except Exception as e:
            print(f"[RMD angle 설정 실패] {e}")

    def const_angle_start_btn_clicked(self):
        if self.checkboxes['desired_angle_move_check']['status'] == True:
            self.rehab.control_active = 1

    def const_angle_stop_btn_clicked(self):
        if self.checkboxes['desired_angle_move_check']['status'] == True:
            self.rehab.control_active = 0

    def exercise_setting_btn_clicked(self):
        exercise_dict = self.textedit_groups['exercise']
        passive_dict = self.textedit_groups['passive']
        assistance_dict = self.textedit_groups['assistance']
        resistance_dict = self.textedit_groups['resistance']

        try:
            self.rehab.exercise_para_dict['desired_velocity'] = float(exercise_dict['exercise_desired_velocity'].toPlainText())
            self.rehab.exercise_para_dict['desired_acceleration'] = float(exercise_dict['exercise_desired_acceleration'].toPlainText())
            self.rehab.exercise_para_dict['repeatation_number'] = int(exercise_dict['exercise_repeatation_number'].toPlainText())
            self.rehab.exercise_para_dict['hold_time'] = float(exercise_dict['exercise_hold_time'].toPlainText())
            self.rehab.exercise_para_dict['rom_upper'] = float(exercise_dict['exercise_rom_upper'].toPlainText())
            self.rehab.exercise_para_dict['rom_lower'] = float(exercise_dict['exercise_rom_lower'].toPlainText())

            self.rehab.passive_para_dict['passive_p'] = float(passive_dict['passive_p_input'].toPlainText())
            self.rehab.passive_para_dict['passive_i'] = float(passive_dict['passive_i_input'].toPlainText())
            self.rehab.passive_para_dict['passive_d'] = float(passive_dict['passive_d_input'].toPlainText())

            self.rehab.assistance_para_dict['assistance_gain_k'] = float(assistance_dict['assistance_gain_k_input'].toPlainText())
            self.rehab.assistance_para_dict['assistance_p'] = float(assistance_dict['assistance_p_input'].toPlainText())
            self.rehab.assistance_para_dict['assistance_i'] = float(assistance_dict['assistance_i_input'].toPlainText())
            self.rehab.assistance_para_dict['assistance_d'] = float(assistance_dict['assistance_d_input'].toPlainText())
            self.rehab.assistance_para_dict['assistance_muscle_p'] = float(assistance_dict['assistance_muscle_p_input'].toPlainText())
            self.rehab.assistance_para_dict['assistance_muscle_i'] = float(assistance_dict['assistance_muscle_i_input'].toPlainText())
            self.rehab.assistance_para_dict['assistance_muscle_d'] = float(assistance_dict['assistance_muscle_d_input'].toPlainText())

            self.rehab.resistance_para_dict['resistance_moment'] = float(resistance_dict['resistance_moment_input'].toPlainText())
            self.rehab.resistance_para_dict['resistance_muscle_p'] = float(resistance_dict['resistance_muscle_p_input'].toPlainText())
            self.rehab.resistance_para_dict['resistance_muscle_i'] = float(resistance_dict['resistance_muscle_i_input'].toPlainText())
            self.rehab.resistance_para_dict['resistance_muscle_d'] = float(resistance_dict['resistance_muscle_d_input'].toPlainText())

        except Exception as e:
            print(f"[RMD exercise 설정 실패] {e}")

    def exercise_start_btn_clicked(self):
        if self.checkboxes['exercise_passive_check']['status'] == True or self.checkboxes['exercise_assistance_check']['status'] == True or self.checkboxes['exercise_resistance_check']['status'] == True:
            self.rehab.control_active = 1

    def exercise_stop_btn_clicked(self):
        if self.checkboxes['exercise_passive_check']['status'] == True or self.checkboxes['exercise_assistance_check']['status'] == True or self.checkboxes['exercise_resistance_check']['status'] == True:
            self.rehab.control_active = 0

    def muscle_start_btn_clicked(self):
        if self.checkboxes['muscle_passive_component_check']['status'] == True:
            self.rehab.control_active = 1
    
    def muscle_stop_btn_clicked(self):
        if self.checkboxes['muscle_passive_component_check']['status'] == True:
            self.rehab.control_active = 0

    def enable_btn_group(self, group_name):
        if group_name in self.button_groups:
            for btn_name in self.button_groups[group_name]:
                self.button_widgets[btn_name].setEnabled(True)

    def disable_btn_group(self, group_name):
        if group_name in self.button_groups:
            for btn_name in self.button_groups[group_name]:
                self.button_widgets[btn_name].setDisabled(True)

    def btn_on(self):
        self.enable_btn_group('hydro')
        self.enable_btn_group('const_angle')
        self.enable_btn_group('exercise')
        self.enable_btn_group('muscle')
    
    def btn_off(self):
        self.disable_btn_group('hydro')
        self.disable_btn_group('const_angle')
        self.disable_btn_group('exercise')
        self.disable_btn_group('muscle')

    def update_data(self):

        knee_angle = self.rehab.knee_angle
        velocity = self.rehab.motor_velocity
        power_enabled_status = self.rehab.power_enabled
        motor_control_active = self.rehab.control_active

        self.angle_text.setText(f"{knee_angle:.2f}")
        self.velocity_text.setText(f"{velocity:.2f}")
        self.power_enabled_status_text.setText(f"{power_enabled_status:.2f}")
        self.control_active_text.setText(f"{motor_control_active:.2f}")


class Ros2Thread(QThread):
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
        QApplication.quit()

    signal.signal(signal.SIGINT, signal_handler)

    rclpy.init(args=args)
    rehab_program = Rehab_program()
    ros2_thread = Ros2Thread(rehab_program)
    ros2_thread.start()

    app = QApplication(sys.argv)
    main_window = RehabWindow(rehab_program)

    try:
        sys.exit(app.exec_())
    finally:
        ros2_thread.stop()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
