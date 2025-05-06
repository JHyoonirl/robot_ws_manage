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



class Motor(Node):
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


        with open("custom_json/motor_info.json", "r") as fr:
            motor_data = json.load(fr)

        self.motor_common_parameter_publisher = self.create_publisher(
            Float64MultiArray, 'Motor_common_parameter', self.qos_profile)

        self.motor_control_mode_publisher = self.create_publisher(
            Float64MultiArray, 'Motor_control_mode', self.qos_profile)

        self.motor_hydrodynamic_control_publisher = self.create_publisher(
            Float64MultiArray, 'Motor_hydrodynamic_control_info', self.qos_profile)
        
        self.motor_info_subscriber = self.create_subscription(
            Float64MultiArray, 'Motor_info', self.motor_info_update_callback, self.qos_profile)
        

        self.knee_angle = 0.0
        self.motor_velocity = 0.0

        self.motor_power_enabled_sub = 0
        '''
        0. off
        1. on
        '''

        self.motor_power_enabled_pub = 0 # motor published control switch

        self.motor_control_mode = 0
        '''
        1. extension constant velocity
        2. extension constant acceleration
        3. passive exercise
        4. resistance exercise
        5. assistance exercise
        6. angle move
        '''
        self.control_active_sub = 0
        self.control_active_pub = 0
        '''
        0. stop
        1. start
        '''
        self.muscle_passive_component_switch = 0
        '''
        0. stop
        1. start
        '''


        self.common_dict = {'rom_safe_upper': motor_data['rom_safe_upper'],
                    'rom_safe_lower': motor_data['rom_safe_lower'],
                    'perpendicular_angle': motor_data['perpendicular_angle']}
        
        self.hydro_dict = {'desired_velocity_input': motor_data['desired_velocity_input'],
                    'desired_acceleration_input': motor_data['desired_acceleration_input'], 
                    'test_p_input': motor_data['test_p_input'],
                    'test_i_input': motor_data['test_i_input'],
                    'test_d_input': motor_data['test_d_input'],
                    'test_rom_upper_input': motor_data['test_rom_upper_input'],
                    'test_rom_lower_input': motor_data['test_rom_lower_input'],
                    'test_hold_time_input': motor_data['test_hold_time_input']}
        
        self.angle_dict = {'desired_angle_input': motor_data['desired_angle_input']}

        self.timer = self.create_timer(0.01, self.motor_control_pub)

    def motor_control_pub(self):
        self.motor_common_parameter_pub()
        self.motor_control_mode_pub()
        self.motor_hydrodynamic_pub()
        
    def motor_info_update_callback(self, msg):
        msg = msg.data
        '''
        motor_power_enabled
        control_active
        voltage
        torque_current
        motor_velocity
        motor_angle
        knee_angle
        '''

        self.motor_power_enabled_sub = msg[0]
        self.control_active_sub = msg[1]
        self.motor_velocity = msg[4]
        self.knee_angle = msg[6]
        

    def motor_common_parameter_pub(self):
        '''
        rom_safe_upper
        rom_safe_lower
        perpendicular_angle
        '''
        msg = Float64MultiArray()
        msg.data = [self.common_dict['rom_safe_upper'],
                    self.common_dict['rom_safe_lower'],
                    self.common_dict['perpendicular_angle']]
        self.motor_common_parameter_publisher.publish(msg)

    def motor_control_mode_pub(self):
        '''
        motor_power_enabled
        motor_control_mode 
        control_active 
        muscle_passive_component_switch
        '''
        msg = Float64MultiArray()
        msg.data = [self.motor_power_enabled_pub, self.motor_control_mode, self.control_active_pub, self.muscle_passive_component_switch]
        self.motor_control_mode_publisher.publish(msg)

    def motor_hydrodynamic_pub(self):
        msg = Float64MultiArray()
        msg.data = [self.hydro_dict['desired_velocity_input'],
                    self.hydro_dict['desired_acceleration_input'],
                    self.hydro_dict['test_p_input'],
                    self.hydro_dict['test_i_input'],
                    self.hydro_dict['test_d_input'],
                    self.hydro_dict['test_rom_upper_input'],
                    self.hydro_dict['test_rom_lower_input'],
                    self.hydro_dict['test_hold_time_input']]
        self.motor_hydrodynamic_control_publisher.publish(msg)


class MotorWindow(QMainWindow):
    def __init__(self, motor=Motor):
        QMainWindow.__init__(self)
        # self.rmd = RMD(port='COM3')  # 포트는 환경에 따라 변경
        self.motor = motor

        self.ui = uic.loadUi('UI/motor_gui.ui', self)

        self.initUI()
        self.show()

    def initUI(self):
        
        # textbrowser 위젲
        self.angle_text = self.findChild(QTextBrowser, 'angle_data')
        self.velocity_text = self.findChild(QTextBrowser, 'velocity_data')
        self.motor_power_enabled_status_text = self.findChild(QTextBrowser, 'motor_power_enabled_status')
        self.motor_control_active_text = self.findChild(QTextBrowser, 'motor_control_active')

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
            'desired_angle_move_check': {
                'widget': self.findChild(QCheckBox, 'desired_angle_move_check'),
                'handler': self.desired_angle_move_checked,
                'status': False
            },
            'muscle_passive_component_check': {
                'widget': self.findChild(QCheckBox, 'muscle_passive_component_check'),
                'handler': self.muscle_passive_component_checked,
                'status': False
            } 
        }

        # connect는 여기서만 한 번!
        for info in self.checkboxes.values():
            if info['widget'] is not None:
                info['widget'].stateChanged.connect(info['handler'])

        self.control_off()

        # buttons 정의

        self.system_quit_btn = self.findChild(QPushButton, 'system_quit_btn')
        self.motor_on_off_btn = self.findChild(QPushButton, 'motor_on_off_btn')
        self.parameter_setting_btn = self.findChild(QPushButton, 'parameter_setting_btn')
        self.parameter_save_btn = self.findChild(QPushButton, 'parameter_save_btn')

        self.system_quit_btn.clicked.connect(self.system_quit_btn_clicked)
        self.motor_on_off_btn.clicked.connect(self.motor_on_off_btn_clicked)
        self.parameter_setting_btn.clicked.connect(self.parameter_setting_btn_clicked)
        self.parameter_save_btn.clicked.connect(self.parameter_save_btn_clicked)

        # 버튼 그룹별 정의
        self.button_groups = {
            'hydro': {
                'hydrodynamic_test_setting_btn': self.hydrodynamic_test_setting_btn_clicked,
                'hydrodynamic_test_start_btn': self.hydrodynamic_test_start_btn_clicked,
                'hydrodynamic_test_stop_btn': self.hydrodynamic_test_stop_btn_clicked,
            },
            'angle': {
                'angle_setting_btn': self.angle_setting_btn_clicked,
                'angle_start_btn': self.angle_start_btn_clicked,
                'angle_stop_btn': self.angle_stop_btn_clicked,
            },
            'muscle': {
                'muscle_start_btn': self.muscle_start_btn_clicked,
                'muscle_stop_btn': self.muscle_stop_btn_clicked,
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
            'common': {
                'rom_safe_upper': None,
                'rom_safe_lower': None,
                'perpendicular_angle': None,
            },
            'hydro': {
                'desired_velocity_input': None,
                'desired_acceleration_input': None,
                'test_p_input': None,
                'test_i_input': None,
                'test_d_input': None,
                'test_rom_upper_input': None,
                'test_rom_lower_input': None,
                'test_hold_time_input': None,
            },
            'angle': {
                'desired_angle_input': None,
            }
        }

        # 각 QTextEdit 위젯 findChild로 바인딩
        for group in self.textedit_groups.values():
            for name in group:
                widget = self.findChild(QTextEdit, name)
                group[name] = widget

        self.QtextEdit_init()
    
    def QtextEdit_init(self):
        # 각 QTextEdit 위젯에 초기값 설정
        self.textedit_groups['common']['rom_safe_upper'].setPlainText(str(self.motor.common_dict['rom_safe_upper']))
        self.textedit_groups['common']['rom_safe_lower'].setPlainText(str(self.motor.common_dict['rom_safe_lower']))
        self.textedit_groups['common']['perpendicular_angle'].setPlainText(str(self.motor.common_dict['perpendicular_angle']))

        self.textedit_groups['hydro']['desired_velocity_input'].setPlainText(str(self.motor.hydro_dict['desired_velocity_input']))
        self.textedit_groups['hydro']['desired_acceleration_input'].setPlainText(str(self.motor.hydro_dict['desired_acceleration_input']))
        self.textedit_groups['hydro']['test_p_input'].setPlainText(str(self.motor.hydro_dict['test_p_input']))
        self.textedit_groups['hydro']['test_i_input'].setPlainText(str(self.motor.hydro_dict['test_i_input']))
        self.textedit_groups['hydro']['test_d_input'].setPlainText(str(self.motor.hydro_dict['test_d_input']))
        self.textedit_groups['hydro']['test_rom_upper_input'].setPlainText(str(self.motor.hydro_dict['test_rom_upper_input']))
        self.textedit_groups['hydro']['test_rom_lower_input'].setPlainText(str(self.motor.hydro_dict['test_rom_lower_input']))
        self.textedit_groups['hydro']['test_hold_time_input'].setPlainText(str(self.motor.hydro_dict['test_hold_time_input']))

        self.textedit_groups['angle']['desired_angle_input'].setPlainText(str(self.motor.angle_dict['desired_angle_input']))
        
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
            self.disable_btn_group('angle')
            self.motor.motor_control_mode = 1

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
            self.disable_btn_group('angle')
            self.motor.motor_control_mode = 2

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

    def desired_angle_move_checked(self, state):
        if state == Qt.Checked:
            
            self.disable_btn_group('hydro')
            self.enable_btn_group('angle')
            self.motor.motor_control_mode = 6

            for name, info in self.checkboxes.items():                
                if name == 'desired_angle_move_check':
                    info['status'] = True
                else:
                    if info['widget'].checkState() == 2:
                        info['widget'].toggle()
                    info['status'] = False
                
        else:
            for name, info in self.checkboxes.items():
                if name == 'desired_angle_move_check':
                    info['status'] = False
            self.disable_btn_group('angle')

    def muscle_passive_component_checked(self, state):
        if state == Qt.Checked:
            self.motor.muscle_passive_component_switch = 1
            self.enable_btn_group('muscle')
            for name, info in self.checkboxes.items():                
                if name == 'muscle_passive_component_check':
                    info['status'] = True

        else:
            self.motor.muscle_passive_component_switch = 0
            self.disable_btn_group('muscle')
            for name, info in self.checkboxes.items():                
                if name == 'muscle_passive_component_check':
                    info['status'] = False

    def system_quit_btn_clicked(self):
        self.motor.motor_power_enabled_pub = 0
        time.sleep(0.5)
        sys.exit()
             
    def motor_on_off_btn_clicked(self):
        if self.motor.motor_power_enabled_pub == 1:
            self.motor.motor_power_enabled_pub = 0
        else:
            self.motor.motor_power_enabled_pub = 1

    def parameter_setting_btn_clicked(self):
        common_dict = self.textedit_groups['common']

        try:
            self.motor.common_dict['rom_safe_upper'] = float(common_dict['rom_safe_upper'].toPlainText())
            self.motor.common_dict['rom_safe_lower'] = float(common_dict['rom_safe_lower'].toPlainText())
            self.motor.common_dict['perpendicular_angle'] = float(common_dict['perpendicular_angle'].toPlainText())

        except Exception as e:
            print(f"[RMD 설정 실패] {e}")

    def parameter_save_btn_clicked(self):
        data = {}
        data['rom_safe_upper'] = self.motor.common_dict['rom_safe_upper']
        data['rom_safe_lower'] = self.motor.common_dict['rom_safe_lower']
        data['perpendicular_angle'] = self.motor.common_dict['perpendicular_angle']
        data['desired_velocity_input'] = self.motor.hydro_dict['desired_velocity_input']
        data['desired_acceleration_input'] = self.motor.hydro_dict['desired_acceleration_input']
        data['test_p_input'] = self.motor.hydro_dict['test_p_input']
        data['test_i_input'] = self.motor.hydro_dict['test_i_input']
        data['test_d_input'] = self.motor.hydro_dict['test_d_input']
        data['test_rom_upper_input'] = self.motor.hydro_dict['test_rom_upper_input']
        data['test_rom_lower_input'] = self.motor.hydro_dict['test_rom_lower_input']
        data['test_hold_time_input'] = self.motor.hydro_dict['test_hold_time_input']
        data['desired_angle_input'] = self.motor.angle_dict['desired_angle_input']

        with open("custom_json/motor_info.json", "w") as fw:
            json.dump(data, fw, indent=4)

    def hydrodynamic_test_setting_btn_clicked(self):
        hydro_dict = self.textedit_groups['hydro']
        try:
            self.motor.hydro_dict['desired_velocity_input'] = float(hydro_dict['desired_velocity_input'].toPlainText())
            self.motor.hydro_dict['desired_acceleration_input'] = float(hydro_dict['desired_acceleration_input'].toPlainText())
            self.motor.hydro_dict['test_p_input'] = float(hydro_dict['test_p_input'].toPlainText())
            self.motor.hydro_dict['test_i_input'] = float(hydro_dict['test_i_input'].toPlainText())
            self.motor.hydro_dict['test_d_input'] = float(hydro_dict['test_d_input'].toPlainText())
            self.motor.hydro_dict['test_rom_upper_input'] = float(hydro_dict['test_rom_upper_input'].toPlainText())
            self.motor.hydro_dict['test_rom_lower_input'] = float(hydro_dict['test_rom_lower_input'].toPlainText())
            self.motor.hydro_dict['test_hold_time_input'] = float(hydro_dict['test_hold_time_input'].toPlainText())
        except Exception as e:
            print(f"[RMD 설정 실패] {e}")   

    def hydrodynamic_test_start_btn_clicked(self):
        if self.checkboxes['hydrodynamic_constant_velocity_check']['status'] == True or self.checkboxes['hydrodynamic_constant_acceleration_check']['status'] == True:
            self.motor.control_active_pub = 1

    def hydrodynamic_test_stop_btn_clicked(self):
        if self.checkboxes['hydrodynamic_constant_velocity_check']['status'] == True or self.checkboxes['hydrodynamic_constant_acceleration_check']['status'] == True:
            self.motor.control_active_pub = 0

    def angle_setting_btn_clicked(self):
        angle_dict = self.textedit_groups['angle']
        try:
            self.motor.angle_dict['desired_angle_input'] = float(angle_dict['desired_angle_input'].toPlainText())
        except Exception as e:
            print(f"[RMD angle 설정 실패] {e}")

    def angle_start_btn_clicked(self):
        if self.checkboxes['desired_angle_move_check']['status'] == True:
            self.motor.control_active_pub = 1

    def angle_stop_btn_clicked(self):
        if self.checkboxes['desired_angle_move_check']['status'] == True:
            self.motor.control_active_pub = 0

    def muscle_start_btn_clicked(self):
        if self.checkboxes['muscle_passive_component_check']['status'] == True:
            self.motor.control_active_pub = 1
    
    def muscle_stop_btn_clicked(self):
        if self.checkboxes['muscle_passive_component_check']['status'] == True:
            self.motor.control_active_pub = 0

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
        self.enable_btn_group('angle')
        self.enable_btn_group('muscle')
    
    def btn_off(self):
        self.disable_btn_group('hydro')
        self.disable_btn_group('angle')
        self.disable_btn_group('muscle')

    def update_data(self):

        knee_angle = self.motor.knee_angle
        velocity = self.motor.motor_velocity
        motor_power_enabled_status = self.motor.motor_power_enabled_sub
        motor_control_active = self.motor.control_active_sub

        self.angle_text.setText(f"{knee_angle:.2f}")
        self.velocity_text.setText(f"{velocity:.2f}")
        self.motor_power_enabled_status_text.setText(f"{motor_power_enabled_status:.2f}")
        self.motor_control_active_text.setText(f"{motor_control_active:.2f}")


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
    motor = Motor()
    ros2_thread = Ros2Thread(motor)
    ros2_thread.start()

    app = QApplication(sys.argv)
    main_window = MotorWindow(motor)

    try:
        sys.exit(app.exec_())
    finally:
        ros2_thread.stop()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
