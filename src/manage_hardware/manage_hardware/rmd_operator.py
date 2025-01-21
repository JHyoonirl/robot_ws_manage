from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QLabel, QWidget, QDial, QPushButton, QTextBrowser, QCheckBox, QTextEdit
from PyQt5.QtCore import QTimer, Qt
from PyQt5 import uic
from pyqtgraph import PlotWidget  # 그래프를 위한 라이브러리
import sys
import rclpy
from std_msgs.msg import Float64
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

        self.Motor_angle_publisher = self.create_publisher(Float64, 'Motor_angle', self.qos_profile)
        self.Motor_velocity_publisher = self.create_publisher(Float64, 'Motor_velocity', self.qos_profile)
        
        ## 모터 반시계 회전(-)은 Extension
        ## 모터 시계 회전(+)은 flexion

        self.MOTOR_ID = 1
        self.ANGLE_INIT = 10
        self.VELOCITY_LIMIT = 5000
        self.Desired_angle = 0.0
        self.control_check_status = False
        self.position_control_check_status = False # 작동 Main Switch
        self.sinusoidal_control_check_status = False # 작동 Main Switch
        self.position_control_activate_status = False # step 작동 on/off Switch
        self.sinusoidal_control_activate_status = False # step 작동 on/off Switch
        self.neutral_angle = 0
        self.neutral_torque = 0 # 중립 위치에서 발생하는 토크는?
        self.position_error = 0

        with open("RMD/PID.json", "r") as fr:
            data = json.load(fr)
        # print(data)

        self.Kp = float(data["kp"])
        self.Ki = float(data["ki"])
        self.Kd = float(data["kd"])

        self.pos_error = 0
        self.pos_error_prev = 0
        self.pos_error_integral = 0
        self.dt = 0.005

        self.amplitude = 0.0
        self.period = 0.0
        self.pos_offset = 0.0
        self.RMD_timer = 0.0

        self.voltage = 0
        self.temperature = 0
        self.torque_current = 0
        self.velocity = 0
        self.angle = 0

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
        # print(self.kp_cur, self.ki_cur)
        
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
                self.voltage, self.temperature, self.torque_current, self.velocity, self.angle = self.RMD.status_motor() #
                time.sleep(0.001)
            except Exception as e:
                print(f'Error1: {e}')
            if self.control_check_status == False:
                self.motor_off()
                
            if self.position_control_check_status == True and self.position_control_activate_status == True: # step 제어 가능
                # if self.position_control_activate_status == True: #제어 활성화
                self.motor_step()
            elif self.sinusoidal_control_check_status == True and self.sinusoidal_control_activate_status == True: # sine 제어 가능
                self.motor_sine()
            else:
                # if self.sinusoidal_control_activate_status == True:
                self.motor_off()

            time.sleep(self.dt)


    def motor_off(self):
        self.RMD.raw_motor_off()
        self.pos_error = 0
        self.pos_error_prev = 0
        self.pos_error_integral = 0

    def motor_step(self):
        try:
            # _ = self.RMD.position_closed_loop(self.Desired_pos, self.VELOCITY_LIMIT)
            # print('control')
            self.pos_error = self.Desired_angle - self.angle

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

            _ = self.RMD.torque_closed_loop(int(output))

        except Exception as e:
            print(f'Error2: {e}')

    def motor_sine(self):
        try:
            sine_time = time.time() - self.RMD_timer
            self.Desired_angle = self.amplitude * math.sin(self.period * sine_time) + self.pos_offset

            self.pos_error = self.Desired_angle - self.angle
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

            _ = self.RMD.torque_closed_loop(int(output))
                        

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
        self.Speed_list = []
        self.threadhold = 500
        self.ui = uic.loadUi('UI/motor.ui', self)

        self.initUI()
        # self.initTimer()
        self.show()

    def initUI(self):
        # plot 위젯 찾기
        self.Plot_Angle = self.findChild(PlotWidget, 'Plot_Angle')
        self.Plot_Velocity = self.findChild(PlotWidget, 'Plot_Velocity')
        
        self.Plot_Angle_desired_data = self.Plot_Angle.plot(pen='r', name='Angle_desired')
        self.Plot_Angle_current_data = self.Plot_Angle.plot(pen='b', name='Angle_current')
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
        self.timer.start(10)  # 100ms 간격으로 업데이트
        
        # 체크 박스 정의
        self.control_on_off = self.findChild(QCheckBox, 'Control_on_off_check')
        self.position_control_check = self.findChild(QCheckBox, 'position_control_check')
        self.sinusoidal_control_check = self.findChild(QCheckBox, 'sinusoidal_control_check')

        self.control_on_off.stateChanged.connect(self.control_on_off_changed)
        self.position_control_check.setDisabled(True)
        self.sinusoidal_control_check.setDisabled(True)

        self.position_control_check.stateChanged.connect(self.position_control_checked)
        self.sinusoidal_control_check.stateChanged.connect(self.sinusoidal_control_checked)

        #textedit 정의
        self.Desired_pos = self.findChild(QTextEdit, 'desired_pos')
        self.Kp = self.findChild(QTextEdit, 'Pos_kp')
        self.Ki = self.findChild(QTextEdit, 'Pos_ki')
        self.Kd = self.findChild(QTextEdit, 'Pos_kd')
        # self.vel_ki = self.findChild(QTextEdit, 'vel_ki')
        # self.cur_kp = self.findChild(QTextEdit, 'cur_kp')
        # self.cur_ki = self.findChild(QTextEdit, 'cur_ki')

        self.Kp.setPlainText(f"{self._RMD.Kp}")
        self.Ki.setPlainText(f"{self._RMD.Ki}")
        self.Kd.setPlainText(f"{self._RMD.Kd}")
        # self.vel_ki.setPlainText(f"{self._RMD.ki_vel}")
        # self.cur_kp.setPlainText(f"{self._RMD.kp_cur}")
        # self.cur_ki.setPlainText(f"{self._RMD.ki_cur}")


        self.amplitude = self.findChild(QTextEdit, 'amplitude')
        self.period = self.findChild(QTextEdit, 'period')
        self.pos_offset = self.findChild(QTextEdit, 'pos_offset')

        # buttons 정의

        self.system_quit_btn = self.findChild(QPushButton, 'system_quit_btn')

        self.parameter_setting_btn = self.findChild(QPushButton, 'parameter_setting_btn')

        self.pos_setting_btn = self.findChild(QPushButton, 'pos_setting_btn')
        self.pos_start_btn = self.findChild(QPushButton, 'pos_start_btn')
        self.pos_stop_btn = self.findChild(QPushButton, 'pos_stop_btn')

        self.sinusoidal_setting_btn = self.findChild(QPushButton, 'sinusoidal_setting_btn')
        self.sinusoidal_start_btn = self.findChild(QPushButton, 'sinusoidal_start_btn')
        self.sinusoidal_stop_btn = self.findChild(QPushButton, 'sinusoidal_stop_btn')

        self.system_quit_btn.clicked.connect(self.system_quit_btn_clicked)

        self.parameter_setting_btn.clicked.connect(self.parameter_setting_btn_clicked)
        
        self.pos_setting_btn.clicked.connect(self.pos_setting_btn_clicked)
        self.pos_start_btn.clicked.connect(self.pos_start_btn_clicked)
        self.pos_stop_btn.clicked.connect(self.pos_stop_btn_clicked)

        self.sinusoidal_setting_btn.clicked.connect(self.sinusoidal_setting_btn_clicked)
        self.sinusoidal_start_btn.clicked.connect(self.sinusoidal_start_btn_clicked)
        self.sinusoidal_stop_btn.clicked.connect(self.sinusoidal_stop_btn_clicked)

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
                # ki_vel = self.vel_ki.toPlainText()
                # kp_cur = self.cur_kp.toPlainText()
                # ki_cur = self.cur_ki.toPlainText()
                time.sleep(0.005)
                self._RMD.Kp = float(Kp)
                self._RMD.Ki = float(Ki)
                self._RMD.Kd = float(Kd)

                data = {"kp": self._RMD.Kp, "ki": self._RMD.Ki, "kd": self._RMD.Kd}
                with open("RMD/PID.json", "w") as fr:
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


    def sinusoidal_start_btn_clicked(self):
        if self._RMD.sinusoidal_control_check_status != False:
            self._RMD.sinusoidal_control_activate_status = True
            self._RMD.RMD_timer = time.time()

    def sinusoidal_stop_btn_clicked(self):
        if self._RMD.sinusoidal_control_check_status != False:
            self._RMD.sinusoidal_control_activate_status = False
    
    def control_on_off_changed(self, state):
        if state == Qt.Checked:
            self._RMD.control_check_status = True
            self.position_control_check.setEnabled(True)
            self.sinusoidal_control_check.setEnabled(True)
            # self.btn_on()
        else:
            self._RMD.control_check_status = False
            self._RMD.position_control_check_status = False
            self._RMD.sinusoidal_control_check_status = False
            self.position_control_check.setDisabled(True)
            self.sinusoidal_control_check.setDisabled(True)
            # self.btn_off()

    def btn_on(self):
        self.pos_setting_btn.setEnabled(True)
        self.pos_start_btn.setEnabled(True)
        self.pos_stop_btn.setEnabled(True)
    
        self.sinusoidal_setting_btn.setEnabled(True)
        self.sinusoidal_start_btn.setEnabled(True)
        self.sinusoidal_stop_btn.setEnabled(True)

    def pos_btn_on(self):
        self.pos_setting_btn.setEnabled(True)
        self.pos_start_btn.setEnabled(True)
        self.pos_stop_btn.setEnabled(True)
    
        self.sinusoidal_setting_btn.setDisabled(True)
        self.sinusoidal_start_btn.setDisabled(True)
        self.sinusoidal_stop_btn.setDisabled(True)

    def sinusoidal_btn_on(self):
        self.pos_setting_btn.setDisabled(True)
        self.pos_start_btn.setDisabled(True)
        self.pos_stop_btn.setDisabled(True)
    
        self.sinusoidal_setting_btn.setEnabled(True)
        self.sinusoidal_start_btn.setEnabled(True)
        self.sinusoidal_stop_btn.setEnabled(True)
    
    def btn_off(self):
        self.pos_setting_btn.setDisabled(True)
        self.pos_start_btn.setDisabled(True)
        self.pos_stop_btn.setDisabled(True)
    
        self.sinusoidal_setting_btn.setDisabled(True)
        self.sinusoidal_start_btn.setDisabled(True)
        self.sinusoidal_stop_btn.setDisabled(True)

    def position_control_checked(self, state):
        # state == 0 : unchecked, state == 2 : checked
        if state == Qt.Checked:
            self._RMD.position_control_check_status = True
            self._RMD.sinusoidal_control_check_status = False
            self.pos_btn_on()

            if self._RMD.sinusoidal_control_check_status == False and self.sinusoidal_control_check.checkState() == 2:
                self.sinusoidal_control_check.toggle()
                # print(self._RMD.position_control_check_status, self._RMD.sinusoidal_control_check_status)
                
        else:
            self.position_control_check_status = False
    
    def sinusoidal_control_checked(self, state):
        if state == Qt.Checked:
            self._RMD.sinusoidal_control_check_status = True
            self._RMD.position_control_check_status = False
            self.sinusoidal_btn_on()
            if self._RMD.position_control_check_status == False and self.position_control_check.checkState() == 2:
                self.position_control_check.toggle()
                # print(self._RMD.position_control_check_status, self._RMD.sinusoidal_control_check_status)
        else:
            self.sinusoidal_control_check_status = False

    def update_data(self):
        
        Current_angle = self._RMD.angle
        Desired_angle = self._RMD.Desired_angle

        velocity = self._RMD.velocity
        self.Angle_text.setText(f"{Current_angle:.2f}")
        
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
        self.Desired_Angle_list.append(Desired_angle)
        self.Current_Angle_list.append(Current_angle)
        self.Speed_list.append(velocity* 360 / 60)
        self.Plot_Angle_desired_data.setData(self.Desired_Angle_list)
        self.Plot_Angle_current_data.setData(self.Current_Angle_list)
        self.Plot_Velocity_data.setData(self.Speed_list)

def run_node(node):
    rclpy.spin(node)


def main(args=None):
    rclpy.init(args=args)
    motor = Motor()
    thread = threading.Thread(target=run_node, args=(motor, ), daemon=True)
    # motor_thread = threading.Thread(target=motor.controller, daemon=True)
    thread.start()
    time.sleep(0.5)
    app = QApplication(sys.argv)
    main_window = MotorWindow(motor)
    # main_window.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
    
