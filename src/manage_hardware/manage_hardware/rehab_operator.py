from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import QTimer, Qt, QThread

import sys
import rclpy
from std_msgs.msg import Float64, Float64MultiArray
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.executors import MultiThreadedExecutor

import time
import json
import signal

from custom_module.ui_module import RehabWindow

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
        const_angle_parameter = rehab_data['const_angle_parameter']

        ## ROS2 topic publisher/subscriber 설정

        self.exercise_info_publisher = self.create_publisher(
            Float64MultiArray, 'Exercise_info', self.qos_profile)
        
        self.exercise_parameter_publisher = self.create_publisher(
            Float64MultiArray, 'Exercise_parameter', self.qos_profile)
        
        self.const_angle_parameter_publisher = self.create_publisher(
            Float64MultiArray, 'Const_angle_parameter', self.qos_profile)
        
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

        self.knee_angle = 0.0

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
        2. sine
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

        self.const_angle = 0.0

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
                    'test_omega_input': hydrodynamic_parameter['test_omega_input'], 
                    'test_p_input': hydrodynamic_parameter['test_p_input'],
                    'test_i_input': hydrodynamic_parameter['test_i_input'],
                    'test_d_input': hydrodynamic_parameter['test_d_input'],
                    'test_rom_upper_input': hydrodynamic_parameter['test_rom_upper_input'],
                    'test_rom_lower_input': hydrodynamic_parameter['test_rom_lower_input'],
                    'test_hold_time_input': hydrodynamic_parameter['test_hold_time_input'],}
        self.const_angle_para_dict = {'const_angle': self.const_angle,
                                "const_angle_p": const_angle_parameter['const_angle_p'],
                                "const_angle_i": const_angle_parameter['const_angle_i'],
                                "const_angle_d": const_angle_parameter['const_angle_d']
                                 }

        self.timer = self.create_timer(0.01, self.rehab_control_pub)

    def rehab_control_pub(self):
        
        self.exercise_info_pub()
        self.exercise_parameter_pub()
        self.passive_parameter_pub()
        self.assistance_parameter_pub()
        self.resistance_parameter_pub()
        self.const_angle_parameter_pub()
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
        self.assistance_parameter_publisher.publish(msg)
    
    def resistance_parameter_pub(self):
        msg = Float64MultiArray()
        msg.data = [self.resistance_para_dict['resistance_moment'],
                    self.resistance_para_dict['resistance_muscle_p'],
                    self.resistance_para_dict['resistance_muscle_i'],
                    self.resistance_para_dict['resistance_muscle_d']]
        self.resistance_parameter_publisher.publish(msg)
    def const_angle_parameter_pub(self):
        msg = Float64MultiArray()
        msg.data = [self.const_angle_para_dict['const_angle'],
                    self.const_angle_para_dict['const_angle_p'],
                    self.const_angle_para_dict['const_angle_i'],
                    self.const_angle_para_dict['const_angle_d']]
        self.const_angle_parameter_publisher.publish(msg)

    def motor_setting_parameter_pub(self):
        msg = Float64MultiArray()
        msg.data = [self.motor_setting_dict['rom_safe_upper'],
                    self.motor_setting_dict['rom_safe_lower'],
                    self.motor_setting_dict['perpendicular_angle']]
        self.motor_setting_parameter_publisher.publish(msg)


    def motor_hydrodynamic_pub(self):
        msg = Float64MultiArray()
        msg.data = [self.hydro_dict['test_velocity_input'],
                    self.hydro_dict['test_omega_input'],
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
        self.knee_angle = self.motor_knee_angle

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
