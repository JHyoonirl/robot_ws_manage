import sys
import rclpy
from std_msgs.msg import Float64, Float64MultiArray
from rclpy.node import Node
from rclpy.qos import QoSProfile

import warnings

# from typing import TYPE_CHECKING
# if TYPE_CHECKING:
from custom_module.Muscle import Muscle
from custom_module.RMD_custom import RMD# 가정한 모듈과 클래스 이름
from dataclasses import dataclass, field
import threading
import time
import json

@dataclass
class PIDGains:
    proportional: float = 0.0
    integral: float = 0.0
    derivative: float = 0.0

@dataclass
class ROMConfig:
    upper: float = 0.0
    lower: float = 0.0

@dataclass
class ControlError:
    errorprev: float = 0.0
    errorintegral: float = 0.0
    errorderivative: float = 0.0


class Motor(Node):
    def __init__(self):
        
        '''
        코드 구성
        1) 전체 구성 자체는 ROS2 노드로 구성되어 있음.
        2) RMD motor 동작을 위해서 사용하는 torque/speed control 제어 입력은 외부 ROS2 topic을 통해서 subscribe함.
        3) RMD motor의 상태 정보는 ROS2 topic을 통해서 publish함.
        
        '''

        ### ROS2 진행을 위한 코드 ###
        super().__init__('RMD_Motor')
        
        # Parameters
        self.declare_parameter('usb_port', '/dev/ttyACM0')
        usb_port = self.get_parameter('usb_port').get_parameter_value().string_value
        self.qos_profile = QoSProfile(depth=10)

        # Muscle Component
        self.muscle = Muscle()

        # RMD Motor Setup
        self.RMD = RMD(0x141)
        self.RMD.setup('slcan', usb_port)

       

        # Publishers
        self.motor_info_publisher = self.create_publisher(Float64MultiArray, 'Motor_info', self.qos_profile)

        # Subscribers
        self.exercise_info_subscriber = self.create_subscription(
            Float64MultiArray, 'Exercise_info', self.exercise_info_callback, self.qos_profile
        )
        self.motor_setting_parameter_subscriber = self.create_subscription(
            Float64MultiArray, 'Motor_setting_parameter', self.motor_setting_parameter_callback, self.qos_profile
        )
        self.motor_hydrodynamic_control_subscriber = self.create_subscription(
            Float64MultiArray, 'Motor_hydrodynamic_control_info', self.motor_hydrodynamic_callback, self.qos_profile
        )
        self.motor_assistance_control_subscriber = self.create_subscription(
            Float64MultiArray, 'Motor_assistance_control_info', self.motor_assistance_callback, self.qos_profile
        )
        self.motor_resistance_control_subscriber = self.create_subscription(
            Float64MultiArray, 'Motor_resistance_control_info', self.motor_resistance_callback, self.qos_profile
        )
        self.const_angle_parameter_subscriber = self.create_subscription(
            Float64MultiArray, 'Const_angle_parameter', self.const_angle_parameter_callback, self.qos_profile
        )


        ##############
        # motor information
        ##############

        self.voltage = 0.0         # Motor voltage
        self.temperature = 0.0     # Motor temperature
        self.torque_current = 0.0  # Motor torque current
        self.motor_velocity = 0.0  # Motor angular velocity
        self.motor_angle = 0.0     # Motor angle
        self.motor_knee_angle = 0.0 # Motor knee angle

        
        
        ##############
        #  exercise info subscribed information
        ##############
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
        self.repeatation_number = 0
        self.hold_time = 0 
        self.power_enabled = 0
        '''
        0. off
        1. on
        '''

        # power 관련 내부 변수
        self.power_enabled_prev = None
        self.power_locked = False  # 안전장치 잠금 상태
        self.raw_power_enabled = 0
        self.raw_power_enabled_prev = 0  # 이전 입력 상태 추적

        self.control_mode = 0
        '''
        1. extension constant velocity
        2. extension constant acc
        3. passive exercise
        4. resistance exercise
        5. assistance exercise
        6. const angle move
        '''
        self.control_active = 0
        # 제어 관련 내부 변수
        self.control_active_prev = 0 # switch memory
        self.control_time_stamp = None # control time stamp

        self.muscle_passive_component_switch = 0
        '''
        모터에 무릎의 passive component 추가

        0. muscle off
        1. muscle on
        '''

        ############
        # motor setting parameter
        ############
        self.rom_safe_upper = 0 # motor의 안전 upper ROM
        self.rom_safe_lower = 0 # motor의 안전 lower ROM
        self.perpendicular_angle = 0 # 무릎의 90도에 해당하는 motor encoder의 각도
        
        # Hydrodynamic test setup
        self.hydrodynamic_test_err_state = ControlError()

        self.hydrodynamic_test_desired_velocity = 0.0
        self.hydrodynamic_test_desired_acceleration = 0.0
        
        self.hydrodynamic_test_gains = PIDGains()

        self.hydrodynamic_test_romconfig = ROMConfig()
        self.hydrodynamic_test_hold_time = 0.0
        
        
        # resistance parameter
        self.resistance_gains = PIDGains() # resistance gains

        # assistance parameter
        self.assistance_gains = PIDGains() # assistance gains


        self.const_angle = 90


        status = self.RMD.motor_initialization()
        self.past_time = time.time()
        
        if status == True:
            self.timer = self.create_timer(0.01, self.ros2_callback) # 10ms 마다 motor 상태 정보 publish
            print('Motor thread started')

    def ros2_callback(self):
        self.motor_info_pub() # motor 상태 정보 publish
        status = self.motor_control()
    
    def motor_info_pub(self):

        self.voltage, self.temperature, self.torque_current, self.motor_velocity, self.motor_angle, error = self.RMD.status_motor() # motor raw status 호출
        self.motor_knee_angle = self.motor_angle - self.perpendicular_angle + 90


        motor_info = Float64MultiArray()
        motor_info.data = [self.power_enabled, self.control_active, self.voltage, self.torque_current, self.motor_velocity, self.motor_angle, self.motor_knee_angle]
        self.motor_info_publisher.publish(motor_info)

    def motor_control(self):
        
        try:
            motor_knee_angle = self.motor_knee_angle
            if motor_knee_angle > self.rom_safe_upper or motor_knee_angle < self.rom_safe_lower:
                self.power_enabled = 0
                self.power_locked = True
                
            if self.power_enabled == 0:
                
                if self.power_enabled_prev == self.power_enabled:
                    return self.power_enabled
                else:
                    self.power_enabled_prev = self.power_enabled
                    
                self.RMD.raw_motor_off()
                self.get_logger().info('Motor off')
                time.sleep(0.005)
                
                return self.power_enabled
            
            if self.control_active != 1:
                return self.power_enabled
            
            if self.muscle_passive_component_switch == 1:
                torque = self.muscle.M_passive(self.motor_knee_angle, self.motor_velocity)
                self.get_logger().info(f"muscle passive torque: {torque}")
                torque_LSB = self.muscle.torque_to_LSB(torque)
                self.get_logger().info(f"muscle passive torque LSB: {torque_LSB}")
                self.RMD.torque_closed_loop(int(torque_LSB))
            if self.control_mode == 1: # 등각속도 운동
                self.motor_constant_velocity()
            elif self.control_mode == 2: # 등각가속도 운동
                # pass
                self.motor_constant_acc()
            elif self.control_mode == 3: # passive exercise
                pass
                # self.motor_passive()
            elif self.control_mode == 4: # resistance exercise
                pass
                # self.motor_resistance()
            elif self.control_mode == 5: # assistance exercise
                pass
                # self.motor_assistance()
            elif self.control_mode == 6: # angle move
                pass
                # self.motor_angle_move()
            self.past_time = time.time()
            self.power_enabled_prev = self.power_enabled
            return self.power_enabled
        except Exception as e:
            print(f'Error - motor control -: {e}')
            self.RMD.raw_motor_off()
            time.sleep(0.005)

    ############### motor control function ###############
    def motor_constant_velocity(self):
        
        hydrodynamic_test_desired_angle = self.const_vel_angle_generator(self.hydrodynamic_test_desired_velocity)
        self.get_logger().info(f"desired_angle: {hydrodynamic_test_desired_angle }")
        # self.get_logger().info(f"Desired angle: {self.hydrodynamic_test_desired_angle}")
        try:
            if self.control_active == 0:
                self.hydrodynamic_test_err_state.errorintegral = 0.0
                self.hydrodynamic_test_err_state.errorprev = 0.0
                self.hydrodynamic_test_err_state.errorderivative = 0.0
            
            self.dt = time.time() - self.past_time
            self.get_logger().info(f"dt: {self.dt}")
            self.pos_error = hydrodynamic_test_desired_angle - self.motor_knee_angle

            # Proportional term
            proportional = self.hydrodynamic_test_gains.proportional * self.pos_error

            # Integral term
            self.hydrodynamic_test_err_state.errorintegral += self.pos_error * self.dt
            integral = self.hydrodynamic_test_gains.integral * self.hydrodynamic_test_err_state.errorintegral

            # Derivative term
            derivative = self.hydrodynamic_test_gains.derivative * (self.pos_error - self.hydrodynamic_test_err_state.errorprev) / self.dt

            # Update previous error
            self.hydrodynamic_test_err_state.errorprev = self.pos_error

            # Calculate the control output
            output = proportional + integral + derivative

            temperature, torque, speed, angle = self.RMD.torque_closed_loop(int(output))
            # self.get_logger().info('rmd torque, speed, angle: {0}'.format(torque, speed, angle))

        except Exception as e:
            print(f'Error2: {e}')


    def const_vel_angle_generator(self, velocity):
        now = time.time()
        t = now - self.control_time_stamp  # 제어 시작 이후 경과 시간
        self.get_logger().info(f"t: {t}")
        if t < self.hydrodynamic_test_hold_time:
            return self.hydrodynamic_test_romconfig.upper # 일정 시간 유지 후 감속 시작
        
        t = now - self.control_time_stamp - self.hydrodynamic_test_hold_time  # 감속 시작 이후 경과 시간
        
        desired_angle = self.hydrodynamic_test_romconfig.upper - velocity * (t)
        
        if desired_angle < self.hydrodynamic_test_romconfig.lower:
            desired_angle = self.hydrodynamic_test_romconfig.lower
        
        return desired_angle
        
    def motor_constant_acc(self):
        try:
            self.dt = time.time() - self.past_time
            self.test_desired_angle = self.const_acc_angle_generator(self.hydrodynamic_test_desired_acceleration)
            self.pos_error = self.test_desired_angle - self.motor_knee_angle

            # PID 계산
            proportional = self.hydrodynamic_test_gains.proportional * self.pos_error
            self.pos_error_integral += self.pos_error * self.dt
            integral = self.hydrodynamic_test_gains.integral * self.pos_error_integral
            derivative = self.hydrodynamic_test_gains.derivative * (self.pos_error - self.pos_error_prev) / self.dt
            self.pos_error_prev = self.pos_error

            output = proportional + integral + derivative
            temperature, torque, speed, angle = self.RMD.torque_closed_loop(int(output))

        except Exception as e:
            print(f'Error2: {e}')


    def const_acc_angle_generator(self, acc):
        
        now = time.time()
        t = now - self.control_time_stamp  # 제어 시작 이후 경과 시간

        if t < self.hydrodynamic_test_hold_time:
            return self.hydrodynamic_test_romconfig.upper # 일정 시간 유지 후 감속 시작

        # 감속 phase
        t_acc = t - self.hydrodynamic_test_hold_time
        theta_0 = self.hydrodynamic_test_romconfig.upper  # 감속 시작 시점의 각도
        omega_0 = 0.0  # 정지 상태에서 출발
        alpha = -abs(acc)  # 음의 가속도로 감소

        desired_angle = theta_0 + omega_0 * t_acc + 0.5 * alpha * t_acc**2

        if desired_angle < self.hydrodynamic_test_romconfig.lower:
            desired_angle = self.hydrodynamic_test_romconfig.lower
        return desired_angle
    
    ############### motor ROS2 callback function ###############
    def exercise_info_callback(self, msg):
        exercise_info = msg.data
        self.desired_angle = exercise_info[0] # 운동 각도 [deg/s]
        self.exercise_state = int(exercise_info[1]) # 운동 상태 [0: stop, 1: start, 2: hold]
        self.repeatation_number = int(exercise_info[2]) # 반복 횟수
        self.hold_time = exercise_info[3] # 운동 유지 시간 [s]
        raw_power_enabled = int(exercise_info[4]) # 전원 상태 [0: off, 1: on]
        control_active = int(exercise_info[5]) # 제어 활성화 상태 [0: off, 1: on]
        control_mode = int(exercise_info[6]) # 제어 모드 [0: off, 1: extension constant velocity, 2: extension constant acc, 3: passive exercise, 4: resistance exercise, 5: assistance exercise, 6: position move]
        muscle_passive_component_switch = int(exercise_info[7]) # 모터에 무릎의 passive component 추가 [0: muscle off, 1: muscle on]

        if raw_power_enabled == 1 and self.raw_power_enabled_prev == 0:
            if self.power_locked:
                self.get_logger().info("Safety lock released.")

            self.power_locked = False

        self.raw_power_enabled_prev = raw_power_enabled

        if not self.power_locked:
            self.power_enabled = raw_power_enabled
        else:
            self.get_logger().warn("Motor power is locked due to safety trigger.")

        if self.power_enabled == 1:
            self.control_active = control_active
            self.control_mode = control_mode
            if self.control_active == 1 and self.control_active_prev != self.control_active:
                self.control_time_stamp = time.time()
            self.control_active_prev = self.control_active
            self.muscle_passive_component_switch = muscle_passive_component_switch
        else:
            self.control_mode = 0
            self.control_active = 0
            self.control_active_prev = 0
            self.muscle_passive_component_switch = 0

    def motor_setting_parameter_callback(self, msg):
        self.rom_safe_upper = msg.data[0] # motor의 안전 upper ROM
        self.rom_safe_lower = msg.data[1] # motor의 안전 lower ROM
        self.perpendicular_angle = msg.data[2] # 무릎의 90도에 해당하는 motor encoder의 각도
    
    def motor_hydrodynamic_callback(self, msg):
        try:
            # self.get_logger().info(f"Received hydrodynamic control info: {msg.data}")
            self.hydrodynamic_test_desired_velocity = msg.data[0]
            self.hydrodynamic_test_desired_acceleration = msg.data[1]
            self.hydrodynamic_test_gains.proportional = msg.data[2]
            self.hydrodynamic_test_gains.integral = msg.data[3]
            self.hydrodynamic_test_gains.derivative = msg.data[4]
            self.hydrodynamic_test_romconfig.upper = msg.data[5]
            self.hydrodynamic_test_romconfig.lower = msg.data[6]
            self.hydrodynamic_test_hold_time = msg.data[7]

        except Exception as e:
            print(f'Error1: {e}')
    
    def motor_assistance_callback(self, msg):
        try:
            # self.get_logger().info(f"Received assistance control info: {msg.data}")
            self.assistance_gains.proportional = msg.data[0]
            self.assistance_gains.integral = msg.data[1]
            self.assistance_gains.derivative = msg.data[2]
        except Exception as e:
            print(f'Error assistance: {e}')

    def motor_resistance_callback(self, msg):
        try:
            # self.get_logger().info(f"Received resistance control info: {msg.data}")
            self.resistance_gains.proportional = msg.data[0]
            self.resistance_gains.integral = msg.data[1]
            self.resistance_gains.derivative = msg.data[2]
        except Exception as e:
            print(f'Error resistance: {e}')

    def const_angle_parameter_callback(self, msg):
        try:
            # self.get_logger().info(f"Received const angle parameter: {msg.data}")
            self.const_angle = msg.data[0]
        except Exception as e:
            print(f'Error const angle: {e}')

def main(args=None):
    rclpy.init(args=args)
    # muscle = Muscle()
    motor = Motor()

    try:
        rclpy.spin(motor)
    except KeyboardInterrupt:
        print('Keyboard Interrupt (SIGINT)')
    finally:
        motor.RMD.raw_motor_off()
        time.sleep(0.005)
        motor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

