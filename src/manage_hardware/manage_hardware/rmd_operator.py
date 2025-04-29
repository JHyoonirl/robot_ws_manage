import sys
import rclpy
from std_msgs.msg import Float64, Float64MultiArray
from rclpy.node import Node
from rclpy.qos import QoSProfile

import warnings

from manage_hardware.RMD_custom import RMD# 가정한 모듈과 클래스 이름
import threading
import time
import json
from Muscle import Muscle



class Motor(Node):
    def __init__(self):
        
        '''
        코드 구성
        1) 전체 구성 자체는 ROS2 노드로 구성되어 있음.
        2) RMD motor 동작을 위해서 사용하는 torque/speed control 제어 입력은 외부 ROS2 topic을 통해서 subscribe함.
        --> torque/speed control 제어는 동시에 켜질 수 없도록 함.
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

        self.MOTOR_ID = 1

        # RMD Motor Setup
        self.RMD = RMD(0x141)
        self.RMD.setup('slcan', usb_port)

        # Motor Control Thread
        self.motor_senser_thread = threading.Thread(target=self.sensing, daemon=True)
        self.motor_controller_thread = threading.Thread(target=self.controller, daemon=True)

        # Motor Safety Range (ROM)
        self.ROM_SAFE_UPPER = 120
        self.ROM_SAFE_LOWER = 40

        # Publishers
        self.motor_info_publisher = self.create_publisher(Float64MultiArray, 'Motor_info', self.qos_profile)

        # Motor State Variables
        self.voltage = 0.0         # Motor voltage
        self.temperature = 0.0     # Motor temperature
        self.torque_current = 0.0  # Motor torque current
        self.motor_velocity = 0.0  # Motor angular velocity
        self.motor_angle = 0.0     # Motor angle
        self.knee_angle = 0.0      # Knee joint angle

        # Motor Control Mode
        self.motor_main_control_switch = 0
        '''
        0. off
        1. on
        '''

        self.motor_control_mode = 0
        '''
        1. extension constant velocity
        2. extension constant acc
        3. passive exercise
        4. resistance exercise
        5. assistance exercise
        6. position move
        '''

        self.control_switch_status = 0
        '''
        0. stop
        1. start
        '''

        self.muscle_passive_component_switch = 1
        '''
        모터에 무릎의 passive component 추가

        0. muscle off
        1. muscle on
        '''

        ## 모터 반시계 회전(-)은 Extension
        ## 모터 시계 회전(+)은 flexion

        # Subscriptions
        self.motor_control_mode_subscriber = self.create_subscription(
            Float64MultiArray, 'Motor_control_mode', self.motor_control_mode_callback, self.qos_profile
        )
        self.motor_hydrodynamic_control_subscriber = self.create_subscription(
            Float64MultiArray, 'Motor_hydrodynamic_control_info', self.motor_hydrodynamic_callback, self.qos_profile
        )
        self.motor_resistance_control_subscriber = self.create_subscription(
            Float64MultiArray, 'Motor_resistance_control_info', self.motor_resistance_callback, self.qos_profile
        )
        self.motor_assistance_control_subscriber = self.create_subscription(
            Float64MultiArray, 'Motor_assistance_control_info', self.motor_assistance_callback, self.qos_profile
        )

        # Control Switch Inputs
        
        self.motor_control_mode_info = [0.0, 0.0, 0.0, 0.0]  
        '''
        motor control mode setting parameter

        [main control switch, control mode switch, control switch, muscle switch]
        '''
        
        # Hydrodynamic experimental setup
        self.desired_velocity = 0.0
        self.desired_acceleration = 0.0
        self.test_rom_upper = 0.0
        self.test_rom_lower = 0.0

        # motor exercise control parameter setup
        
        # common parameter
        self.exercise_desired_angle = 0.0
        self.knee_angle_imu = 0.0
        
        # resistance parameter
        self.resistance_pid = [0.0, 0.0, 0.0] # resistance PID gain
        self.resistance_state = 0.0 # resistance hold time

        # assistance parameter
        self.assistance_pid = [0.0, 0.0, 0.0] # assistance PID gain
        self.assistance_state = 0.0 # assistance hold time

        with open("custom_json/motor_info.json", "r") as fr:
            data = json.load(fr)
        # print(data)

        self.Kp = float(data["kp"])
        self.Ki = float(data["ki"])
        self.Kd = float(data["kd"])

        self.Neutral_angle = float(data["neutral_angle"]) # 무릎의 0도에 해당하는 motor encoder의 각도
        self.knee_angle = 0 # 무릎 각도 = encoder angle - self.Neutral_angle

        status_1 = self.init_motor()
        status_2 = self.init_acceleration()
        if status_1 == True and status_2 == True:
            self.motor_senser_thread.start()
            self.motor_controller_thread.start()    
    
    def init_motor(self):
        # 스케일링된 값들을 바이트 배열로 변환하여 전달
        response = self.RMD.read_pid()
        init_data = response.data
        self.kp_cur = init_data[2]
        self.ki_cur = init_data[3]
        self.kp_vel = init_data[4]
        self.ki_vel = init_data[5]
        self.kp_pos = init_data[6]
        self.ki_pos = init_data[7]

        print(self.kp_cur, self.ki_cur)
        print(self.kp_vel, self.ki_vel)
        print(self.kp_pos, self.ki_pos)
        init_data = [
            self.RMD.byteArray(self.kp_cur, 1) ,
            self.RMD.byteArray(self.ki_cur, 1),
            self.RMD.byteArray(self.kp_vel, 1),
            self.RMD.byteArray(self.ki_vel, 1),
            self.RMD.byteArray(self.kp_pos, 1),
            self.RMD.byteArray(self.ki_pos, 1)
        ]
        # 바이트 배열을 하나의 플랫 리스트로 변환
        flat_data = [item for sublist in init_data for item in sublist]
        self.RMD.write_pid_ram(flat_data)
        print('initialized Motor')
        return True

    def init_acceleration(self):
        for i in range(4):
            index = self.RMD.byteArray(i, 1)
            response = self.RMD.read_acceleration(index)
            data = response.data
            acc = int.from_bytes(data[4:8], byteorder='little', signed=True)
            input_acc = self.RMD.byteArray(60000, 4)
            self.RMD.write_acceleration(index, input_acc)
        print('initialized Motor acc')
        return True
    
    def sensing(self):
        while True:
        
            self.voltage, self.temperature, self.torque_current, self.motor_velocity, self.motor_angle, error = self.RMD.status_motor() # motor raw status 호출

            self.knee_angle = self.motor_angle - self.Neutral_angle # encoder angle - neutral angle

            self.motor_info = Float64MultiArray()
            self.motor_info.data = [self.motor_main_control_switch, self.voltage, self.torque_current, self.motor_velocity, self.motor_angle, self.knee_angle]
            
            self.motor_info_publisher.publish(self.motor_info)

    def controller(self):
        while True:
            
            if self.knee_angle > self.knee_safe_rom_upper or self.knee_angle < self.knee_safe_rom_lower :
                self.motor_main_control_switch = 0

            
            if self.motor_main_control_switch == 0:
                self.motor_off()
                print('motor off')
                time.sleep(1)
                continue

            else:  
                if self.motor_control_mode == 1: # 등각속도 운동
                    self.motor_constant_velocity()
                elif self.motor_control_mode == 2: # 등각가속도 운동
                    self.motor_constant_acc()
                elif self.motor_control_mode == 3: # passive exercise
                    self.motor_passive()
                elif self.motor_control_mode == 4: # resistance exercise
                    self.motor_resistance()
                elif self.motor_control_mode == 5: # assistance exercise
                    self.motor_assistance()
                elif self.motor_control_mode == 6: # angle move
                    self.motor_angle_move()
            self.past_time = time.time()


    def motor_off(self):
        self.RMD.raw_motor_off()

    def motor_constant_velocity(self):
        # motor position control을 위한 함수
        # option 1: 정해진 구간 안에서 desired position을 설정
        try:
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
            '''
            # self.dt = time.time() - self.past_time
            # sine_time = time.time() - self.RMD_timer_sinusoidal
            # self.Desired_angle = self.amplitude * math.sin(self.period * sine_time) + self.pos_offset

            # self.pos_error = self.Desired_angle - self.knee_angle
            # print(self.pos_error)

            # # Proportional term
            # proportional = self.Kp * self.pos_error

            # # Integral term
            # self.pos_error_integral += self.pos_error * self.dt
            # integral = self.Ki * self.pos_error_integral

            # # Derivative term
            # derivative = self.Kd * (self.pos_error - self.pos_error_prev) / self.dt

            # # Update previous error
            # self.pos_error_prev = self.pos_error

            # # Calculate the control output
            # output = proportional + integral + derivative
            # if abs(output) > self.torque_threshold:
            #     if output > 0:
            #         output = self.torque_threshold
            #     elif output < 0:
            #         output = - self.torque_threshold
            # self.torque_out = output
            # temperature, torque, speed, angle = self.RMD.torque_closed_loop(int(output))
            # self.get_logger().info('rmd torque, speed, angle: {0}'.format(torque, speed, angle))
            '''
            self.dt = time.time() - self.past_time
            sine_time = time.time() - self.RMD_timer_sine
            if sine_time < 3:
                sine_time = 0


                self.Desired_angle = self.amplitude_sine

                if self.Desired_angle < self.pos_offset_sine:
                    self.Desired_angle = self.pos_offset_sine

                
                self.get_logger().info('rself.state_sine: {0}'.format(self.state_sine))

                self.pos_error = self.Desired_angle - self.knee_angle
                # print(self.pos_error)

                if sine_time < 3:
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
            elif sine_time > 3:
                acc = - self.period_sine
                vel_for_constant_acc_input = 100*(sine_time - 3)*acc
                sine_time = sine_time - 3
                
                '''
                1 torque : 1 deg /s^2
                '''
                if self.knee_angle < self.pos_offset_sine:
                    self.state_sine = 2
                
                if self.knee_angle > self.pos_offset_ramp and self.state_sine ==1:
                    _a = self.RMD.speed_closed_loop(int(vel_for_constant_acc_input))
                    self.get_logger().info('rself.state_ramp: {0}'.format(_a))
                    
                elif self.state_sine == 2:
                    _a = self.RMD.speed_closed_loop(int(0))
                    vel_for_constant_acc_input = 0.0
                     

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


                self.Desired_angle = self.amplitude_ramp

                if self.Desired_angle < self.pos_offset_ramp:
                    self.Desired_angle = self.pos_offset_ramp

                
                

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
                


            elif ramp_time > 3:
                try:
                    ramp_time = ramp_time - 3
                    # print(ramp_time)
                    velocity = - 100*self.velocity_ramp
                    '''
                    1 velocity : 1 deg/s
                    '''
                    if self.knee_angle < self.pos_offset_ramp:
                        self.state_ramp = 2
                    
                    if self.knee_angle > self.pos_offset_ramp and self.state_ramp ==1:
                        _a = self.RMD.speed_closed_loop(int(velocity))
                        
                        
                        
                    elif self.state_ramp ==2:
                        _a = self.RMD.speed_closed_loop(int(0))
                        
                except Exception as e:
                    print(f'Error3: {e}')

            # sign = -1

            
            # self.get_logger().info('rmd torque, speed, angle: {0}'.format(torque, speed, angle))
                        

        except Exception as e:
            print(f'Error3: {e}')

    # def muscle_passive(self):
    #     theta = self.angle
    #     dtheta = self.velocity


    #     pass

def run_node(node):
    rclpy.spin(node)


def main(args=None):
    rclpy.init(args=args)
    motor = Motor()

    try:
        thread = threading.Thread(target=run_node, args=(motor, ), daemon=True)
        thread.start()
    except KeyboardInterrupt:
        print('Keyboard Interrupt (SIGINT)')
    finally:
        motor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    
