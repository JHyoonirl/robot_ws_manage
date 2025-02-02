import numpy as np
import math
import time
import threading


class Rehab:
    def __init__(self):
        '''
        재활 운동에 사용되는 기본적인 안전 정보를 가지고 있는 객체
        '''

        self.theta_max = 0
        self.theta_min = 0
        self.hold_time = 0
        self.Dtheta_max = 0
        self.DDtheta_time = 0
        self.repeatation_number = 0 #설정 반복 횟수
        self.Thruster_acc = 0 # thrust의 변화률
        self.ACCELERATION_TIME = 1
        self.dtheta_desired_current = 0
        self.dt = 0.01

        '''
        운동을 진행하는 프로토콜과 관련된 method와 정보를 가지고 있는 객체
        '''
        self.passive_mode = Passive_mode(self)
        self.active_resist_mode = Active_Resistance_mode(self)
        self.active_assist_mode = Active_Assistance_mode(self)

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
        self.Rehab = rehab
        self.theta_max = self.Rehab.theta_max
        self.theta_min = self.Rehab.theta_min
        self.hold_time = self.Rehab.hold_time
        self.Dtheta_max = self.Rehab.Dtheta_max
        self.DDtheta_time = self.Rehab.DDtheta_time
        self.repeatation_number = self.Rehab.repeatation_number #설정 반복 횟수
        self.Thruster_acc = self.Rehab.Thruster_acc # thrust의 변화률
        self.ACCELERATION_TIME = self.Rehab.ACCELERATION_TIME
        self.dtheta_desired_current = self.Rehab.dtheta_desired_current
        self.dt = self.Rehab.dt # dt는 real time으로 해야 함으로 time.time() 사용 필요
        self.past_time = time.time()
        self.time_stamp = 0

        self.stop_event = threading.Event()

        self.desired_position_end = 0
        self.desired_position_start = 0

        self.desired_position_trajectory_currnet = 0
        self.current_position = 0
        self.current_velocity = 0

        self.repeatation_memory = -1 #현재 반복 횟수
        self.protocol_start_time = 0

        self.signal = 0 ## signal: 0(현재 state 유지), 1(Generate desired trajectory), 2(Hold position)
        self.state = 0 ## state: 0(움직이지 않음), 1(Desired trajectory를 따라 움직임)), 2(현재 위치 유지), 3(운동 종료)
        self.integer_init_signal = 0 # 굳이 필요한가? 추후 확인 필요

    def reset(self):
        # self.Rehab = Rehab()
        self.theta_max = self.Rehab.theta_max
        self.theta_min = self.Rehab.theta_min
        self.hold_time = self.Rehab.hold_time
        self.Dtheta_max = self.Rehab.Dtheta_max
        self.DDtheta_time = self.Rehab.DDtheta_time
        self.repeatation_number = self.Rehab.repeatation_number #설정 반복 횟수
        self.Thruster_acc = self.Rehab.Thruster_acc # thrust의 변화률
        self.ACCELERATION_TIME = self.Rehab.ACCELERATION_TIME
        self.dtheta_desired_current = self.Rehab.dtheta_desired_current

    def signal_generator(self):
        '''
        Passive mode에서 시간에 따른 signal 생성
        '''

        if self.signal == 1 or self.state == 2:
            self.signal = 0
        self.integer_init_signal = 0

        if self.signal == 0 and self.state == 1 and abs(self.desired_position_end - self.current_position) < 1 and abs(self.current_velocity) < 0.05:
            self.signal = 2
        
        if self.signal == 0 and self.state == 2 and time.time() > self.time_stamp + self.hold_time:
            self.signal = 1
            self.integer_init_signal = 1

        if self.signal == 1:
            self.repeatation_memory += 1
            self.time_stamp = time.time()
            self.state = 1
        elif self.signal == 2:
            self.time_stamp = time.time()
            self.state = 2
        
        if self.repeatation_memory > self.repeatation_number:
            self.state = 3

    def desired_position_generator(self):
        if self.repeatation_memory > -1:
            if self.signal == 1:
                if self.repeatation_memory % 2 == 0:
                    self.desired_position_start = self.current_position
                    self.desired_position_end = self.theta_max
                    self.desired_position_trajectory_currnet = self.desired_position_end
                else:
                    self.desired_position_start = self.current_position
                    self.desired_position_end = self.theta_min
                    self.desired_position_trajectory_currnet = self.desired_position_end

        else:
            self.desired_position_start = 0
            self.desired_position_end = 0
    

    def desired_velocity_profile_generator(self):
        '''
        Passive mode에서 시간에 따른 프로파일 생성
        '''
        time_now = time.time() - self.time_stamp
        self.dt = time.time() - self.past_time
        acc_calculated = 0
        distance = abs(self.desired_position_end - self.desired_position_start)
        constant_vel_time = distance/self.Dtheta_max - self.ACCELERATION_TIME
        Acc = self.Dtheta_max/self.ACCELERATION_TIME
        if self.repeatation_memory > -1 and self.state  == 1:
            if time_now < self.ACCELERATION_TIME:
                acc_calculated = Acc
            elif time_now < constant_vel_time + self.ACCELERATION_TIME:
                acc_calculated = 0
            elif time_now < constant_vel_time + 2*self.ACCELERATION_TIME:
                acc_calculated = -Acc
            else:
                acc_calculated = 0
            
            if self.desired_position_end < self.desired_position_start:
                self.dtheta_desired_current -= acc_calculated*self.dt
            else:
                self.dtheta_desired_current += acc_calculated*self.dt
        
        if self.state == 2:
            self.dtheta_desired_current = 0
        self.past_time = time.time() # 따로 loop에서 진행해야 할 수 도?
    
    def desired_position_trajectory_generator(self):
        '''
        Passive mode에서 desired_velocity에 의한 위치 프로파일 생성
        오일러 적분으로 진행
        '''
        self.desired_position_trajectory_currnet += self.dtheta_desired_current * self.dt

    def Passive_exercise(self):
        '''
        Passive mode 진행 프로토콜을 Main loop로 실행되는 부분
        '''
        while not self.stop_event.is_set():
            self.signal_generator()
            self.desired_position_generator()
            self.desired_velocity_profile_generator()
            self.desired_position_trajectory_generator()
            time.sleep(self.dt)
    

class Active_Resistance_mode:
    def __init__(self, rehab):
        self.Rehab = rehab
        self.theta_max = self.Rehab.theta_max
        self.theta_min = self.Rehab.theta_min
        self.hold_time = self.Rehab.hold_time
        self.Dtheta_max = self.Rehab.Dtheta_max
        self.DDtheta_time = self.Rehab.DDtheta_time
        self.repeatation_number = self.Rehab.repeatation_number #설정 반복 횟수
        self.Thruster_acc = self.Rehab.Thruster_acc # thrust의 변화률
        self.ACCELERATION_TIME = self.Rehab.ACCELERATION_TIME
        self.dtheta_desired_current = self.Rehab.dtheta_desired_current
        self.dt = self.Rehab.dt # dt는 real time으로 해야 함으로 time.time() 사용 필요

    def reset(self):
        # self.Rehab = Rehab()
        self.theta_max = self.Rehab.theta_max
        self.theta_min = self.Rehab.theta_min
        self.hold_time = self.Rehab.hold_time
        self.Dtheta_max = self.Rehab.Dtheta_max
        self.DDtheta_time = self.Rehab.DDtheta_time
        self.repeatation_number = self.Rehab.repeatation_number #설정 반복 횟수
        self.Thruster_acc = self.Rehab.Thruster_acc # thrust의 변화률
        self.ACCELERATION_TIME = self.Rehab.ACCELERATION_TIME
        self.dtheta_desired_current = self.Rehab.dtheta_desired_current

    def Active_resistance_exercise(self):
        pass

class Active_Assistance_mode:
    def __init__(self, rehab):
        self.Rehab = rehab
        self.theta_max = self.Rehab.theta_max
        self.theta_min = self.Rehab.theta_min
        self.hold_time = self.Rehab.hold_time
        self.Dtheta_max = self.Rehab.Dtheta_max
        self.DDtheta_time = self.Rehab.DDtheta_time
        self.repeatation_number = self.Rehab.repeatation_number #설정 반복 횟수
        self.Thruster_acc = self.Rehab.Thruster_acc # thrust의 변화률
        self.ACCELERATION_TIME = self.Rehab.ACCELERATION_TIME
        self.dtheta_desired_current = self.Rehab.dtheta_desired_current
        self.dt = self.Rehab.dt # dt는 real time으로 해야 함으로 time.time() 사용 필요

    def reset(self):
        # self.Rehab = Rehab()
        self.theta_max = self.Rehab.theta_max
        self.theta_min = self.Rehab.theta_min
        self.hold_time = self.Rehab.hold_time
        self.Dtheta_max = self.Rehab.Dtheta_max
        self.DDtheta_time = self.Rehab.DDtheta_time
        self.repeatation_number = self.Rehab.repeatation_number #설정 반복 횟수
        self.Thruster_acc = self.Rehab.Thruster_acc # thrust의 변화률
        self.ACCELERATION_TIME = self.Rehab.ACCELERATION_TIME
        self.dtheta_desired_current = self.Rehab.dtheta_desired_current


    def Active_assistance_exercise(self, theta_max, theta_min, hold_time, Dtheta_max, DDtheta_time, repeatation_number, Thruster_acc):
        pass
def main(args=None):
    rehab_info = Rehab()
    passive_mode = Passive_mode(rehab_info)
    active_resist_mode = Active_Resistance_mode(rehab_info)


if __name__ == '__main__':
    main()
