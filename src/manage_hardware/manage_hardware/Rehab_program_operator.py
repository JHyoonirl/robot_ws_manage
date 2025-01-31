import numpy as np
import math
import time


class Rehab:
    def __init__(self):
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
        
        self.time_stamp = 0

        self.desired_position_end = 0
        self.desired_position_start = 0
        self.current_position = 0
        self.current_velocity = 0

        self.repeatation_memory = -1 #현재 반복 횟수
        self.protocol_start_time = 0

        self.signal = 0 ## signal: 0(현재 state 유지), 1(Generate desired trajectory), 2(Hold position)
        self.state = 0 ## state: 0(움직이지 않음), 1(Desired trajectory를 따라 움직임)), 2(현재 위치 유지), 3(운동 종료)
        self.integer_init_signal = 0 # 굳이 필요한가? 추후 확인 필요

        
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
                else:
                    self.desired_position_start = self.current_position
                    self.desired_position_end = self.theta_min

        else:
            self.desired_position_start = 0
            self.desired_position_end = 0
    

    def desired_velocity_profile_generator(self):
        '''
        Passive mode에서 시간에 따른 프로파일 생성
        '''
        time_now = time.time() - self.time_stamp
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
    
    def desired_position_trajectory_generator(self):
        '''
        Passive mode에서 desired_velocity에 의한 위치 프로파일 생성
        오일러 적분으로 진행
        '''

        pass

class Passive_mode:
    def __init__(self):
        self.rmd = None
        self.muscle = None
        self.reha = Rehab()
    def Passive_exercise(self):
        '''
        Passive mode 진행 프로토콜을 Main loop로 실행되는 부분
        '''
        pass

class Active_Resist_mode:
    def __init__(self, rmd, muscle):
        self.rmd = rmd
        self.muscle = muscle
        self.reha = Rehab()

class Active_Assistance_mode:
   
        

    def Active_exercise_Resistance(self, theta_max, theta_min, hold_time, Dtheta_max, DDtheta_time, repeatation_number, Thruster_acc):
        '''
        Passive mode 진행 프로토콜을 Main loop로 실행되는 부분
        '''
        pass

    def Active_exercise_Assistance(self, theta_max, theta_min, hold_time, Dtheta_max, DDtheta_time, repeatation_number, Thruster_acc):
        pass
