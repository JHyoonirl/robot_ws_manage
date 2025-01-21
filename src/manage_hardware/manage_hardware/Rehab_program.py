import numpy as np


class Rehab:
    def __init__(self):
        self.theta_max = 0
        self.theta_min = 0
        self.hold_time = 0
        self.Dtheta_max = 0
        self.DDtheta_time = 0
        self.repeatation_number = 0
        self.Thruster_acc = 0 # thrust의 변화률


        self.repeatation_memory = 0
        self.protocol_start_time = 0

        self.signal = 0 ## signal: 0(현재 state 유지), 1(Generate desired trajectory), 2(Hold position)
        self.state = 0 ## state: 0(움직이지 않음), 1(Desired trajectory를 따라 움직임)), 2(현재 위치 유지), 3(운동 종료)

        
    def signal_generator(self, theta_max, theta_min, hold_time, Dtheta_max, DDtheta_time, repeatation_number, Thruster_acc):
        pass
    def desired_velocity_profile_generator(self, theta_max, theta_min, hold_time, Dtheta_max, DDtheta_time):
        pass
    
    def desired_position_trajectory_generator(self, theta_max, theta_min, hold_time, Dtheta_max, DDtheta_time):
        pass


    def Passive_exercise(self, theta_max, theta_min, hold_time, Dtheta_max, DDtheta_time, repeatation_number, Thruster_acc):
        pass

    def Active_exercise_Resistance(self, theta_max, theta_min, hold_time, Dtheta_max, DDtheta_time, repeatation_number, Thruster_acc):
        pass

    def Active_exercise_Assistance(self, theta_max, theta_min, hold_time, Dtheta_max, DDtheta_time, repeatation_number, Thruster_acc):
        pass