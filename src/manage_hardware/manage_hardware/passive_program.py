import time

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
        self.current_position = self.rehab.imu_current_angle
        '''
        나중에 센서값을 변경해야 함!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        
        '''
        self.current_velocity = 0

        self.repeatation_memory = 0 
        '''
        짝수: Extension
        홀수: Flexion
        '''

        self.signal = 0
        '''
        signal: 0(현재 state 유지), 1(Generate desired trajectory), 2(Hold position)
        '''
        
        self.state = 0
        '''
        ## state: 0(최초 움직임), 1(Desired trajectory를 따라 움직임), 2(Holding), 3(운동 종료)
        '''

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
        # print(self.state, self.repeatation_memory)
        

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

        # self.desired_angle += self.dtheta_desired_current * self.dt
        

        distance = abs(self.desired_position_end - self.desired_position_start)
        # constant_vel_time = distance/self.rehab.dThetaMax - self.rehab.acc_time
        constant_vel_time = distance/self.rehab.dThetaMax - self.rehab.dThetaMax/self.rehab.ddThetaMax
        acc_vel_time = self.rehab.dThetaMax/self.rehab.ddThetaMax
        t_fall = t_rise = acc_vel_time
        v_max = self.rehab.dThetaMax
        t_high = constant_vel_time
        t_total = t_high + t_rise*2

        if self.desired_position_end - self.desired_position_start > 0: # 전진
            sign =  1
        else:
            sign =  - 1

        t = time.time() - self.time_stamp
        if self.state == 1:
            if t <= t_rise:  # 가속 구간
                des_dis = (v_max / (2 * t_rise)) * t**2
            elif t <= t_rise + t_high:  # 등속 구간
                x_rise = (v_max / (2 * t_rise)) * t_rise**2  # 상승 구간 끝 위치
                des_dis =  x_rise + v_max * (t - t_rise)
            elif t <= t_total:  # 감속 구간
                x_rise = (v_max / (2 * t_rise)) * t_rise**2  # 상승 구간 끝 위치
                x_high = x_rise + v_max * t_high  # 등속 구간 끝 위치
                t_fall_start = t_rise + t_high
                des_dis =  x_high + (v_max * (t - t_fall_start)) - (v_max / (2 * t_fall)) * (t - t_fall_start)**2
            else:
                des_dis = distance
        else:
            des_dis = distance
        self.desired_angle = self.desired_position_start + sign * des_dis
        
        
        
    def Passive_exercise(self):
        '''
        Passive mode 진행 프로토콜을 Main loop로 실행되는 부분
        '''
        self.current_position = self.rehab.imu_current_angle
        
        self.control_generator()
        self.desired_position_generator()
        self.desired_velocity_profile_generator()
        self.desired_position_trajectory_generator()
        # self.current_position = self.desired_angle
        # print(self.desired_position_start, self.desired_position_end)
        return self.desired_angle, self.dtheta_desired_current
def main():
    passive_mode = Passive_mode()
    # th_converter.visualize_torque_to_rpm()

if __name__ == '__main__':
    main()
    