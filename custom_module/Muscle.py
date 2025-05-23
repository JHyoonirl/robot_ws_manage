import numpy as np

class Muscle:
    def __init__(self):
        '''
        설멍:
        muscle의 passive component를 적용하기 위한 객체와 Muscle의 Active force를 적용하기 위한 객체
        + 현재 무릎 모사 장치의 중성 부력 상태가 아니기 때문에 중력 보상 필요
        '''
        self.angle = 0
        '''
        현재 무릎의 각도 UNIT[deg]
        '''
        self.velocity = 0
        '''
        현재 무릎의 각속도 UNIT[deg/s]
        '''
        self.neutral_torque = 0
        '''
        중력 보상을 위한 토크값 --> force scalar가 되어야 할 수도?(수학적 증명 필요)
        '''
        self.Control_mode = False
        '''
        False: Only for involuntary / True: Control with voluntary
        '''

        self.LSB_torque_constant = 35


    def M_stiffness(self, angle):
        '''
        angle : Unit[deg]
        rad_angle : Unit[rad]
        torque : Unit[Nm]
        '''
        rad_angle = np.deg2rad(angle)
        if angle >= -5 and angle  < 55:
            torque = 11*np.exp(-3.2*rad_angle)
        elif angle >= 55 and angle < 95:
            torque = -2.48*rad_angle + 2.84
        elif angle >= 95 and angle < 155:
            torque = -0.02*np.exp(2.5*rad_angle)
        else:
            torque = 0
            
        return torque

    def M_damping(self, angle, velocity):
        '''
        angle : Unit[deg]
        velocity : Unit[deg/s]
        rad_angle : Unit[rad]
        rad_velocity : Unit[rad/s]
        torque : Unit[Nm/(deg/s)]
        '''
        
        def damping_coef(angle):

            '''
            angle : Unit[deg]
            rad_angle : Unit[rad]
            coef : Unit[Nm/(deg/s)]
            '''
            rad_angle = np.deg2rad(angle)
            if angle <= 90:
                coef = -0.237 * rad_angle + 1.12
            else:
                coef = 1.6 * rad_angle ** 2 - 4.87 * rad_angle + 4.44
            return coef

        rad_velocity = np.deg2rad(velocity)
        damping_coefficient = damping_coef(angle)

        # Adjust damping coefficient based on velocity direction
        # if rad_velocity < 0:
        #     damping_coefficient = -damping_coefficient

        torque = - damping_coefficient * rad_velocity  # Use absolute value of velocity
        return torque
    
    def M_passive(self, angle, velocity) -> float:

        total_torque = self.M_stiffness(angle) + self.M_damping(angle, velocity)
        
        return float(total_torque)
    
    def torque_to_LSB(self, torque:float)-> int:
        '''
        torque constant : 0.32 Nm/A
        torque closed loop constant : 0.01 A / LSB
        LSB torque constant : 312.5 LSB/Nm
        torque : Unit[Nm]
        '''

        LSB_input = self.LSB_torque_constant * torque

        return int(LSB_input)
