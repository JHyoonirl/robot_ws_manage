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


    def M_stiffness(self, angle):
        '''
        angle : Unit[deg]
        '''
        if angle >= -5 and angle  <= 55:
            torque = 22.5*np.exp(-0.0367*angle)
        elif angle >= 55 and angle <= 95:
            torque = -0.149*self.angle + 11.17
        elif angle >= 95 and angle <= 155:
            torque = -0.002*np.exp(0.0768*angle)
        else:
            torque = 0
            
        return torque

    def M_damping(self, angle, velocity):
        
        def damping_coef(angle):
            coef = 0.000213 * angle ** 2 - 0.0266* angle + 1.583
            return coef
        
        torque = damping_coef(angle) * velocity

        return torque
    
    def M_passive(self, angle, velocity):

        total_torque = self.M_damping(angle, velocity) + self.M_stiffness(angle)
        return total_torque
    
    def M_active(self):
        torque = 0
        return torque
