import numpy as np
import sympy as sp
import math
import matplotlib.pyplot as plt

class thruster_converter:
    def __init__(self):
        self.moment_arm = 0.325
        self.percentage = np.linspace(0,100,21)
        self.torque_list = np.array([-7.0538, -6.9458, -7.0074, -5.8711, -4.6310
                                      ,-3.2947, -2.2889, -1.5748, -0.8979, -0.3149, 0
                                      ,0.0808, 0.5475, 1.0392, 1.5914, 2.2437, 3.181
                                      ,4.1786, 5.3522, 5.4559, 5.4534])
        # reverse 더 큰 힘을 낼 수 있음
        self.reverse_per = np.linspace(45, 0, 1000)
        self.reverse_rpm = np.linspace(600, 4400, 1000)
        self.reverse_per_for_torque = self.percentage[2:10][::-1]
        self.reverse_torque = self.torque_list[2:10][::-1]
        # print(self.reverse_per_for_torque)
        # print(self.reverse_torque)
        self.reverse_fcn()
        
        # forward 더 작은 힘을 낼 수 있음
        self.forward_per = np.linspace(52, 100, 1000)
        self.forward_rpm = np.linspace(750, 4750, 1000)
        self.forward_per_for_torque = self.percentage[11:-2]
        self.forward_torque = self.torque_list[11:19] + 0.1
        # print(self.forward_per_for_torque)
        # print(self.forward_torque)
        

        self.forward_fcn()

    def reverse_fcn(self):

        # rad/s 에서 percentage로의 변환 다항식
        self.per_to_rpm_reverse_poly = np.polyfit(self.reverse_per, self.reverse_rpm, 1); # percentage 에서 rpm으로 변환식
        self.rpm_for_torque_reverse = np.polyval(self.per_to_rpm_reverse_poly, self.reverse_per_for_torque) # percentage(45~10)에 해당하는 rpm
        self.per_to_rpm_reverse_fcn = np.poly1d(self.per_to_rpm_reverse_poly)

        # rad/s <-> torque 변환 다항식
        self.rpm_to_torque_reverse_poly = np.polyfit(self.rpm_for_torque_reverse, self.reverse_torque, 2)
        self.rpm_to_torque_reverse_fcn = np.poly1d(self.rpm_to_torque_reverse_poly)

        # SymPy를 사용하여 역함수 계산
        x, y = sp.symbols('x y')
        self.per_to_rpm_reverse_expr = sp.lambdify(x, self.per_to_rpm_reverse_fcn(x), 'numpy')
        self.rpm_to_per_reverse_fcn = sp.solve(y - self.per_to_rpm_reverse_expr(x), x)  # 역함수 계산
        # print(self.rpm_to_per_reverse_fcn)

        self.rpm_to_torque_reverse_expr = sp.lambdify(x, self.rpm_to_torque_reverse_fcn(x), 'numpy')
        print(self.rpm_to_torque_reverse_fcn)
        self.torque_to_rpm_reverse_fcn = sp.solve(y - self.rpm_to_torque_reverse_expr(x), x)  # 역함수 계산
        print(self.torque_to_rpm_reverse_fcn)
 
    def reverse_convert_torque_to_percentage(self, torque_value):
        # Convert torque to rad/s
        x, y = sp.symbols('x y')
        rpm_value = float(sp.N(self.torque_to_rpm_reverse_fcn[1].subs(y, torque_value)))
        
        # Convert rad/s to percentage
        percentage_value = float(sp.N(self.rpm_to_per_reverse_fcn[0].subs(y, rpm_value)))
        return percentage_value
    
    def reverse_convert_torque_to_rpm(self, torque_value):
        # Convert torque to rad/s
        x, y = sp.symbols('x y')
        rpm_value = float(sp.N(self.torque_to_rpm_reverse_fcn[1].subs(y, torque_value)))
        
        # Convert rad/s to percentage
        # percentage_value = float(sp.N(self.rpm_to_per_reverse_fcn[0].subs(y, rpm_value)))
        return rpm_value

    def forward_fcn(self):

        # rad/s 에서 percentage로의 변환 다항식
        self.per_to_rpm_forward_poly = np.polyfit(self.forward_per, self.forward_rpm, 1); # percentage 에서 rpm으로 변환식
        self.rpm_for_torque_forward = np.polyval(self.per_to_rpm_forward_poly, self.forward_per_for_torque) # percentage(45~10)에 해당하는 rpm
        self.per_to_rpm_forward_fcn = np.poly1d(self.per_to_rpm_forward_poly)

        # rad/s <-> torque 변환 다항식
        self.rpm_to_torque_forward_poly = np.polyfit(self.rpm_for_torque_forward, self.forward_torque, 2)
        self.rpm_to_torque_forward_fcn = np.poly1d(self.rpm_to_torque_forward_poly)

        # SymPy를 사용하여 역함수 계산
        x, y = sp.symbols('x y')
        self.per_to_rpm_forward_expr = sp.lambdify(x, self.per_to_rpm_forward_fcn(x), 'numpy')
        self.rpm_to_per_forward_fcn = sp.solve(y - self.per_to_rpm_forward_expr(x), x)  # 역함수 계산
        # print(self.rpm_to_per_forward_fcn)
        self.rpm_to_torque_forward_expr = sp.lambdify(x, self.rpm_to_torque_forward_fcn(x), 'numpy')
        print(self.rpm_to_torque_forward_fcn)
        self.torque_to_rpm_forward_fcn = sp.solve(y - self.rpm_to_torque_forward_expr(x), x)  # 역함수 계산
        print(self.torque_to_rpm_forward_fcn)
    def forward_convert_torque_to_percentage(self, torque_value):
        # Convert torque to rad/s
        x, y = sp.symbols('x y')
        rpm_value = float(sp.N(self.torque_to_rpm_forward_fcn[1].subs(y, torque_value)))
        
        # Convert rad/s to percentage
        percentage_value = float(sp.N(self.rpm_to_per_forward_fcn[0].subs(y, rpm_value)))
        return percentage_value
    
    def forward_convert_torque_to_rpm(self, torque_value):
        # Convert torque to rad/s
        x, y = sp.symbols('x y')
        rpm_value = float(sp.N(self.torque_to_rpm_forward_fcn[1].subs(y, torque_value)))
        
        # Convert rad/s to percentage
        # percentage_value = float(sp.N(self.rpm_to_per_forward_fcn[0].subs(y, rpm_value)))
        return rpm_value
    
    def torque_to_percentage(self, torque):
        if torque < 0:
            percentage = self.reverse_convert_torque_to_percentage(torque)
                # actual_rpm.append(np.interp(torque, self.reverse_torque[::-1], self.reverse_rpm[::-1]))
        else:
            percentage = self.forward_convert_torque_to_percentage(torque)
        return percentage
    
    def torque_to_rpm(self, torque):
        if torque < 0:
            percentage = self.reverse_convert_torque_to_percentage(torque)
                # actual_rpm.append(np.interp(torque, self.reverse_torque[::-1], self.reverse_rpm[::-1]))
        else:
            percentage = self.forward_convert_torque_to_percentage(torque)
        return percentage
    
    def visualize_torque_to_rpm(self):
        torques = np.linspace(-7, 6, 100)  # Sample range of torque values
        estimated_rpm = []
        actual_rpm = []
        
        for torque in torques:
            if torque < 0:
                estimated_rpm.append(self.reverse_convert_torque_to_rpm(torque))
                
            else:
                estimated_rpm.append(self.forward_convert_torque_to_rpm(torque))
                
        plt.figure(figsize=(10, 6))
        plt.plot(self.forward_torque, self.rpm_for_torque_forward, 'r-', label='Actual Rad/s')
        plt.plot(self.reverse_torque, self.rpm_for_torque_reverse, 'g-', label='Actual Rad/s--')
        plt.plot(torques, estimated_rpm, 'b-', label='estimated Rad/s--')
        plt.title('Comparison of Actual and Estimated Rad/s Values')
        plt.xlabel('Torque (Nm)')
        plt.ylabel('Rad/s')
        plt.legend()
        plt.grid(True)
        plt.show()

def main():
    th_converter = thruster_converter()
    th_converter.visualize_torque_to_rpm()

if __name__ == '__main__':
    main()