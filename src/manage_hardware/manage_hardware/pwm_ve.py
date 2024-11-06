import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from custominterface.srv import Status
from rclpy.qos import QoSProfile
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout, QSpinBox, QLabel
from threading import Thread
from datetime import datetime

class PwmClient(Node):
    def __init__(self):
        super().__init__('pwm_ve')
        self.cli = self.create_client(Status, 'pwm_server')
        qos_profile = QoSProfile(depth=10)
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.pwm_publisher = self.create_publisher(Float64, 'pwm_signal', qos_profile)
        self.pwm = 50.0  # 초기 PWM 값 설정
        self.pwm_response = False  # 서비스 응답 상태 초기화

        self.pwm_time = self.create_publisher(Float64, 'pwm_time', qos_profile)

        self.timer = self.create_timer(0.005, self.publish_pwm)

    def send_request(self, pwm_switch):
        req = Status.Request()
        req.pwm_switch = pwm_switch
        future = self.cli.call_async(req)
        future.add_done_callback(self.handle_service_response)

    def handle_service_response(self, future):
        try:
            response = future.result()
            self.pwm_response = response.pwm_result
        except Exception as e:
            self.get_logger().error(f'Service call failed {e}')

    def publish_pwm(self):
        msg = Float64()
        time_ = Float64()
        if self.pwm_response:
            msg.data = self.pwm
        else:
            msg.data = 50.0  # PWM 값을 기본값으로 재설정
        time_.data = float(datetime.now().timestamp())
        self.pwm_time.publish(time_)
        self.pwm_publisher.publish(msg)
        # self.get_logger().info('Published pwm: {0}'.format(msg.data))

class PwmApp(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.init_ui()
        
    def init_ui(self):
        layout = QVBoxLayout()
        
        self.spin_box = QSpinBox(self)
        self.spin_box.setRange(0, 100)  # PWM 범위 설정
        self.spin_box.setValue(int(self.node.pwm))  # 초기값 설정
        self.spin_box.valueChanged.connect(self.pwm_value_changed)

        self.btn_on = QPushButton('Turn ON PWM', self)
        self.btn_off = QPushButton('Turn OFF PWM', self)
        self.btn_on.clicked.connect(lambda: self.node.send_request(True))
        self.btn_off.clicked.connect(lambda: self.node.send_request(False))
        
        layout.addWidget(QLabel('Set PWM Value:'))
        layout.addWidget(self.spin_box) 
        layout.addWidget(self.btn_on)
        layout.addWidget(self.btn_off)
        
        self.setLayout(layout)
        self.setWindowTitle('PWM Controller')
        
        self.show()
        QApplication.processEvents()
        self.move(500, 500)

    def pwm_value_changed(self, value):
        self.node.pwm = float(value)
        self.node.get_logger().info(f'PWM value set to {value}')

    
def run_node(node):
    rclpy.spin(node)

def main(args=None):
    rclpy.init(args=args)
    node = PwmClient()
    node_thread = Thread(target=run_node, args=(node,))
    node_thread.start()
    
    app = QApplication(sys.argv)
    ex = PwmApp(node)
    app.exec_()
    
    node.destroy_node()
    rclpy.shutdown()
    node_thread.join()

if __name__ == '__main__':
    main()
