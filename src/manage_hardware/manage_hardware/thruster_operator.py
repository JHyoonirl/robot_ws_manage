import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from custominterface.srv import Status
from rclpy.qos import QoSProfile
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout, QSpinBox, QLabel
from threading import Thread
from datetime import datetime

class ThrusterClient(Node):
    def __init__(self):
        super().__init__('Thruster_operator')
        self.cli = self.create_client(Status, 'Thruster_server')
        qos_profile = QoSProfile(depth=10)
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.Thruster_publisher = self.create_publisher(Float64, 'Thruster_signal', qos_profile)
        self.Thruster = 50.0  # 초기 Thruster 값 설정
        self.Thruster_response = False  # 서비스 응답 상태 초기화

        # 실험을 위해 주석 처리
        self.Thruster_time = self.create_publisher(Float64, 'Thruster_time', qos_profile)

        # 실험을 위해 주석 처리
        self.timer = self.create_timer(0.005, self.publish_Thruster)

    def send_request(self, Thruster_switch):
        req = Status.Request()
        req.Thruster_switch = Thruster_switch
        future = self.cli.call_async(req)
        future.add_done_callback(self.handle_service_response)

    def handle_service_response(self, future):
        try:
            response = future.result()
            self.Thruster_response = response.Thruster_result
        except Exception as e:
            self.get_logger().error(f'Service call failed {e}')

    def publish_Thruster(self):
        msg = Float64()
        time_ = Float64()
        if self.Thruster_response:
            msg.data = self.Thruster
        else:
            msg.data = 50.0  # Thruster 값을 기본값으로 재설정
        time_.data = float(datetime.now().timestamp())
        self.Thruster_time.publish(time_)
        self.Thruster_publisher.publish(msg)
        # self.get_logger().info('Published Thruster: {0}'.format(msg.data))

class ThrusterApp(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.init_ui()
        
    def init_ui(self):
        layout = QVBoxLayout()
        
        self.spin_box = QSpinBox(self)
        self.spin_box.setRange(0, 100)  # Thruster 범위 설정
        self.spin_box.setValue(int(self.node.Thruster))  # 초기값 설정
        self.spin_box.valueChanged.connect(self.Thruster_value_changed)

        self.btn_on = QPushButton('Turn ON Thruster', self)
        self.btn_off = QPushButton('Turn OFF Thruster', self)
        self.btn_on.clicked.connect(lambda: self.node.send_request(True))
        self.btn_off.clicked.connect(lambda: self.node.send_request(False))
        
        layout.addWidget(QLabel('Set Thruster Value:'))
        layout.addWidget(self.spin_box) 
        layout.addWidget(self.btn_on)
        layout.addWidget(self.btn_off)
        
        self.setLayout(layout)
        self.setWindowTitle('Thruster Controller')
        
        self.show()
        QApplication.processEvents()
        self.move(500, 500)

    def Thruster_value_changed(self, value):
        self.node.Thruster = float(value)
        self.node.get_logger().info(f'Thruster value set to {value}')

    
def run_node(node):
    rclpy.spin(node)

def main(args=None):
    rclpy.init(args=args)
    node = ThrusterClient()
    node_thread = Thread(target=run_node, args=(node,))
    node_thread.start()
    
    app = QApplication(sys.argv)
    ex = ThrusterApp(node)
    app.exec_()
    
    node.destroy_node()
    rclpy.shutdown()
    node_thread.join()

if __name__ == '__main__':
    main()
