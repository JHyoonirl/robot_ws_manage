import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from custominterface.srv import Status
from rclpy.qos import QoSProfile
from PyQt5.QtWidgets import *
from PyQt5.QtCore import QTimer, Qt
from threading import Thread
from datetime import datetime
from PyQt5 import uic
from thruster_torque_converter import thruster_converter

form_class = uic.loadUiType("UI/thruster.ui")[0]

class thrusterClient(Node):
    def __init__(self):
        super().__init__('thruster_operator')
        self.cli = self.create_client(Status, 'thruster_server')
        qos_profile = QoSProfile(depth=10)
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.thruster_publisher = self.create_publisher(Float64, 'thruster_signal', qos_profile)
        self.thruster = 50.0  # 초기 thruster 값 설정
        self.torque = 0
        self.thruster_response = False  # 서비스 응답 상태 초기화
        self.control_mode = 0 # 0: thruster direct, 1: thruster controlled by torque, 2: neutral
        self.torque_converter = thruster_converter()
        # 실험을 위해 주석 처리
        self.thruster_time = self.create_publisher(Float64, 'thruster_time', qos_profile)

        # 실험을 위해 주석 처리
        self.timer = self.create_timer(0.005, self.publish_thruster)

    def send_request(self, thruster_switch):
        req = Status.Request()
        req.thruster_switch = thruster_switch
        future = self.cli.call_async(req)
        future.add_done_callback(self.handle_service_response)

    def handle_service_response(self, future):
        try:
            response = future.result()
            self.thruster_response = response.thruster_result
        except Exception as e:
            self.get_logger().error(f'Service call failed {e}')

    def publish_thruster(self):
        msg = Float64()
        time_ = Float64()
        if self.thruster_response:
            if self.control_mode == 0:
                msg.data = self.thruster
            elif self.control_mode == 1:
                msg.data = self.torque_converter.torque_to_percentage(self.torque)
                if msg.data > 100:
                    msg.data = 100
                elif msg.data  < 0:
                    msg.data = 0
            else: # self.control_mode == 2
                msg.data = 50.0
        else:
            msg.data = 50.0  # thruster 값을 기본값으로 재설정
        # time_.data = float(datetime.now().timestamp())
        # self.thruster_time.publish(time_)
        # print(self.control_mode)
        self.thruster_publisher.publish(msg)
        # self.get_logger().info('Published thruster: {0}'.format(msg.data))

class thrusterApp(QMainWindow, form_class):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setupUi(self)
        self.percentage = 50
        self.torque = 0
        self.init_ui()
        
    def init_ui(self):
        # layout = QVBoxLayout()
        
        # self.thruster_percentage_spinbox
        self.thruster_percentage_spinbox.setRange(0, 100)  # thruster 범위 설정
        self.thruster_percentage_spinbox.setSingleStep(0.05)
        self.thruster_percentage_spinbox.setValue(int(self.node.thruster))  # 초기값 설정
        self.thruster_percentage_spinbox.valueChanged.connect(self.thruster_percentage_changed)
        

        self.thruster_torque_spinbox.setRange(-7, 6)  # thruster 범위 설정
        self.thruster_torque_spinbox.setSingleStep(0.05)
        self.thruster_torque_spinbox.setValue(int(self.node.torque))  # 초기값 설정
        self.thruster_torque_spinbox.valueChanged.connect(self.thruster_torque_changed)

        
        self.percentage_checkBox.stateChanged.connect(self.control_thruster_changed)
        self.torque_checkBox.stateChanged.connect(self.control_torque_changed)
        
        
        self.button_on.clicked.connect(lambda: self.node.send_request(True))
        self.button_off.clicked.connect(lambda: self.node.send_request(False))
        self.percentage_btn.clicked.connect(self.percentage_changed)
        self.torque_btn.clicked.connect(self.torque_changed)
        
        self.show()
        QApplication.processEvents()
        # self.move(500, 500)

    def control_thruster_changed(self, state):
        # state == 0 : unchecked, state == 2 : checked
        if state == Qt.Checked:
            self.node.control_mode = 0

            if self.torque_checkBox.checkState() == 2:
                self.torque_checkBox.toggle()
                # print(self._RMD.position_control_check_status, self._RMD.sinusoidal_control_check_status)

    def control_torque_changed(self, state):
        # state == 0 : unchecked, state == 2 : checked
        if state == Qt.Checked:
            self.node.control_mode = 1

            if self.percentage_checkBox.checkState() == 2:
                self.percentage_checkBox.toggle()
                # print(self._RMD.position_control_check_status, self._RMD.sinusoidal_control_check_status)
        
    def thruster_percentage_changed(self, value):
        self.percentage = float(value)

    def thruster_torque_changed(self, value):
        self.torque = float(value)

    def percentage_changed(self):
        self.node.thruster = float(self.percentage)

    def torque_changed(self):
        self.node.torque = float(self.torque)
        

    
def run_node(node):
    rclpy.spin(node)

def main(args=None):
    rclpy.init(args=args)
    node = thrusterClient()
    node_thread = Thread(target=run_node, args=(node,))
    
    
    app = QApplication(sys.argv)
    ex = thrusterApp(node)
    node_thread.start()
    app.exec_()
    
    node.destroy_node()
    rclpy.shutdown()
    node_thread.join()

if __name__ == '__main__':
    main()
