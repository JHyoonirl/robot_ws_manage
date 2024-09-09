import sys
import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3
from PyQt5.QtWidgets import QApplication, QVBoxLayout, QWidget, QPushButton
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import matplotlib.pyplot as plt
from collections import deque
from threading import Thread
from rclpy.qos import QoSProfile

# from custominterface.srv import Status

class VisualSensor(Node):
    def __init__(self, canvas):
        super().__init__('FT_graph')
        self.canvas = canvas
        self.ax = canvas.figure.subplots(2, 1)
        qos_profile = QoSProfile(depth=10)

        self.subscription_force = self.create_subscription(
            Vector3,
            'force_data',
            self.listen_force,
            10)
        self.subscription_torque = self.create_subscription(
            Vector3,
            'torque_data',
            self.listen_torque,
            10)
        self.pwm_subscribed = self.create_subscription(
            Float64,
            'pwm_signal',
            self.listen_pwm,
            qos_profile)
        
        # self.cli = self.create_client(Status, 'sensor_server')
        self.force_data = deque(maxlen=100)
        self.torque_data = deque(maxlen=100)
        self.timer = self.create_timer(0.01, self.update_graph)
        self.timer = self.create_timer(0.01, self.update_data)

    def listen_force(self, msg):
        current_time = self.get_clock().now().nanoseconds*10^6
        self.force_data.append((current_time, msg))
        self.force_tmp = msg
        # self.get_logger().info('Subscribed force: {0}'.format(msg))

    def listen_torque(self, msg):
        current_time = self.get_clock().now().nanoseconds*10^6
        self.torque_data.append((current_time, msg))
        self.torque_tmp = msg
        # self.get_logger().info('Subscribed torque: {0}'.format(msg))s

    def listen_pwm(self, msg):
        self.get_logger().info('Subscribed pwm: {0}'.format(msg.data))
        self.pwm_tmp = msg.data

    def update_graph(self):
        if self.force_data:
            times, forces = zip(*[(t, (data.x, data.y, data.z)) for t, data in self.force_data])
            self.ax[0].clear()
            self.ax[0].plot(times, [f[0] for f in forces], label='X', color='red')
            self.ax[0].plot(times, [f[1] for f in forces], label='Y', color='green')
            self.ax[0].plot(times, [f[2] for f in forces], label='Z', color='blue')
            self.ax[0].legend(loc=(0.85, 0.85))
            self.ax[0].set_title('Force Data')
            self.ax[0].set_xlabel('Time (ns)')
            self.ax[0].set_ylabel('Force')
        if self.torque_data:
            times, torques = zip(*[(t, (data.x, data.y, data.z)) for t, data in self.torque_data])
            self.ax[1].clear()
            self.ax[1].plot(times, [t[0] for t in torques], label='X', color='red')
            self.ax[1].plot(times, [t[1] for t in torques], label='Y', color='green')
            self.ax[1].plot(times, [t[2] for t in torques], label='Z', color='blue')
            self.ax[1].legend()
            self.ax[1].set_title('Torque Data')
            self.ax[1].set_xlabel('Time (ns)')
            self.ax[1].set_ylabel('Torque')
        self.canvas.draw()

    def update_data(self):
        try:
            self.get_logger().info('Subscribed: {0}, {1}, {2}'.format(self.pwm_tmp, self.force_tmp, self.torque_tmp))
        except:
            pass

class App(QWidget):
    def __init__(self):
        super().__init__()
        self.init_ui()

    def init_ui(self):
        self.layout = QVBoxLayout(self)
        # Creating the FigureCanvas object correctly
        self.canvas = FigureCanvas(plt.Figure())
        self.layout.addWidget(self.canvas)
        self.node = VisualSensor(self.canvas)
        self.ros_thread = Thread(target=self.ros_spin)
        self.ros_thread.start()
        self.show()

    def ros_spin(self):
        rclpy.spin(self.node)

    def closeEvent(self, event):
        self.node.destroy_node()
        rclpy.shutdown()
        self.ros_thread.join()

def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)

    ex = App()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
