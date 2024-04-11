import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QPushButton
from PyQt5.QtCore import QCoreApplication
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib import animation
import numpy as np
import random
import time


global data_receive
data_receive = []

class MyMplCanvas(FigureCanvas):
    '''
    그래프 생성을 위한 초기화 부분
    '''
    def __init__(self, parent=None, width=5, height=3, dpi=200, data_num=100):
        fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = fig.add_subplot(211, xlim=(0, data_num), ylim=(-100, 100))
        self.axes.set_title('Force data')
        # self.axes.set_xlabel('data number')
        self.axes.set_ylabel('force (N)')
        self.axes2 = fig.add_subplot(212, xlim=(0, data_num), ylim=(-5, 5))
        self.axes2.set_title('Torque data')
        # self.axes2.set_xlabel('data number')
        self.axes2.set_ylabel('Torque (N*m)')

        self.compute_initial_figure()
        FigureCanvas.__init__(self, fig)
        self.setParent(parent)
    
    def compute_initial_figure(self):
        pass

class AnimationWidget(QWidget):
    def __init__(self):
        QMainWindow.__init__(self)
        
        vbox = QVBoxLayout()
        self.canvas = MyMplCanvas(self, width=10, height=8, dpi=100, data_num=200)
        vbox.addWidget(self.canvas)
        
        hbox = QHBoxLayout()
        self.start_button = QPushButton("start", self)
        self.stop_button = QPushButton("stop", self)
        self.quit_button = QPushButton("quit", self)
        self.start_button.clicked.connect(self.on_start)
        self.stop_button.clicked.connect(self.on_stop)
        self.quit_button.clicked.connect(QCoreApplication.instance().quit)
        hbox.addWidget(self.start_button)
        hbox.addWidget(self.stop_button)
        hbox.addWidget(self.quit_button)        
        vbox.addLayout(hbox)
        self.setLayout(vbox)
        self.data_num = 200

        self.x_ = np.arange(self.data_num)
        self.force_x = np.ones(self.data_num)*np.nan
        self.force_y = np.ones(self.data_num)*np.nan
        self.force_z = np.ones(self.data_num)*np.nan
        self.line1_x, = self.canvas.axes.plot(self.x_, self.force_x, animated=True, lw=2, label='force_x')
        self.line1_y, = self.canvas.axes.plot(self.x_, self.force_y, animated=True, lw=2, label='force_y')
        self.line1_z, = self.canvas.axes.plot(self.x_, self.force_z, animated=True, lw=2, label='force_z')

        self.torque_x = np.ones(self.data_num)*np.nan
        self.torque_y = np.ones(self.data_num)*np.nan
        self.torque_z = np.ones(self.data_num)*np.nan
        self.line2_x, = self.canvas.axes2.plot(self.x_, self.torque_x, animated=True, lw=2, label='torque_x')
        self.line2_y, = self.canvas.axes2.plot(self.x_, self.torque_y, animated=True, lw=2, label='torque_y')
        self.line2_z, = self.canvas.axes2.plot(self.x_, self.torque_z, animated=True, lw=2, label='torque_z')

        self.line = [self.line1_x, self.line1_y, self.line1_z, self.line2_x, self.line2_y, self.line2_z]

        # 범례 추가
        self.canvas.axes.legend()
        self.canvas.axes2.legend()
        self.canvas.figure.suptitle('FT Sensor data')


    def update_line(self, i):
        if data_receive != False:
            for data, line in zip(data_receive, self.line):
                old_y = line.get_ydata()
                new_y = np.r_[old_y[-200 + 1:], data]
                print(new_y)
                self.line[self.line.index(line)].set_ydata(new_y)
                

        return self.line

    def on_start(self):
        self.ani = animation.FuncAnimation(self.canvas.figure, self.update_line, blit=True, interval=10)
    
    def on_stop(self):
        self.ani._stop()


class QtFTsensor(Node):
    
    def __init__(self):
        super().__init__('qt_ftsensor')
        qos_profile = QoSProfile(depth=10)
        self.force_data = []
        self.torque_data = []
        data_receive = []

        self.force_receive = self.create_subscription(
            String,
            'force_data',
            self.force_sub,
            qos_profile
        )
        
        self.torque_receive = self.create_subscription(
            String,
            'torque_data',
            self.torque_sub,
            qos_profile
        )

        self.create_timer(0.01, self.data_sum)

        
        # self.timer = self.create_timer(0.01, self.publish_pwm)

    def force_sub(self, msg):
        # msg is the data which i want to receive
        # print(eval(msg.data))
        self.force_data = eval(msg.data)
        # if msg.data == 'w':
        #     self.pwm += 1
        # elif msg.data =='s':
        #     self.pwm -= 1
        # elif msg.data == 'q':
        #     self.pwm = 77
    def torque_sub(self, msg):
        # msg is the data which i want to receive
        self.torque_data = eval(msg.data)
    def data_sum(self):
        data_receive = self.force_data + self.torque_data
        # print(data_receive)


    # def publish_pwm(self):
    #     msg = String()
    #     msg.data = str(self.pwm)
    #     self.pwm_publisher.publish(msg)
    #     self.get_logger().info('Published pwm: {0}'.format(msg.data))
    #     # if self.count == 100:
    #     #     self.count = 1
    #     # else:
    #     #     self.count += 1


def main(args=None):
    rclpy.init(args=args)
    node = QtFTsensor()
    qApp = QApplication(sys.argv)
    aw = AnimationWidget()
    # aw.show()
    try:
        aw.show()
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(qApp.exec_())


if __name__ == '__main__':
    main()
    
    # aw.show()
    # sys.exit(qApp.exec_())