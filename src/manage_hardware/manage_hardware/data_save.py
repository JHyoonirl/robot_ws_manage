import sys
import rclpy
import pandas as pd
from rclpy.node import Node
from std_msgs.msg import Float64
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Vector3
from PyQt5.QtWidgets import QApplication, QVBoxLayout, QWidget, QLabel, QLineEdit, QGroupBox, QPushButton, QHBoxLayout
from PyQt5.QtCore import QTimer, QDateTime
from threading import Thread, Lock
from collections import deque

class DataSaver(Node):
    def __init__(self):
        super().__init__('pwm_botton')
        self.qos_profile = QoSProfile(depth=10)
        self.pwm = 0.0
        self.force_x = self.force_y = self.force_z = 0.0
        self.torque_x = self.torque_y = self.torque_z = 0.0
        
        # self.prev_pwm = None
        self.prev_force = (None, None, None)
        self.prev_torque = (None, None, None)

        self.pwm_sub = self.create_subscription(
            Float64,
            'pwm_signal',
            self.pwm_subscriber,
            self.qos_profile)
        
        self.force_sub = self.create_subscription(
            Vector3,
            'force_data',
            self.force_subscriber,
            self.qos_profile)
        
        self.torque_sub = self.create_subscription(
            Vector3,
            'torque_data',
            self.torque_subscriber,
            self.qos_profile)

        # Lock for thread safety when accessing node data
        self.data_lock = Lock()

    def pwm_subscriber(self, msg):
        with self.data_lock:
            self.pwm = msg.data

    def force_subscriber(self, msg):
        with self.data_lock:
            self.force_x, self.force_y, self.force_z = msg.x, msg.y, msg.z

    def torque_subscriber(self, msg):
        with self.data_lock:
            self.torque_x, self.torque_y, self.torque_z = msg.x, msg.y, msg.z

class Form(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.data = deque()  # Using deque for more efficient append operations
        self.saving = False

        # QTimer to periodically update the data
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_data)
        self.timer.start(1)  # Update every 10ms

        # Main layout setup
        main_layout = QVBoxLayout()

        # Group boxes for PWM, Force, and Torque values
        self.label_pwm = QLabel('0.0', self)
        self.label_force_x = QLabel('0.0', self)
        self.label_force_y = QLabel('0.0', self)
        self.label_force_z = QLabel('0.0', self)
        self.label_torque_x = QLabel('0.0', self)
        self.label_torque_y = QLabel('0.0', self)
        self.label_torque_z = QLabel('0.0', self)

        # Group box layout
        horizontal_layout = QHBoxLayout()
        horizontal_layout.addWidget(self.create_group_box('PWM', self.label_pwm))
        horizontal_layout.addWidget(self.create_group_box('Force X', self.label_force_x))
        horizontal_layout.addWidget(self.create_group_box('Force Y', self.label_force_y))
        horizontal_layout.addWidget(self.create_group_box('Force Z', self.label_force_z))
        horizontal_layout.addWidget(self.create_group_box('Torque X', self.label_torque_x))
        horizontal_layout.addWidget(self.create_group_box('Torque Y', self.label_torque_y))
        horizontal_layout.addWidget(self.create_group_box('Torque Z', self.label_torque_z))
        main_layout.addLayout(horizontal_layout)

        # Buttons
        button_layout = QHBoxLayout()
        self.button_load = QPushButton('Data Load', self)
        self.button_load.clicked.connect(self.data_load)
        self.button_stop = QPushButton('Data Stop', self)
        self.button_stop.clicked.connect(self.data_stop)
        self.button_save = QPushButton('Data Save', self)
        self.button_save.clicked.connect(self.data_save)
        self.file_name_input = QLineEdit(self)
        self.file_name_input.setPlaceholderText('Enter file name')

        button_layout.addWidget(self.button_load)
        button_layout.addWidget(self.button_stop)
        button_layout.addWidget(self.button_save)
        button_layout.addWidget(self.file_name_input)
        main_layout.addLayout(button_layout)

        self.setLayout(main_layout)
        self.setWindowTitle('Data Save')
        self.resize(800, 200)
        self.show()

    def create_group_box(self, title, label):
        group_box = QGroupBox(title)
        layout = QVBoxLayout()
        layout.addWidget(label)
        group_box.setLayout(layout)
        return group_box

    def update_data(self):
        with self.node.data_lock:
            self.label_pwm.setText(f'{self.node.pwm:.2f}')
            self.label_force_x.setText(f'{self.node.force_x:.2f}')
            self.label_force_y.setText(f'{self.node.force_y:.2f}')
            self.label_force_z.setText(f'{self.node.force_z:.2f}')
            self.label_torque_x.setText(f'{self.node.torque_x:.2f}')
            self.label_torque_y.setText(f'{self.node.torque_y:.2f}')
            self.label_torque_z.setText(f'{self.node.torque_z:.2f}')

        # Save data if the saving flag is set and values have changed
        if self.saving:
            current_time = QDateTime.currentDateTime().toString("hh:mm:ss.zzz")  # Time with ms precision
            pwm = self.node.pwm
            force = (self.node.force_x, self.node.force_y, self.node.force_z)
            torque = (self.node.torque_x, self.node.torque_y, self.node.torque_z)

            # Check if values are the same as the previous saved values
            if (force != self.node.prev_force or 
                torque != self.node.prev_torque):
            
                # Append data to deque
                self.data.append([current_time, pwm, *force, *torque])

                # Update previous values
                # self.node.prev_pwm = pwm
                self.node.prev_force = force
                self.node.prev_torque = torque

    def data_load(self):
        self.saving = True

    def data_stop(self):
        self.saving = False

    def data_save(self):
        file_name = self.file_name_input.text()
        if file_name:
            df = pd.DataFrame(self.data, columns=['Time', 'PWM', 'Force_X', 'Force_Y', 'Force_Z', 'Torque_X', 'Torque_Y', 'Torque_Z'])
            df.to_excel(f'{file_name}.xlsx', index=False)
            print(f'Data saved to {file_name}.xlsx')
        else:
            print("Please enter a file name.")

def run_node(node):
    rclpy.spin(node)

def main(args=None):
    rclpy.init(args=args)
    node = DataSaver()
    node_thread = Thread(target=run_node, args=(node,))
    node_thread.start()

    app = QApplication(sys.argv)
    ex = Form(node)
    app.exec_()

    node.destroy_node()
    rclpy.shutdown()
    node_thread.join()

if __name__ == '__main__':
    main()
