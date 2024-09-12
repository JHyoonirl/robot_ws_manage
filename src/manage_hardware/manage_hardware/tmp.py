import sys
import pandas as pd
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QLineEdit, QGroupBox, QTimer)
from PyQt5.QtCore import QTimer, QTime
import numpy as np

class Form(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node

        # 데이터를 저장할 배열 초기화
        self.data = []
        self.saving = False

        # 데이터 업데이트를 주기적으로 호출할 QTimer 설정
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_data)
        self.timer.start(100)  # 100ms마다 업데이트

        # 메인 레이아웃 설정
        main_layout = QVBoxLayout()

        # PWM 값 박스
        pwm_group = QGroupBox('PWM')
        pwm_layout = QVBoxLayout()
        self.label_pwm = QLabel('0.0', self)
        pwm_layout.addWidget(self.label_pwm)
        pwm_group.setLayout(pwm_layout)
        
        # Force X 값 박스
        force_x_group = QGroupBox('Force X')
        force_x_layout = QVBoxLayout()
        self.label_force_x = QLabel('0.0', self)
        force_x_layout.addWidget(self.label_force_x)
        force_x_group.setLayout(force_x_layout)

        # Force Y 값 박스
        force_y_group = QGroupBox('Force Y')
        force_y_layout = QVBoxLayout()
        self.label_force_y = QLabel('0.0', self)
        force_y_layout.addWidget(self.label_force_y)
        force_y_group.setLayout(force_y_layout)

        # Force Z 값 박스
        force_z_group = QGroupBox('Force Z')
        force_z_layout = QVBoxLayout()
        self.label_force_z = QLabel('0.0', self)
        force_z_layout.addWidget(self.label_force_z)
        force_z_group.setLayout(force_z_layout)

        # torque X 값 박스
        torque_x_group = QGroupBox('Torque X')
        torque_x_layout = QVBoxLayout()
        self.label_torque_x = QLabel('0.0', self)
        torque_x_layout.addWidget(self.label_torque_x)
        torque_x_group.setLayout(torque_x_layout)

        # torque Y 값 박스
        torque_y_group = QGroupBox('Torque X')
        torque_y_layout = QVBoxLayout()
        self.label_torque_y = QLabel('0.0', self)
        torque_y_layout.addWidget(self.label_torque_y)
        torque_y_group.setLayout(torque_y_layout)

        # torque Z 값 박스
        torque_z_group = QGroupBox('Torque X')
        torque_z_layout = QVBoxLayout()
        self.label_torque_z = QLabel('0.0', self)
        torque_z_layout.addWidget(self.label_torque_z)
        torque_z_group.setLayout(torque_z_layout)

        # 가로로 배치
        horizontal_layout = QHBoxLayout()
        horizontal_layout.addWidget(pwm_group)
        horizontal_layout.addWidget(force_x_group)
        horizontal_layout.addWidget(force_y_group)
        horizontal_layout.addWidget(force_z_group)
        horizontal_layout.addWidget(torque_x_group)
        horizontal_layout.addWidget(torque_y_group)
        horizontal_layout.addWidget(torque_z_group)

        # 메인 레이아웃에 가로 레이아웃 추가
        main_layout.addLayout(horizontal_layout)

        # 버튼 레이아웃
        button_layout = QHBoxLayout()

        # Data Load 버튼
        self.button_load = QPushButton('Data Load', self)
        self.button_load.clicked.connect(self.data_load)
        button_layout.addWidget(self.button_load)

        # Data Stop 버튼
        self.button_stop = QPushButton('Data Stop', self)
        self.button_stop.clicked.connect(self.data_stop)
        button_layout.addWidget(self.button_stop)

        # Data Save 버튼
        self.button_save = QPushButton('Data Save', self)
        self.button_save.clicked.connect(self.data_save)
        button_layout.addWidget(self.button_save)

        # 파일 이름 입력 공간
        self.file_name_input = QLineEdit(self)
        self.file_name_input.setPlaceholderText('Enter file name')
        button_layout.addWidget(self.file_name_input)

        # 버튼 레이아웃 추가
        main_layout.addLayout(button_layout)

        self.setLayout(main_layout)

        self.setWindowTitle('Data Save')
        self.resize(800, 200)
        self.show()

    def update_data(self):
        # 데이터가 업데이트될 때마다 GUI를 갱신합니다.
        if hasattr(self.node, 'pwm'):
            self.label_pwm.setText(f'{self.node.pwm:.2f}')
        if hasattr(self.node, 'force_x') and hasattr(self.node, 'force_y') and hasattr(self.node, 'force_z'):
            self.label_force_x.setText(f'{self.node.force_x:.2f}')
            self.label_force_y.setText(f'{self.node.force_y:.2f}')
            self.label_force_z.setText(f'{self.node.force_z:.2f}')
            self.label_torque_x.setText(f'{self.node.torque_x:.2f}')
            self.label_torque_y.setText(f'{self.node.torque_y:.2f}')
            self.label_torque_z.setText(f'{self.node.torque_z:.2f}')

        # 데이터 로드 중이면 데이터를 저장
        if self.saving:
            current_time = QTime.currentTime().toString("hh:mm:ss")
            pwm = self.node.pwm if hasattr(self.node, 'pwm') else 0.0
            force_x = self.node.force_x if hasattr(self.node, 'force_x') else 0.0
            force_y = self.node.force_y if hasattr(self.node, 'force_y') else 0.0
            force_z = self.node.force_z if hasattr(self.node, 'force_z') else 0.0
            torque_x = self.node.torque_x if hasattr(self.node, 'torque_x') else 0.0
            torque_y = self.node.torque_y if hasattr(self.node, 'torque_y') else 0.0
            torque_z = self.node.torque_y if hasattr(self.node, 'torque_y') else 0.0

            # 데이터를 배열에 추가
            self.data.append([current_time, pwm, force_x, force_y, force_z, torque_x, torque_y, torque_z])

    def data_load(self):
        # 데이터를 저장 시작
        self.saving = True

    def data_stop(self):
        # 데이터를 저장 정지
        self.saving = False

    def data_save(self):
        # 입력한 파일 이름을 사용하여 데이터 저장
        file_name = self.file_name_input.text()
        if file_name:
            df = pd.DataFrame(self.data, columns=['Time', 'PWM', 'Force_X', 'Force_Y', 'Force_Z', 'Torque_X', 'Torque_Y', 'Torque_Z'])
            df.to_excel(f'{file_name}.xlsx', index=False)
            print(f'Data saved to {file_name}.xlsx')
        else:
            print("Please enter a file name.")

# 노드 클래스 예시 (임시 데이터 제공)
class Node:
    def __init__(self):
        self.pwm = 0.5
        self.force_x = 1.0
        self.force_y = 1.5
        self.force_z = 2.0
        self.torque_x = 0.1
        self.torque_y = 0.2
        self.torque_z = 0.3

if __name__ == '__main__':
    app = QApplication(sys.argv)
    node = Node()  # 예시 노드 데이터
    form = Form(node)
    sys.exit(app.exec_())
