from __future__ import annotations
from typing import TYPE_CHECKING

from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QLabel, QWidget, QDial, QPushButton, QTextBrowser, QCheckBox, QTextEdit
from PyQt5.QtCore import QTimer, Qt, QThread
from PyQt5 import uic

if TYPE_CHECKING:
    from src.manage_hardware.manage_hardware.rehab_operator import Rehab_program

import time
import json
import sys


class RehabWindow(QMainWindow):
    def __init__(self, rehab: Rehab_program):
        QMainWindow.__init__(self)
        self.rehab = rehab

        self.ui = uic.loadUi('UI/rehab_gui.ui', self)

        self.move(0,1000)
        self.initUI()
        time.sleep(0.5)


        ### 타이머 설정 ###
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_data)
        
        self.timer.start(100)  # 100ms 간격으로 업데이트

        self.show()

    def initUI(self):
        
        # textbrowser 위젲
        self.angle_text = self.findChild(QTextBrowser, 'angle_data')
        self.velocity_text = self.findChild(QTextBrowser, 'velocity_data')
        self.motor_power_enabled_status_text = self.findChild(QTextBrowser, 'motor_power_enabled_status')
        self.motor_control_active_text = self.findChild(QTextBrowser, 'motor_control_active')
        self.thruster_power_enabled_status_text = self.findChild(QTextBrowser, 'thruster_power_enabled_status')
        self.thruster_control_active_text = self.findChild(QTextBrowser, 'thruster_control_active')
        self.gui_power_enabled_status_text = self.findChild(QTextBrowser, 'gui_power_enabled_status')
        self.gui_control_active_text = self.findChild(QTextBrowser, 'gui_control_active')

        
        # 체크 박스 정의
        self.control_on_off_check = self.findChild(QCheckBox, 'control_on_off_check')

        self.control_on_off_check.stateChanged.connect(self.control_on_off_changed)

        # 체크박스 이름과 연결 함수 딕셔너리 정의
        self.checkboxes = {
            'hydrodynamic_constant_velocity_check': {
                'widget': self.findChild(QCheckBox, 'hydrodynamic_constant_velocity_check'),
                'handler': self.hydrodynamic_constant_velocity_checked,
                'status': False
            },
            'hydrodynamic_omega_check': {
                'widget': self.findChild(QCheckBox, 'hydrodynamic_omega_check'),
                'handler': self.hydrodynamic_omega_checked,
                'status': False
            },
            'const_angle_move_check': {
                'widget': self.findChild(QCheckBox, 'const_angle_move_check'),
                'handler': self.const_angle_move_checked,
                'status': False
            },
            'muscle_passive_component_check': {
                'widget': self.findChild(QCheckBox, 'muscle_passive_component_check'),
                'handler': self.muscle_passive_component_checked,
                'status': False
            },
            'exercise_passive_check': {
                'widget': self.findChild(QCheckBox, 'exercise_passive_check'),
                'handler': self.exercise_passive_checked,
                'status': False
            },
            'exercise_assistance_check': {
                'widget': self.findChild(QCheckBox, 'exercise_assistance_check'),
                'handler': self.exercise_assistance_checked,
                'status': False
            },
            'exercise_resistance_check': {
                'widget': self.findChild(QCheckBox, 'exercise_resistance_check'),
                'handler': self.exercise_resistance_checked,
                'status': False
            },
        }

        # connect는 여기서만 한 번!
        for info in self.checkboxes.values():
            if info['widget'] is not None:
                info['widget'].stateChanged.connect(info['handler'])

        self.control_off()

        # buttons 정의

        self.system_quit_btn = self.findChild(QPushButton, 'system_quit_btn')
        self.power_on_off_btn = self.findChild(QPushButton, 'power_on_off_btn')
        self.parameter_setting_btn = self.findChild(QPushButton, 'parameter_setting_btn')
        self.parameter_save_btn = self.findChild(QPushButton, 'parameter_save_btn')

        self.system_quit_btn.clicked.connect(self.system_quit_btn_clicked)
        self.power_on_off_btn.clicked.connect(self.power_on_off_btn_clicked)
        self.parameter_setting_btn.clicked.connect(self.parameter_setting_btn_clicked)
        self.parameter_save_btn.clicked.connect(self.parameter_save_btn_clicked)

        # 버튼 그룹별 정의
        self.button_groups = {
            'hydro': {
                'hydrodynamic_test_setting_btn': self.hydrodynamic_test_setting_btn_clicked,
                'hydrodynamic_test_start_btn': self.hydrodynamic_test_start_btn_clicked,
                'hydrodynamic_test_stop_btn': self.hydrodynamic_test_stop_btn_clicked,
            },
            'const_angle': {
                'const_angle_setting_btn': self.const_angle_setting_btn_clicked,
                'const_angle_start_btn': self.const_angle_start_btn_clicked,
                'const_angle_stop_btn': self.const_angle_stop_btn_clicked,
            },
            'muscle': {
                'muscle_start_btn': self.muscle_start_btn_clicked,
                'muscle_stop_btn': self.muscle_stop_btn_clicked,
            },
            'exercise': {
                'exercise_setting_btn': self.exercise_setting_btn_clicked,
                'exercise_start_btn': self.exercise_start_btn_clicked,
                'exercise_stop_btn': self.exercise_stop_btn_clicked,
            }
        }

        # 위젯 객체 저장
        self.button_widgets = {}

        # findChild + connect
        for group_name, btn_dict in self.button_groups.items():
            for btn_name, handler in btn_dict.items():
                btn = self.findChild(QPushButton, btn_name)
                if btn:
                    btn.clicked.connect(handler)
                    self.button_widgets[btn_name] = btn

        self.btn_off()

        # QTextEdit 그룹별 정의
        self.textedit_groups = {
            'motor_setting': {
                'rom_safe_upper': None,
                'rom_safe_lower': None,
                'perpendicular_angle': None,
            },
            'hydro': {
                'test_velocity_input': None,
                'test_omega_input': None,
                'test_p_input': None,
                'test_i_input': None,
                'test_d_input': None,
                'test_rom_upper_input': None,
                'test_rom_lower_input': None,
                'test_hold_time_input': None,
            },
            'const_angle': {
                'const_angle_input': None,
                'const_angle_p_input': None,
                'const_angle_i_input': None,
                'const_angle_d_input': None,
            },
            'exercise': {
                'exercise_desired_velocity': None,
                'exercise_desired_acceleration': None,
                'exercise_repeatation_number': None,
                'exercise_hold_time': None,
                'exercise_rom_upper': None,
                'exercise_rom_lower': None,
            },
            'passive': {
                'passive_p_input': None,
                'passive_i_input': None,
                'passive_d_input': None,
            },
            'assistance': {
                'assistance_gain_k_input': None,
                'assistance_p_input': None,
                'assistance_i_input': None,
                'assistance_d_input': None,
                'assistance_muscle_p_input': None,
                'assistance_muscle_i_input': None,
                'assistance_muscle_d_input': None,
            },
            'resistance': {
                'resistance_moment_input': None,
                'resistance_muscle_p_input': None,
                'resistance_muscle_i_input': None,
                'resistance_muscle_d_input': None,
            },
        }

        # 각 QTextEdit 위젯 findChild로 바인딩
        for group in self.textedit_groups.values():
            for name in group:
                widget = self.findChild(QTextEdit, name)
                group[name] = widget

        self.QtextEdit_init()

        
    
    def QtextEdit_init(self):
        # 각 QTextEdit 위젯에 초기값 설정
        self.textedit_groups['motor_setting']['rom_safe_upper'].setPlainText(str(self.rehab.motor_setting_dict['rom_safe_upper']))
        self.textedit_groups['motor_setting']['rom_safe_lower'].setPlainText(str(self.rehab.motor_setting_dict['rom_safe_lower']))
        self.textedit_groups['motor_setting']['perpendicular_angle'].setPlainText(str(self.rehab.motor_setting_dict['perpendicular_angle']))

        self.textedit_groups['hydro']['test_velocity_input'].setPlainText(str(self.rehab.hydro_dict['test_velocity_input']))
        self.textedit_groups['hydro']['test_omega_input'].setPlainText(str(self.rehab.hydro_dict['test_omega_input']))
        self.textedit_groups['hydro']['test_p_input'].setPlainText(str(self.rehab.hydro_dict['test_p_input']))
        self.textedit_groups['hydro']['test_i_input'].setPlainText(str(self.rehab.hydro_dict['test_i_input']))
        self.textedit_groups['hydro']['test_d_input'].setPlainText(str(self.rehab.hydro_dict['test_d_input']))
        self.textedit_groups['hydro']['test_rom_upper_input'].setPlainText(str(self.rehab.hydro_dict['test_rom_upper_input']))
        self.textedit_groups['hydro']['test_rom_lower_input'].setPlainText(str(self.rehab.hydro_dict['test_rom_lower_input']))
        self.textedit_groups['hydro']['test_hold_time_input'].setPlainText(str(self.rehab.hydro_dict['test_hold_time_input']))
        
        self.textedit_groups['exercise']['exercise_desired_velocity'].setPlainText(str(self.rehab.exercise_para_dict['desired_velocity']))
        self.textedit_groups['exercise']['exercise_desired_acceleration'].setPlainText(str(self.rehab.exercise_para_dict['desired_acceleration']))
        self.textedit_groups['exercise']['exercise_repeatation_number'].setPlainText(str(self.rehab.exercise_para_dict['repeatation_number']))
        self.textedit_groups['exercise']['exercise_hold_time'].setPlainText(str(self.rehab.exercise_para_dict['hold_time']))
        self.textedit_groups['exercise']['exercise_rom_upper'].setPlainText(str(self.rehab.exercise_para_dict['rom_upper']))
        self.textedit_groups['exercise']['exercise_rom_lower'].setPlainText(str(self.rehab.exercise_para_dict['rom_lower']))

        self.textedit_groups['passive']['passive_p_input'].setPlainText(str(self.rehab.passive_para_dict['passive_p']))
        self.textedit_groups['passive']['passive_i_input'].setPlainText(str(self.rehab.passive_para_dict['passive_i']))
        self.textedit_groups['passive']['passive_d_input'].setPlainText(str(self.rehab.passive_para_dict['passive_d']))

        self.textedit_groups['assistance']['assistance_gain_k_input'].setPlainText(str(self.rehab.assistance_para_dict['assistance_gain_k']))
        self.textedit_groups['assistance']['assistance_p_input'].setPlainText(str(self.rehab.assistance_para_dict['assistance_p']))
        self.textedit_groups['assistance']['assistance_i_input'].setPlainText(str(self.rehab.assistance_para_dict['assistance_i']))
        self.textedit_groups['assistance']['assistance_d_input'].setPlainText(str(self.rehab.assistance_para_dict['assistance_d']))
        self.textedit_groups['assistance']['assistance_muscle_p_input'].setPlainText(str(self.rehab.assistance_para_dict['assistance_muscle_p']))
        self.textedit_groups['assistance']['assistance_muscle_i_input'].setPlainText(str(self.rehab.assistance_para_dict['assistance_muscle_i']))
        self.textedit_groups['assistance']['assistance_muscle_d_input'].setPlainText(str(self.rehab.assistance_para_dict['assistance_muscle_d']))

        self.textedit_groups['resistance']['resistance_moment_input'].setPlainText(str(self.rehab.resistance_para_dict['resistance_moment']))
        self.textedit_groups['resistance']['resistance_muscle_p_input'].setPlainText(str(self.rehab.resistance_para_dict['resistance_muscle_p']))
        self.textedit_groups['resistance']['resistance_muscle_i_input'].setPlainText(str(self.rehab.resistance_para_dict['resistance_muscle_i']))
        self.textedit_groups['resistance']['resistance_muscle_d_input'].setPlainText(str(self.rehab.resistance_para_dict['resistance_muscle_d']))

        self.textedit_groups['const_angle']['const_angle_p_input'].setPlainText(str(self.rehab.const_angle_para_dict['const_angle_p']))
        self.textedit_groups['const_angle']['const_angle_i_input'].setPlainText(str(self.rehab.const_angle_para_dict['const_angle_i']))
        self.textedit_groups['const_angle']['const_angle_d_input'].setPlainText(str(self.rehab.const_angle_para_dict['const_angle_d']))

    def control_on_off_changed(self, state):
        if state == Qt.Checked:
            for info in self.checkboxes.values():
                info['widget'].setEnabled(True)
            self.control_on()
        else:
            self.control_off()

    def control_on(self):
        for info in self.checkboxes.values():
            if info['widget'] is not None:
                info['widget'].setEnabled(True)

    def control_off(self):
        for info in self.checkboxes.values():
            if info['widget'] is not None:
                info['widget'].setDisabled(True)

    def hydrodynamic_constant_velocity_checked(self, state):
        # state == 0 : unchecked, state == 2 : checked
        if state == Qt.Checked:
            
            self.enable_btn_group('hydro')
            self.disable_btn_group('const_angle')
            self.rehab.exercise_info_dict['control_mode'] = 1

            for name, info in self.checkboxes.items():                
                if name == 'hydrodynamic_constant_velocity_check':
                    info['status'] = True
                else:
                    if info['widget'].checkState() == 2:
                        info['widget'].toggle()
                    info['status'] = False
                
        else:
            for name, info in self.checkboxes.items():
                if name == 'hydrodynamic_constant_velocity_check':
                    info['status'] = False
            self.rehab.exercise_info_dict['control_mode'] = 0
            self.disable_btn_group('hydro')
    
    def hydrodynamic_omega_checked(self, state):
        if state == Qt.Checked:
            
            self.enable_btn_group('hydro')
            self.disable_btn_group('const_angle')
            self.rehab.exercise_info_dict['control_mode'] = 2

            for name, info in self.checkboxes.items():                
                if name == 'hydrodynamic_omega_check':
                    info['status'] = True
                else:
                    if info['widget'].checkState() == 2:
                        info['widget'].toggle()
                    info['status'] = False
                
        else:
            for name, info in self.checkboxes.items():
                if name == 'hydrodynamic_omega_check':
                    info['status'] = False
            self.rehab.exercise_info_dict['control_mode'] = 0
            self.disable_btn_group('hydro')

    def const_angle_move_checked(self, state):
        if state == Qt.Checked:
            
            self.disable_btn_group('hydro')
            self.enable_btn_group('const_angle')
            self.rehab.exercise_info_dict['control_mode'] = 6

            for name, info in self.checkboxes.items():                
                if name == 'const_angle_move_check':
                    info['status'] = True
                else:
                    if info['widget'].checkState() == 2:
                        info['widget'].toggle()
                    info['status'] = False
                
        else:
            for name, info in self.checkboxes.items():
                if name == 'const_angle_move_check':
                    info['status'] = False
            self.rehab.exercise_info_dict['control_mode'] = 0
            self.disable_btn_group('const_angle')

    def exercise_passive_checked(self, state):
        if state == Qt.Checked:
            self.enable_btn_group('exercise')
            self.disable_btn_group('hydro')
            self.disable_btn_group('const_angle')

            self.rehab.exercise_info_dict['control_mode'] = 3
            for name, info in self.checkboxes.items():                
                if name == 'exercise_passive_check':
                    info['status'] = True
                else:
                    if info['widget'].checkState() == 2:
                        info['widget'].toggle()
                    info['status'] = False

        else:
            for name, info in self.checkboxes.items():
                if name == 'exercise_passive_check':
                    info['status'] = False
            self.rehab.exercise_info_dict['control_mode'] = 0
            self.disable_btn_group('exercise')

    def exercise_assistance_checked(self, state):
        if state == Qt.Checked:
            self.enable_btn_group('exercise')
            self.disable_btn_group('hydro')
            self.disable_btn_group('const_angle')

            self.rehab.exercise_info_dict['control_mode'] = 4
            for name, info in self.checkboxes.items():                
                if name == 'exercise_assistance_check':
                    info['status'] = True
                else:
                    if info['widget'].checkState() == 2:
                        info['widget'].toggle()
                    info['status'] = False

        else:
            for name, info in self.checkboxes.items():
                if name == 'exercise_assistance_check':
                    info['status'] = False
            self.rehab.exercise_info_dict['control_mode'] = 0
            self.disable_btn_group('exercise')
    
    def exercise_resistance_checked(self, state):
        if state == Qt.Checked:
            self.enable_btn_group('exercise')
            self.disable_btn_group('hydro')
            self.disable_btn_group('const_angle')

            self.rehab.exercise_info_dict['control_mode'] = 5
            for name, info in self.checkboxes.items():                
                if name == 'exercise_resistance_check':
                    info['status'] = True
                else:
                    if info['widget'].checkState() == 2:
                        info['widget'].toggle()
                    info['status'] = False

        else:
            for name, info in self.checkboxes.items():
                if name == 'exercise_resistance_check':
                    info['status'] = False
            self.rehab.exercise_info_dict['control_mode'] = 0
            self.disable_btn_group('exercise')

    def muscle_passive_component_checked(self, state):
        if state == Qt.Checked:
            self.rehab.exercise_info_dict['muscle_passive_component_switch'] = 1
            self.enable_btn_group('muscle')
            for name, info in self.checkboxes.items():                
                if name == 'muscle_passive_component_check':
                    info['status'] = True

        else:
            self.rehab.exercise_info_dict['muscle_passive_component_switch'] = 0
            self.disable_btn_group('muscle')
            for name, info in self.checkboxes.items():                
                if name == 'muscle_passive_component_check':
                    info['status'] = False

    def system_quit_btn_clicked(self):
        self.rehab.exercise_info_dict['power_enabled'] == 0
        time.sleep(0.5)
        sys.exit()
             
    def power_on_off_btn_clicked(self):
        if self.rehab.exercise_info_dict['power_enabled'] == 1:
            self.rehab.exercise_info_dict['power_enabled'] = 0
        else:
            self.rehab.exercise_info_dict['power_enabled'] = 1

    def parameter_setting_btn_clicked(self):
        motor_setting_dict = self.textedit_groups['motor_setting']

        try:
            self.rehab.motor_setting_dict['rom_safe_upper'] = float(motor_setting_dict['rom_safe_upper'].toPlainText())
            self.rehab.motor_setting_dict['rom_safe_lower'] = float(motor_setting_dict['rom_safe_lower'].toPlainText())
            self.rehab.motor_setting_dict['perpendicular_angle'] = float(motor_setting_dict['perpendicular_angle'].toPlainText())

        except Exception as e:
            print(f"[RMD 설정 실패] {e}")

    def parameter_save_btn_clicked(self):
        

        data = {
                'exercise_parameter': {
                    'desired_velocity': self.rehab.exercise_para_dict['desired_velocity'],
                    'desired_acceleration': self.rehab.exercise_para_dict['desired_acceleration'],
                    'repeatation_number': self.rehab.exercise_para_dict['repeatation_number'],
                    'hold_time': self.rehab.exercise_para_dict['hold_time'],
                    'rom_upper': self.rehab.exercise_para_dict['rom_upper'],
                    'rom_lower': self.rehab.exercise_para_dict['rom_lower'],
                },
                'passive_parameter': {
                    'passive_p': self.rehab.passive_para_dict['passive_p'],
                    'passive_i': self.rehab.passive_para_dict['passive_i'],
                    'passive_d': self.rehab.passive_para_dict['passive_d'],
                },
                'assistance_parameter': {
                    'assistance_gain_k': self.rehab.assistance_para_dict['assistance_gain_k'],
                    'assistance_p': self.rehab.assistance_para_dict['assistance_p'],
                    'assistance_i': self.rehab.assistance_para_dict['assistance_i'],
                    'assistance_d': self.rehab.assistance_para_dict['assistance_d'],
                    'assistance_muscle_p': self.rehab.assistance_para_dict['assistance_muscle_p'],
                    'assistance_muscle_i': self.rehab.assistance_para_dict['assistance_muscle_i'],
                    'assistance_muscle_d': self.rehab.assistance_para_dict['assistance_muscle_d'],
                },
                'resistance_parameter': {
                    'resistance_moment': self.rehab.resistance_para_dict['resistance_moment'],
                    'resistance_muscle_p': self.rehab.resistance_para_dict['resistance_muscle_p'],
                    'resistance_muscle_i': self.rehab.resistance_para_dict['resistance_muscle_i'],
                    'resistance_muscle_d': self.rehab.resistance_para_dict['resistance_muscle_d'],
                },
                'motor_setting_parameter': {
                    'rom_safe_upper': self.rehab.motor_setting_dict['rom_safe_upper'],
                    'rom_safe_lower': self.rehab.motor_setting_dict['rom_safe_lower'],
                    'perpendicular_angle': self.rehab.motor_setting_dict['perpendicular_angle'],
                },
                'hydrodynamic_parameter': {
                    'test_velocity_input': self.rehab.hydro_dict['test_velocity_input'],
                    'test_omega_input': self.rehab.hydro_dict['test_omega_input'],
                    'test_p_input': self.rehab.hydro_dict['test_p_input'],
                    'test_i_input': self.rehab.hydro_dict['test_i_input'],
                    'test_d_input': self.rehab.hydro_dict['test_d_input'],
                    'test_rom_upper_input': self.rehab.hydro_dict['test_rom_upper_input'],
                    'test_rom_lower_input': self.rehab.hydro_dict['test_rom_lower_input'],
                    'test_hold_time_input': self.rehab.hydro_dict['test_hold_time_input'],
                },
                'const_angle_parameter': {
                    'const_angle_p': self.rehab.const_angle_para_dict['const_angle_p'],
                    'const_angle_i': self.rehab.const_angle_para_dict['const_angle_i'],
                    'const_angle_d': self.rehab.const_angle_para_dict['const_angle_d'],
                }
            }


        with open("custom_json/rehab.json", "w") as fw:
            json.dump(data, fw, indent=4)


    def hydrodynamic_test_setting_btn_clicked(self):
        hydro_dict = self.textedit_groups['hydro']
        try:
            self.rehab.hydro_dict['test_velocity_input'] = float(hydro_dict['test_velocity_input'].toPlainText())
            self.rehab.hydro_dict['test_omega_input'] = float(hydro_dict['test_omega_input'].toPlainText())
            self.rehab.hydro_dict['test_p_input'] = float(hydro_dict['test_p_input'].toPlainText())
            self.rehab.hydro_dict['test_i_input'] = float(hydro_dict['test_i_input'].toPlainText())
            self.rehab.hydro_dict['test_d_input'] = float(hydro_dict['test_d_input'].toPlainText())
            self.rehab.hydro_dict['test_rom_upper_input'] = float(hydro_dict['test_rom_upper_input'].toPlainText())
            self.rehab.hydro_dict['test_rom_lower_input'] = float(hydro_dict['test_rom_lower_input'].toPlainText())
            self.rehab.hydro_dict['test_hold_time_input'] = float(hydro_dict['test_hold_time_input'].toPlainText())
        except Exception as e:
            print(f"[RMD 설정 실패] {e}")   

    def hydrodynamic_test_start_btn_clicked(self):
        if self.checkboxes['hydrodynamic_constant_velocity_check']['status'] == True or self.checkboxes['hydrodynamic_omega_check']['status'] == True:
            self.rehab.exercise_info_dict['control_active'] = 1

    def hydrodynamic_test_stop_btn_clicked(self):
        if self.checkboxes['hydrodynamic_constant_velocity_check']['status'] == True or self.checkboxes['hydrodynamic_omega_check']['status'] == True:
            self.rehab.exercise_info_dict['control_active'] = 0

    def const_angle_setting_btn_clicked(self):
        angle_dict = self.textedit_groups['const_angle']
        try:
            self.rehab.const_angle_para_dict['const_angle'] = float(angle_dict['const_angle_input'].toPlainText())
            self.rehab.const_angle_para_dict['const_angle_p'] = float(angle_dict['const_angle_p_input'].toPlainText())
            self.rehab.const_angle_para_dict['const_angle_i'] = float(angle_dict['const_angle_i_input'].toPlainText())
            self.rehab.const_angle_para_dict['const_angle_d'] = float(angle_dict['const_angle_d_input'].toPlainText())
        except Exception as e:
            print(f"[RMD angle 설정 실패] {e}")

    def const_angle_start_btn_clicked(self):
        if self.checkboxes['const_angle_move_check']['status'] == True:
            self.rehab.exercise_info_dict['control_active'] = 1

    def const_angle_stop_btn_clicked(self):
        if self.checkboxes['const_angle_move_check']['status'] == True:
            self.rehab.exercise_info_dict['control_active'] = 0

    def exercise_setting_btn_clicked(self):
        exercise_dict = self.textedit_groups['exercise']
        passive_dict = self.textedit_groups['passive']
        assistance_dict = self.textedit_groups['assistance']
        resistance_dict = self.textedit_groups['resistance']

        try:
            self.rehab.exercise_para_dict['desired_velocity'] = float(exercise_dict['exercise_desired_velocity'].toPlainText())
            self.rehab.exercise_para_dict['desired_acceleration'] = float(exercise_dict['exercise_desired_acceleration'].toPlainText())
            self.rehab.exercise_para_dict['repeatation_number'] = int(exercise_dict['exercise_repeatation_number'].toPlainText())
            self.rehab.exercise_para_dict['hold_time'] = float(exercise_dict['exercise_hold_time'].toPlainText())
            self.rehab.exercise_para_dict['rom_upper'] = float(exercise_dict['exercise_rom_upper'].toPlainText())
            self.rehab.exercise_para_dict['rom_lower'] = float(exercise_dict['exercise_rom_lower'].toPlainText())

            self.rehab.passive_para_dict['passive_p'] = float(passive_dict['passive_p_input'].toPlainText())
            self.rehab.passive_para_dict['passive_i'] = float(passive_dict['passive_i_input'].toPlainText())
            self.rehab.passive_para_dict['passive_d'] = float(passive_dict['passive_d_input'].toPlainText())

            self.rehab.assistance_para_dict['assistance_gain_k'] = float(assistance_dict['assistance_gain_k_input'].toPlainText())
            self.rehab.assistance_para_dict['assistance_p'] = float(assistance_dict['assistance_p_input'].toPlainText())
            self.rehab.assistance_para_dict['assistance_i'] = float(assistance_dict['assistance_i_input'].toPlainText())
            self.rehab.assistance_para_dict['assistance_d'] = float(assistance_dict['assistance_d_input'].toPlainText())
            self.rehab.assistance_para_dict['assistance_muscle_p'] = float(assistance_dict['assistance_muscle_p_input'].toPlainText())
            self.rehab.assistance_para_dict['assistance_muscle_i'] = float(assistance_dict['assistance_muscle_i_input'].toPlainText())
            self.rehab.assistance_para_dict['assistance_muscle_d'] = float(assistance_dict['assistance_muscle_d_input'].toPlainText())

            self.rehab.resistance_para_dict['resistance_moment'] = float(resistance_dict['resistance_moment_input'].toPlainText())
            self.rehab.resistance_para_dict['resistance_muscle_p'] = float(resistance_dict['resistance_muscle_p_input'].toPlainText())
            self.rehab.resistance_para_dict['resistance_muscle_i'] = float(resistance_dict['resistance_muscle_i_input'].toPlainText())
            self.rehab.resistance_para_dict['resistance_muscle_d'] = float(resistance_dict['resistance_muscle_d_input'].toPlainText())

        except Exception as e:
            print(f"[RMD exercise 설정 실패] {e}")

    def exercise_start_btn_clicked(self):
        if self.checkboxes['exercise_passive_check']['status'] == True or self.checkboxes['exercise_assistance_check']['status'] == True or self.checkboxes['exercise_resistance_check']['status'] == True:
            self.rehab.exercise_info_dict['control_active'] = 1

    def exercise_stop_btn_clicked(self):
        if self.checkboxes['exercise_passive_check']['status'] == True or self.checkboxes['exercise_assistance_check']['status'] == True or self.checkboxes['exercise_resistance_check']['status'] == True:
            self.rehab.exercise_info_dict['control_active'] = 0

    def muscle_start_btn_clicked(self):
        if self.checkboxes['muscle_passive_component_check']['status'] == True:
            self.rehab.exercise_info_dict['control_active'] = 1
    
    def muscle_stop_btn_clicked(self):
        if self.checkboxes['muscle_passive_component_check']['status'] == True:
            self.rehab.exercise_info_dict['control_active'] = 0

    def enable_btn_group(self, group_name):
        if group_name in self.button_groups:
            for btn_name in self.button_groups[group_name]:
                self.button_widgets[btn_name].setEnabled(True)

    def disable_btn_group(self, group_name):
        if group_name in self.button_groups:
            for btn_name in self.button_groups[group_name]:
                self.button_widgets[btn_name].setDisabled(True)

    def btn_on(self):
        self.enable_btn_group('hydro')
        self.enable_btn_group('const_angle')
        self.enable_btn_group('exercise')
        self.enable_btn_group('muscle')
    
    def btn_off(self):
        self.disable_btn_group('hydro')
        self.disable_btn_group('const_angle')
        self.disable_btn_group('exercise')
        self.disable_btn_group('muscle')

    def update_data(self):

        knee_angle = self.rehab.knee_angle
        velocity = self.rehab.motor_velocity
        motor_power_enabled = self.rehab.motor_power_enabled
        motor_control_active = self.rehab.motor_control_active
        thruster_power_enabled = self.rehab.thruster_power_enabled
        thruster_control_active = self.rehab.thruster_control_active
        gui_power_enabled = self.rehab.exercise_info_dict['power_enabled']
        gui_control_active = self.rehab.exercise_info_dict['control_active']


        self.angle_text.setText(f"{knee_angle:.2f}")
        self.velocity_text.setText(f"{velocity:.2f}")
        self.motor_power_enabled_status_text.setText(f"{motor_power_enabled:.2f}")
        self.motor_control_active_text.setText(f"{motor_control_active:.2f}")
        self.thruster_power_enabled_status_text.setText(f"{thruster_power_enabled:.2f}")
        self.thruster_control_active_text.setText(f"{thruster_control_active:.2f}")
        self.gui_power_enabled_status_text.setText(f"{gui_power_enabled:.2f}")
        self.gui_control_active_text.setText(f"{gui_control_active:.2f}")