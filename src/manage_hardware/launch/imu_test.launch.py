#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():
   
    usb_port_motor = DeclareLaunchArgument(
            'usb_port_motor', default_value='/dev/ttyACM0',
            description='USB port for motor'
            )
    
    usb_port_imu = DeclareLaunchArgument(
            'usb_port_imu', default_value='/dev/ttyUSB0',
            description='USB port for imu'
            )
    
    usb_port_sensor = DeclareLaunchArgument(
            'usb_port_sensor', default_value='/dev/ttyUSB1',
            description='USB port for torque sensor'
            )

    # thruster 제어하는 노드
    rehab_operator = Node(
            package='manage_hardware',
            executable='rehab_program_operator',
            name='rehab_program_operator',
            output='screen',
            parameters=[{
                'usb_port_motor': LaunchConfiguration('usb_port_motor')
            }]
            )
    
    # sensor 불러오는 노드
    imu_operator = Node(
            package='manage_hardware',
            executable='esp_imu',
            name='esp_imu',
            output='screen',
            parameters=[{
                'usb_port_imu': LaunchConfiguration('usb_port_imu')
            }]
            )
    
    sensor_operator = Node(
            package='manage_hardware',
            executable='sensor_operator',
            name='sensor_operator',
            output='screen',
            parameters=[{
                'usb_port': LaunchConfiguration('usb_port_sensor')
            }]
            )
    
    # data 저장하는 노드
    data_save = Node(
            package='manage_hardware',
            executable='data_save',
            name='data_save',
            output='screen')

    return LaunchDescription([
        usb_port_imu,
        usb_port_motor,
        usb_port_sensor,
        rehab_operator,
        imu_operator,
        sensor_operator,
        data_save
        
    ])