#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():
   
    usb_port_sensor = DeclareLaunchArgument(
            'usb_port_sensor', default_value='/dev/ttyUSB0',
            description='USB port for FT sensor'
            )
    
    usb_port_motor = DeclareLaunchArgument(
            'usb_port_motor', default_value='/dev/ttyACM0',
            description='USB port for motor'
            )

    # thruster 제어하는 노드
    thruster_operator = Node(
            package='manage_hardware',
            executable='thruster_operator',
            name='thruster_operator',
            output='screen')
    
    # sensor 불러오는 노드
    sensor_operator = Node(
            package='manage_hardware',
            executable='sensor_operator',
            name='sensor_operator',
            output='screen',
            parameters=[{
                'usb_port': LaunchConfiguration('usb_port_sensor')
            }]
            )
    
    # RMD 모터 제어하는 논드
    rmd_operator = Node(
            package='manage_hardware',
            executable='rmd_operator',
            name='rmd_operator',
            output='screen',
            parameters=[{
                'usb_port': LaunchConfiguration('usb_port_motor')
            }]
            )
    
    # data 저장하는 노드
    data_save = Node(
            package='manage_hardware',
            executable='data_save',
            name='data_save',
            output='screen')

    return LaunchDescription([
        usb_port_sensor,
        usb_port_motor,
        sensor_operator,
        rmd_operator,
        data_save,
        thruster_operator
        
    ])