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
    
    sensor_data = Node(
            package='manage_hardware',
            executable='sensor_data',
            name='sensor_data',
            output='screen',
            parameters=[{
                'usb_port': LaunchConfiguration('usb_port_sensor')
            }]
            )
    
    data_save = Node(
            package='manage_hardware',
            executable='data_save',
            name='data_save',
            output='screen')

    pwm_ve = Node(
            package='manage_hardware',
            executable='pwm_ve',
            name='pwm_ve',
            output='screen')



    return LaunchDescription([
        usb_port_sensor,
        sensor_data,
        data_save,
        pwm_ve
        
    ])