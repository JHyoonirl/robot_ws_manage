#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():
   
    return LaunchDescription([
        Node(
            package='pwm_msg',
            executable='keyboard_publisher',
            name='KeyboardInput',
            output='screen'),
        Node(
            package='pwm_msg',
            executable='pwm_publisher',
            name='PwmPublisher',
            output='screen'),
    ])