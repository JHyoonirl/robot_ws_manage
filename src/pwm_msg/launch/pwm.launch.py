#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():
   
    return LaunchDescription([
        Node(
            package='pwm_msg',
            executable='KeyboardInput',
            name='KeyboardInput',
            output='screen'),
        Node(
            package='pwm_msg',
            executable='PwmPublisher',
            name='PwmPublisher',
            output='screen'),
    ])