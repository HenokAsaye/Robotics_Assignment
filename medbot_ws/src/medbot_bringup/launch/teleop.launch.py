#!/usr/bin/env python3
"""
Teleop Launch File
Launch teleop controls for manual robot driving.
Useful for testing and mapping.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),

        # Keyboard teleop node
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard',
            output='screen',
            prefix='xterm -e',  # Opens in new terminal
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[('/cmd_vel', '/cmd_vel')]
        ),
    ])
