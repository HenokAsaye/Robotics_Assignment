#!/usr/bin/env python3
"""
SLAM Launch File
Module Owner: Abenezer (Localization)

Launches SLAM Toolbox for simultaneous localization and mapping.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Package directory
    pkg_localization = get_package_share_directory('medbot_localization')

    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    slam_params = os.path.join(pkg_localization, 'config', 'slam_toolbox_params.yaml')

    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),

        # SLAM Toolbox Node
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                slam_params,
                {'use_sim_time': use_sim_time}
            ],
        ),
    ])
