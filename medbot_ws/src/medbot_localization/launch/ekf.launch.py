#!/usr/bin/env python3
"""
EKF Launch File
Module Owner: Abenezer (Localization)

Launches the Extended Kalman Filter for sensor fusion.
Fuses odometry and IMU data for improved state estimation.
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
    ekf_params = os.path.join(pkg_localization, 'config', 'ekf_params.yaml')

    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),

        # EKF Node
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                ekf_params,
                {'use_sim_time': use_sim_time}
            ],
            remappings=[
                ('odometry/filtered', 'odom_filtered')
            ]
        ),
    ])
