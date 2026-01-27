#!/usr/bin/env python3
"""
Mission Control Launch File
Launches all mission-related nodes.
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

        # Delivery Manager
        Node(
            package='medbot_mission',
            executable='delivery_manager',
            name='delivery_manager',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # Waypoint Publisher
        Node(
            package='medbot_mission',
            executable='waypoint_publisher',
            name='waypoint_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # Status Monitor
        Node(
            package='medbot_mission',
            executable='status_monitor',
            name='status_monitor',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # Obstacle Detector
        Node(
            package='medbot_mission',
            executable='obstacle_detector',
            name='obstacle_detector',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # Emergency Stop
        Node(
            package='medbot_mission',
            executable='emergency_stop',
            name='emergency_stop',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
    ])
