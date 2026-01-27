#!/usr/bin/env python3
"""
Robot Description Display Launch File
Module Owner: Abel (Modeling)

Launches robot_state_publisher and RViz for URDF visualization.
Use this to verify the robot model before simulation.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    # Package directories
    pkg_description = get_package_share_directory('medbot_description')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_gui = LaunchConfiguration('use_gui', default='true')

    # Robot description from xacro
    robot_description_content = Command([
        'xacro ', os.path.join(pkg_description, 'urdf', 'medbot.urdf.xacro'),
        ' use_sim:=', use_sim_time
    ])

    robot_description = {'robot_description': robot_description_content}

    # RViz config
    rviz_config = os.path.join(pkg_description, 'rviz', 'display.rviz')

    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'use_gui',
            default_value='true',
            description='Use joint state publisher GUI'
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description, {'use_sim_time': use_sim_time}]
        ),

        # Joint State Publisher (GUI)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            condition=None  # Always run for visualization
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}]
        ),
    ])
