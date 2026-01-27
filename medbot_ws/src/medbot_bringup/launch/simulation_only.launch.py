#!/usr/bin/env python3
"""
Simulation Only Launch File
Launches Gazebo simulation with robot, without navigation.
Useful for initial testing and development.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Package directories
    pkg_bringup = get_package_share_directory('medbot_bringup')
    pkg_gazebo = get_package_share_directory('medbot_gazebo')

    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    rviz = LaunchConfiguration('rviz', default='true')
    world = LaunchConfiguration('world')

    # Default world file
    default_world = os.path.join(
        get_package_share_directory('medbot_gazebo'),
        'worlds',
        'addis_ababa_urban.world'
    )

    # RViz config for simulation viewing
    rviz_config = os.path.join(
        get_package_share_directory('medbot_description'),
        'rviz',
        'display.rviz'
    )

    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Launch RViz visualization'
        ),
        DeclareLaunchArgument(
            'world',
            default_value=default_world,
            description='World file for Gazebo simulation'
        ),

        # Gazebo simulation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo, 'launch', 'gazebo_simulation.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'world': world
            }.items()
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(rviz),
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}]
        ),
    ])
