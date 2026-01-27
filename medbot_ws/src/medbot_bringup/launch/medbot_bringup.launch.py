#!/usr/bin/env python3
"""
Complete System Bringup Launch File
Launches the full simulation with SLAM and Navigation.

This is the main entry point for running the complete system.
Team members can use this to test integrated functionality.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetRemap


def generate_launch_description():
    # Package directories
    pkg_bringup = get_package_share_directory('medbot_bringup')
    pkg_gazebo = get_package_share_directory('medbot_gazebo')
    pkg_navigation = get_package_share_directory('medbot_navigation')
    pkg_localization = get_package_share_directory('medbot_localization')

    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    slam = LaunchConfiguration('slam', default='true')
    nav = LaunchConfiguration('nav', default='true')
    rviz = LaunchConfiguration('rviz', default='true')
    world = LaunchConfiguration('world')

    # Default world file
    default_world = os.path.join(
        get_package_share_directory('medbot_gazebo'),
        'worlds',
        'addis_ababa_urban.world'
    )

    # RViz config
    rviz_config = os.path.join(pkg_bringup, 'rviz', 'navigation.rviz')

    return LaunchDescription([
        # ==================== ARGUMENTS ====================
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock'
        ),
        DeclareLaunchArgument(
            'slam',
            default_value='true',
            description='Launch SLAM Toolbox for mapping'
        ),
        DeclareLaunchArgument(
            'nav',
            default_value='true',
            description='Launch Nav2 navigation stack'
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

        # ==================== GAZEBO SIMULATION ====================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo, 'launch', 'gazebo_simulation.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'world': world
            }.items()
        ),

        # ==================== SLAM TOOLBOX ====================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_localization, 'launch', 'slam.launch.py')
            ),
            condition=IfCondition(slam),
            launch_arguments={
                'use_sim_time': use_sim_time
            }.items()
        ),

        # ==================== NAV2 NAVIGATION ====================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_navigation, 'launch', 'navigation.launch.py')
            ),
            condition=IfCondition(nav),
            launch_arguments={
                'use_sim_time': use_sim_time
            }.items()
        ),

        # ==================== RVIZ ====================
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
