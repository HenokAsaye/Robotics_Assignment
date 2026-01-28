#!/usr/bin/env python3
"""
Complete System Bringup Launch File
Launches the full simulation with SLAM, Navigation, EKF, and Mission Control.

This is the main entry point for running the complete system.
Team members can use this to test integrated functionality.

Usage:
    ros2 launch medbot_bringup medbot_bringup.launch.py
    ros2 launch medbot_bringup medbot_bringup.launch.py slam:=false  # Use pre-built map
    ros2 launch medbot_bringup medbot_bringup.launch.py mission:=false  # Without mission control
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
    pkg_mission = get_package_share_directory('medbot_mission')

    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    slam = LaunchConfiguration('slam', default='true')
    nav = LaunchConfiguration('nav', default='true')
    rviz = LaunchConfiguration('rviz', default='true')
    mission = LaunchConfiguration('mission', default='true')
    ekf = LaunchConfiguration('ekf', default='true')
    world = LaunchConfiguration('world')

    # Default world file
    default_world = os.path.join(
        get_package_share_directory('medbot_gazebo'),
        'worlds',
        'simple_urban.world'
    )

    # RViz config
    rviz_config = os.path.join(pkg_bringup, 'rviz', 'navigation.rviz')

    # Nav2 params
    nav2_params = os.path.join(pkg_navigation, 'config', 'nav2_params.yaml')

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
        DeclareLaunchArgument(
            'mission',
            default_value='true',
            description='Launch mission control nodes (delivery manager, obstacle detector)'
        ),
        DeclareLaunchArgument(
            'ekf',
            default_value='true',
            description='Launch EKF for sensor fusion (odometry + IMU)'
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
                'use_sim_time': use_sim_time,
                'params_file': nav2_params
            }.items()
        ),

        # ==================== EKF SENSOR FUSION ====================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_localization, 'launch', 'ekf.launch.py')
            ),
            condition=IfCondition(ekf),
            launch_arguments={
                'use_sim_time': use_sim_time
            }.items()
        ),

        # ==================== MISSION CONTROL ====================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_mission, 'launch', 'mission.launch.py')
            ),
            condition=IfCondition(mission),
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
