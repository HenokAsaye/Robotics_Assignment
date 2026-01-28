#!/usr/bin/env python3
"""
Gazebo Harmonic (New Gazebo) Simulation Launch File
Launches Gazebo Harmonic with the MedBot world.

Module Owner: Systems Integration
"""

import os
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Package directories
    pkg_gazebo = get_package_share_directory('medbot_gazebo')
    pkg_description = get_package_share_directory('medbot_description')

    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world')

    # World file path
    world_file = os.path.join(pkg_gazebo, 'worlds', 'addis_ababa_urban.world')
    
    # URDF file
    urdf_file = os.path.join(pkg_description, 'urdf', 'medbot.urdf.xacro')
    
    # Model path for Gazebo
    model_path = os.path.join(pkg_gazebo, 'models')

    # Robot description
    robot_description_content = Command([
        'xacro ', urdf_file,
        ' use_sim:=true'
    ])
    robot_description = ParameterValue(robot_description_content, value_type=str)

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'world',
            default_value=world_file,
            description='World file to load'
        ),

        # ==================== GAZEBO HARMONIC (NEW) ====================
        # Use the 'gz' command to launch Gazebo Harmonic directly
        # Set model path so it can find robot models
        ExecuteProcess(
            cmd=['bash', '-c', f'export GZ_SIM_RESOURCE_PATH={model_path}:$GZ_SIM_RESOURCE_PATH && gz sim -r {world_file}'],
            output='screen',
            name='gazebo_harmonic',
            additional_env={'GZ_SIM_RESOURCE_PATH': model_path}
        ),

        # ==================== ROBOT STATE PUBLISHER ====================
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time
            }]
        ),

        # ==================== SPAWN ROBOT ====================
        # Spawn robot using dedicated Python script with logging
        ExecuteProcess(
            cmd=['python3', os.path.join(pkg_gazebo, 'scripts', 'spawn_robot.py')],
            output='screen',
            name='spawn_medbot'
        ),
    ])


