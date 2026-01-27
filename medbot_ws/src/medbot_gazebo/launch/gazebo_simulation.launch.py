#!/usr/bin/env python3
"""
Gazebo Simulation Launch File
Launches Gazebo with the Ethiopian urban world and spawns the robot.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    # Package directories
    pkg_gazebo = get_package_share_directory('medbot_gazebo')
    pkg_description = get_package_share_directory('medbot_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    z_pose = LaunchConfiguration('z_pose', default='0.1')

    # World file path
    world_file = os.path.join(pkg_gazebo, 'worlds', 'addis_ababa_urban.world')

    # Robot description
    robot_description_content = Command([
        'xacro ', os.path.join(pkg_description, 'urdf', 'medbot.urdf.xacro'),
        ' use_sim:=true'
    ])

    # Set Gazebo model path
    gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.path.join(pkg_gazebo, 'models')
    )

    return LaunchDescription([
        # Environment variables
        gazebo_model_path,

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
        DeclareLaunchArgument(
            'x_pose',
            default_value='0.0',
            description='Robot initial X position'
        ),
        DeclareLaunchArgument(
            'y_pose',
            default_value='0.0',
            description='Robot initial Y position'
        ),
        DeclareLaunchArgument(
            'z_pose',
            default_value='0.1',
            description='Robot initial Z position'
        ),

        # Launch Gazebo server
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world}.items()
        ),

        # Launch Gazebo client
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            )
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description_content,
                'use_sim_time': use_sim_time
            }]
        ),

        # Spawn robot in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_medbot',
            output='screen',
            arguments=[
                '-entity', 'medbot',
                '-topic', 'robot_description',
                '-x', x_pose,
                '-y', y_pose,
                '-z', z_pose
            ]
        ),
    ])
