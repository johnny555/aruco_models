#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Declare launch arguments
    world_file_arg = DeclareLaunchArgument(
        'world_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('aruco_models'),
            'worlds',
            'aruco_test.world'
        ]),
        description='Path to the world file'
    )
    
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Start Gazebo with GUI'
    )
    
    verbose_arg = DeclareLaunchArgument(
        'verbose',
        default_value='false',
        description='Enable verbose output'
    )

    # Gazebo process
    gazebo_process = ExecuteProcess(
        cmd=[
            'gazebo',
            '--verbose' if LaunchConfiguration('verbose') else '',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            LaunchConfiguration('world_file')
        ],
        output='screen',
        condition=None
    )

    return LaunchDescription([
        world_file_arg,
        gui_arg,
        verbose_arg,
        gazebo_process
    ])
