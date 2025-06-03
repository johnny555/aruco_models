#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='krytn',
        description='Name of the robot to spawn'
    )
    
    robot_x_arg = DeclareLaunchArgument(
        'robot_x',
        default_value='0.0',
        description='Robot initial X position'
    )
    
    robot_y_arg = DeclareLaunchArgument(
        'robot_y',
        default_value='0.0',
        description='Robot initial Y position'
    )
    
    robot_z_arg = DeclareLaunchArgument(
        'robot_z',
        default_value='0.1',
        description='Robot initial Z position'
    )

    # Launch the ArUco test world
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('aruco_models'),
                'launch',
                'aruco_test_world.launch.py'
            ])
        ])
    )

    # ArUco detection node (if using aruco_detect package)
    aruco_detect_node = Node(
        package='aruco_detect',
        executable='aruco_detect',
        name='aruco_detect',
        parameters=[{
            'image_transport': 'raw',
            'publish_images': True,
            'fiducial_len': 0.14,  # 14cm markers
            'dictionary': 0,  # DICT_4X4_50
            'do_pose_estimation': True,
            'ignore_fiducials': '',
            'fiducial_len_override': ''
        }],
        remappings=[
            ('/camera/image', '/camera/image_raw'),
            ('/camera/camera_info', '/camera/camera_info')
        ],
        condition=None
    )

    # RViz for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('aruco_models'),
            'rviz',
            'aruco_test.rviz'
        ])],
        condition=None
    )

    return LaunchDescription([
        robot_name_arg,
        robot_x_arg,
        robot_y_arg,
        robot_z_arg,
        world_launch,
        # aruco_detect_node,  # Uncomment if you want to use real ArUco detection
        # rviz_node,  # Uncomment if you have an RViz config
    ])
