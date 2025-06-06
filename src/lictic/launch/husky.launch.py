#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource 
import launch_ros.actions

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('lictic'),
        'config',
        'params_husky.yaml'
    )
    
    camera_launch = os.path.join(
        get_package_share_directory('lictic'),
        'launch',
        'camera.launch.py'
    )
    
    velodyne_launch = os.path.join(
        get_package_share_directory('velodyne'),
        'launch',
        'velodyne-all-nodes-VLP32C-launch.py'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(camera_launch)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(velodyne_launch)
        ),
        launch_ros.actions.Node(
            package='merger',
            executable='odom',
            name='odom',
            output='screen',
            parameters=[config_file],
        ),
        launch_ros.actions.Node(
            package='yolosed',
            executable='segmentation',
            name='segmentation',
            output='screen',
            parameters=[config_file],
        ),
        launch_ros.actions.Node(
            package='gmapper',
            executable='semap',
            name='semap',
            output='screen',
        ),
        launch_ros.actions.Node(
            package='visual',
            executable='qt',
            name='qt',
            output='screen',
        ),
    ])