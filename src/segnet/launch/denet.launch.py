#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.launch_description_sources import XMLLaunchDescriptionSource 
import launch_ros.actions

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('segnet'),
        'config',
        'denet_params.yaml'
    )
    
    camera_launch = os.path.join(
        get_package_share_directory('ros_deep_learning'),
        'launch',
        'video_source.ros2v2.launch'
    )

    return LaunchDescription([
        launch_ros.actions.Node(
            package='segnet',
            executable='denet',
            name='denet',
            output='screen',
            parameters=[config_file],
        ),
        launch_ros.actions.Node(
            package='visual',
            executable='qt',
            name='qt',
            output='screen',
        ),
        launch_ros.actions.Node(
            package='gmapper',
            executable='semap',
            name='semap',
            output='screen',
        ),
        launch_ros.actions.Node(
            package='merger',
            executable='odom',
            name='odom',
            output='screen',
        ),
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(camera_launch)
        ),
    ])