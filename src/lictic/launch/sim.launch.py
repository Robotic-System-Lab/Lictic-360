#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('yolosed'),
        'config',
        'segmentation_params.yaml'
    )
    
    turtlebot3_launch = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'launch',
        'callibrate_cam.launch.py'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(turtlebot3_launch),
            launch_arguments={'gui': 'true'}.items(),
        ),
        launch_ros.actions.Node(
            package='yolosed',
            executable='segmentation',
            name='segmentation',
            output='screen',
            parameters=[config_file],
        ),
        # launch_ros.actions.Node(
        #     package='gmapper',
        #     executable='semap',
        #     name='semap',
        #     output='screen',
        # ),
        launch_ros.actions.Node(
            package='visual',
            executable='qt',
            name='qt',
            output='screen',
        ),
    ])