#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('yolosed'),
        'config',
        'segmentation_params.yaml'
    )

    return LaunchDescription([
        launch_ros.actions.Node(
            package='yolosed',
            executable='segmentation',
            name='segmentation',
            output='screen',
            parameters=[config_file],
        ),
    ])