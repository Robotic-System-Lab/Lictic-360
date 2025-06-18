#!/usr/bin/env python3
from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='lictic_teleop',
            executable='teleop_keyboard',
            name='teleop_keyboard',
            output='screen',
        ),
    ])