#!/usr/bin/env python3
from launch import LaunchDescription
import launch.actions
import launch_ros.actions

def generate_launch_description():
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time', default='true')
    return LaunchDescription([
        launch_ros.actions.Node(
            package='yolosed',
            executable='yolosed',
            name='yolosed',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])