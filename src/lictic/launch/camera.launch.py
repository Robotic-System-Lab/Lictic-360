#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.actions

def generate_launch_description():
    # Declare launch arguments with their default values
    input_cam_6 = LaunchConfiguration('input_cam_6')
    input_cam_5 = LaunchConfiguration('input_cam_5')
    input_cam_4 = LaunchConfiguration('input_cam_4')
    input_cam_3 = LaunchConfiguration('input_cam_3')
    input_cam_2 = LaunchConfiguration('input_cam_2')
    input_cam_1 = LaunchConfiguration('input_cam_1')

    input_width   = LaunchConfiguration('input_width')
    input_height  = LaunchConfiguration('input_height')
    input_codec   = LaunchConfiguration('input_codec')
    input_loop    = LaunchConfiguration('input_loop')
    input_latency = LaunchConfiguration('input_latency')

    return LaunchDescription([
        DeclareLaunchArgument('input_cam_6', default_value='csi://0'),
        DeclareLaunchArgument('input_cam_5', default_value='csi://1'),
        DeclareLaunchArgument('input_cam_4', default_value='csi://2'),
        DeclareLaunchArgument('input_cam_3', default_value='csi://3'),
        DeclareLaunchArgument('input_cam_2', default_value='csi://4'),
        DeclareLaunchArgument('input_cam_1', default_value='csi://5'),
        DeclareLaunchArgument('input_width', default_value='360'),
        DeclareLaunchArgument('input_height', default_value='240'),
        DeclareLaunchArgument('input_codec', default_value='unknown'),
        DeclareLaunchArgument('input_loop', default_value='0'),
        DeclareLaunchArgument('input_latency', default_value='2000'),

        # Node for Kamera 1 (video_source_6 in XML)
        launch_ros.actions.Node(
            package='ros_deep_learning',
            executable='video_source',
            name='video_source_6',
            output='screen',
            parameters=[{
                'resource': input_cam_6,
                'width': input_width,
                'height': input_height,
                'codec': input_codec,
                'loop': input_loop,
                'latency': input_latency,
            }],
            remappings=[('/video_source/raw', '/camera_6/image_raw')]
        ),

        # Node untuk Kamera 2 (video_source_5 in XML)
        launch_ros.actions.Node(
            package='ros_deep_learning',
            executable='video_source',
            name='video_source_5',
            output='screen',
            parameters=[{
                'resource': input_cam_5,
                'width': input_width,
                'height': input_height,
                'codec': input_codec,
                'loop': input_loop,
                'latency': input_latency,
            }],
            remappings=[('/video_source/raw', '/camera_5/image_raw')]
        ),

        # Node untuk Kamera 3 (video_source_4 in XML)
        launch_ros.actions.Node(
            package='ros_deep_learning',
            executable='video_source',
            name='video_source_4',
            output='screen',
            parameters=[{
                'resource': input_cam_4,
                'width': input_width,
                'height': input_height,
                'codec': input_codec,
                'loop': input_loop,
                'latency': input_latency,
            }],
            remappings=[('/video_source/raw', '/camera_4/image_raw')]
        ),

        # Node untuk Kamera 4 (video_source_3 in XML)
        launch_ros.actions.Node(
            package='ros_deep_learning',
            executable='video_source',
            name='video_source_3',
            output='screen',
            parameters=[{
                'resource': input_cam_3,
                'width': input_width,
                'height': input_height,
                'codec': input_codec,
                'loop': input_loop,
                'latency': input_latency,
            }],
            remappings=[('/video_source/raw', '/camera_3/image_raw')]
        ),

        # Node untuk Kamera 5 (video_source_2 in XML)
        launch_ros.actions.Node(
            package='ros_deep_learning',
            executable='video_source',
            name='video_source_2',
            output='screen',
            parameters=[{
                'resource': input_cam_2,
                'width': input_width,
                'height': input_height,
                'codec': input_codec,
                'loop': input_loop,
                'latency': input_latency,
            }],
            remappings=[('/video_source/raw', '/camera_2/image_raw')]
        ),

        # Node untuk Kamera 6 (video_source_1 in XML)
        launch_ros.actions.Node(
            package='ros_deep_learning',
            executable='video_source',
            name='video_source_1',
            output='screen',
            parameters=[{
                'resource': input_cam_1,
                'width': input_width,
                'height': input_height,
                'codec': input_codec,
                'loop': input_loop,
                'latency': input_latency,
            }],
            remappings=[('/video_source/raw', '/camera_1/image_raw')]
        ),
    ])