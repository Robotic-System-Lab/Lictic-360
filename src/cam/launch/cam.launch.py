import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Common parameters
    input_width = 360
    input_height = 240
    input_codec = "unknown"
    input_loop = 0
    input_latency = 2000

    # Define each camera node with remappings.
    node_cam_6 = Node(
        package="cam",
        executable="video_source",
        name="video_source_6",
        output="screen",
        parameters=[{
            "resource": "csi://0",
            "width": input_width,
            "height": input_height,
            "codec": input_codec,
            "loop": input_loop,
            "latency": input_latency,
        }],
        remappings=[("/video_source/raw", "/camera_6/image_raw")]
    )

    node_cam_5 = Node(
        package="cam",
        executable="video_source",
        name="video_source_5",
        output="screen",
        parameters=[{
            "resource": "csi://1",
            "width": input_width,
            "height": input_height,
            "codec": input_codec,
            "loop": input_loop,
            "latency": input_latency,
        }],
        remappings=[("/video_source/raw", "/camera_5/image_raw")]
    )

    node_cam_4 = Node(
        package="cam",
        executable="video_source",
        name="video_source_4",
        output="screen",
        parameters=[{
            "resource": "csi://2",
            "width": input_width,
            "height": input_height,
            "codec": input_codec,
            "loop": input_loop,
            "latency": input_latency,
        }],
        remappings=[("/video_source/raw", "/camera_4/image_raw")]
    )

    node_cam_3 = Node(
        package="cam",
        executable="video_source",
        name="video_source_3",
        output="screen",
        parameters=[{
            "resource": "csi://3",
            "width": input_width,
            "height": input_height,
            "codec": input_codec,
            "loop": input_loop,
            "latency": input_latency,
        }],
        remappings=[("/video_source/raw", "/camera_3/image_raw")]
    )

    node_cam_2 = Node(
        package="cam",
        executable="video_source",
        name="video_source_2",
        output="screen",
        parameters=[{
            "resource": "csi://4",
            "width": input_width,
            "height": input_height,
            "codec": input_codec,
            "loop": input_loop,
            "latency": input_latency,
        }],
        remappings=[("/video_source/raw", "/camera_2/image_raw")]
    )

    node_cam_1 = Node(
        package="cam",
        executable="video_source",
        name="video_source_1",
        output="screen",
        parameters=[{
            "resource": "csi://5",
            "width": input_width,
            "height": input_height,
            "codec": input_codec,
            "loop": input_loop,
            "latency": input_latency,
        }],
        remappings=[("/video_source/raw", "/camera_1/image_raw")]
    )

    ld = LaunchDescription([
        node_cam_6,
        node_cam_5,
        node_cam_4,
        node_cam_3,
        node_cam_2,
        node_cam_1,
    ])

    return ld