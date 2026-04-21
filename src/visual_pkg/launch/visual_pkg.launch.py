from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    params = {
        "camera_index": 0,
        "width": 640,
        "height": 480,
        "camera_fps": 30,
        "process_fps": 15.0,
        "show_display": False,
        "apriltag_code": -1,
    }

    return LaunchDescription([
        Node(
            package="visual_pkg",
            executable="visual_node",
            name="visual_node",
            output="screen",
            parameters=[params],
        )
    ])
