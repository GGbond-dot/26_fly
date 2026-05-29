from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="barcode_camera_pkg",
            executable="barcode_camera_node",
            name="barcode_camera_node",
            output="screen",
            parameters=[
                {
                    "camera_device": "/dev/video0",
                    "frame_width": 640,
                    "frame_height": 480,
                    "fps": 15.0,
                    "barcode_topic": "/barcode_text",
                    "show_preview": True,
                    "window_name": "barcode_camera_preview",
                    "publish_duplicates": False,
                    "stop_after_first_publish": True,
                }
            ],
        )
    ])
