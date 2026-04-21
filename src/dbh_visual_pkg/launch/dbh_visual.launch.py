from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package="dbh_visual_pkg",
            executable="dbh_visual_node",
            name="dbh_visual_node",
            output="screen",
            arguments=[
                "--camera", "0",
                "--camera-name", "DECXIN",
                "--width", "640",
                "--height", "480",
                "--fps", "60",
                "--method", "adaptive",
                "--ros2-topic", "visual_result",
            ],
        )
    ])
