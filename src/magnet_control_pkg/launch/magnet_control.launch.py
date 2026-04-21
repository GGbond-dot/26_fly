from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    params = {
        "pin": 10,
        "on_level": 0,
        "off_level": 1,
        "initial_off": True,
        "pulse_duration": 1.0,
        "status_period": 0.5,
    }

    return LaunchDescription([
        Node(
            package="magnet_control_pkg",
            executable="magnet_control_node",
            name="magnet_control_node",
            output="screen",
            parameters=[params],
        )
    ])
