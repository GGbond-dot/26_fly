from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # /dev/ttyS3(串口三)：原气压计 data_comm_analysis 链路已停用，ttyS3 让给 VL53。
    # 只要不同时启动 data_comm_analysis 就不会抢口。
    serial_port = LaunchConfiguration("serial_port")
    baud_rate = LaunchConfiguration("baud_rate")

    return LaunchDescription([
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyS3"),
        DeclareLaunchArgument("baud_rate", default_value="115200"),
        Node(
            package="vl53_ranging_pkg",
            executable="vl53_ranging_node",
            name="vl53_ranging",
            output="screen",
            parameters=[{
                "serial_port": serial_port,
                "baud_rate": baud_rate,
            }],
        ),
    ])
