#!/usr/bin/env python3
"""
D题 立体货架盘点 — 任务总启动（单相机方案）

拉起三部分：
  1) qr_vision        单相机二维码识别 + 激光指示（/qr_vision/*）
  2) qr_fine_tune     归一化偏移 → 机体系 cm 微调量（/qr_vision/fine_offset_body_cm）
  3) inventory_mission_node  盘点任务状态机（遍历 / 定向）

不含 PID / uart / cartographer —— 那些复用本仓现有 launch（与搬运/植保同款），
实飞时与本 launch 一起起。

常用：
  ros2 launch inventory_control_pkg inventory_mission.launch.py            # 遍历盘点
  ros2 launch inventory_control_pkg inventory_mission.launch.py mode:=directed   # 定向盘点
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    mode = LaunchConfiguration("mode")
    camera_device = LaunchConfiguration("camera_device")
    laser_pin = LaunchConfiguration("laser_pin")
    rotate_code = LaunchConfiguration("rotate_code")

    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="traverse",
                              description="traverse=遍历盘点 / directed=定向盘点"),
        DeclareLaunchArgument("camera_device", default_value="/dev/video0",
                              description="盘点相机设备（按 by-path 稳定路径配置更稳）"),
        DeclareLaunchArgument("laser_pin", default_value="-1",
                              description="激光 wiringPi 引脚号，-1=不控（无硬件调试）"),
        DeclareLaunchArgument("rotate_code", default_value="-1",
                              description="相机旋转 -1/0/1/2，按当前安装方位标定"),

        Node(
            package="qr_vision_pkg",
            executable="qr_vision",
            name="qr_vision_node",
            output="screen",
            parameters=[{
                "camera_device": camera_device,
                "laser_pin": laser_pin,
                "rotate_code": rotate_code,
                "fourcc": "MJPG",
            }],
        ),
        Node(
            package="qr_vision_pkg",
            executable="qr_fine_tune",
            name="qr_fine_tune_node",
            output="screen",
            parameters=[{
                "input_prefix": "/qr_vision",
                "output_topic": "/qr_vision/fine_offset_body_cm",
            }],
        ),
        Node(
            package="inventory_control_pkg",
            executable="inventory_mission_node",
            name="inventory_mission_node",
            output="screen",
            parameters=[{
                "mode": mode,
                "flight_height_cm": 150.0,
            }],
        ),
    ])
