#!/usr/bin/env python3
"""
D题 立体货架盘点 — 任务总启动（单相机方案）

拉起四部分：
  1) qr_vision           单相机二维码识别 + 激光指示（/qr_vision/*）
  2) standoff_estimator  雷达 /scan 测板面距离（/standoff/distance），喂 qr_fine_tune 动态增益
  3) qr_fine_tune        归一化偏移 → 机体系 cm 微调量（/qr_vision/fine_offset_body_cm）
  4) inventory_mission_node  盘点任务状态机（遍历 / 定向）

不含 PID / uart / cartographer —— 那些复用本仓现有 launch（与搬运/植保同款），
实飞时与本 launch 一起起（standoff 需要 /scan，由 cartographer/雷达那条 launch 提供）。

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
    traverse_faces = LaunchConfiguration("traverse_faces")
    camera_device = LaunchConfiguration("camera_device")
    laser_pin = LaunchConfiguration("laser_pin")
    rotate_code = LaunchConfiguration("rotate_code")

    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="traverse",
                              description="traverse=遍历盘点 / directed=定向盘点"),
        DeclareLaunchArgument("traverse_faces", default_value="A,B,C,D",
                              description="遍历哪些面（逗号分隔）。只有货架1时设 A,B 只扫前后两面"),
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
            package="standoff_estimator_pkg",
            executable="standoff_estimator_node",
            name="standoff_estimator",
            output="screen",
            parameters=[{
                "scan_topic": "/scan",
                "sector_center_deg": 0.0,       # 相机/雷达同朝正前方
                "sector_half_width_deg": 30.0,
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
                "use_standoff": True,
                "standoff_topic": "/standoff/distance",
                "standoff_valid_topic": "/standoff/valid",
                "hfov_deg": 60.0,               # ⚠ 待标定相机实际视场角
                "vfov_deg": 37.0,               # ⚠ 待标定
            }],
        ),
        Node(
            package="inventory_control_pkg",
            executable="inventory_mission_node",
            name="inventory_mission_node",
            output="screen",
            parameters=[{
                "mode": mode,
                "traverse_faces": traverse_faces,
                "flight_height_cm": 150.0,
            }],
        ),
    ])
