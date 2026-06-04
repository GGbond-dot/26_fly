#!/usr/bin/env python3
"""
D题 立体货架盘点 — 「识别二维码 → 打激光」链路最小测试 launch

只起 qr_vision 一件事，把链路串通、方便地面台架验证（不起飞、不带 PID/standoff/mission）：
  下视相机识别二维码，识别到且对准 → 子线程把香橙派 wiringPi GPIO(laser_pin) 拉低 0.5s → 激光亮。

激光走香橙派 GPIO 直驱（同 24fly）。植保那条 /electromagnet_control(STM32 0x33) 链路实测不稳，
已弃用，所以这里不需要 uart_to_stm32。

用法（飞机端先 source DDS + install）：
  ros2 launch inventory_control_pkg qr_laser_test.launch.py laser_pin:=10
验证：
  把数字二维码拿到下视相机前并大致对中 → 激光闪 0.5s；预览窗看是否对中；
  ros2 topic echo /qr_vision/id        看识别到的编号
  ros2 topic echo /qr_vision/aligned   看是否判定对准
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 盘点相机 = 植保下视相机 down_cam(/dev/video0)。by-path 稳定路径，插拔不变号。
DOWN_CAM = ("/dev/v4l/by-path/"
            "platform-xhci-hcd.11.auto-usb-0:1:1.0-video-index0")


def generate_launch_description() -> LaunchDescription:
    camera_device = LaunchConfiguration("camera_device")
    rotate_code = LaunchConfiguration("rotate_code")
    laser_pin = LaunchConfiguration("laser_pin")

    return LaunchDescription([
        DeclareLaunchArgument("camera_device", default_value=DOWN_CAM,
                              description="盘点相机（下视 down_cam by-path）"),
        DeclareLaunchArgument("rotate_code", default_value="2",
                              description="下视画面逆时针90°转正=2"),
        DeclareLaunchArgument("laser_pin", default_value="13",
                              description="激光 WiringOP 引脚号（香橙派 gpio 命令，实测=13），-1=不控"),

        Node(
            package="qr_vision_pkg",
            executable="qr_vision",
            name="qr_vision_node",
            output="screen",
            parameters=[{
                "camera_device": camera_device,
                "rotate_code": rotate_code,
                "laser_pin": laser_pin,
                "fourcc": "MJPG",
                "enable_debug_image": True,    # 台架调试出 /qr_vision/debug_image 方便看对没对中
                "enable_gui": True,            # 弹本地预览窗（cv2.imshow，需有 DISPLAY）
            }],
        ),
    ])
