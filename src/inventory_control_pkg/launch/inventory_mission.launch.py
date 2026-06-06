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
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    mode = LaunchConfiguration("mode")
    traverse_faces = LaunchConfiguration("traverse_faces")
    camera_device = LaunchConfiguration("camera_device")
    laser_pin = LaunchConfiguration("laser_pin")
    rotate_code = LaunchConfiguration("rotate_code")
    enable_gui = ParameterValue(LaunchConfiguration("enable_gui"), value_type=bool)
    # 用 ParameterValue 显式指定类型，避免 launch 字符串传给类型化参数(bool/double)时类型不匹配。
    land_at_circle = ParameterValue(LaunchConfiguration("land_at_circle"), value_type=bool)
    land_x_cm = ParameterValue(LaunchConfiguration("land_x_cm"), value_type=float)
    land_y_cm = ParameterValue(LaunchConfiguration("land_y_cm"), value_type=float)

    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="traverse",
                              description="traverse=遍历盘点 / directed=定向盘点"),
        DeclareLaunchArgument("traverse_faces", default_value="A,B",
                              description="遍历哪些面（逗号分隔）。默认 A,B 只扫货架1（C/D 坐标尚未标定，"
                                          "标定货架2 后传 A,B,C,D 打开）"),
        # 盘点相机 = 之前植保的下视相机 down_cam(/dev/video0)。by-path 稳定路径，插拔不变号。
        DeclareLaunchArgument(
            "camera_device",
            default_value="/dev/v4l/by-path/platform-xhci-hcd.11.auto-usb-0:1:1.0-video-index0",
            description="盘点相机设备（下视 down_cam by-path 稳定路径）"),
        # 激光走香橙派 WiringOP `gpio` 命令控脚（实测 pin=13，on=低）。植保 /electromagnet_control(0x33) 实测不稳，已弃用。
        DeclareLaunchArgument("laser_pin", default_value="13",
                              description="激光 WiringOP 引脚号（香橙派 gpio 命令，实测=13），-1=不控"),
        DeclareLaunchArgument("rotate_code", default_value="2",
                              description="相机旋转 -1/0/1/2，下视相机画面逆时针90°转正=2"),
        # 预览窗：开了 qr_vision 会弹本地 cv2 窗口（连显示器/VNC 时可见），待命态也持续出画面，
        # 用来确认自启动跑起来、相机正常。需有 DISPLAY（autostart_fly.sh 已兜底 :0）。
        DeclareLaunchArgument("enable_gui", default_value="false",
                              description="true=qr_vision 弹本地预览窗（确认相机/自启动）；需 DISPLAY"),
        # 降落点：默认 false=落末面正前方 y 轴（单货架 A,B 测试）；四面全跑传 true 落黑圆 land_x/y。
        DeclareLaunchArgument("land_at_circle", default_value="false",
                              description="true=遍历跑完飞黑圆(land_x/y)降落；false=落末面正前方y轴"),
        DeclareLaunchArgument("land_x_cm", default_value="0.0",
                              description="黑圆降落点 x(cm)，land_at_circle=true 时生效"),
        DeclareLaunchArgument("land_y_cm", default_value="0.0",
                              description="黑圆降落点 y(cm)，land_at_circle=true 时生效"),

        Node(
            package="qr_vision_pkg",
            executable="qr_vision",
            name="qr_vision_node",
            output="screen",
            parameters=[{
                "camera_device": camera_device,
                "laser_pin": laser_pin,
                "rotate_code": rotate_code,
                "enable_gui": enable_gui,
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
                "land_at_circle": land_at_circle,
                "land_x_cm": land_x_cm,
                "land_y_cm": land_y_cm,
                # 超时补救：超时后沿机头反方向后撤这么多 cm 再试一次（0=关闭，超时直接跳过）。
                "scan_retreat_cm": 10.0,
                # 后退重试时收严激光（standoff 变大窗口变松→强制 strict_vertical 卡纵向）。
                "tighten_laser_on_retreat": True,
            }],
        ),
    ])
