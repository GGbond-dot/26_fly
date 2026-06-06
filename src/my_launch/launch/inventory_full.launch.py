#!/usr/bin/env python3
"""
D题 立体货架盘点 — 要求1「整套基础部分」一把启动（飞控底座 + 视觉 + 任务）。

⚠ 2026-06-05 起：飞机上电后**先原地待命 WAIT_MODE 不起飞**，等地面站点「普通任务/进阶任务」
按钮下发 /inventory_mode 才动作（飞机重启后由地面站告知本轮跑要求1 还是要求2）。
收到 traverse → 起飞 A,B,C,D 四面 24 码遍历（逐码激光 0.5s + 上报）→ 飞黑圆 (land_x/y) 降落；
收到 directed → 地面读抽取码 → 直飞目标货位盘点 → 降落。

= rotate_test_basic 那套飞控底座（carto + uart + 标准位置PID）
+ inventory_mission.launch.py（qr_vision / standoff / qr_fine_tune / 任务节点），
默认参数已配成：mode=traverse、traverse_faces=A,B,C,D、land_at_circle=true、落点 (350,250)。

⚠ 货架几何（A/B/C/D 实测坐标）写死在 inventory_mission_node.cpp::faceGeometry()，改了要重编。
⚠ 偏航转速默认 30°/s（B/D 面 yaw180 + 换货架旋转都用它），想更柔和传 max_angular_velocity:=xx。

放在 my_launch（统一总启动位置，与 rotate_test_basic / demo_basic / spray_basic 并列）。

常用：
  ros2 launch my_launch inventory_full.launch.py                 # 要求1 整套
  ros2 launch my_launch inventory_full.launch.py traverse_faces:=A,B   # 只跑货架1
  ros2 launch my_launch inventory_full.launch.py max_angular_velocity:=20.0
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def _include(package_name, launch_file, launch_arguments=None):
    share = FindPackageShare(package=package_name).find(package_name)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", launch_file)),
        launch_arguments=(launch_arguments or {}).items(),
    )


def generate_launch_description() -> LaunchDescription:
    traverse_faces = LaunchConfiguration("traverse_faces")
    land_at_circle = LaunchConfiguration("land_at_circle")
    land_x_cm = LaunchConfiguration("land_x_cm")
    land_y_cm = LaunchConfiguration("land_y_cm")
    max_angular_velocity = LaunchConfiguration("max_angular_velocity")
    enable_gui = LaunchConfiguration("enable_gui")

    args = [
        DeclareLaunchArgument("traverse_faces", default_value="A,B,C,D",
                              description="遍历哪些面（逗号分隔）。要求1 全跑=A,B,C,D；只测货架1=A,B"),
        DeclareLaunchArgument("land_at_circle", default_value="true",
                              description="true=跑完飞黑圆降落点；false=落末面正前方y轴(单货架测试)"),
        DeclareLaunchArgument("land_x_cm", default_value="350.0",
                              description="黑圆降落点 x(cm)，实测"),
        DeclareLaunchArgument("land_y_cm", default_value="250.0",
                              description="黑圆降落点 y(cm)，实测"),
        DeclareLaunchArgument("max_angular_velocity", default_value="30.0",
                              description="偏航最大角速度 deg/s（yaw180 转速上限，默认30；更柔和传更小值）"),
        # 自启动默认弹相机预览窗：上电待命(WAIT_MODE)时也持续出画面 → 一眼确认自启动跑起来、相机正常。
        # 不想要窗口（纯无头/省 CPU）：enable_gui:=false。需 DISPLAY（autostart_fly.sh 已兜底 :0）。
        DeclareLaunchArgument("enable_gui", default_value="true",
                              description="true=弹相机预览窗确认自启动/相机；无头省CPU传 false"),
    ]

    # ── 飞控底座（与 my_launch/rotate_test_basic 同款三件套）────────────────────
    #   carto 出 tf(map->laser_link)+/scan；uart 发 /height + 转 /target_velocity(0x31)；
    #   标准位置 PID（z 走 /height，不用面阵激光）。standoff_estimator 需要 /scan，由 carto 那条提供。
    fly_carto = _include("my_carto_pkg", "fly_carto.launch.py", {"use_rviz": "false"})
    uart = _include("uart_to_stm32", "uart_to_stm32.launch.py")
    position_pid = _include(
        "pid_control_pkg", "position_pid_controller.launch.py",
        {"max_angular_velocity": max_angular_velocity},
    )

    # ── 视觉 + 盘点任务（复用 inventory_mission.launch.py，透传降落点参数）────────
    mission = _include(
        "inventory_control_pkg", "inventory_mission.launch.py",
        {
            "mode": "traverse",
            "traverse_faces": traverse_faces,
            "land_at_circle": land_at_circle,
            "land_x_cm": land_x_cm,
            "land_y_cm": land_y_cm,
            "enable_gui": enable_gui,
        },
    )

    return LaunchDescription(args + [fly_carto, uart, position_pid, mission])
