#!/usr/bin/env python3
"""
D题 旋转飞控测试 — 验证「起飞→前进→原地 yaw 180°→返航→降落」

只起 inventory_mission_node（mode:=rotate_test），不开相机/激光/standoff，
纯粹验证 PID 位置控制器的 yaw 旋转 + 带偏航平移能否稳住。

⚠ 不含 PID / uart / cartographer —— 这条 launch 只发任务航点，飞控链路要另起
（与遍历盘点 inventory_mission.launch.py 相同：carto + uart + 位置PID）。
rotate_test 需要 tf map→laser_link（carto 提供）和 /height（uart 上报）才能判到点。

航线（全部为非盘点过渡航点）：
  起飞 home@test_height → 前进 +x test_forward → 原地分步转 0→test_yaw（保持）
  → 保持 test_yaw 平移飞回 home → 在 home 垂直降落 → /mission_complete + 停控制器

⚠ 本测试「转到 test_yaw 后带偏航平移返航」会真正触发带偏航平移：PID 发的 /target_velocity 是
  map 系、uart_to_stm32(帧0x31) 未旋到机体系，若 STM32 固件按机体系解释，返航会反向飞车。
  这正是盘点扫背面（yaw 180）同款动作 → 首飞务必小油门、人随时接管，重点看「转到180后往home飞方向对不对」。
  旋转分步限制单步误差；±180 微分跳变已在位置PID（yaw通道角度归一化）根治。想更慢：降 max_angular_velocity。

常用：
  ros2 launch inventory_control_pkg rotate_test.launch.py                       # 默认连续转 180
  ros2 launch inventory_control_pkg rotate_test.launch.py test_forward_cm:=150 test_yaw_deg:=90
  ros2 launch inventory_control_pkg rotate_test.launch.py test_yaw_step_deg:=45 # 想分段观察才设小值
  # 转速由位置PID 的 max_angular_velocity 决定（不是步长）：默认 30°/s（180°约6s）。
  # 想更柔和：max_angular_velocity:=xx 传给 rotate_test_basic（见 my_launch/rotate_test_basic）。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    test_height_cm = LaunchConfiguration("test_height_cm")
    test_forward_cm = LaunchConfiguration("test_forward_cm")
    test_yaw_deg = LaunchConfiguration("test_yaw_deg")
    test_yaw_step_deg = LaunchConfiguration("test_yaw_step_deg")

    return LaunchDescription([
        DeclareLaunchArgument("test_height_cm", default_value="100.0",
                              description="测试悬停高度 cm（起飞/前进/旋转/返航都用它）"),
        DeclareLaunchArgument("test_forward_cm", default_value="200.0",
                              description="沿 map +x 前进距离 cm"),
        DeclareLaunchArgument("test_yaw_deg", default_value="180.0",
                              description="原地旋转目标偏航 deg（会转过去再转回 0）"),
        DeclareLaunchArgument("test_yaw_step_deg", default_value="180.0",
                              description="偏航分步步长 deg：默认 180=一口气连续转；设更小值(如90/45)则分段转每步停一下"),

        Node(
            package="inventory_control_pkg",
            executable="inventory_mission_node",
            name="inventory_mission_node",
            output="screen",
            parameters=[{
                "mode": "rotate_test",
                "test_height_cm": test_height_cm,
                "test_forward_cm": test_forward_cm,
                "test_yaw_deg": test_yaw_deg,
                "test_yaw_step_deg": test_yaw_step_deg,
                # home 默认 (0,0)：起飞点即 cartographer 地图原点。若起飞点非原点，按场地标定覆盖。
                "home_x_cm": 0.0,
                "home_y_cm": 0.0,
            }],
        ),
    ])
