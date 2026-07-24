#!/usr/bin/env python3
"""
G 题 空地协同智能消防系统 — 无人机侧整套一把启动（飞控底座 + 任务 + 机车链路 + 视觉）。

= 飞控底座（carto + uart + 标准位置PID，与 inventory_full / spray_basic 同款三件套）
+ fire_control_pkg（巡逻/灭火状态机）
+ fire_link_pkg   （四条 UDP 通道：遥测/火源上报 → 车，按键启动/车端状态 ← 车）
+ fire_vision_pkg （下视红色火花识别 + 反投影，enable_fire_task=true 时才有意义）

⚠ 默认 enable_fire_task=false + auto_start=false：
  上电后**原地待命**，等消防车按键（UDP 8893）才起飞 → 起飞 18dm → 5 条带蛇形全覆盖
  → 返航降落。视觉不参与决策（仍会起节点出话题，方便边飞边看识别对不对）。
  视觉标定完（颜色阈值 + 相机内参 + 反投影方向）再开 enable_fire_task:=true 跑发挥部分。

组网（见《G题_机车通信接口约定.md》）：
  无人机自己做热点：机 10.42.0.1（网关）/ 消防车 10.42.0.163（接机端热点）。

常用：
  ros2 launch my_launch fire_full.launch.py                      # 比赛默认：等按键，只跑基本要求
  ros2 launch my_launch fire_full.launch.py auto_start:=true     # 调试：不等按键直接起飞
  ros2 launch my_launch fire_full.launch.py enable_fire_task:=true   # 带视觉跑发挥部分
  ros2 launch my_launch fire_full.launch.py enable_vision:=false     # 不起相机（省 CPU/纯链路调试）
  ros2 launch my_launch fire_full.launch.py enable_gui:=true         # 弹识别预览窗（需 DISPLAY，远程调试用）
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _include(package_name, launch_file, launch_arguments=None, condition=None):
    share = FindPackageShare(package=package_name).find(package_name)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", launch_file)),
        launch_arguments=(launch_arguments or {}).items(),
        condition=condition,
    )


def generate_launch_description() -> LaunchDescription:
    enable_fire_task = LaunchConfiguration("enable_fire_task")
    auto_start = LaunchConfiguration("auto_start")
    enable_vision = LaunchConfiguration("enable_vision")
    enable_gui = LaunchConfiguration("enable_gui")
    laser_pin = LaunchConfiguration("laser_pin")
    max_angular_velocity = LaunchConfiguration("max_angular_velocity")

    args = [
        DeclareLaunchArgument(
            "enable_fire_task", default_value="false",
            description="true=接视觉跑发挥部分(见红→降高→悬停3s→抛包→上报车)；false=只跑基本要求主链路"),
        DeclareLaunchArgument(
            "auto_start", default_value="false",
            description="true=不等消防车按键直接起飞（仅调试）。⚠ 比赛时必须 false"),
        DeclareLaunchArgument(
            "enable_vision", default_value="true",
            description="是否起下视识别节点。false=纯链路/主链路调试，省 CPU"),
        DeclareLaunchArgument(
            "enable_gui", default_value="false",
            description="true=弹识别预览窗+掩膜窗（需 DISPLAY，远程调试用；比赛关掉）"),
        # 机腹激光笔：香橙派 40pin PB0，WiringOP `gpio write`，低电平点亮。
        # ⚠ 这里填的是 WiringOP 的 wPi 编号，不是物理脚号也不是 PB0 这个名字。
        #   上板 `gpio readall` 查 PB0 那一行的 wPi 列，对不上就用 laser_pin:= 覆盖。
        DeclareLaunchArgument(
            "laser_pin", default_value="13",
            description="机腹激光笔 WiringOP wPi 引脚号(PB0)。用 gpio readall 核对"),
        DeclareLaunchArgument(
            "max_angular_velocity", default_value="30.0",
            description="偏航最大角速度 deg/s。本题全程 yaw=0 不转向，只在异常纠偏时用到"),
    ]

    # ── 飞控底座 ───────────────────────────────────────────────────────────
    #   carto 出 tf(map->laser_link)；uart 发 /height + 转 /target_velocity(0x31)
    #   + /drop_package → 飞控舵机帧 0x11（1=开1400抛投, 0=关700复位）；
    #   标准位置 PID（z 走 /height，不用面阵激光）。
    fly_carto = _include("my_carto_pkg", "fly_carto.launch.py", {"use_rviz": "false"})
    uart = _include("uart_to_stm32", "uart_to_stm32.launch.py")
    position_pid = _include(
        "pid_control_pkg", "position_pid_controller.launch.py",
        {"max_angular_velocity": max_angular_velocity},
    )

    # ── 任务状态机 ─────────────────────────────────────────────────────────
    mission = _include(
        "fire_control_pkg", "fire_mission.launch.py",
        {"enable_fire_task": enable_fire_task, "auto_start": auto_start},
    )

    # ── 机腹激光笔（香橙派 GPIO，非飞控帧）─────────────────────────────────
    # fire_mission_node 发 /magnet/cmd (1=亮/2=灭)，本节点执行 `gpio write`。
    # 不起它 → 激光全程不亮，基本要求的航迹指示直接没分。
    laser = Node(
        package="magnet_control_pkg",
        executable="magnet_control_node",
        name="laser_gpio_node",
        output="screen",
        parameters=[{
            # magnet_node 里 pin 声明为 int，launch 参数是字符串，必须显式给类型
            "pin": ParameterValue(laser_pin, value_type=int),
            "on_level": 0,      # PB0 低电平点亮
            "off_level": 1,
            "initial_off": True,
        }],
    )

    # ── 机-车 UDP 桥（车 10.42.0.163 / 本机 10.42.0.1，本机为热点源）────────
    # 必须起：按键启动包走这里进来，不起就永远等不到 /fire_start。
    link = _include("fire_link_pkg", "fire_link.launch.py")

    # ── 下视火源识别 ───────────────────────────────────────────────────────
    vision = _include(
        "fire_vision_pkg", "fire_vision.launch.py",
        {"enable_gui": enable_gui},
        condition=IfCondition(enable_vision),
    )

    return LaunchDescription(
        args + [fly_carto, uart, position_pid, mission, laser, link, vision])
