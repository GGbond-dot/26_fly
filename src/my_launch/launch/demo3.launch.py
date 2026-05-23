import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """demo3：在 demo2 的基础上，把扫描任务换成"抓取任务"(pillar_pickup_mission)，
    同时：
      1) visual_node 发布方框几何中心到 /fine_data，供 PID 视觉接管使用；
      2) PID 的 /height 重映射到 /laser_array/ground_height，z 轴闭环始终用面阵
         激光（抗高台），避免下降到高台正上方时点阵读数突变导致失稳。
    本文件不改动 demo2 相关启动。
    """
    show_display = LaunchConfiguration("show_display")

    # 包路径
    my_carto_pkg_share        = FindPackageShare(package='my_carto_pkg').find('my_carto_pkg')
    uart_to_stm32_pkg_share   = FindPackageShare(package='uart_to_stm32').find('uart_to_stm32')
    pid_control_pkg_share     = FindPackageShare(package='pid_control_pkg').find('pid_control_pkg')
    activity_control_pkg_share= FindPackageShare(package='activity_control_pkg').find('activity_control_pkg')
    pillar_detector_pkg_share = FindPackageShare(package='pillar_detector_pkg').find('pillar_detector_pkg')
    laser_array_pkg_share     = FindPackageShare(package='laser_array_pkg').find('laser_array_pkg')
    # 磁铁硬件已迁到 STM32，由 uart_to_stm32 通过 /electromagnet_control (帧 0x33) 控制。
    # GPIO 版 magnet_control_pkg 不再启动。

    fly_carto_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(my_carto_pkg_share, 'launch', 'fly_carto.launch.py')
        )
    )
    uart_to_stm32_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(uart_to_stm32_pkg_share, 'launch', 'uart_to_stm32.launch.py')
        )
    )
    # demo3 专用：PID z 轴用面阵（抗高台）
    position_pid_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pid_control_pkg_share, 'launch', 'position_pid_controller_ground.launch.py')
        )
    )

    # demo3 专用：抓取任务节点
    pillar_pickup_mission_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(activity_control_pkg_share, 'launch', 'pillar_pickup_mission.launch.py')
        )
    )
    pillar_detector_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pillar_detector_pkg_share, 'launch', 'pillar_detector_tf.launch.py')
        )
    )

    # visual_node 直接起（覆盖 center_source=square），不走 visual_pkg.launch.py
    visual_node = Node(
        package="visual_pkg",
        executable="visual_node",
        name="visual_node",
        output="screen",
        parameters=[{
            "camera_index": 0,
            "width": 640,
            "height": 480,
            "camera_fps": 30,
            "process_fps": 15.0,
            "show_display": show_display,
            "apriltag_code": -1,
            "center_source": "square",
            # 中心框优先指数：压掉对齐柱①时挤进画面边角的起停区 A 大方框(误锁→占比/对准跑偏)。
            # 越大越偏向画面正中的框；0=关闭(纯面积，旧行为)。试飞嫌还抢锁就加大(3~4)。
            "rect_center_bias": 2.0,
        }],
    )

    # 面阵激光地面高度（抗高台版）
    laser_array_ground_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(laser_array_pkg_share, 'launch', 'laser_array_ground.launch.py')
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument("show_display", default_value="false"),
        fly_carto_launch,
        uart_to_stm32_launch,
        position_pid_controller_launch,
        pillar_pickup_mission_launch,
        pillar_detector_tf_launch,
        visual_node,
        laser_array_ground_launch,
    ])
