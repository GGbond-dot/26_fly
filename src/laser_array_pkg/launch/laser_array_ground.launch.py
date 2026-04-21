from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    params = {
        "serial_port": "/dev/ttyS3",
        "baud_rate": 921600,
        # 空间滤波
        "percentile": 1.0,        # 无双峰时: 1.0=最大值, 0.8=80分位
        "cluster_gap": 0.20,      # m, 相邻束差距 > 此值认为柱子/地面双峰
        # 时间滤波
        "max_slew_rate": 1.5,     # m/s, 高度变化速率上限
        "ema_alpha": 0.6,         # 0~1, 越大越跟手
        "jump_threshold": 0.15,   # m, 降幅超此值 + 无远簇 -> 冻结输出
        "max_hold_frames": 20,    # 最多保持上一帧多少帧 (50Hz下20帧=0.4s)
        # 障碍检测
        "obstacle_margin": 0.20,  # m
        # 日志
        "log_period_sec": 0.5,    # 高度日志打印间隔
    }

    return LaunchDescription([
        Node(
            package="laser_array_pkg",
            executable="laser_array_ground_node",
            name="laser_array_ground_node",
            output="screen",
            parameters=[params],
        )
    ])
