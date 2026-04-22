from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """demo3 专用 PID 启动文件：

    唯一区别于 position_pid_controller.launch.py 的地方是把 /height 重映射到
    /laser_array/ground_height，让 z 轴闭环使用面阵激光（抗高台），避免飞过
    或下降到高台正上方时因点阵激光突变导致高度失稳。
    """
    pid_params = {
        "control_frequency": 50.0,
        "map_frame": "map",
        "laser_link_frame": "laser_link",

        "kp_xy": 0.8, "ki_xy": 0.0, "kd_xy": 0.2,
        "kp_yaw": 1.0, "ki_yaw": 0.0, "kd_yaw": 0.2,
        "kp_z": 1.0, "ki_z": 0.0, "kd_z": 0.2,

        "max_linear_velocity": 33.0,
        "max_angular_velocity": 30.0,
        "max_vertical_velocity": 30.0,

        "visual_kp_x": 0.08, "visual_ki_x": 0.0, "visual_kd_x": 0.01,
        "visual_kp_y": 0.08, "visual_ki_y": 0.0, "visual_kd_y": 0.01,
        "visual_pixel_deadzone": 5.0,
        "visual_max_xy_velocity": 20.0,
        "visual_data_timeout_sec": 0.5,
    }

    return LaunchDescription([
        Node(
            package="pid_control_pkg",
            executable="position_pid_controller",
            name="position_pid_controller",
            output="screen",
            parameters=[pid_params],
            remappings=[("/height", "/laser_array/ground_height")],
        )
    ])
