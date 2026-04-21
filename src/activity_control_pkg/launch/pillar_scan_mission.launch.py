from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="activity_control_pkg",
            executable="pillar_scan_mission",
            name="pillar_scan_mission",
            output="screen",
            parameters=[{
                "map_frame": "map",
                "laser_link_frame": "laser_link",
                "target_topic": "/target_position",
                "enable_topic": "/pillar_detect_enable",

                # 到达容差
                "position_tolerance_cm": 9.0,
                "yaw_tolerance_deg": 5.0,
                "height_tolerance_cm": 12.0,

                # 航点数值
                "flight_height_cm": 40.0,
                "land_height_cm": 4.0,
                "scan_end_x_cm": 250.0,

                # 访问 + 降落
                "landing_x_cm": 250.0,
                "landing_y_cm": -250.0,
                "pillar_visit_height_cm": 150.0,
                "pillar_hover_sec": 1.0,
                "pillar_wait_timeout_sec": 3.0,
            }],
        )
    ])
