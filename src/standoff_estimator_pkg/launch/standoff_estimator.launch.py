from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    scan_topic = LaunchConfiguration("scan_topic")
    sector_center_deg = LaunchConfiguration("sector_center_deg")
    sector_half_width_deg = LaunchConfiguration("sector_half_width_deg")

    return LaunchDescription([
        DeclareLaunchArgument("scan_topic", default_value="/scan"),
        # 雷达 x 轴若和相机朝向有偏差，用它把扇区中心转到相机正前方
        DeclareLaunchArgument("sector_center_deg", default_value="0.0"),
        DeclareLaunchArgument("sector_half_width_deg", default_value="30.0"),
        Node(
            package="standoff_estimator_pkg",
            executable="standoff_estimator_node",
            name="standoff_estimator",
            output="screen",
            parameters=[{
                "scan_topic": scan_topic,
                "sector_center_deg": sector_center_deg,
                "sector_half_width_deg": sector_half_width_deg,
                "min_points": 10,
                "min_range_m": 0.05,
                "max_range_m": 8.0,
                "outlier_thresh_m": 0.08,
                "ema_alpha": 0.5,
            }],
        ),
    ])
