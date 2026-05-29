from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # G 题用：单杆塔检测，地图范围内找一根柱子，输出 /detected_pillar。
    return LaunchDescription([
        Node(
            package="pillar_detector_pkg",
            executable="pillar_detector_single_node",
            name="pillar_detector_single",
            output="screen",
            parameters=[{
                "scan_topic": "/scan",
                "output_topic": "/detected_pillar",
                "map_x_min_m": 0.5,
                "map_x_max_m": 2.0,
                "map_y_min_m": -2.0,
                "map_y_max_m": -0.5,
                "group_dist_m": 0.25,
                "min_pts_per_group": 4,
                "min_pillar_separation_m": 0.40,
                "accumulation_frames": 20,
                "cluster_merge_dist_m": 0.20,
                "min_votes": 8,
            }],
        )
    ])
