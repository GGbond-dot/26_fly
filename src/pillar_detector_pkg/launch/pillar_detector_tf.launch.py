from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pillar_detector_pkg',
            executable='pillar_detector_tf',
            name='pillar_detector_tf',
            output='screen',
            parameters=[{
                # ── 话题 / 坐标系 ──
                'scan_topic': '/scan',
                'enable_topic': '/pillar_detect_enable',
                'map_frame': 'map',
                'laser_link_frame': 'laser_link',

                # ── 柱子有效世界 bbox (map 系, 单位 m) ──
                # 起飞点 arena(25,25) = map(0,0)
                # arena 柱子区 x∈[50,250]cm, y∈[50,250]cm
                #   → map_x = arena_x - 25,  map_y = 25 - arena_y
                #   → map_x ∈ [0.25, 2.25] m,  map_y ∈ [-2.25, -0.25] m
                'map_x_min_m':  0.25,
                'map_x_max_m':  2.25,
                'map_y_min_m': -2.25,
                'map_y_max_m': -0.25,

                # ── 单帧分组 ──
                'group_dist_m': 0.25,
                'min_pts_per_group': 4,
                'min_pillar_separation_m': 0.40,

                # ── 多帧聚类 ──
                'cluster_merge_dist_m': 0.20,
                'min_votes': 8,
                'max_pillars': 4,

                # ── TF 查询超时 ──
                'tf_timeout_sec': 0.05,
            }],
        )
    ])
