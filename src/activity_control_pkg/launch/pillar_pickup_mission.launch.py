from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="activity_control_pkg",
            executable="pillar_pickup_mission",
            name="pillar_pickup_mission",
            output="screen",
            parameters=[{
                "map_frame": "map",
                "laser_link_frame": "laser_link",

                # 到达容差
                "position_tolerance_cm": 9.0,
                "yaw_tolerance_deg": 5.0,
                "height_tolerance_cm": 12.0,

                # 航线数值
                "flight_height_cm": 40.0,
                "land_height_cm": 4.0,
                "scan_end_x_cm": 250.0,
                "landing_x_cm": 250.0,
                "landing_y_cm": -250.0,
                "pillar_visit_height_cm": 150.0,
                "pillar_wait_timeout_sec": 3.0,

                # 视觉对准
                "visual_align1_timeout_sec": 4.0,
                "visual_align2_timeout_sec": 1.5,
                "visual_pixel_tol_px": 15,
                "visual_align_required_hits": 3,
                "visual_jump_px": 100,
                "visual_stale_sec": 0.5,

                # 摄像头→点阵激光 xy 偏置（cm，机体系）——待测
                "cam_offset_dx_cm": 0.0,
                "cam_offset_dy_cm": 0.0,

                # 高度采样
                "sample_duration_sec": 2.5,
                "sample_min_pillar_frames": 8,
                "sample_pillar_drop_thresh_cm": 20.0,

                # 下降 / 抓取
                "mid_clearance_cm": 30.0,
                "grab_clearance_cm": 5.0,
                "hover_grab_sec": 2.0,
            }],
        )
    ])
