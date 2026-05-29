import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _include(package_name, launch_file, launch_arguments=None):
    package_share = FindPackageShare(package=package_name).find(package_name)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, "launch", launch_file)
        ),
        launch_arguments=(launch_arguments or {}).items(),
    )


def generate_launch_description():
    fly_carto_launch = _include(
        "my_carto_pkg",
        "fly_carto.launch.py",
        {"use_rviz": "false"},
    )

    uart_to_stm32_launch = _include(
        "uart_to_stm32",
        "uart_to_stm32.launch.py",
    )

    position_pid_controller_launch = _include(
        "pid_control_pkg",
        "position_pid_controller.launch.py",
    )

    spray_mission_launch = _include(
        "spray_control_pkg",
        "spray_mission.launch.py",
    )

    # 下视相机 down_cam = /dev/video0（指向地面，撒药看绿色）。
    # 用 by-path 稳定路径，重启/插拔后 videoN 号变了也不受影响。
    # 强制 MJPG；绿色识别不需要高清，用 640x480@30 减轻 USB/CPU（发挥部分两相机同开更稳）。
    DOWN_CAM = ("/dev/v4l/by-path/"
                "platform-xhci-hcd.11.auto-usb-0:1:1.0-video-index0")

    drone_camera_node = Node(
        package="drone_camera_pkg",
        executable="drone_camera_node",
        name="drone_camera_node",
        output="screen",
        parameters=[{
            "camera_device": DOWN_CAM,
            "fourcc": "MJPG",
            "frame_width": 640,
            "frame_height": 480,
            "fps": 30.0,
            "rotate_code": -1,   # 下视若装歪了：2=逆时针90
            "window_name": "down_camera_preview",
            "spray_allowed_topic": "/spray_allowed",
            "center_roi_width": 50,
            "center_roi_height": 50,
            "green_h_min": 35,   # 去掉黄色区
            "green_h_max": 85,   # 去掉青蓝区，只保纯绿
            "green_s_min": 45,   # 60→45：上次收太猛导致大量漏打，放宽
            "green_v_min": 45,   # 60→45：放宽，防止暗区绿格被误判灰
            "green_pixel_threshold": 180,  # 250→180：8%阈值，仍能排除白格偶发像素
        }],
    )

    # 前视/水平相机 front_cam = /dev/video2（朝前读杆塔条码）。
    # 默认一起开（两个相机都开）。基本要求阶段场上没杆塔，它只是出预览 +
    # 空跑 /barcode_text，mission 侧 enable_barcode_task=False 不会理会。
    # 条码细节多用 1280x720；画面逆时针90°才正 → rotate_code=2。
    FRONT_CAM = ("/dev/v4l/by-path/"
                 "platform-xhci-hcd.14.auto-usb-0:1:1.0-video-index0")

    barcode_camera_node = Node(
        package="barcode_camera_pkg",
        executable="barcode_camera_node",
        name="barcode_camera_node",
        output="screen",
        parameters=[{
            "camera_device": FRONT_CAM,
            "fourcc": "MJPG",
            "frame_width": 1280,
            "frame_height": 720,
            "fps": 30.0,
            "rotate_code": 2,   # 逆时针90°转正
            "barcode_topic": "/barcode_text",
            "show_preview": True,
            "window_name": "front_camera_preview",
            "publish_duplicates": False,
            # 基本要求阶段不希望它读到一次就把相机关掉（否则预览窗没了），设 False
            "stop_after_first_publish": False,
        }],
    )

    # 柱子检测（tf 版，map 系 ROI）：起飞后由 spray_mission_node 发 /pillar_detect_enable
    # 开/关检测窗，窗内持续把 /scan 点变到 map 系按 ROI 攒帧，关窗聚类后发 /detected_pillars。
    # 因为在 map 系框 ROI，无人机边飞边转都不影响——这是“空中识别”的关键。
    #
    # bbox 已按实测柱子位置 ~map(100,-100)cm 框 ±50cm（见下）。柱子细(3cm)，ROI 收窄到柱子
    # 真实位置附近才不会把墙/支架当柱子；上场跑 home 记 tf 校验栅格对齐后，若柱子实际位置偏了再微调。
    pillar_detector_tf_node = Node(
        package="pillar_detector_pkg",
        executable="pillar_detector_tf",
        name="pillar_detector_tf",
        output="screen",
        parameters=[{
            "scan_topic": "/scan",
            "enable_topic": "/pillar_detect_enable",
            "map_frame": "map",
            "laser_link_frame": "laser_link",

            # bbox：柱子实测约 map(100,-100)cm，框 ±50cm（1m×1m，容 carto 漂移、排除远处墙）。
            # 单位 m，坐标系=本项目 spray frame（home=原点, X 行向上, Y 列右为负）。
            "map_x_min_m":  0.5,
            "map_x_max_m":  1.5,
            "map_y_min_m": -1.5,
            "map_y_max_m": -0.5,

            # 细杆(3cm)+空中：单帧点少，放宽分组门槛
            "group_dist_m": 0.18,        # 3cm 柱子的几个点彼此很近
            "min_pts_per_group": 2,      # 4→2：2~3m 外细杆单帧可能只打到 2 个点
            "min_pillar_separation_m": 0.40,

            # 多帧投票（4s 窗 @ ~10-15Hz ≈ 40-60 帧）：靠投票压掉偶发噪点
            "cluster_merge_dist_m": 0.20,
            "min_votes": 8,
            "max_pillars": 4,

            "tf_timeout_sec": 0.05,
        }],
    )

    return LaunchDescription([
        fly_carto_launch,
        uart_to_stm32_launch,
        position_pid_controller_launch,
        drone_camera_node,
        barcode_camera_node,
        pillar_detector_tf_node,
        spray_mission_launch,
    ])
