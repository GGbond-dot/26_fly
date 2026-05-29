from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 前视/条码相机 front_cam = /dev/video2（水平朝前，读杆塔条码）。
    # by-path 稳定路径；强制 MJPG（否则 YUYV 高分辨率仅 3fps，条码读不出）。
    # 条码细节多，用 1280x720。相机装向导致画面逆时针 90° 才正 → rotate_code=2。
    FRONT_CAM = ("/dev/v4l/by-path/"
                 "platform-xhci-hcd.14.auto-usb-0:1:1.0-video-index0")

    return LaunchDescription([
        Node(
            package="barcode_camera_pkg",
            executable="barcode_camera_node",
            name="barcode_camera_node",
            output="screen",
            parameters=[
                {
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
                    "stop_after_first_publish": True,
                }
            ],
        )
    ])
