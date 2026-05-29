from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 单独测条形码视觉：只起 barcode_camera_node，不开飞控/PID/撒药。
    #   ros2 launch my_launch barcode_test.launch.py
    #   另开一个终端：ros2 topic echo /barcode_text
    #
    # 与正式 spray_basic 里的 barcode 节点的区别（测试友好）：
    #   stop_after_first_publish=False → 读到一次不关相机，可连续对着调距离
    #   publish_duplicates=True        → 每帧解出来都发，topic 持续刷，方便看稳定性
    #
    # 前视/条码相机 front_cam = /dev/video2（水平朝前读杆塔条码）。
    # by-path 稳定路径；强制 MJPG（YUYV 高分辨率仅 3fps，条码会糊读不出）。
    # 条码细节多用 1280x720；相机装向导致画面逆时针 90° 才正 → rotate_code=2。
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
            "rotate_code": 2,             # 预览里条码是斜/倒的就调这个(0/1/2)转正
            "barcode_topic": "/barcode_text",
            "show_preview": True,
            "window_name": "barcode_test_preview",
            "publish_duplicates": True,   # 测试：每帧解出都发，看稳定性
            "stop_after_first_publish": False,  # 测试：读到一次不关相机，连续调距离
        }],
    )

    return LaunchDescription([
        barcode_camera_node,
    ])
