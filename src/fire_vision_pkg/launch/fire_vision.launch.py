from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        # 下视相机 = 之前植保/盘点用的 down_cam(/dev/video0)。by-path 稳定路径，插拔不变号。
        DeclareLaunchArgument(
            "camera_device",
            default_value="/dev/v4l/by-path/platform-xhci-hcd.11.auto-usb-0:1:1.0-video-index0",
            description="下视相机设备（by-path 稳定路径）"),
        DeclareLaunchArgument("enable_gui", default_value="false",
                              description="true=弹本地预览窗+掩膜窗（远程调试用，需 DISPLAY）"),

        Node(
            package='fire_vision_pkg',
            executable='fire_detector',
            name='fire_detector',
            output='screen',
            parameters=[{
                'camera_device': LaunchConfiguration("camera_device"),
                'frame_width': 640,
                'frame_height': 480,
                'fourcc': 'MJPG',        # 双USB相机必须 MJPG，否则 YUYV 只有 3fps
                'rotate_code': 2,        # 下视相机画面逆时针90°转正
                'lock_white_balance': True,

                # ── 相机内参：⚠️ 必须现场标定 ──
                # fx/fy<=0 时按 hfov_deg 粗推，只够把流程跑通。
                # 反投影误差会直接变成火源坐标误差，而抛包判定半径只有 3dm，
                # 18dm 高度下 1° 的角度误差就是 ~3cm 的地面误差，务必用棋盘格标一次。
                'fx_px': -1.0,
                'fy_px': -1.0,
                'cx_px': -1.0,           # <0 = 取图像中心
                'cy_px': -1.0,
                'hfov_deg': 60.0,

                # ── 相机安装 ──
                # cam_yaw_offset_deg=0 的约定：图像"上"=机体+x(前)，图像"右"=机体-y(右)
                # 装反了整个反投影方向就错，第一次试飞务必用"把图案放在机头正前方"验证。
                'cam_yaw_offset_deg': 0.0,
                'cam_offset_x_cm': 0.0,  # 相机光心相对 laser_link 的机体系偏移
                'cam_offset_y_cm': 0.0,

                # ── 颜色阈值：⚠️ 现场用 fire_color_tune 标 ──
                # 主判据是 LAB 的 a 通道（偏红程度），对光照不均比 HSV 色相稳。
                # 淡灰底布(240,240,240)的 S≈0，淡蓝坐标线(180,230,255)色相在另一头，
                # 光靠 S 这一条就能把场地背景干掉。
                'lab_a_min': 150,
                'hsv_s_min': 90,
                'hsv_v_min': 50,
                'morph_kernel': 5,

                # ── 形状/尺寸过滤 ──
                # 挡红色停车区的**主力**是物理尺寸判据：火苗 10×5cm，停车区 50×50cm，
                # 长边差 5 倍。它只需要激光高度，不需要 tf/定位/场地标定——
                # 而几何黑名单(blacklist_dm)排在反投影之后，定位一挂就是摆设，
                # 所以真正扛事的是这两行。
                'min_size_cm': 2.0,
                'max_size_cm': 20.0,     # 火苗长边 10cm，留 2 倍容差

                # 拿不到高度时的兜底像素阈值。固定像素值会随高度平方漂，
                # 18dm 标好的到 8dm 就差 5 倍，只是聊胜于无。
                'min_area_px': 60,
                'max_area_px': 6000,
                'min_fill_ratio': 0.25,
                'max_aspect_ratio': 4.0,

                # ── 几何黑名单（场地系 dm，[xmin,ymin,xmax,ymax,...]）──
                # 挡住左下角的起降区(黑,7×7)和消防车停车区(红,5×5)。
                # 红停车区与火源同色，面积过滤可能被边缘裁切骗过，
                # 而这块区域坐标已知——几何判据比任何颜色阈值都可靠。
                #
                # ⚠️ 别图省事框一个大矩形。左下那个街区实测 x6~14 / y11~20，火源就放在街区里——
                # 黑名单框大了会把街区下边缘的火源误丢。
                # 宁可框小也不框大：漏挡最多多一次误检（还有面积+投票兜着），框大直接丢分。
                'blacklist_dm': [
                    0.0, 0.0, 7.0, 7.0,     # 无人机起降区（黑），实测 7×7 不是 11×7
                    11.0, 0.0, 16.0, 5.0,   # 消防车停车区（红）
                ],
                # 下面三个必须与 fire_control_pkg 的同名参数完全一致，否则黑名单会框错地方
                'home_field_x_dm': 3.5,
                'home_field_y_dm': 3.5,
                # 必须与 fire_control_pkg 一致：飞机 map +x 指向场地 +y（40dm 边方向）
                'field_yaw_offset_deg': -90.0,

                # ── 多帧投票 ──
                # 单帧误检（顶部照明反光、红衣服飘过视野）不予采信。
                'vote_window': 5,
                'vote_min': 3,
                'vote_jitter_cm': 25.0,
                'min_height_cm': 30.0,   # 低于此高度不反投影（还在地面/刚起飞）

                # 默认 True：车端仪表盘要看无人机画面（通道 5），而桥是单独 launch 的，
                # 起桥时不用再记着加参数。代价只是常驻编一路 320 宽 @10Hz 的 JPEG。
                'enable_debug_image': True,

                # ── 通用图传（经 fire_video_link_pkg 桥给车/地面站看）──
                # 与火源检测**无关**的独立通道：读到帧就发 /camera/down/compressed，
                # 检测关掉也照发，默认发原始画面（不叠检测框）。
                # 优先级：UDP 控制包 > 视频帧率 > 画质。这几个值是照着
                # "绝不挤占 fire_link_pkg 控制包" 定的，往大调之前先想清楚。
                'stream_enable': True,
                'stream_annotate': False,   # True=画面上叠红色检测框（调视觉时才开）
                'stream_max_width': 320,
                'stream_jpeg_quality': 50,  # 超码率时自动往下降，最低 15
                'stream_max_hz': 10.0,
                'stream_max_kbps': 800.0,

                # 曝光：默认自动。要手动就**必须**同时给 exposure_value，
                # 只切手动不给值 = 曝光停在驱动默认的极短值 = 画面全黑。
                # 调法：让节点跑着，另开终端用 v4l2-ctl 实时试，见效再把值填这儿。
                #   v4l2-ctl -d $DEV --list-ctrls              # 看范围
                #   v4l2-ctl -d $DEV --set-ctrl=auto_exposure=1
                #   v4l2-ctl -d $DEV --set-ctrl=exposure_time_absolute=300
                #   v4l2-ctl -d $DEV --set-ctrl=gain=50
                'auto_exposure': True,
                'exposure_value': -1,      # auto_exposure=false 时必填
                'gain_value': -1,          # <0=不设。暗环境优先加这个，不掉帧率
                'brightness_value': -1,

                'debug_image_raw': False,   # 原始 Image，只在本机调试时开

                # ⚠ 必须包 ParameterValue(value_type=bool)：substitution 默认按字符串传，
                #   rclpy 侧 declare_parameter("enable_gui", False) 已推断成 BOOL，
                #   收到字符串会直接 InvalidParameterTypeException，节点起不来。
                'enable_gui': ParameterValue(
                    LaunchConfiguration("enable_gui"), value_type=bool),
                'map_frame': 'map',
                'base_frame': 'laser_link',
            }],
        ),
    ])
