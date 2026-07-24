#!/usr/bin/env python3
"""红色火花图案检测 + 地面坐标反投影（2023 电赛 G 题 空地协同智能消防系统）。

场上三个火花图案中**只有目标是红色**，另两个是其它颜色，所以本节点不做筛选——
检测到红色即认定为目标，交给 fire_control_pkg 处理。

关键设计
--------
1) **不用亮度门控**。火源已从"电池供电的红色 LED"降级为**印刷的红色火花图案**，
   是被动反光的，亮度随环境光走（题目明说会有顶部照明和窗外自然光、光照不均）。
   所以主判据是 **LAB 的 a 通道**（偏红程度，对光照不均比 HSV 稳），
   再用 HSV 的 S 做二次确认。淡灰底布 (240,240,240) 的 S 几乎为 0，
   淡蓝坐标线 (180,230,255) 的色相在另一头，这一条就能把整个场地背景干掉。

2) **必须锁白平衡**。相机自动白平衡会让红色在不同光照下漂，现场标的阈值到测试时就废了。
   开相机时把 AUTO_WB / AUTO_EXPOSURE 关掉（能不能关取决于驱动，关不掉会 WARN）。

3) **最大的误检源是红色消防车停车区**。它和火源图案是同一类红，但面积大得多
   （5dm vs ~1dm，18dm 高度下面积差 25 倍以上）。三重防线：
   面积上下限 + 形状过滤 + **几何黑名单**（落在起降区/停车区矩形内的检测直接丢弃）。
   几何黑名单比任何颜色阈值都可靠。

4) **多帧投票**。单帧误检（顶部照明反光、评委的红色衣物飘过视野）不予采信，
   要求最近 vote_window 帧里至少 vote_min 帧检测到、且位置抖动在 vote_jitter_cm 内。

反投影
------
相机垂直向下，针孔模型：地面偏移 = 像素偏移 × 高度 / 焦距。

    dx_img = (u - cx) * h / fx      # 图像"右"方向的地面偏移
    dy_img = (v - cy) * h / fy      # 图像"下"方向的地面偏移

再按 cam_yaw_offset_deg 转到机体系、按当前 yaw 转到 map 系、加上无人机自身位置。
机体系用 ROS REP-103：+x 前、+y 左。约定 cam_yaw_offset_deg=0 时
图像"上"(-v) 对应机体 +x、图像"右"(+u) 对应机体 -y。

发布
----
  /fire/detected      std_msgs/Bool                 本帧是否看到红色图案（原始，未投票）
  /fire/position_map  std_msgs/Float32MultiArray    [x_cm, y_cm]，**map 系**，已投票确认
  /fire/debug_image   sensor_msgs/Image             调试图原图（debug_image_raw 时，本机调试用）
  /camera/down/compressed       sensor_msgs/CompressedImage
                                    下视相机通用图传（JPEG，限频限带宽，经跨域桥给车/地面站）。
                                    **与火源检测无关**：读到帧就发，/fire/enable 关掉也照发，
                                    默认发原始画面；stream_annotate=True 才叠检测框。

订阅
----
  /fire/enable        std_msgs/Bool                 检测总开关（默认 True）
  /height             std_msgs/Int16                离地高度 cm，反投影必需
  tf: map -> laser_link                             无人机位姿
"""

from __future__ import annotations

import math
import os
import subprocess
import time

import cv2
import yaml
import numpy as np

import rclpy
from rclpy.node import Node

from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import Bool, Float32MultiArray, Int16
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge

import tf2_ros


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """四元数取 yaw。不依赖 tf_transformations —— 那个包不在 ros-humble 默认安装里，
    板子上未必有，为两行数学引一个可能缺失的依赖不划算。"""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class FireDetector(Node):
    def __init__(self) -> None:
        super().__init__("fire_detector")

        # ---------------- 相机 ----------------
        # 下视相机 = 之前植保/盘点用的 down_cam(/dev/video0)。by-path 稳定路径，插拔不变号。
        self.declare_parameter(
            "camera_device",
            "/dev/v4l/by-path/platform-xhci-hcd.11.auto-usb-0:1:1.0-video-index0")
        self.declare_parameter("frame_width", 640)
        self.declare_parameter("frame_height", 480)
        self.declare_parameter("fourcc", "MJPG")      # 双USB相机必须 MJPG，否则 YUYV 只有 3fps
        # rotate_code: -1=不转, 0=顺时针90, 1=180, 2=逆时针90（= cv2.ROTATE_* 取值）
        self.declare_parameter("rotate_code", 2)      # 下视相机画面逆时针90°转正=2
        self.declare_parameter("lock_white_balance", True)

        # ---------------- 相机内参（必须现场标定）----------------
        # fx/fy<=0 时用 hfov_deg 反推：fx = (W/2) / tan(hfov/2)。粗略但能先跑起来。
        self.declare_parameter("fx_px", -1.0)
        self.declare_parameter("fy_px", -1.0)
        self.declare_parameter("cx_px", -1.0)         # <0 = 取图像中心
        self.declare_parameter("cy_px", -1.0)
        self.declare_parameter("hfov_deg", 60.0)

        # ---------------- 相机安装 ----------------
        self.declare_parameter("cam_yaw_offset_deg", 0.0)
        self.declare_parameter("cam_offset_x_cm", 0.0)   # 相机光心相对 laser_link 的机体系偏移
        self.declare_parameter("cam_offset_y_cm", 0.0)

        # ---------------- 颜色阈值（现场必标）----------------
        self.declare_parameter("lab_a_min", 150)      # LAB a 通道，128=中性，越大越红
        self.declare_parameter("hsv_s_min", 90)       # 饱和度下限，淡灰底布 S≈0
        self.declare_parameter("hsv_v_min", 50)       # 亮度下限，只用来去掉纯黑
        self.declare_parameter("morph_kernel", 5)

        # ---------------- 形状过滤 ----------------
        self.declare_parameter("min_area_px", 60)
        self.declare_parameter("max_area_px", 6000)   # 拿不到高度时的兜底，会随高度漂
        # 物理尺寸判据（主力）：火苗 10x5cm，停车区 50x50cm。
        # 只需要激光高度，不需要 tf/定位/场地标定 —— 定位挂了它照样拦得住停车区。
        self.declare_parameter("min_size_cm", 2.0)
        self.declare_parameter("max_size_cm", 20.0)   # 火苗长边 10cm，留 2 倍容差
        self.declare_parameter("min_fill_ratio", 0.25)  # 轮廓面积 / 外接矩形面积
        self.declare_parameter("max_aspect_ratio", 4.0)

        # ---------------- 几何黑名单（场地系 dm）----------------
        # 起降区(黑,7×7) + 消防车停车区(红,5×5)。红停车区和火源同色，靠这条兜底。
        # 格式 [xmin,ymin,xmax,ymax, ...]，落在任一矩形内的检测直接丢弃。
        #
        # ⚠️ 别图省事框一个大矩形。左下那个街区实测 x6~14 / y11~20，火源就放在街区里——
        # 黑名单框大了会把街区下边缘的火源误丢。两块区域按题目图 1 像素反算，严格贴边框。
        self.declare_parameter("blacklist_dm", [
            0.0, 0.0, 7.0, 7.0,     # 无人机起降区（黑），实测 7×7 不是 11×7
            11.0, 0.0, 16.0, 5.0,   # 消防车停车区（红）——与火源同色，主要靠它
        ])
        self.declare_parameter("home_field_x_dm", 3.5)   # 与 fire_control_pkg 保持一致
        self.declare_parameter("home_field_y_dm", 3.5)
        self.declare_parameter("field_yaw_offset_deg", 0.0)

        # ---------------- 多帧投票 ----------------
        self.declare_parameter("vote_window", 5)
        self.declare_parameter("vote_min", 3)
        self.declare_parameter("vote_jitter_cm", 25.0)
        self.declare_parameter("min_height_cm", 30.0)    # 低于此高度不反投影（地面上/刚起飞）

        # ---------------- 调试 ----------------
        self.declare_parameter("enable_debug_image", False)
        self.declare_parameter("enable_gui", False)

        # ---- 通用图传（经 fire_video_link_pkg 的跨域桥给消防车/地面站看）----
        # 这是一条**与火源检测无关**的通用视频通道：只要相机读到帧就发，
        # 不受 /fire/enable 影响，也不管有没有检测到红色。默认发**原始画面**，
        # 想在画面上叠检测框才把 stream_annotate 打开。
        #
        # 优先级是死的：UDP 控制包(遥测/按键) > 视频流帧率 > 画质。
        # 所以限频限带宽，超带宽时**降画质保帧率**。
        self.declare_parameter("stream_enable", True)
        self.declare_parameter("stream_annotate", False)   # True=画面上叠检测框
        self.declare_parameter("stream_max_width", 320)
        self.declare_parameter("stream_jpeg_quality", 50)
        self.declare_parameter("stream_max_hz", 10.0)
        self.declare_parameter("stream_max_kbps", 800.0)
        self.declare_parameter("debug_image_raw", False)   # 原始 Image，本机调试才开
        self.declare_parameter("auto_exposure", True)      # False 时必须给 exposure_value
        self.declare_parameter("exposure_value", -1)
        # 暗环境优先加 gain 而不是拉曝光：曝光时间长会压帧率还拖影，
        # gain 是电子增益，不影响帧率，代价只是噪点。<0 = 不设，用驱动默认。
        self.declare_parameter("gain_value", -1)
        self.declare_parameter("brightness_value", -1)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "laser_link")

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.camera_device = str(g("camera_device"))
        self.frame_width = int(g("frame_width"))
        self.frame_height = int(g("frame_height"))
        self.fourcc = str(g("fourcc"))
        self.rotate_code = int(g("rotate_code"))
        self.lock_wb = bool(g("lock_white_balance"))
        self.fx = float(g("fx_px"))
        self.fy = float(g("fy_px"))
        self.cx = float(g("cx_px"))
        self.cy = float(g("cy_px"))
        self.hfov_deg = float(g("hfov_deg"))
        self.cam_yaw_offset = math.radians(float(g("cam_yaw_offset_deg")))
        self.cam_off_x_cm = float(g("cam_offset_x_cm"))
        self.cam_off_y_cm = float(g("cam_offset_y_cm"))
        self.lab_a_min = int(g("lab_a_min"))
        self.hsv_s_min = int(g("hsv_s_min"))
        self.hsv_v_min = int(g("hsv_v_min"))
        self.morph_k = max(1, int(g("morph_kernel")))
        self.min_area = int(g("min_area_px"))
        self.max_area = int(g("max_area_px"))
        self.min_size_cm = float(g("min_size_cm"))
        self.max_size_cm = float(g("max_size_cm"))
        self.min_fill = float(g("min_fill_ratio"))
        self.max_aspect = float(g("max_aspect_ratio"))
        blacklist = list(g("blacklist_dm"))
        self.blacklist = [tuple(blacklist[i:i + 4])
                          for i in range(0, len(blacklist) - 3, 4)]
        self.home_x_dm = float(g("home_field_x_dm"))
        self.home_y_dm = float(g("home_field_y_dm"))
        self.field_yaw = math.radians(float(g("field_yaw_offset_deg")))
        self.vote_window = max(1, int(g("vote_window")))
        self.vote_min = max(1, int(g("vote_min")))
        self.vote_jitter = float(g("vote_jitter_cm"))
        self.min_height_cm = float(g("min_height_cm"))
        self.enable_debug = bool(g("enable_debug_image"))
        self.enable_gui = bool(g("enable_gui"))
        self.stream_enable = bool(g("stream_enable"))
        self.stream_annotate = bool(g("stream_annotate"))
        self.dbg_raw = bool(g("debug_image_raw"))
        self.dbg_max_width = max(80, int(g("stream_max_width")))
        self.dbg_quality_max = min(95, max(10, int(g("stream_jpeg_quality"))))
        self.dbg_max_hz = max(0.5, float(g("stream_max_hz")))
        self.dbg_max_kbps = max(50.0, float(g("stream_max_kbps")))
        self.auto_exposure = bool(g("auto_exposure"))
        self.exposure_value = int(g("exposure_value"))
        self.gain_value = int(g("gain_value"))
        self.brightness_value = int(g("brightness_value"))
        self.map_frame = str(g("map_frame"))
        self.base_frame = str(g("base_frame"))

        # ---------------- 状态 ----------------
        self.enabled = True
        self.height_cm = None
        self.votes = []          # 最近若干帧的 (x_cm, y_cm) 或 None
        self.bridge = CvBridge()

        # ---------------- ROS 接口 ----------------
        self.detected_pub = self.create_publisher(Bool, "/fire/detected", 10)
        self.position_pub = self.create_publisher(
            Float32MultiArray, "/fire/position_map", 10)
        # 视觉伺服误差 [ex_norm, ey_norm, bx_cm, by_cm]，见 _process() 里的说明
        self.servo_pub = self.create_publisher(
            Float32MultiArray, "/fire/servo_error", 10)
        self.image_pub = self.create_publisher(Image, "/fire/debug_image", 10)
        # 压缩图走 sensor_data QoS（best_effort + depth 1）：图像丢帧无所谓，
        # 但 reliable 的重传会跟 fire_link_pkg 的 UDP 控制包抢带宽，那是要命的。
        self.image_pub_c = self.create_publisher(
            CompressedImage, "/camera/down/compressed", qos_profile_sensor_data)

        # 码率自适应状态：窗口内实际发了多少字节，据此在 quality 上下限间浮动
        self.dbg_quality = self.dbg_quality_max
        self._dbg_last_pub = 0.0
        self._dbg_win_start = 0.0
        self._dbg_win_bytes = 0

        self.create_subscription(Bool, "/fire/enable", self._on_enable, 10)
        self.create_subscription(Int16, "/height", self._on_height, 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._open_camera()
        self.timer = self.create_timer(1.0 / 30.0, self._tick)

        self.get_logger().info(
            f"火源检测节点已启动：{self.camera_device} "
            f"{self.frame_width}x{self.frame_height} rotate={self.rotate_code} "
            f"fx={self.fx:.1f} fy={self.fy:.1f} "
            f"投票 {self.vote_min}/{self.vote_window}")

    # ---------------------------------------------------------------- 相机
    def _open_camera(self) -> None:
        self.cap = cv2.VideoCapture(self.camera_device, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().error(f"打不开相机 {self.camera_device}")
            return
        if self.fourcc:
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)

        if self.lock_wb:
            # 自动白平衡会让红色随光照漂移，现场标好的阈值到测试时就失效了。
            # 有些 UVC 驱动不支持这个属性，set 会静默失败——所以读回来确认并 WARN。
            #
            # ⚠️ 这里**只锁白平衡，不碰曝光**。旧代码顺手设了 AUTO_EXPOSURE=1（手动）
            # 却从没设过曝光值，曝光量就停在驱动默认的极短值上 → 画面全黑
            # （2026-07-23 车端看到的黑屏就是这个）。曝光要手动就必须给具体值，
            # 见下面的 auto_exposure / exposure_value。
            self.cap.set(cv2.CAP_PROP_AUTO_WB, 0)
            if self.cap.get(cv2.CAP_PROP_AUTO_WB) != 0:
                self.get_logger().warning(
                    "白平衡锁定失败（驱动可能不支持 CAP_PROP_AUTO_WB）。"
                    "红色阈值会随光照漂移，现场务必在实际光照下重标 lab_a_min。")

        if not self.auto_exposure:
            # 手动曝光：V4L2 语义 1=手动 / 3=自动。必须同时给曝光值，否则就是黑屏。
            self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
            if self.exposure_value > 0:
                self.cap.set(cv2.CAP_PROP_EXPOSURE, self.exposure_value)
                self.get_logger().info(f"手动曝光，exposure={self.exposure_value}")
            else:
                self.get_logger().warning(
                    "auto_exposure=false 但没给 exposure_value，"
                    "曝光会停在驱动默认值（很可能是黑屏）。要么给值，要么用自动曝光。")

        if self.gain_value >= 0:
            self.cap.set(cv2.CAP_PROP_GAIN, self.gain_value)
        if self.brightness_value >= 0:
            self.cap.set(cv2.CAP_PROP_BRIGHTNESS, self.brightness_value)
        self.get_logger().info(
            f"曝光={'自动' if self.auto_exposure else self.exposure_value} "
            f"gain={self.gain_value if self.gain_value >= 0 else '默认'} "
            f"brightness={self.brightness_value if self.brightness_value >= 0 else '默认'}")

        self._apply_tuned_file()

    def _apply_tuned_file(self) -> None:
        """应用 camera_tune.py 存下来的相机参数（如果存过）。

        放在最后，所以它会**覆盖**上面那些 launch 参数——现场拖滑条调出来的值
        肯定比代码里写死的默认值靠谱。不想要了就删掉那个文件。

        用 v4l2-ctl 而不是 cap.set()：OpenCV 的属性映射在不同后端上时灵时不灵，
        而调参窗口本来就是用 v4l2-ctl 调的，用同一条路才能保证存进去的值
        和当时看到的画面一致。
        """
        path = os.path.expanduser("~/.ros/fire_vision_camera.yaml")
        if not os.path.exists(path):
            return
        try:
            with open(path) as f:
                data = yaml.safe_load(f) or {}
        except (OSError, yaml.YAMLError) as e:
            self.get_logger().warning(f"读 {path} 失败，忽略：{e}")
            return

        controls = data.get("controls") or {}
        if not controls:
            return

        applied, failed = [], []
        for name, value in controls.items():
            try:
                r = subprocess.run(
                    ["v4l2-ctl", "-d", self.camera_device,
                     f"--set-ctrl={name}={int(value)}"],
                    capture_output=True, text=True, timeout=5)
                (applied if r.returncode == 0 else failed).append(f"{name}={value}")
            except (subprocess.SubprocessError, FileNotFoundError, ValueError):
                failed.append(f"{name}={value}")

        if applied:
            self.get_logger().info(f"已应用调好的相机参数: {', '.join(applied)}")
        if failed:
            self.get_logger().warning(f"这些相机参数设置失败: {', '.join(failed)}")

        # 实际生效的分辨率可能与请求的不同，以实际为准算主点
        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or self.frame_width
        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or self.frame_height
        # rotate 90/270 会交换宽高，主点和焦距要按旋转后的画面算
        if self.rotate_code in (cv2.ROTATE_90_CLOCKWISE, cv2.ROTATE_90_COUNTERCLOCKWISE):
            actual_w, actual_h = actual_h, actual_w
        self.img_w, self.img_h = actual_w, actual_h

        if self.cx < 0:
            self.cx = self.img_w / 2.0
        if self.cy < 0:
            self.cy = self.img_h / 2.0
        if self.fx <= 0 or self.fy <= 0:
            # 用视场角粗推。只够把流程跑通，正式测试前必须用棋盘格标定。
            f = (self.img_w / 2.0) / math.tan(math.radians(self.hfov_deg) / 2.0)
            self.fx = self.fy = f
            self.get_logger().warning(
                f"未提供 fx/fy，按 hfov={self.hfov_deg}° 粗推 f={f:.1f}px。"
                "反投影误差会直接变成火源坐标误差（抛包判定半径只有 3dm），"
                "正式测试前必须标定。")

    # ---------------------------------------------------------------- 回调
    def _on_enable(self, msg: Bool) -> None:
        if msg.data != self.enabled:
            self.get_logger().info(f"检测{'开启' if msg.data else '关闭'}")
        self.enabled = msg.data

    def _on_height(self, msg: Int16) -> None:
        self.height_cm = float(msg.data)

    # ---------------------------------------------------------------- 坐标
    def _get_pose(self):
        """返回 (x_cm, y_cm, yaw_rad)，map 系。取不到返回 None。"""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time())
        except Exception as exc:                       # noqa: BLE001
            self.get_logger().warning(
                f"取不到 {self.map_frame}->{self.base_frame} 的 tf：{exc}",
                throttle_duration_sec=2.0)
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        return t.x * 100.0, t.y * 100.0, _yaw_from_quaternion(q.x, q.y, q.z, q.w)

    def _pixel_to_body(self, u: float, v: float, height_cm: float):
        """像素坐标 → 机体系地面偏移 cm（REP-103：+x 前，+y 左）。相机垂直向下，针孔模型。"""
        # 1) 像素偏移 → 地面偏移（图像坐标：+u 右，+v 下）
        dx_img = (u - self.cx) * height_cm / self.fx
        dy_img = (v - self.cy) * height_cm / self.fy

        # 2) 图像 → 机体。约定 cam_yaw_offset=0 时：
        #    图像"上"(-v)=机体 +x，图像"右"(+u)=机体 -y（已由实机确认）
        bx, by = -dy_img, -dx_img
        ca, sa = math.cos(self.cam_yaw_offset), math.sin(self.cam_yaw_offset)
        bx, by = ca * bx - sa * by, sa * bx + ca * by
        return bx + self.cam_off_x_cm, by + self.cam_off_y_cm

    def _pixel_to_map(self, u: float, v: float, pose, height_cm: float):
        """像素坐标 → map 系 cm。"""
        drone_x, drone_y, yaw = pose
        bx, by = self._pixel_to_body(u, v, height_cm)
        cy_, sy_ = math.cos(yaw), math.sin(yaw)
        return (drone_x + cy_ * bx - sy_ * by,
                drone_y + sy_ * bx + cy_ * by)

    def _map_to_field_dm(self, x_cm: float, y_cm: float):
        """map 系 cm → 场地系 dm。与 fire_control_pkg::mapToField 保持一致。"""
        c, s = math.cos(self.field_yaw), math.sin(self.field_yaw)
        dx = c * x_cm + s * y_cm
        dy = -s * x_cm + c * y_cm
        return dx / 10.0 + self.home_x_dm, dy / 10.0 + self.home_y_dm

    def _in_blacklist(self, x_cm: float, y_cm: float) -> bool:
        """落在起降区/消防车停车区里的检测直接丢弃。

        红色停车区和火源图案同色，面积过滤有可能被边缘裁切骗过去，
        而这两个区域的坐标是已知的——几何判据比任何颜色阈值都可靠。
        """
        fx_dm, fy_dm = self._map_to_field_dm(x_cm, y_cm)
        for xmin, ymin, xmax, ymax in self.blacklist:
            if xmin <= fx_dm <= xmax and ymin <= fy_dm <= ymax:
                return True
        return False

    # ---------------------------------------------------------------- 检测
    def _find_red(self, frame):
        """返回最佳候选的 (u, v, area, bbox)，没有则 None。"""
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 主判据 LAB 的 a 通道（偏红程度），对光照不均比 HSV 色相稳；
        # S/V 只做二次确认，去掉淡灰底布(S≈0)和纯黑。
        mask = ((lab[:, :, 1] >= self.lab_a_min) &
                (hsv[:, :, 1] >= self.hsv_s_min) &
                (hsv[:, :, 2] >= self.hsv_v_min)).astype(np.uint8) * 255

        k = np.ones((self.morph_k, self.morph_k), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
        self._last_mask = mask

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best = None
        for c in contours:
            area = cv2.contourArea(c)
            if not (self.min_area <= area <= self.max_area):
                continue
            x, y, w, h = cv2.boundingRect(c)
            if w == 0 or h == 0:
                continue

            # 物理尺寸判据：把像素长边按当前高度换算成实际厘米。
            # 目标在地面、相机下视，故 长边_cm = 长边_px * 高度_cm / fx。
            #
            # 这是**唯一不依赖 tf / 定位 / 场地标定**的几何防线，只要激光测得到高度就能用。
            # 火苗 10x5cm，消防车停车区 50x50cm —— 差 5 倍，比任何颜色阈值都硬。
            # 几何黑名单（_in_blacklist）排在反投影之后，定位一挂就是摆设，
            # 这条挡在前面，定位挂了照样拦得住停车区。
            #
            # 拿不到高度时退回下面的固定像素阈值兜底（会随高度漂，只是聊胜于无）。
            if (self.height_cm is not None
                    and self.height_cm >= self.min_height_cm and self.fx > 0):
                long_cm = max(w, h) * float(self.height_cm) / self.fx
                if not (self.min_size_cm <= long_cm <= self.max_size_cm):
                    continue

            if area / float(w * h) < self.min_fill:
                continue
            aspect = max(w / h, h / w)
            if aspect > self.max_aspect:
                continue
            m = cv2.moments(c)
            if m["m00"] == 0:
                continue
            u = m["m10"] / m["m00"]
            v = m["m01"] / m["m00"]
            if best is None or area > best[2]:
                best = (u, v, area, (x, y, w, h))
        return best

    def _vote(self, position):
        """把本帧结果压入投票窗；够票且位置收敛则返回确认坐标，否则 None。"""
        self.votes.append(position)
        if len(self.votes) > self.vote_window:
            del self.votes[:len(self.votes) - self.vote_window]

        hits = [p for p in self.votes if p is not None]
        if len(hits) < self.vote_min:
            return None
        xs = [p[0] for p in hits]
        ys = [p[1] for p in hits]
        mx, my = sum(xs) / len(xs), sum(ys) / len(ys)
        # 位置抖动太大说明是误检或反投影不稳，不予采信
        if max(math.hypot(x - mx, y - my) for x, y in hits) > self.vote_jitter:
            return None
        return mx, my

    # ---------------------------------------------------------------- 主循环
    def _tick(self) -> None:
        if not hasattr(self, "cap") or not self.cap.isOpened():
            return
        ok, frame = self.cap.read()
        if not ok or frame is None:
            self.get_logger().warning("读相机失败", throttle_duration_sec=2.0)
            return
        if self.rotate_code in (cv2.ROTATE_90_CLOCKWISE, cv2.ROTATE_180,
                                cv2.ROTATE_90_COUNTERCLOCKWISE):
            frame = cv2.rotate(frame, self.rotate_code)

        # 通用图传：读到帧就发，**放在检测流程之前**。
        # 这样 /fire/enable 关掉、或者压根没在找火源时，车端画面照样是连续的
        # ——它是一条独立的图传通道，不是检测的副产品。
        # stream_annotate=True 时改在 _publish_debug 里发标注图（那里才有框）。
        if self.stream_enable and not self.stream_annotate:
            self._publish_stream(frame)

        if not self.enabled:
            self.votes.clear()
            return

        self._last_mask = None
        best = self._find_red(frame)

        det = Bool()
        det.data = best is not None
        self.detected_pub.publish(det)

        # 视觉伺服误差：每帧发，供 fire_control_pkg 做闭环对中。
        # 与 /fire/position_map 的区别是**不做投票、不查黑名单、不需要 tf** ——
        # 对中是相对量，只要看得见就能修正；投票那条是给"确认+上报"用的绝对坐标。
        #   [0] ex_norm  归一化像素误差，+ = 目标在画面右侧（机体 -y）
        #   [1] ey_norm  归一化像素误差，+ = 目标在画面下方（机体 -x）
        #   [2] bx_cm    机体系前向偏移（+x 前）
        #   [3] by_cm    机体系左向偏移（+y 左）
        # 相机与抛投口视为同 z 轴，故对中目标就是画面中心，无需额外瞄准点标定。
        if best is not None and self.height_cm is not None:
            ex = (best[0] - self.cx) / float(self.img_w)
            ey = (best[1] - self.cy) / float(self.img_h)
            bx, by = self._pixel_to_body(best[0], best[1], float(self.height_cm))
            servo = Float32MultiArray()
            servo.data = [float(ex), float(ey), float(bx), float(by)]
            self.servo_pub.publish(servo)

        position = None
        rejected = None
        if best is not None:
            pose = self._get_pose()
            if pose is None:
                rejected = "无 tf"
            elif self.height_cm is None:
                rejected = "无高度"
            elif self.height_cm < self.min_height_cm:
                rejected = f"高度过低({self.height_cm:.0f}cm)"
            else:
                x_cm, y_cm = self._pixel_to_map(best[0], best[1], pose, self.height_cm)
                if self._in_blacklist(x_cm, y_cm):
                    rejected = "落在黑名单区（起降区/停车区）"
                else:
                    position = (x_cm, y_cm)

        if rejected:
            self.get_logger().info(f"检测到红色但丢弃：{rejected}",
                                   throttle_duration_sec=2.0)

        confirmed = self._vote(position)
        if confirmed is not None:
            msg = Float32MultiArray()
            msg.data = [float(confirmed[0]), float(confirmed[1])]
            self.position_pub.publish(msg)
            fx_dm, fy_dm = self._map_to_field_dm(*confirmed)
            self.get_logger().info(
                f"火源确认：map=({confirmed[0]:.0f}, {confirmed[1]:.0f})cm "
                f"场地=({fx_dm:.1f}, {fy_dm:.1f})dm",
                throttle_duration_sec=1.0)

        if self.enable_debug or self.enable_gui:
            self._publish_debug(frame, best, confirmed)

    def _publish_debug(self, frame, best, confirmed) -> None:
        vis = frame.copy()
        if best is not None:
            x, y, w, h = best[3]
            cv2.rectangle(vis, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(vis, (int(best[0]), int(best[1])), 4, (0, 255, 255), -1)
            cv2.putText(vis, f"area={int(best[2])}", (x, max(14, y - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        txt = "CONFIRMED" if confirmed else ("seen" if best else "-")
        cv2.putText(vis, f"{txt}  h={self.height_cm}", (8, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        if self.enable_debug and self.dbg_raw:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(vis, "bgr8"))
        if self.stream_enable and self.stream_annotate:
            self._publish_stream(vis)
        if self.enable_gui:
            cv2.imshow("fire_detector", vis)
            if self._last_mask is not None:
                cv2.imshow("fire_mask", self._last_mask)
            cv2.waitKey(1)

    def _publish_stream(self, vis) -> None:
        """通用图传：发 JPEG 画面给消防车/地面站（经跨域桥）。

        三条限制按优先级叠加，目的是**绝不挤占 fire_link_pkg 的 UDP 控制包**：
          1. 限频   —— 检测循环 30Hz，但最多按 stream_max_hz 发
          2. 限分辨率 —— 缩到 stream_max_width 以内
          3. 限码率 —— 超过 stream_max_kbps 就降 JPEG 质量，**不降帧率**
                       （定好的优先级：速度比画质重要）
        """
        now = time.monotonic()
        if now - self._dbg_last_pub < 1.0 / self.dbg_max_hz:
            return
        self._dbg_last_pub = now

        h, w = vis.shape[:2]
        if w > self.dbg_max_width:
            scale = self.dbg_max_width / float(w)
            vis = cv2.resize(vis, (self.dbg_max_width, max(1, int(round(h * scale)))),
                             interpolation=cv2.INTER_AREA)

        ok, buf = cv2.imencode(".jpg", vis,
                               [int(cv2.IMWRITE_JPEG_QUALITY), int(self.dbg_quality)])
        if not ok:
            self.get_logger().warn("JPEG 编码失败，跳过本帧", throttle_duration_sec=5.0)
            return

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.format = "jpeg"
        msg.data = buf.tobytes()
        self.image_pub_c.publish(msg)

        self._adapt_quality(now, len(msg.data))

    def _adapt_quality(self, now: float, nbytes: int) -> None:
        """按实测码率在 [15, stream_jpeg_quality] 之间调 JPEG 质量。

        每秒结算一次：超上限就砍质量，明显富余就慢慢加回去。
        慢加快减是防止在阈值附近来回抖，画面质量忽好忽坏比一直偏糊更难看。
        """
        if self._dbg_win_start == 0.0:
            self._dbg_win_start = now
        self._dbg_win_bytes += nbytes

        elapsed = now - self._dbg_win_start
        if elapsed < 1.0:
            return

        kbps = self._dbg_win_bytes * 8.0 / 1000.0 / elapsed
        self._dbg_win_start = now
        self._dbg_win_bytes = 0

        old = self.dbg_quality
        if kbps > self.dbg_max_kbps:
            self.dbg_quality = max(15, self.dbg_quality - 10)
        elif kbps < self.dbg_max_kbps * 0.7:
            self.dbg_quality = min(self.dbg_quality_max, self.dbg_quality + 2)

        if self.dbg_quality != old:
            self.get_logger().info(
                f"调试图码率 {kbps:.0f} kbps（上限 {self.dbg_max_kbps:.0f}），"
                f"JPEG 质量 {old} → {self.dbg_quality}",
                throttle_duration_sec=5.0)

    def destroy_node(self) -> bool:
        if hasattr(self, "cap") and self.cap.isOpened():
            self.cap.release()
        if self.enable_gui:
            cv2.destroyAllWindows()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FireDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
