#!/usr/bin/env python3
"""
单相机二维码识别节点（D 题 立体货架盘点）

来源：24fly opencv01.decoder_common 的 QRVisionNodeBase，按教练组要求改成
"单相机 + 空中 yaw 旋转扫面"方案：
  - 去掉了左右相机 split（camera_side_expected / current_target_camera）。
  - 新增 rotate_code 适配当前相机安装方位（与 24fly 不同）。
  - 新增 /qr_vision/enable 门控：任务节点只在到位盘点时开识别/激光，
    平时关闭，避免飞行途中误打激光、误盘点。

发布（topic_prefix 默认 /qr_vision）：
  {prefix}/id          std_msgs/String   识别到的二维码文本（货物编号 "1".."24"）
  {prefix}/offset_norm geometry_msgs/Point  归一化像素偏移 x=ex(右正), y=ey(下正)
  {prefix}/aligned     std_msgs/Bool     是否已对准中心
  {prefix}/debug_image sensor_msgs/Image 调试图（enable_debug_image 时）

订阅：
  /qr_vision/enable    std_msgs/Bool     识别+激光总开关（默认 True，便于单测）

激光：识别到二维码且横向已进入 eps_x_laser 窗口、且与上次不同码时，
      起一个子线程把 laser_pin 拉低 0.5s（亮）再恢复，对应题目"激光在二维码
      范围内点亮 0.5 秒"。laser_pin=-1 时不操作 GPIO（无硬件调试用）。
"""

from __future__ import annotations

import os
import threading
import time

import cv2
from pyzbar import pyzbar

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

from std_msgs.msg import String, Bool
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

try:
    import wiringpi
    from wiringpi import GPIO
    _HAS_WIRINGPI = True
except ImportError:  # 本机/无硬件环境下允许跑（laser_pin 会被强制 -1）
    wiringpi = None
    GPIO = None
    _HAS_WIRINGPI = False


class QRVisionNode(Node):
    def __init__(self, node_name: str = "qr_vision_node",
                 topic_prefix: str = "/qr_vision",
                 default_camera_device: str = "/dev/video0") -> None:
        super().__init__(node_name)

        # ---------------- 参数声明 ----------------
        self.declare_parameter("camera_device", default_camera_device)
        self.declare_parameter("frame_width", 640)
        self.declare_parameter("frame_height", 480)
        self.declare_parameter("fourcc", "MJPG")          # 双USB相机必须 MJPG 否则 YUYV 3fps
        # rotate_code: -1=不转, 0=顺时针90, 1=180, 2=逆时针90（= cv2.ROTATE_* 取值）
        self.declare_parameter("rotate_code", -1)         # 当前相机方位与 24fly 不同，按需设
        self.declare_parameter("eps_x", 0.40)             # 对准判定横向阈值（归一化）
        self.declare_parameter("eps_y", 0.40)             # 对准判定纵向阈值
        self.declare_parameter("eps_x_laser", 0.25)       # 触发激光的更严横向阈值
        self.declare_parameter("stable_frames", 1)        # 连续 N 帧在窗内才算 aligned
        self.declare_parameter("enable_debug_image", False)
        self.declare_parameter("enable_gui", False)
        self.declare_parameter("decode_interval", 3)      # 每 N 帧解码一次（降 CPU）
        self.declare_parameter("laser_pin", -1)           # wiringPi 引脚号，-1=不控激光

        # ---------------- 参数读取 ----------------
        self.camera_device = self.get_parameter("camera_device").value
        self.frame_width = int(self.get_parameter("frame_width").value)
        self.frame_height = int(self.get_parameter("frame_height").value)
        self.fourcc = str(self.get_parameter("fourcc").value)
        self.rotate_code = int(self.get_parameter("rotate_code").value)
        self.eps_x = self.get_parameter("eps_x").value
        self.eps_y = self.get_parameter("eps_y").value
        self.eps_x_laser = self.get_parameter("eps_x_laser").value
        self.stable_frames = self.get_parameter("stable_frames").value
        self.enable_debug = self.get_parameter("enable_debug_image").value
        self.enable_gui = self.get_parameter("enable_gui").value
        self.decode_interval = max(1, int(self.get_parameter("decode_interval").value))
        self.laser_pin = int(self.get_parameter("laser_pin").value)
        if not _HAS_WIRINGPI:
            self.laser_pin = -1

        self.add_on_set_parameters_callback(self._on_parameter_change)
        self._update_window_status()

        # 话题前缀
        prefix = topic_prefix.strip().rstrip("/")
        self.topic_prefix = prefix if prefix.startswith("/") else "/" + prefix

        # 发布器
        self.qr_id_pub = self.create_publisher(String, f"{self.topic_prefix}/id", 10)
        self.offset_pub = self.create_publisher(Point, f"{self.topic_prefix}/offset_norm", 10)
        self.aligned_pub = self.create_publisher(Bool, f"{self.topic_prefix}/aligned", 10)
        self.image_pub = self.create_publisher(Image, f"{self.topic_prefix}/debug_image", 10)

        # 识别+激光总开关：任务节点到位盘点时置 True，平时 False
        self.detect_enabled = True
        self.enable_sub = self.create_subscription(
            Bool, "/qr_vision/enable", self._on_enable, 10)

        self.bridge = CvBridge()

        # 摄像头初始化
        self.cap = cv2.VideoCapture(self.camera_device)
        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open camera {self.camera_device}")
            raise RuntimeError("Camera open failed")
        if self.fourcc:
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)

        self.stable_count = 0
        self.last_qr_id = ""
        self.frame_count = 0
        self.previous_qr_data = ""

        self.gpio_initialized = False
        self._init_gpio()

        self.timer = self.create_timer(1.0 / 30.0, self.process_frame)
        self.get_logger().info(
            f"{node_name} started on {self.camera_device} "
            f"(rotate_code={self.rotate_code}, laser_pin={self.laser_pin}).")

    # ------------------------------------------------------ GPIO / 激光
    def _init_gpio(self):
        if not self.gpio_initialized and self.laser_pin != -1:
            if wiringpi.wiringPiSetup() == -1:
                self.get_logger().error("GPIO 初始化失败！请检查权限。")
                self.laser_pin = -1
            else:
                wiringpi.pinMode(self.laser_pin, GPIO.OUTPUT)
                wiringpi.digitalWrite(self.laser_pin, GPIO.HIGH)  # 高=灭
                self.gpio_initialized = True
                self.get_logger().info(f"GPIO {self.laser_pin} 初始化成功 (wPi编号)")

    def _fire_laser_worker(self):
        """子线程：拉低 0.5s（激光亮）再恢复，不阻塞识别主循环。"""
        if self.laser_pin == -1:
            return
        try:
            self.get_logger().info(f"==> 激光发射! (Pin {self.laser_pin})")
            wiringpi.digitalWrite(self.laser_pin, GPIO.LOW)
            time.sleep(0.5)
            wiringpi.digitalWrite(self.laser_pin, GPIO.HIGH)
            self.get_logger().info("==> 激光关闭")
        except Exception as e:
            self.get_logger().error(f"激光发射失败: {e}")

    # ------------------------------------------------------ 动态控制
    def _on_enable(self, msg: Bool):
        self.detect_enabled = bool(msg.data)

    def _update_window_status(self):
        has_display = os.environ.get("DISPLAY") is not None
        self.should_show_window = self.enable_gui and has_display
        if not self.should_show_window:
            try:
                cv2.destroyWindow(self.get_name())
            except Exception:
                pass

    def _on_parameter_change(self, params):
        for param in params:
            if param.name == "enable_gui":
                self.enable_gui = param.value
                self._update_window_status()
                self.get_logger().info(f"GUI status -> {self.enable_gui}")
        return SetParametersResult(successful=True)

    # ------------------------------------------------------ 主循环
    def process_frame(self) -> None:
        ret, frame = self.cap.read()
        if not ret:
            return

        # 关闭识别时只读帧清缓冲，不解码不打激光
        if not self.detect_enabled:
            self.stable_count = 0
            return

        if self.rotate_code in (cv2.ROTATE_90_CLOCKWISE,
                                cv2.ROTATE_180,
                                cv2.ROTATE_90_COUNTERCLOCKWISE):
            frame = cv2.rotate(frame, self.rotate_code)

        self.frame_count += 1
        img_h, img_w = frame.shape[:2]
        img_cx, img_cy = img_w / 2.0, img_h / 2.0

        decoded_objects = []
        if self.frame_count % self.decode_interval == 0:
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            decoded_objects = pyzbar.decode(frame_rgb)

        aligned = False
        found = False

        for obj in decoded_objects:
            found = True
            try:
                qr_data = obj.data.decode("utf-8")
                self.last_qr_id = qr_data
            except Exception:
                continue

            cx = obj.rect.left + obj.rect.width / 2.0
            cy = obj.rect.top + obj.rect.height / 2.0
            ex = (cx - img_cx) / img_cx
            ey = (cy - img_cy) / img_cy

            if abs(ex) < self.eps_x and abs(ey) < self.eps_y:
                self.stable_count += 1
                laser_ready = abs(ex) < self.eps_x_laser
            else:
                self.stable_count = 0
                laser_ready = False

            aligned = self.stable_count >= self.stable_frames

            if laser_ready and qr_data != self.previous_qr_data:
                threading.Thread(target=self._fire_laser_worker, daemon=True).start()
                self.previous_qr_data = qr_data

            self.qr_id_pub.publish(String(data=qr_data))
            self.offset_pub.publish(Point(x=float(ex), y=float(ey), z=0.0))

            if self.enable_debug or self.should_show_window:
                cv2.circle(frame, (int(cx), int(cy)), 6, (0, 255, 0), -1)
                cv2.line(frame, (int(img_cx), int(img_cy)), (int(cx), int(cy)), (255, 0, 0), 2)
                cv2.putText(frame, f"ID:{qr_data[:10]}", (obj.rect.left, max(0, obj.rect.top - 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            break

        if not found:
            self.stable_count = 0

        self.aligned_pub.publish(Bool(data=bool(aligned)))

        if self.enable_debug:
            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
            except Exception:
                pass

        if self.should_show_window:
            try:
                cv2.imshow(self.get_name(), frame)
                cv2.waitKey(1)
            except Exception as e:
                self.get_logger().error(f"imshow failed: {e}")
                self.should_show_window = False

    def destroy_node(self):
        try:
            self.cap.release()
        except Exception:
            pass
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = QRVisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
