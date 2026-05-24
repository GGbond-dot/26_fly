from __future__ import annotations

import math
from typing import Optional

import cv2
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import String

from visual_pkg.detector_core import (
    DetectorConfig,
    RobustRatioFilter,
    StableRectangleTracker,
    build_black_mask,
    detect_circle_in_square,
    detect_landing_box,
    detect_rectangle_with_fallback,
    draw_overlay,
    fuse_binary_masks,
    preprocess,
)


class VisualNode(Node):
    def __init__(self) -> None:
        super().__init__("visual_node")

        self.declare_parameter("camera_index", 0)
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("camera_fps", 30)
        self.declare_parameter("process_fps", 15.0)
        self.declare_parameter("show_display", False)
        self.declare_parameter("apriltag_code", -1)
        # center_source: "circle" (默认，兼容 demo2) | "square" (demo3 抓取对准用方框几何中心)
        self.declare_parameter("center_source", "circle")
        # 方框“中心框优先”指数：越大越偏向画面正中的框，压掉边角的 A/B 起停区方框。0=关闭(纯面积)。
        self.declare_parameter("rect_center_bias", 2.0)
        # 降落专用检测参数（vision_mode="land" 时生效，和柱子检测解耦；见 detector_core.detect_landing_box）。
        self.declare_parameter("landing_min_area", 600.0)
        self.declare_parameter("landing_center_bias", 2.0)
        self.declare_parameter("landing_oversize_frac", 0.72)
        self.declare_parameter("landing_roi_margin_frac", 0.12)
        self.declare_parameter("landing_min_black_frac", 0.015)

        cam_idx = int(self.get_parameter("camera_index").value)
        width = int(self.get_parameter("width").value)
        height = int(self.get_parameter("height").value)
        cam_fps = int(self.get_parameter("camera_fps").value)
        process_fps = float(self.get_parameter("process_fps").value)
        self._show = bool(self.get_parameter("show_display").value)
        src = str(self.get_parameter("center_source").value).strip().lower()
        self._center_source = "square" if src == "square" else "circle"

        self._config = DetectorConfig()
        self._config.rect_center_bias = float(self.get_parameter("rect_center_bias").value)
        self._config.landing_min_area = float(self.get_parameter("landing_min_area").value)
        self._config.landing_center_bias = float(self.get_parameter("landing_center_bias").value)
        self._config.landing_oversize_frac = float(self.get_parameter("landing_oversize_frac").value)
        self._config.landing_roi_margin_frac = float(self.get_parameter("landing_roi_margin_frac").value)
        self._config.landing_min_black_frac = float(self.get_parameter("landing_min_black_frac").value)
        # 视觉任务模式：mission 进降落阶段会发 /vision_mode="land"，切到降落专用检测分支
        # （只找 B 框中心、关圆检测），其余时间 "pillar"（柱子抓取/放置那套，行为不变）。
        self._mode = "pillar"
        self._tracker = StableRectangleTracker(
            pred_decay=self._config.track_pred_decay,
            pred_max_step_ratio=self._config.track_pred_max_step_ratio,
            gate_base=self._config.track_gate_base,
            gate_miss_gain=self._config.track_gate_miss_gain,
            gate_speed_gain=self._config.track_gate_speed_gain,
            gate_speed_norm=self._config.track_gate_speed_norm,
            size_ratio_min=self._config.track_size_ratio_min,
            size_ratio_max=self._config.track_size_ratio_max,
            size_ratio_min_miss=self._config.track_size_ratio_min_miss,
            size_ratio_max_miss=self._config.track_size_ratio_max_miss,
            velocity_blend_old=self._config.track_velocity_blend_old,
            switch_required_hits=self._config.track_switch_required_hits,
            pending_iou_min=self._config.track_pending_iou_min,
        )
        self._ratio_filter = RobustRatioFilter(
            alpha=self._config.circle_ratio_ema_alpha,
            outlier_percent=self._config.ratio_outlier_percent,
            window_size=self._config.ratio_window_size,
            confidence_min=self._config.ratio_confidence_min,
        )
        self._circle_last: Optional[dict] = None
        self._circle_misses = 0

        self._cap: Optional[cv2.VideoCapture] = None
        self._camera_ok = self._open_camera(cam_idx, width, height, cam_fps)
        if not self._camera_ok:
            self.get_logger().error(
                f"Failed to open camera index {cam_idx} (/dev/video{cam_idx}). "
                "Publishing NaN for visual outputs until camera becomes available."
            )

        self._pub_center = self.create_publisher(PointStamped, "/circle_center", 10)
        self._pub_ratio = self.create_publisher(Float32, "/circle_area_ratio", 10)
        self._pub_fine_data = self.create_publisher(Int32MultiArray, "/fine_data", 10)
        self._pub_apriltag_code = self.create_publisher(Int32, "/apriltag_code", 10)

        # /vision_mode 由 mission 锁存发布（transient_local），晚订阅也能拿到最后值。
        mode_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._sub_mode = self.create_subscription(
            String, "/vision_mode", self._on_vision_mode, mode_qos
        )

        timer_period = 1.0 / max(1.0, process_fps)
        self._timer = self.create_timer(timer_period, self._timer_callback)

        self.get_logger().info(
            f"VisualNode started: camera={cam_idx} ({width}x{height}), "
            f"process_fps={process_fps:.1f}, show_display={self._show}"
        )

    def _open_camera(self, index: int, width: int, height: int, fps: int) -> bool:
        cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap = cv2.VideoCapture(index)
        if not cap.isOpened():
            return False
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, fps)
        self._cap = cap
        return True

    def _on_vision_mode(self, msg: String) -> None:
        mode = (msg.data or "").strip().lower()
        mode = "land" if mode == "land" else "pillar"
        if mode != self._mode:
            self.get_logger().info(f"vision_mode: {self._mode} -> {mode}")
            self._mode = mode

    def _publish_apriltag_code(self) -> None:
        msg = Int32()
        msg.data = int(self.get_parameter("apriltag_code").value)
        self._pub_apriltag_code.publish(msg)

    def _publish_nan(self) -> None:
        now = self.get_clock().now().to_msg()

        pt = PointStamped()
        pt.header.stamp = now
        pt.header.frame_id = "camera"
        pt.point.x = math.nan
        pt.point.y = math.nan
        pt.point.z = 0.0
        self._pub_center.publish(pt)

        ratio = Float32()
        ratio.data = math.nan
        self._pub_ratio.publish(ratio)

        # /fine_data 第三位=valid(0=丢框/无目标)：x,y 给 0 让 PID 保持不动，
        # mission 降落阶段靠 valid 区分“丢框”和“真对准”，不再把 [0,0] 当成已对准。
        fine = Int32MultiArray()
        fine.data = [0, 0, 0]
        self._pub_fine_data.publish(fine)

        self._publish_apriltag_code()

    def _timer_callback(self) -> None:
        if not self._camera_ok or self._cap is None:
            cam_idx = int(self.get_parameter("camera_index").value)
            width = int(self.get_parameter("width").value)
            height = int(self.get_parameter("height").value)
            fps = int(self.get_parameter("camera_fps").value)
            self._camera_ok = self._open_camera(cam_idx, width, height, fps)
            if self._camera_ok:
                self.get_logger().info("Camera re-opened successfully.")
            self._publish_nan()
            return

        ok, frame = self._cap.read()
        if not ok:
            self.get_logger().warn("cap.read() failed; will attempt camera re-open next tick.")
            self._camera_ok = False
            self._publish_nan()
            return

        # 降落模式：单独一条链路（只找 B 框中心、关圆检测），不走柱子那套检测/跟踪。
        if self._mode == "land":
            self._process_landing(frame)
            return

        _, binary_primary = preprocess(frame, self._config)
        binary_black = build_black_mask(frame, self._config)
        binary_fused = fuse_binary_masks(binary_primary, binary_black, self._config)

        detection = detect_rectangle_with_fallback(binary_fused, binary_primary, self._config)
        state, confirmed = self._tracker.update(detection)

        circle_current: Optional[dict] = None
        allow_hold = (
            state.misses <= self._config.circle_detect_miss_tolerance
            and state.stable_hits >= max(2, self._tracker.required_hits)
        )
        if state.polygon is not None and (confirmed or allow_hold):
            circle_current = detect_circle_in_square(binary_black, state.polygon, self._config)

        if circle_current is not None:
            smooth_ratio = self._ratio_filter.update(
                circle_current["ratio"], circle_current["confidence"]
            )
            if smooth_ratio is not None:
                circle_current["ratio_smooth"] = smooth_ratio
                self._circle_last = circle_current
                self._circle_misses = 0
        else:
            self._circle_misses += 1
            self._circle_last = None
            self._ratio_filter.reset()

        circle_for_pub = circle_current
        circle_hold = False

        now = self.get_clock().now().to_msg()
        frame_h, frame_w = frame.shape[:2]

        pt = PointStamped()
        pt.header.stamp = now
        pt.header.frame_id = "camera"
        ratio_msg = Float32()

        fine_x_px = 0
        fine_y_px = 0

        # 发布中心选择：square=方框几何中心(抓取/放置对准)，circle=圆心(默认兼容 demo2)
        center_xy: Optional[tuple] = None
        if self._center_source == "square":
            if state.polygon is not None:
                poly = state.polygon.reshape(-1, 2)
                center_xy = (float(poly[:, 0].mean()), float(poly[:, 1].mean()))
        else:
            if circle_for_pub is not None:
                center_xy = (float(circle_for_pub["center"][0]),
                             float(circle_for_pub["center"][1]))

        if center_xy is not None:
            cx, cy = center_xy
            dx = cx - frame_w / 2.0
            dy = cy - frame_h / 2.0
            pt.point.x = dx
            pt.point.y = dy
            pt.point.z = 0.0
            fine_x_px = int(round(dx))
            fine_y_px = int(round(dy))
        else:
            pt.point.x = math.nan
            pt.point.y = math.nan
            pt.point.z = 0.0

        if circle_for_pub is not None:
            ratio_val = circle_for_pub.get("ratio_smooth", circle_for_pub.get("ratio", math.nan))
            ratio_msg.data = float(ratio_val)
        else:
            ratio_msg.data = math.nan

        self._pub_center.publish(pt)
        self._pub_ratio.publish(ratio_msg)

        # 第三位=valid：方框中心有效=1，丢框=0（下游降落逻辑用；柱子/放置逻辑只读前两位，行为不变）。
        fine = Int32MultiArray()
        fine.data = [fine_x_px, fine_y_px, 1 if center_xy is not None else 0]
        self._pub_fine_data.publish(fine)

        self._publish_apriltag_code()

        if self._show:
            vis = frame.copy()
            draw_overlay(
                vis,
                state,
                confirmed,
                self._config.threshold_method,
                0.0,
                0.0,
                circle_for_pub,
                circle_hold,
            )
            cv2.imshow("visual_pkg", vis)
            cv2.waitKey(1)

    def _process_landing(self, frame) -> None:
        """降落专用处理：只在黑掩码里找 B 框中心，发 /fine_data(带 valid)；圆占比发 NaN。"""
        now = self.get_clock().now().to_msg()
        frame_h, frame_w = frame.shape[:2]

        binary_black = build_black_mask(frame, self._config)
        det = detect_landing_box(binary_black, self._config)

        pt = PointStamped()
        pt.header.stamp = now
        pt.header.frame_id = "camera"
        fine = Int32MultiArray()
        if det is not None:
            cx, cy = det["center"]
            dx = cx - frame_w / 2.0
            dy = cy - frame_h / 2.0
            pt.point.x = dx
            pt.point.y = dy
            pt.point.z = 0.0
            fine.data = [int(round(dx)), int(round(dy)), 1]
        else:
            pt.point.x = math.nan
            pt.point.y = math.nan
            pt.point.z = 0.0
            fine.data = [0, 0, 0]   # 丢框：x,y 给 0 让 PID 保持，valid=0 让 mission 不当对准
        self._pub_center.publish(pt)

        ratio = Float32()
        ratio.data = math.nan       # 降落不需要圆占比
        self._pub_ratio.publish(ratio)

        self._pub_fine_data.publish(fine)
        self._publish_apriltag_code()

        if self._show:
            vis = frame.copy()
            if det is not None:
                c = (int(round(det["center"][0])), int(round(det["center"][1])))
                cv2.circle(vis, c, 6, (0, 255, 0), 2)
                if det.get("polygon") is not None:
                    cv2.polylines(
                        vis, [det["polygon"].reshape(-1, 1, 2).astype(int)],
                        True, (0, 255, 0), 2,
                    )
            tag = det["method"] if det is not None else "none"
            cv2.putText(vis, f"LAND {tag}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imshow("visual_pkg", vis)
            cv2.waitKey(1)

    def destroy_node(self) -> None:
        if self._cap is not None:
            self._cap.release()
        if self._show:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisualNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
