from __future__ import annotations

import math
from typing import Optional

import cv2
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray

from visual_pkg.detector_core import (
    DetectorConfig,
    RobustRatioFilter,
    StableRectangleTracker,
    build_black_mask,
    detect_circle_in_square,
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

        cam_idx = int(self.get_parameter("camera_index").value)
        width = int(self.get_parameter("width").value)
        height = int(self.get_parameter("height").value)
        cam_fps = int(self.get_parameter("camera_fps").value)
        process_fps = float(self.get_parameter("process_fps").value)
        self._show = bool(self.get_parameter("show_display").value)

        self._config = DetectorConfig()
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

        fine = Int32MultiArray()
        fine.data = [0, 0]
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
            circle_current = detect_circle_in_square(binary_fused, state.polygon, self._config)

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
            if self._circle_misses > self._config.circle_hold_frames:
                self._circle_last = None
                self._ratio_filter.reset()

        circle_for_pub = circle_current if circle_current is not None else self._circle_last
        circle_hold = circle_current is None and self._circle_last is not None

        now = self.get_clock().now().to_msg()
        frame_h, frame_w = frame.shape[:2]

        pt = PointStamped()
        pt.header.stamp = now
        pt.header.frame_id = "camera"
        ratio_msg = Float32()

        fine_x_px = 0
        fine_y_px = 0
        if circle_for_pub is not None:
            cx, cy = circle_for_pub["center"]
            dx = float(cx) - frame_w / 2.0
            dy = float(cy) - frame_h / 2.0
            pt.point.x = dx
            pt.point.y = dy
            pt.point.z = 0.0
            fine_x_px = int(round(dx))
            fine_y_px = int(round(dy))

            ratio_val = circle_for_pub.get("ratio_smooth", circle_for_pub.get("ratio", math.nan))
            ratio_msg.data = float(ratio_val)
        else:
            pt.point.x = math.nan
            pt.point.y = math.nan
            pt.point.z = 0.0
            ratio_msg.data = math.nan

        self._pub_center.publish(pt)
        self._pub_ratio.publish(ratio_msg)

        fine = Int32MultiArray()
        fine.data = [fine_x_px, fine_y_px]
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
