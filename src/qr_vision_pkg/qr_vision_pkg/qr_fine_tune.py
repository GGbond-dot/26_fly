#!/usr/bin/env python3
"""
二维码微调节点（独立、解耦）

职责：
- 订阅二维码识别节点发布的话题（左右相机用不同前缀隔离）：
  - {input_prefix}/offset_norm (geometry_msgs/Point): x=ex, y=ey，归一化像素偏移
  - {input_prefix}/aligned (std_msgs/Bool): 是否对准
- 将归一化偏移转换为“机身坐标系微调位移（厘米）”，发布给 Route 做仲裁：
  - {output_topic} (geometry_msgs/Point): x=body_dx_cm, y=body_dy_cm, z=body_dz_cm

说明：
- 本节点不发布速度，不参与 PID 计算，完全解耦。
- 映射规则通过参数控制（支持右/左相机不同符号约定）。
"""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from std_msgs.msg import Bool


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class QRFineTuneNode(Node):
    def __init__(self) -> None:
        super().__init__("qr_fine_tune_node")

        # 参数：输入/输出话题
        self.declare_parameter("input_prefix", "/qr_vision")
        self.declare_parameter("output_topic", "/qr_vision/fine_offset_body_cm")

        # 参数：发布频率与滤波/限幅（用于防抖：避免目标频繁跳动）
        self.declare_parameter("publish_hz", 10.0)          # 建议 5~10Hz
        self.declare_parameter("ema_alpha", 0.8)            # 0~1，越大越平滑
        self.declare_parameter("deadband_ex", 0.02)         # |ex| 小于该值视为 0
        self.declare_parameter("deadband_ey", 0.02)         # |ey| 小于该值视为 0
        self.declare_parameter("max_step_cm", 2.0)          # 每次发布最大变化（cm），用于“积分式”平滑逼近

        # 参数：归一化偏移 -> 机身(cm) 映射与限幅
        # 约定：decoder 的 ex(右为正)、ey(下为正)
        # 你可以通过 invert_* 来适配左右相机不同安装方向。
        self.declare_parameter("k_body_x_cm", 20.0)  # ex -> body_x(cm)
        self.declare_parameter("k_body_y_cm", 0.0)   # 默认不微调 body_y
        self.declare_parameter("k_body_z_cm", 20.0)  # ey -> body_z(cm)
        self.declare_parameter("max_cm", 15.0)
        self.declare_parameter("invert_body_x", True)
        self.declare_parameter("invert_body_y", False)
        self.declare_parameter("invert_body_z", True)
        self.declare_parameter("publish_zero_when_aligned", True)

        self.input_prefix = str(self.get_parameter("input_prefix").value).rstrip("/")
        if not self.input_prefix.startswith("/"):
            self.input_prefix = "/" + self.input_prefix

        self.output_topic = str(self.get_parameter("output_topic").value)

        self.publish_hz = float(self.get_parameter("publish_hz").value)
        self.ema_alpha = float(self.get_parameter("ema_alpha").value)
        self.deadband_ex = float(self.get_parameter("deadband_ex").value)
        self.deadband_ey = float(self.get_parameter("deadband_ey").value)
        self.max_step_cm = float(self.get_parameter("max_step_cm").value)

        self.k_body_x_cm = float(self.get_parameter("k_body_x_cm").value)
        self.k_body_y_cm = float(self.get_parameter("k_body_y_cm").value)
        self.k_body_z_cm = float(self.get_parameter("k_body_z_cm").value)
        self.max_cm = float(self.get_parameter("max_cm").value)
        self.invert_body_x = bool(self.get_parameter("invert_body_x").value)
        self.invert_body_y = bool(self.get_parameter("invert_body_y").value)
        self.invert_body_z = bool(self.get_parameter("invert_body_z").value)
        self.publish_zero_when_aligned = bool(self.get_parameter("publish_zero_when_aligned").value)

        self._last_offset_norm: Optional[Point] = None
        self._aligned: bool = False
        self._ex_f: float = 0.0
        self._ey_f: float = 0.0
        self._has_filt: bool = False
        self._out_dx: float = 0.0
        self._out_dy: float = 0.0
        self._out_dz: float = 0.0

        # 正确顺序：类型, 话题, 回调函数, 深度
        self.offset_sub = self.create_subscription(Point, f"{self.input_prefix}/offset_norm", self._on_offset, 10)
        self.aligned_sub = self.create_subscription(Bool, f"{self.input_prefix}/aligned", self._on_aligned, 10)

        self.pub = self.create_publisher(Point, self.output_topic, 10)

        # 低频定时发布（避免频繁更新导致下游目标点跳变）
        hz = max(1.0, self.publish_hz)
        self.timer = self.create_timer(1.0 / hz, self._publish)

        self.get_logger().info(
            f"QRFineTuneNode started: subscribe({self.input_prefix}/offset_norm,{self.input_prefix}/aligned) -> publish({self.output_topic})"
        )

    def _on_offset(self, msg: Point) -> None:
        self._last_offset_norm = msg

    def _on_aligned(self, msg: Bool) -> None:
        self._aligned = bool(msg.data)

    def _publish(self) -> None:
        out = Point()

        if self.publish_zero_when_aligned and self._aligned:
            # 对准时输出 0，并复位滤波与输出状态，避免下一次进入时带上历史积分
            self._has_filt = False
            self._out_dx = 0.0
            self._out_dy = 0.0
            self._out_dz = 0.0
            out.x = 0.0
            out.y = 0.0
            out.z = 0.0
            self.pub.publish(out)
            return

        if self._last_offset_norm is None:
            # 未收到 offset 时输出 0，保持系统可预测
            out.x = 0.0
            out.y = 0.0
            out.z = 0.0
            self.pub.publish(out)
            return

        ex = float(self._last_offset_norm.x)
        ey = float(self._last_offset_norm.y)

        # 死区：小误差直接当 0，避免抖动导致频繁微调
        if math.fabs(ex) < self.deadband_ex:
            ex = 0.0
        if math.fabs(ey) < self.deadband_ey:
            ey = 0.0

        # EMA 滤波（降低噪声）
        a = _clamp(self.ema_alpha, 0.0, 0.99)
        if not self._has_filt:
            self._ex_f = ex
            self._ey_f = ey
            self._has_filt = True
        else:
            self._ex_f = a * self._ex_f + (1.0 - a) * ex
            self._ey_f = a * self._ey_f + (1.0 - a) * ey

        desired_dx = self._ex_f * self.k_body_x_cm
        desired_dy = self._ex_f * self.k_body_y_cm  # 默认 0
        desired_dz = self._ey_f * self.k_body_z_cm

        if self.invert_body_x:
            desired_dx = -desired_dx
        if self.invert_body_y:
            desired_dy = -desired_dy
        if self.invert_body_z:
            desired_dz = -desired_dz

        desired_dx = _clamp(desired_dx, -self.max_cm, self.max_cm)
        desired_dy = _clamp(desired_dy, -self.max_cm, self.max_cm)
        desired_dz = _clamp(desired_dz, -self.max_cm, self.max_cm)

        # “积分式”平滑：输出每次只允许变化 max_step_cm（防止目标跳变）
        step = max(0.0, self.max_step_cm)
        self._out_dx += _clamp(desired_dx - self._out_dx, -step, step)
        self._out_dy += _clamp(desired_dy - self._out_dy, -step, step)
        self._out_dz += _clamp(desired_dz - self._out_dz, -step, step)

        # 极小值抑制
        if math.fabs(self._out_dx) < 0.1:
            self._out_dx = 0.0
        if math.fabs(self._out_dy) < 0.1:
            self._out_dy = 0.0
        if math.fabs(self._out_dz) < 0.1:
            self._out_dz = 0.0

        out.x = self._out_dx
        out.y = self._out_dy
        out.z = self._out_dz
        self.pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = QRFineTuneNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()



