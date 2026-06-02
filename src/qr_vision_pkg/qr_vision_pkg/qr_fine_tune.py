#!/usr/bin/env python3
"""
二维码微调节点（独立、解耦）

职责：
- 订阅二维码识别节点发布的话题：
  - {input_prefix}/offset_norm (geometry_msgs/Point): x=ex(右正), y=ey(下正)，按半宽/半高归一化(−1~1)
  - {input_prefix}/aligned (std_msgs/Bool): 是否对准
- 将归一化偏移转换为“机身坐标系微调位移（厘米）”，发布给 Route 做仲裁：
  - {output_topic} (geometry_msgs/Point): x=body_dx_cm, y=body_dy_cm, z=body_dz_cm

轴映射（螃蟹式横走对准）：
- 相机/指示激光/雷达全部正前方安装，机头始终顶着板面。
- ex(横向像素偏移) → body_y(左右平移/蟹行)；
- ey(纵向)        → body_z(高度)；
- body_x(前后到板距离) 不由像素管，交给雷达 standoff 闭环保持，本节点恒输出 0。

像素 → 真实 cm 的尺度（standoff 动态增益）：
- 偏移按半宽归一化，真实偏移 = e × D × tan(FOV/2)，故增益 k = D(米) × tan(FOV/2) × 100。
- 订阅 standoff 距离话题动态算 k_body_y_cm / k_body_z_cm；
- standoff 无效/过期/未启用时，回退到静态默认增益 k_body_y_cm / k_body_z_cm。

说明：
- 本节点不发布速度，不参与 PID 计算，完全解耦。
"""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from std_msgs.msg import Bool, Float32


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

        # 参数：归一化偏移 -> 机身(cm) 静态默认增益（standoff 失效时的回退值）
        # 约定：decoder 的 ex(右为正)、ey(下为正)
        self.declare_parameter("k_body_y_cm", 20.0)  # ex -> body_y(cm) 横移/蟹行
        self.declare_parameter("k_body_z_cm", 20.0)  # ey -> body_z(cm) 高度
        self.declare_parameter("max_cm", 15.0)
        self.declare_parameter("invert_body_y", False)
        self.declare_parameter("invert_body_z", True)
        self.declare_parameter("publish_zero_when_aligned", True)

        # 参数：standoff 动态增益（像素->真实cm 随距离缩放）
        self.declare_parameter("use_standoff", True)
        self.declare_parameter("standoff_topic", "/standoff/distance")
        self.declare_parameter("standoff_valid_topic", "/standoff/valid")
        self.declare_parameter("hfov_deg", 60.0)            # 相机水平视场角，待标定
        self.declare_parameter("vfov_deg", 37.0)            # 相机垂直视场角，待标定
        self.declare_parameter("standoff_timeout_sec", 0.5)  # 距离超过该时长未更新视为过期
        self.declare_parameter("standoff_min_m", 0.10)      # 合理 standoff 下限（防异常值）
        self.declare_parameter("standoff_max_m", 5.0)       # 合理 standoff 上限

        self.input_prefix = str(self.get_parameter("input_prefix").value).rstrip("/")
        if not self.input_prefix.startswith("/"):
            self.input_prefix = "/" + self.input_prefix

        self.output_topic = str(self.get_parameter("output_topic").value)

        self.publish_hz = float(self.get_parameter("publish_hz").value)
        self.ema_alpha = float(self.get_parameter("ema_alpha").value)
        self.deadband_ex = float(self.get_parameter("deadband_ex").value)
        self.deadband_ey = float(self.get_parameter("deadband_ey").value)
        self.max_step_cm = float(self.get_parameter("max_step_cm").value)

        self.k_body_y_cm = float(self.get_parameter("k_body_y_cm").value)
        self.k_body_z_cm = float(self.get_parameter("k_body_z_cm").value)
        self.max_cm = float(self.get_parameter("max_cm").value)
        self.invert_body_y = bool(self.get_parameter("invert_body_y").value)
        self.invert_body_z = bool(self.get_parameter("invert_body_z").value)
        self.publish_zero_when_aligned = bool(self.get_parameter("publish_zero_when_aligned").value)

        self.use_standoff = bool(self.get_parameter("use_standoff").value)
        self.standoff_topic = str(self.get_parameter("standoff_topic").value)
        self.standoff_valid_topic = str(self.get_parameter("standoff_valid_topic").value)
        self.hfov_deg = float(self.get_parameter("hfov_deg").value)
        self.vfov_deg = float(self.get_parameter("vfov_deg").value)
        self.standoff_timeout_sec = float(self.get_parameter("standoff_timeout_sec").value)
        self.standoff_min_m = float(self.get_parameter("standoff_min_m").value)
        self.standoff_max_m = float(self.get_parameter("standoff_max_m").value)

        # 预算半视场角的正切，省得每帧重算
        self._tan_half_hfov = math.tan(math.radians(self.hfov_deg) / 2.0)
        self._tan_half_vfov = math.tan(math.radians(self.vfov_deg) / 2.0)

        self._last_offset_norm: Optional[Point] = None
        self._aligned: bool = False
        self._ex_f: float = 0.0
        self._ey_f: float = 0.0
        self._has_filt: bool = False
        self._out_dy: float = 0.0
        self._out_dz: float = 0.0

        # standoff 状态
        self._standoff_m: Optional[float] = None
        self._standoff_valid: bool = False
        self._standoff_stamp = self.get_clock().now()

        # 正确顺序：类型, 话题, 回调函数, 深度
        self.offset_sub = self.create_subscription(Point, f"{self.input_prefix}/offset_norm", self._on_offset, 10)
        self.aligned_sub = self.create_subscription(Bool, f"{self.input_prefix}/aligned", self._on_aligned, 10)

        if self.use_standoff:
            self.standoff_sub = self.create_subscription(Float32, self.standoff_topic, self._on_standoff, 10)
            self.standoff_valid_sub = self.create_subscription(
                Bool, self.standoff_valid_topic, self._on_standoff_valid, 10)

        self.pub = self.create_publisher(Point, self.output_topic, 10)

        # 低频定时发布（避免频繁更新导致下游目标点跳变）
        hz = max(1.0, self.publish_hz)
        self.timer = self.create_timer(1.0 / hz, self._publish)

        self.get_logger().info(
            f"QRFineTuneNode started: subscribe({self.input_prefix}/offset_norm,{self.input_prefix}/aligned) "
            f"-> publish({self.output_topic}); standoff={'on' if self.use_standoff else 'off'} "
            f"(topic={self.standoff_topic}, hfov={self.hfov_deg}°, vfov={self.vfov_deg}°)"
        )

    def _on_offset(self, msg: Point) -> None:
        self._last_offset_norm = msg

    def _on_aligned(self, msg: Bool) -> None:
        self._aligned = bool(msg.data)

    def _on_standoff(self, msg: Float32) -> None:
        self._standoff_m = float(msg.data)
        self._standoff_stamp = self.get_clock().now()

    def _on_standoff_valid(self, msg: Bool) -> None:
        self._standoff_valid = bool(msg.data)

    def _effective_gains(self) -> tuple[float, float]:
        """返回当前生效的 (k_body_y_cm, k_body_z_cm)。
        standoff 有效且新鲜且在合理范围内 → 用 D 动态算；否则回退静态默认。"""
        if not self.use_standoff or not self._standoff_valid or self._standoff_m is None:
            return self.k_body_y_cm, self.k_body_z_cm

        # 新鲜度检查
        age = (self.get_clock().now() - self._standoff_stamp).nanoseconds * 1e-9
        if age > self.standoff_timeout_sec:
            return self.k_body_y_cm, self.k_body_z_cm

        d = self._standoff_m
        if d < self.standoff_min_m or d > self.standoff_max_m:
            return self.k_body_y_cm, self.k_body_z_cm

        # k = D(米) × tan(FOV/2) × 100  (偏移按半宽/半高归一化)
        k_y = d * self._tan_half_hfov * 100.0
        k_z = d * self._tan_half_vfov * 100.0
        return k_y, k_z

    def _publish(self) -> None:
        out = Point()  # x=body_dx 恒为 0（前后交给 standoff），y=body_dy，z=body_dz

        if self.publish_zero_when_aligned and self._aligned:
            # 对准时输出 0，并复位滤波与输出状态，避免下一次进入时带上历史积分
            self._has_filt = False
            self._out_dy = 0.0
            self._out_dz = 0.0
            self.pub.publish(out)
            return

        if self._last_offset_norm is None:
            # 未收到 offset 时输出 0，保持系统可预测
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

        # 生效增益（standoff 动态 or 静态回退）
        k_y, k_z = self._effective_gains()

        desired_dy = self._ex_f * k_y   # 横向像素 -> 机体 y（蟹行）
        desired_dz = self._ey_f * k_z   # 纵向像素 -> 机体 z（高度）

        if self.invert_body_y:
            desired_dy = -desired_dy
        if self.invert_body_z:
            desired_dz = -desired_dz

        desired_dy = _clamp(desired_dy, -self.max_cm, self.max_cm)
        desired_dz = _clamp(desired_dz, -self.max_cm, self.max_cm)

        # “积分式”平滑：输出每次只允许变化 max_step_cm（防止目标跳变）
        step = max(0.0, self.max_step_cm)
        self._out_dy += _clamp(desired_dy - self._out_dy, -step, step)
        self._out_dz += _clamp(desired_dz - self._out_dz, -step, step)

        # 极小值抑制
        if math.fabs(self._out_dy) < 0.1:
            self._out_dy = 0.0
        if math.fabs(self._out_dz) < 0.1:
            self._out_dz = 0.0

        out.x = 0.0
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
