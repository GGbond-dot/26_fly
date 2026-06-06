from __future__ import annotations

import threading

from PyQt5.QtCore import QObject, pyqtSignal

import rclpy
from std_msgs.msg import Empty, String

from .models import parse_inventory_payload, parse_target_payload


class RosBridge(QObject):
    inventory_result = pyqtSignal(dict)
    target_id = pyqtSignal(int)
    target_confirmed = pyqtSignal(dict)
    led_blink = pyqtSignal()
    mission_complete = pyqtSignal()
    connection_state = pyqtSignal(str)
    uav_status = pyqtSignal(str)        # 飞机状态/心跳文本（/inventory_status）
    error = pyqtSignal(str)

    def __init__(self) -> None:
        super().__init__()
        self._node = None
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._lock = threading.Lock()
        self._target_slot_pub = None
        self._mode_pub = None

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, name="ros-bridge", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        with self._lock:
            node = self._node
        if node is not None:
            try:
                node.executor.wake() if node.executor else None
            except Exception:
                pass
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)

    def is_ready(self) -> bool:
        with self._lock:
            return self._node is not None

    def publish_target_slot(self, slot: str) -> bool:
        with self._lock:
            target_slot_pub = self._target_slot_pub
        if target_slot_pub is None:
            self.error.emit("ROS 尚未连接，无法发布目标库位")
            return False

        info = String()
        info.data = slot

        try:
            target_slot_pub.publish(info)
        except Exception as exc:
            self.error.emit(f"发布目标库位失败: {exc}")
            return False
        return True

    def publish_mode(self, mode: str) -> bool:
        # 告诉重启后待命的飞机本轮任务：traverse(普通) / directed(进阶)。
        with self._lock:
            mode_pub = self._mode_pub
        if mode_pub is None:
            self.error.emit("ROS 尚未连接，无法下发任务模式")
            return False

        info = String()
        info.data = mode
        try:
            mode_pub.publish(info)
        except Exception as exc:
            self.error.emit(f"下发任务模式失败: {exc}")
            return False
        return True

    def _run(self) -> None:
        initialized_here = False
        try:
            if not rclpy.ok():
                rclpy.init(args=None)
                initialized_here = True

            node = rclpy.create_node("rack_inventory_ground_station")
            target_slot_pub = node.create_publisher(
                String,
                "/inventory_target_slot",
                10,
            )
            mode_pub = node.create_publisher(
                String,
                "/inventory_mode",
                10,
            )
            node.create_subscription(
                String,
                "/inventory_result",
                self._handle_inventory_result,
                10,
            )
            node.create_subscription(
                String,
                "/inventory_target",
                self._handle_target,
                10,
            )
            node.create_subscription(
                Empty,
                "/inventory_led",
                self._handle_led,
                10,
            )
            node.create_subscription(
                Empty,
                "/mission_complete",
                self._handle_mission_complete,
                10,
            )
            # 飞机状态/心跳：飞机起好就持续发（含待命态），地面站据此判在线 + 显示当前阶段。
            node.create_subscription(
                String,
                "/inventory_status",
                self._handle_uav_status,
                10,
            )
            with self._lock:
                self._node = node
                self._target_slot_pub = target_slot_pub
                self._mode_pub = mode_pub
            self.connection_state.emit("ROS 已连接")

            while not self._stop_event.is_set() and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.1)
        except Exception as exc:
            self.connection_state.emit("ROS 未连接")
            self.error.emit(f"ROS bridge 错误: {exc}")
        finally:
            with self._lock:
                node = self._node
                self._node = None
                self._target_slot_pub = None
                self._mode_pub = None
            if node is not None:
                try:
                    node.destroy_node()
                except Exception:
                    pass
            if initialized_here and rclpy.ok():
                try:
                    rclpy.shutdown()
                except Exception:
                    pass
            self.connection_state.emit("ROS 已断开")

    def _handle_inventory_result(self, msg: String) -> None:
        try:
            self.inventory_result.emit(parse_inventory_payload(msg.data))
        except Exception as exc:
            self.error.emit(f"盘点结果解析失败: {exc}")

    def _handle_target(self, msg: String) -> None:
        try:
            data = parse_target_payload(msg.data)
        except Exception as exc:
            self.error.emit(f"目标编号解析失败: {exc}")
            return
        # 含货位 = 无人机对地面站下发货位的确认回报；仅编号 = 请求地面站查表下发。
        if "slot" in data:
            self.target_confirmed.emit(data)
        else:
            self.target_id.emit(data["item_id"])

    def _handle_led(self, _msg: Empty) -> None:
        self.led_blink.emit()

    def _handle_mission_complete(self, _msg: Empty) -> None:
        self.mission_complete.emit()

    def _handle_uav_status(self, msg: String) -> None:
        self.uav_status.emit(msg.data)
