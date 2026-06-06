from __future__ import annotations

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, String

if __package__ in (None, ""):
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from ground_station.models import SLOTS
else:
    from .models import SLOTS


class FakeUavNode(Node):
    """模拟 inventory_mission_node 的对外话题，用于无飞机时联调地面站。

    发布的话题/格式与真实无人机一致：
      /inventory_result  std_msgs/String  "编号=N,货位=XY"
      /inventory_led     std_msgs/Empty
      /inventory_target  std_msgs/String  先 "N"，收到货位后回 "目标编号=N,货位=XY"
      /inventory_status  std_msgs/String  状态/心跳（~2Hz），驱动地面站「飞机在线 + 阶段」
      /mission_complete  std_msgs/Empty
    订阅：
      /inventory_target_slot  std_msgs/String  地面站下发的货位，如 "C5"
    """

    def __init__(self, delay_s: float, target_id: int | None) -> None:
        super().__init__("fake_inventory_uav")
        self.delay_s = delay_s
        self.target_id = target_id
        self.result_pub = self.create_publisher(String, "/inventory_result", 10)
        self.led_pub = self.create_publisher(Empty, "/inventory_led", 10)
        self.target_pub = self.create_publisher(String, "/inventory_target", 10)
        self.complete_pub = self.create_publisher(Empty, "/mission_complete", 10)
        # 状态/心跳：与真机一致发 /inventory_status，让地面站顶栏「飞机在线 + 阶段」联调可见。
        self.status_pub = self.create_publisher(String, "/inventory_status", 10)
        self._status_text = "待命中：等地面站下发任务模式（普通/进阶）"
        self.create_timer(0.5, self._publish_status)
        self.create_subscription(
            String,
            "/inventory_target_slot",
            self._handle_target_slot,
            10,
        )
        # 模拟飞机 WAIT_MODE：上电后等地面站下发 /inventory_mode 才动作。
        self.mode: str | None = None
        self.create_subscription(
            String,
            "/inventory_mode",
            self._handle_mode,
            10,
        )

    def _publish_status(self) -> None:
        msg = String()
        msg.data = self._status_text
        self.status_pub.publish(msg)

    def _handle_mode(self, msg: String) -> None:
        mode = msg.data.strip().lower()
        if mode in ("traverse", "directed"):
            self.mode = mode
            self.get_logger().info(f"received mission mode: {mode}")

    def run(self) -> None:
        # 先等地面站下发任务模式（模拟飞机 WAIT_MODE 待命）。
        self.get_logger().info("WAIT_MODE: 等待地面站 /inventory_mode（traverse/directed）…")
        while rclpy.ok() and self.mode is None:
            rclpy.spin_once(self, timeout_sec=0.2)
        if self.mode == "directed":
            # 进阶：不跑全遍历，直接报抽取码编号 → 等地面站下发货位 → 回确认。
            self._run_directed()
            return

        # 普通（遍历盘点）：逐货位发布 "编号=N,货位=XY" + 一次 LED 脉冲。
        for index, slot in enumerate(SLOTS, start=1):
            time.sleep(self.delay_s)
            self._status_text = f"[遍历] 盘点中 货位{slot}"
            result = String()
            result.data = f"编号={index},货位={slot}"
            self.result_pub.publish(result)
            self.led_pub.publish(Empty())
            self.get_logger().info(f"inventory_result: {result.data}")
            rclpy.spin_once(self, timeout_sec=0.01)

        self.complete_pub.publish(Empty())
        self._status_text = "[遍历] 任务完成"
        self.get_logger().info("mission_complete")
        # 遍历盘点结束即收工（要求1）。要求2 是另一次独立飞行/模式，见 _run_directed。
        self.get_logger().info("traverse 完成，spin 保活。")
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)

    def _run_directed(self) -> None:
        # 定向盘点（要求2）：报送抽取码编号，等地面站查表下发货位，收到后回确认（_handle_target_slot）。
        self._status_text = "[定向] 识别抽取码中…（请把抽取码举到机头相机前）"
        if self.target_id is not None:
            time.sleep(0.5)
            target_msg = String()
            target_msg.data = str(self.target_id)
            self.target_pub.publish(target_msg)
            self._status_text = f"[定向] 已识别抽取码 编号{self.target_id}，等地面站下发货位"
            self.get_logger().info(f"inventory_target: {self.target_id}")

        self.get_logger().info("spinning for /inventory_target_slot messages")
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)

    def _handle_target_slot(self, msg: String) -> None:
        slot = msg.data.strip()
        self.get_logger().info(f"received target slot: {slot}")
        if self.target_id is not None and slot:
            # 模拟无人机收到货位后的确认回报。
            ack = String()
            ack.data = f"目标编号={self.target_id},货位={slot}"
            self.target_pub.publish(ack)
            self._status_text = f"[定向] 已收到货位 {slot}，准备起飞"
            self.get_logger().info(f"inventory_target(ack): {ack.data}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Fake UAV publisher for ground station tests")
    parser.add_argument("--delay", type=float, default=0.15, help="Delay between inventory messages")
    parser.add_argument("--target-id", type=int, default=7, help="Target id to publish after traversal")
    parser.add_argument("--no-target", action="store_true", help="Do not publish /inventory_target")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = FakeUavNode(args.delay, None if args.no_target else args.target_id)
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
