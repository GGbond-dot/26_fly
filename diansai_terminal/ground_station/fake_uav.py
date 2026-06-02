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
        self.create_subscription(
            String,
            "/inventory_target_slot",
            self._handle_target_slot,
            10,
        )

    def run(self) -> None:
        # 遍历盘点：逐货位发布 "编号=N,货位=XY" + 一次 LED 脉冲。
        for index, slot in enumerate(SLOTS, start=1):
            time.sleep(self.delay_s)
            result = String()
            result.data = f"编号={index},货位={slot}"
            self.result_pub.publish(result)
            self.led_pub.publish(Empty())
            self.get_logger().info(f"inventory_result: {result.data}")
            rclpy.spin_once(self, timeout_sec=0.01)

        self.complete_pub.publish(Empty())
        self.get_logger().info("mission_complete")

        # 定向盘点：报送抽取码编号，等地面站查表下发货位。
        if self.target_id is not None:
            time.sleep(0.5)
            target_msg = String()
            target_msg.data = str(self.target_id)
            self.target_pub.publish(target_msg)
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
