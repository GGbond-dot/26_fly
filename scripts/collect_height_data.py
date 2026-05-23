#!/usr/bin/env python3
"""Collect the two height signals used by the pickup mission.

Records only:
  - /laser_array/ground_height: processed array-laser ground height, Int16 cm
  - /height: point/flight-controller height from uart_to_stm32, Int16 cm

Each received ROS message is written as one CSV row with a receive timestamp.
The messages do not carry headers, so the timestamp is the local ROS node time
at callback entry.
"""

import argparse
import csv
import os
import signal
from datetime import datetime
from pathlib import Path
from typing import Dict, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Int16


DEFAULT_AREA_TOPIC = "/laser_array/ground_height"
DEFAULT_POINT_TOPIC = "/height"


class HeightCollector(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("height_data_collector")

        self.area_topic = args.area_topic
        self.point_topic = args.point_topic
        self.mount_diff_cm = float(args.mount_diff_cm)
        self.flush_every = max(1, int(args.flush_every))
        self.rows_since_flush = 0
        self.seq = 0
        self.latest: Dict[str, Tuple[float, int]] = {}

        self.output_path = resolve_output_path(args.output)
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self.csv_file = self.output_path.open("w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(
            self.csv_file,
            fieldnames=[
                "seq",
                "ros_time_sec",
                "wall_time_iso",
                "source",
                "topic",
                "height_cm",
                "array_ground_height_cm",
                "point_height_cm",
                "pair_age_sec",
                "drop_cm",
            ],
        )
        self.writer.writeheader()
        self.csv_file.flush()

        qos = QoSProfile(depth=max(1, int(args.qos_depth)))
        self.create_subscription(
            Int16,
            self.area_topic,
            lambda msg: self._record("array_ground", self.area_topic, msg),
            qos,
        )
        self.create_subscription(
            Int16,
            self.point_topic,
            lambda msg: self._record("point_fc", self.point_topic, msg),
            qos,
        )

        if args.duration_sec > 0:
            self.stop_time_sec = self._now_sec() + float(args.duration_sec)
            self.create_timer(0.1, self._stop_when_duration_reached)
        else:
            self.stop_time_sec = None

        self.create_timer(max(0.5, float(args.status_period_sec)), self._print_status)
        self.get_logger().info(
            f"collecting {self.area_topic} and {self.point_topic} -> {self.output_path}"
        )

    def close(self) -> None:
        self.csv_file.flush()
        self.csv_file.close()

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _record(self, source: str, topic: str, msg: Int16) -> None:
        now_sec = self._now_sec()
        value = int(msg.data)
        self.latest[source] = (now_sec, value)

        area = self.latest.get("array_ground")
        point = self.latest.get("point_fc")
        area_value = area[1] if area else None
        point_value = point[1] if point else None

        pair_age: Optional[float] = None
        drop: Optional[float] = None
        if area and point:
            pair_age = abs(area[0] - point[0])
            drop = float(area[1]) - self.mount_diff_cm - float(point[1])

        self.seq += 1
        self.writer.writerow(
            {
                "seq": self.seq,
                "ros_time_sec": f"{now_sec:.9f}",
                "wall_time_iso": datetime.now().isoformat(timespec="milliseconds"),
                "source": source,
                "topic": topic,
                "height_cm": value,
                "array_ground_height_cm": "" if area_value is None else area_value,
                "point_height_cm": "" if point_value is None else point_value,
                "pair_age_sec": "" if pair_age is None else f"{pair_age:.6f}",
                "drop_cm": "" if drop is None else f"{drop:.3f}",
            }
        )

        self.rows_since_flush += 1
        if self.rows_since_flush >= self.flush_every:
            self.csv_file.flush()
            self.rows_since_flush = 0

    def _print_status(self) -> None:
        area = self.latest.get("array_ground")
        point = self.latest.get("point_fc")
        area_text = "NA" if area is None else f"{area[1]}cm"
        point_text = "NA" if point is None else f"{point[1]}cm"
        self.get_logger().info(
            f"rows={self.seq} array_ground={area_text} point_fc={point_text}"
        )

    def _stop_when_duration_reached(self) -> None:
        if self.stop_time_sec is not None and self._now_sec() >= self.stop_time_sec:
            raise KeyboardInterrupt


def resolve_output_path(output: str) -> Path:
    if output:
        return Path(output).expanduser()

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_dir = Path(os.environ.get("FLY_LOG_DIR", "~/fly_logs")).expanduser()
    return log_dir / f"height_data_{timestamp}.csv"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Record timestamped array-laser and uart_to_stm32 height CSV data."
    )
    parser.add_argument("-o", "--output", default="", help="CSV output path")
    parser.add_argument("--area-topic", default=DEFAULT_AREA_TOPIC)
    parser.add_argument("--point-topic", default=DEFAULT_POINT_TOPIC)
    parser.add_argument(
        "--mount-diff-cm",
        type=float,
        default=9.0,
        help="array laser minus point laser z offset, used only for drop_cm",
    )
    parser.add_argument(
        "--duration-sec",
        type=float,
        default=0.0,
        help="stop automatically after this many seconds; 0 means run until Ctrl-C",
    )
    parser.add_argument("--qos-depth", type=int, default=100)
    parser.add_argument(
        "--flush-every",
        type=int,
        default=1,
        help="flush CSV every N rows; 1 is safest for flight tests",
    )
    parser.add_argument("--status-period-sec", type=float, default=2.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = HeightCollector(args)

    shutdown_requested = False

    def _handle_signal(_signum, _frame) -> None:
        nonlocal shutdown_requested
        shutdown_requested = True

    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    try:
        while rclpy.ok() and not shutdown_requested:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        output_path = node.output_path
        row_count = node.seq
        node.close()
        node.destroy_node()
        rclpy.shutdown()
        print(f"[height_data] wrote {row_count} rows -> {output_path}")


if __name__ == "__main__":
    main()
