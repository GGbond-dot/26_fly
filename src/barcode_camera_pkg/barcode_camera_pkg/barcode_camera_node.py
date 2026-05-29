from __future__ import annotations

from typing import Optional

import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    from pyzbar.pyzbar import ZBarSymbol, decode
except ImportError:  # pragma: no cover - depends on target board packages
    ZBarSymbol = None
    decode = None


class BarcodeCameraNode(Node):
    def __init__(self) -> None:
        super().__init__("barcode_camera_node")

        self.declare_parameter("camera_device", "/dev/video0")
        self.declare_parameter("frame_width", 640)
        self.declare_parameter("frame_height", 480)
        self.declare_parameter("fps", 15.0)
        self.declare_parameter("barcode_topic", "/barcode_text")
        self.declare_parameter("show_preview", True)
        self.declare_parameter("window_name", "barcode_camera_preview")
        self.declare_parameter("publish_duplicates", False)
        self.declare_parameter("stop_after_first_publish", True)

        self.camera_device = str(self.get_parameter("camera_device").value)
        self.frame_width = int(self.get_parameter("frame_width").value)
        self.frame_height = int(self.get_parameter("frame_height").value)
        self.fps = float(self.get_parameter("fps").value)
        self.show_preview = bool(self.get_parameter("show_preview").value)
        self.window_name = str(self.get_parameter("window_name").value)
        self.publish_duplicates = bool(self.get_parameter("publish_duplicates").value)
        self.stop_after_first_publish = bool(self.get_parameter("stop_after_first_publish").value)
        barcode_topic = str(self.get_parameter("barcode_topic").value)

        if decode is None or ZBarSymbol is None:
            raise RuntimeError("pyzbar is not installed. Install python3-pyzbar and libzbar0.")

        self.publisher = self.create_publisher(String, barcode_topic, 10)
        self.capture = cv2.VideoCapture(self.camera_device)
        if not self.capture.isOpened():
            raise RuntimeError(f"Failed to open camera device {self.camera_device}")

        if self.frame_width > 0:
            self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        if self.frame_height > 0:
            self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        if self.fps > 0.0:
            self.capture.set(cv2.CAP_PROP_FPS, self.fps)

        self.last_text: Optional[str] = None
        period = 1.0 / max(self.fps, 1.0)
        self.timer = self.create_timer(period, self._read_frame)

        self.get_logger().info(
            f"Code128 barcode camera ready. camera={self.camera_device} topic={barcode_topic}"
        )

    def _read_frame(self) -> None:
        ok, frame = self.capture.read()
        if not ok or frame is None:
            self.get_logger().warn("Failed to read frame from barcode camera.")
            return

        results = decode(frame, symbols=[ZBarSymbol.CODE128])
        for result in results:
            text = result.data.decode("utf-8", errors="replace")
            if self.publish_duplicates or text != self.last_text:
                msg = String()
                msg.data = text
                self.publisher.publish(msg)
                self.last_text = text
                self.get_logger().info(f"Detected Code128 barcode: {text}")
                if self.stop_after_first_publish:
                    self._stop_camera()
                    return
            break

        if self.show_preview:
            self._draw_preview(frame, results)
            cv2.imshow(self.window_name, frame)
            cv2.waitKey(1)

    def _draw_preview(self, frame, results) -> None:
        for result in results:
            points = result.polygon
            if points:
                for index, point in enumerate(points):
                    next_point = points[(index + 1) % len(points)]
                    cv2.line(
                        frame,
                        (point.x, point.y),
                        (next_point.x, next_point.y),
                        (0, 255, 0),
                        2,
                    )
            x, y, width, height = result.rect
            cv2.putText(
                frame,
                result.data.decode("utf-8", errors="replace"),
                (x, max(0, y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )
            cv2.rectangle(frame, (x, y), (x + width, y + height), (0, 255, 0), 2)

    def destroy_node(self) -> None:
        self._stop_camera()
        super().destroy_node()

    def _stop_camera(self) -> None:
        if hasattr(self, "timer") and self.timer is not None:
            self.timer.cancel()
            self.destroy_timer(self.timer)
            self.timer = None
        if self.capture.isOpened():
            self.capture.release()
        cv2.destroyAllWindows()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = BarcodeCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
