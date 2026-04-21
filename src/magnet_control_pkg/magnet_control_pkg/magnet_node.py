import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, String

from magnet_control_pkg.gpio_backend import GpioMagnet, MagnetLevels


class MagnetControlNode(Node):
    def __init__(self) -> None:
        super().__init__("magnet_control_node")

        self.declare_parameter("pin", 10)
        self.declare_parameter("on_level", 0)
        self.declare_parameter("off_level", 1)
        self.declare_parameter("initial_off", True)
        self.declare_parameter("pulse_duration", 1.0)
        self.declare_parameter("status_period", 0.5)

        self.pin = int(self.get_parameter("pin").value)
        on_level = int(self.get_parameter("on_level").value)
        off_level = int(self.get_parameter("off_level").value)
        self.pulse_duration = float(self.get_parameter("pulse_duration").value)

        self.magnet = GpioMagnet(self.pin, MagnetLevels(on=on_level, off=off_level))

        self.status_pub = self.create_publisher(Bool, "magnet/status", 10)
        self.result_pub = self.create_publisher(String, "magnet/result", 10)
        self.cmd_sub = self.create_subscription(Int32, "magnet/cmd", self._cmd_callback, 10)

        status_period = float(self.get_parameter("status_period").value)
        self.status_timer = self.create_timer(status_period, self._publish_status)

        if bool(self.get_parameter("initial_off").value):
            self._set_hardware(False)

        self.get_logger().info(f"Magnet control ready on GPIO1_A4 / WiringOP pin {self.pin}")
        self.get_logger().info("Command topic: /magnet/cmd, 1=on, 2=off, 3=pulse, 4=status")

    def _cmd_callback(self, msg: Int32) -> None:
        try:
            command = int(msg.data)
            if command == 1:
                self._set_hardware(True)
                self._publish_result("1: magnet enabled")
            elif command == 2:
                self._set_hardware(False)
                self._publish_result("2: magnet disabled")
            elif command == 3:
                self._pulse()
            elif command == 4:
                enabled = self._publish_status()
                if enabled is None:
                    self._publish_result("4: failed to read magnet status")
                else:
                    state = "enabled" if enabled else "disabled"
                    self._publish_result(f"4: magnet status is {state}")
            else:
                self._publish_result(f"unknown command {command}; use 1=on, 2=off, 3=pulse, 4=status")
        except Exception as exc:
            error = f"Failed to handle magnet command {msg.data}: {exc}"
            self.get_logger().error(error)
            self._publish_result(error)

    def _pulse(self) -> None:
        self._set_hardware(True)
        time.sleep(self.pulse_duration)
        self._set_hardware(False)
        self._publish_result(f"3: magnet pulsed for {self.pulse_duration:g} second(s)")

    def _publish_status(self) -> bool | None:
        try:
            msg = Bool()
            msg.data = self.magnet.read_enabled()
            self.status_pub.publish(msg)
            return msg.data
        except Exception as exc:
            self.get_logger().warn(f"Failed to read magnet status: {exc}")
            return None

    def _set_hardware(self, enabled: bool) -> None:
        self.magnet.set_enabled(enabled)
        msg = Bool()
        msg.data = enabled
        self.status_pub.publish(msg)

    def _publish_result(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.result_pub.publish(msg)
        self.get_logger().info(text)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MagnetControlNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
