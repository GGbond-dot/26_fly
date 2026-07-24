#!/usr/bin/env python3
"""相机调参窗口 —— 拖滑条实时看效果，点 SAVE 固化。

    ros2 launch fire_video_link_pkg video_test.launch.py tune:=true

窗口上半是画面，下半是滑条。拖动立刻生效（改的是 V4L2 设备级控制项，
不用重启节点）。调好点画面底部的绿色 SAVE 按钮，或按 s 键。

保存到 ~/.ros/fire_vision_camera.yaml，fire_detector 每次启动会自动读它并应用，
**不用重新编译**。想恢复出厂就删掉这个文件。

为什么画面是订阅 ROS 话题而不是自己开相机：相机被 fire_detector 独占了，
同一个 /dev/video* 不能开两次。而 V4L2 的控制项是设备级的，
另一个进程用 v4l2-ctl 去改，正在采图的那个进程立刻就受影响——这是这套东西能工作的原因。
"""

from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys
import threading

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

DEFAULT_DEV = ("/dev/v4l/by-path/"
               "platform-xhci-hcd.11.auto-usb-0:1:1.0-video-index0")
DEFAULT_TOPIC = "/camera/down/compressed"
SAVE_PATH = os.path.expanduser("~/.ros/fire_vision_camera.yaml")
WINDOW = "camera tune"

# 只给这些控制项建滑条。全建的话滑条几十条，窗口塞不下也没意义。
# 顺序就是滑条从上到下的顺序，按调参时的实际使用频率排。
WANTED = [
    "gain",
    "exposure_time_absolute", "exposure_absolute",   # 新/旧内核名字不同
    "auto_exposure", "exposure_auto",
    "brightness",
    "contrast",
    "saturation",
    "white_balance_automatic", "white_balance_temperature_auto",
    "white_balance_temperature",
]

CTRL_RE = re.compile(
    r"^\s*(?P<name>\w+)\s+0x[0-9a-f]+\s+\((?P<type>\w+)\)\s*:\s*(?P<rest>.*)$")


def list_controls(dev: str) -> dict:
    """解析 v4l2-ctl --list-ctrls，拿到每项的 min/max/当前值。"""
    try:
        out = subprocess.run(
            ["v4l2-ctl", "-d", dev, "--list-ctrls"],
            capture_output=True, text=True, timeout=5).stdout
    except FileNotFoundError:
        print("[err] 没装 v4l2-ctl：sudo apt install v4l-utils")
        return {}
    except subprocess.SubprocessError as e:
        print(f"[err] 读相机控制项失败: {e}")
        return {}

    ctrls = {}
    for line in out.splitlines():
        m = CTRL_RE.match(line)
        if not m:
            continue
        rest = m.group("rest")
        kv = dict(re.findall(r"(\w+)=(-?\d+)", rest))
        if "value" not in kv:
            continue
        ctype = m.group("type")
        if ctype == "bool":
            lo, hi = 0, 1
        else:
            if "min" not in kv or "max" not in kv:
                continue
            lo, hi = int(kv["min"]), int(kv["max"])
        if hi <= lo:
            continue
        ctrls[m.group("name")] = {
            "min": lo, "max": hi,
            "value": int(kv["value"]),
            "type": ctype,
            "inactive": "inactive" in rest,
        }
    return ctrls


def set_control(dev: str, name: str, value: int) -> bool:
    try:
        r = subprocess.run(
            ["v4l2-ctl", "-d", dev, f"--set-ctrl={name}={value}"],
            capture_output=True, text=True, timeout=5)
        return r.returncode == 0
    except subprocess.SubprocessError:
        return False


class Viewer(Node):
    """只负责收图，不碰相机。"""

    def __init__(self, topic: str) -> None:
        super().__init__("camera_tune_viewer")
        self.frame = None
        self.lock = threading.Lock()
        self.create_subscription(
            CompressedImage, topic, self._on_image, qos_profile_sensor_data)

    def _on_image(self, msg: CompressedImage) -> None:
        img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if img is not None:
            with self.lock:
                self.frame = img

    def get(self):
        with self.lock:
            return None if self.frame is None else self.frame.copy()


def save_yaml(dev: str, names: list[str]) -> str:
    """把设备上**当前实际生效**的值存下来。

    存的是从设备读回来的值，不是滑条位置——有些控制项驱动会拒绝或钳位，
    存滑条位置会导致下次启动应用了一个设备根本不接受的值。
    """
    cur = list_controls(dev)
    os.makedirs(os.path.dirname(SAVE_PATH), exist_ok=True)
    lines = [
        "# 由 scripts/camera_tune.py 保存，fire_detector 启动时自动应用。",
        "# 想恢复相机默认值，直接删掉本文件。",
        "controls:",
    ]
    for n in names:
        if n in cur:
            lines.append(f"  {n}: {cur[n]['value']}")
    with open(SAVE_PATH, "w") as f:
        f.write("\n".join(lines) + "\n")
    return SAVE_PATH


def main() -> int:
    ap = argparse.ArgumentParser(description="相机调参窗口")
    ap.add_argument("--device", default=DEFAULT_DEV)
    ap.add_argument("--topic", default=DEFAULT_TOPIC)
    args = ap.parse_args()

    ctrls = list_controls(args.device)
    if not ctrls:
        print(f"[err] 读不到 {args.device} 的控制项。设备路径对吗？")
        return 1

    names = [n for n in WANTED if n in ctrls]
    if not names:
        print("[err] 相机不支持任何常用控制项，打印全部：")
        print(", ".join(ctrls))
        return 1

    print("可调项：")
    for n in names:
        c = ctrls[n]
        flag = "  (当前 inactive，可能要先关自动模式)" if c["inactive"] else ""
        print(f"  {n:32s} {c['min']}~{c['max']}  当前={c['value']}{flag}")

    rclpy.init()
    node = Viewer(args.topic)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW, 720, 700)

    # 滑条回调直接打设备。拖动过程中会连发很多次，v4l2-ctl 很轻量，扛得住。
    def make_cb(name: str):
        def cb(v: int) -> None:
            lo = ctrls[name]["min"]
            set_control(args.device, name, v + lo)   # 滑条从 0 起，要加回 min
        return cb

    for n in names:
        c = ctrls[n]
        span = c["max"] - c["min"]
        cv2.createTrackbar(n[:24], WINDOW, c["value"] - c["min"], span, make_cb(n))

    state = {"msg": "", "msg_ttl": 0}

    def on_mouse(event, x, y, flags, _param) -> None:
        if event == cv2.EVENT_LBUTTONDOWN and state.get("save_box"):
            x0, y0, x1, y1 = state["save_box"]
            if x0 <= x <= x1 and y0 <= y <= y1:
                path = save_yaml(args.device, names)
                state["msg"] = f"已保存 -> {path}"
                state["msg_ttl"] = 90

    cv2.setMouseCallback(WINDOW, on_mouse)

    print("\n拖滑条实时生效。点绿色 SAVE 按钮（或按 s）保存，q/ESC 退出。")

    try:
        while rclpy.ok():
            frame = node.get()
            if frame is None:
                canvas = np.full((240, 320, 3), 30, np.uint8)
                cv2.putText(canvas, "waiting video...", (30, 120),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (180, 180, 180), 2)
            else:
                canvas = frame
                if canvas.shape[1] < 480:      # 小图放大点，肉眼才看得出噪点
                    k = 480 / canvas.shape[1]
                    canvas = cv2.resize(canvas, None, fx=k, fy=k,
                                        interpolation=cv2.INTER_NEAREST)

            h, w = canvas.shape[:2]
            bar = np.full((46, w, 3), 25, np.uint8)
            view = np.vstack([canvas, bar])

            bx0, by0 = w - 110, h + 8
            bx1, by1 = w - 10, h + 38
            state["save_box"] = (bx0, by0, bx1, by1)
            cv2.rectangle(view, (bx0, by0), (bx1, by1), (40, 160, 60), -1)
            cv2.putText(view, "SAVE", (bx0 + 22, by1 - 9),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            if state["msg_ttl"] > 0:
                state["msg_ttl"] -= 1
                cv2.putText(view, state["msg"], (10, h + 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (120, 230, 140), 1)
            else:
                cv2.putText(view, "drag sliders / click SAVE / q to quit",
                            (10, h + 30), cv2.FONT_HERSHEY_SIMPLEX,
                            0.45, (150, 150, 150), 1)

            cv2.imshow(WINDOW, view)
            key = cv2.waitKey(30) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("s"):
                path = save_yaml(args.device, names)
                state["msg"] = f"已保存 -> {path}"
                state["msg_ttl"] = 90
                print(f"[save] {path}")
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
