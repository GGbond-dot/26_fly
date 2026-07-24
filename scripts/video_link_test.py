#!/usr/bin/env python3
"""视频链路自测（无人机侧跑）——G 题 通道 5：跨域 DDS 视频桥。

同时在**两个 ROS_DOMAIN_ID** 上订阅，把链路切成三段分别定位，
省得"车端看不到图"时对着一整条链路瞎猜：

    [1] 飞机内部域 26         [2] 桥转发           [3] 共享域 6
    /camera/down/compressed  ──────→  /fly/fire/debug_image/compressed
       (fire_vision_pkg 发)                        (车端订阅这个)

原理和桥一样：一个进程建两个 rclpy.Context，各绑一个域。

用法
----
体检（最常用，飞机上跑，看三段哪段断了）：
    source scripts/setup_dds.sh
    python3 scripts/video_link_test.py

没有相机/没起视觉节点时，先造假图验证桥本身：
    python3 scripts/video_link_test.py --publish        # 终端A：在域26发假图
    python3 scripts/video_link_test.py                  # 终端B：体检

在**消防车**上验收（车端只有域 6，用 --dst-only）：
    python3 video_link_test.py --dst-only

改域号 / 话题：
    python3 scripts/video_link_test.py --src-domain 26 --dst-domain 6
"""

from __future__ import annotations

import argparse
import socket
import sys
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import CompressedImage

DEFAULT_SRC_TOPIC = "/camera/down/compressed"
DEFAULT_DST_TOPIC = "/fly/fire/debug_image/compressed"

# fire_link_pkg 在无人机侧 bind 的 UDP 口。端口被占用 = 那个节点在跑。
DRONE_UDP_PORTS = {8890: "车→机 任务状态回传", 8893: "车→机 按键启动"}


class Probe:
    """一个域上的一路订阅探针。"""

    def __init__(self, name: str, domain: int, topic: str, args) -> None:
        self.name = name
        self.domain = domain
        self.topic = topic

        self.count = 0
        self.bytes = 0
        self.first_at = None
        self.last_at = None
        self.stamps = {}   # header stamp(ns) -> 本地收到时刻，给下游算桥延迟
        self.lock = threading.Lock()

        self.ctx = rclpy.Context()
        rclpy.init(context=self.ctx, args=args, domain_id=domain)
        self.node = Node(f"video_link_probe_{name}", context=self.ctx)
        self.node.create_subscription(
            CompressedImage, topic, self._cb, qos_profile_sensor_data)
        self.exec_ = SingleThreadedExecutor(context=self.ctx)
        self.exec_.add_node(self.node)
        self.thread = threading.Thread(target=self.exec_.spin, daemon=True)

    def _cb(self, msg: CompressedImage) -> None:
        now = time.monotonic()
        with self.lock:
            self.count += 1
            self.bytes += len(msg.data)
            if self.first_at is None:
                self.first_at = now
            self.last_at = now
            key = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            self.stamps[key] = now
            if len(self.stamps) > 200:      # 只留最近的，别无限涨
                for k in sorted(self.stamps)[:100]:
                    del self.stamps[k]

    def start(self) -> None:
        self.thread.start()

    def snapshot(self) -> tuple[int, int]:
        """取出并清零本窗口的计数。"""
        with self.lock:
            c, b = self.count, self.bytes
            self.count = self.bytes = 0
            return c, b

    def shutdown(self) -> None:
        """关闭顺序不能乱：先让 spin 退出并 join，再 destroy_node。

        少了 join 这一步，spin 线程还在回调里而节点已经被销毁，
        rclpy 会在 C++ 层 terminate（表现为退出时 core dump，诊断结果照样打印，
        但退出码是错的，接脚本判断时会误判）。
        """
        self.exec_.shutdown()
        if self.thread.is_alive():
            self.thread.join(timeout=2.0)
        self.node.destroy_node()
        rclpy.shutdown(context=self.ctx)


def check_udp_ports() -> list[str]:
    """检查控制面 UDP 口有没有被 fire_link_node 占着。

    优先级第一条是"UDP 控制包不能被视频流挤掉"，所以体检先看控制面还在不在。
    能 bind 成功 = 没人在监听 = fire_link_node 没起（或崩了）。
    """
    lines = []
    for port, desc in sorted(DRONE_UDP_PORTS.items()):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.bind(("0.0.0.0", port))
            lines.append(f"  ⚠️  UDP {port} ({desc}) 空闲 —— fire_link_node 没在跑")
        except OSError:
            lines.append(f"  ✅ UDP {port} ({desc}) 已被占用 —— 收包节点在跑")
        finally:
            s.close()
    return lines


def run_publisher(args) -> int:
    """在源域上发假 JPEG，用来在没有相机/视觉节点时单独验证桥。"""
    try:
        import cv2
        import numpy as np
        have_cv = True
    except ImportError:
        have_cv = False
        print("[warn] 没有 cv2，改发随机字节（能验证链路通断，但车端显示不出画面）")

    ctx = rclpy.Context()
    rclpy.init(context=ctx, domain_id=args.src_domain)
    node = Node("video_link_fake_pub", context=ctx)
    pub = node.create_publisher(CompressedImage, args.src_topic, qos_profile_sensor_data)

    print(f"[pub] 域 {args.src_domain} 上以 {args.rate} Hz 发 {args.src_topic}，Ctrl-C 停")
    seq = 0
    period = 1.0 / max(0.5, args.rate)
    try:
        while True:
            msg = CompressedImage()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.header.frame_id = "fake"
            msg.format = "jpeg"
            if have_cv:
                img = np.zeros((240, 320, 3), dtype=np.uint8)
                img[:] = (40, 40, 40)
                cv2.putText(img, f"TEST {seq}", (20, 130),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
                ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 50])
                msg.data = buf.tobytes() if ok else b"\xff\xd8\xff\xd9"
            else:
                msg.data = bytes(4096)
            pub.publish(msg)
            seq += 1
            if seq % 20 == 0:
                print(f"[pub] 已发 {seq} 帧")
            time.sleep(period)
    except KeyboardInterrupt:
        print("\n[pub] 停止")
    finally:
        node.destroy_node()
        rclpy.shutdown(context=ctx)
    return 0


def diagnose(src: Probe | None, dst: Probe | None, delays: list[float]) -> int:
    """把三段现象翻译成"该去查什么"，这才是这脚本存在的意义。"""
    print("\n" + "=" * 62)
    print("诊断")
    print("=" * 62)

    src_ok = src is not None and src.first_at is not None
    dst_ok = dst is not None and dst.first_at is not None

    if src is None:                                    # --dst-only，车端验收
        if dst_ok:
            print("✅ 共享域上收到视频流，车端可以直接订阅这个话题。")
            return 0
        print("❌ 共享域上没有视频流。在飞机上依次确认：")
        print("   1) 桥进程在跑吗：ros2 launch fire_video_link_pkg fire_video_bridge.launch.py")
        print("   2) 桥日志里有没有 '[桥] ... Hz'（有=飞机侧没问题，是网络）")
        print("   3) 路由器是否转发 DDS 多播：两机能 ping 通但 topic list 互相看不见就是它")
        print("   4) 订阅 QoS 必须是 best_effort，reliable 的订阅一帧都收不到")
        return 1

    if not src_ok and not dst_ok:
        print("❌ 两段都没数据 —— 问题在**源头**，跟桥和网络无关。")
        print("   • fire_detector 起来了吗？")
        print("   • enable_debug_image 是 True 吗？（默认 False，不开不发图）")
        print("   • debug_image_compressed 是 True 吗？")
        print(f"   • 话题名对吗：ros2 topic list | grep compressed")
        return 1

    if src_ok and not dst_ok:
        print("❌ 内部域有图、共享域没有 —— 问题在**桥**这一环。")
        print("   • 桥进程在跑吗？它的日志打的是什么？")
        print("   • yaml 里的类型名必须逐字是 sensor_msgs/msg/CompressedImage")
        print("   • yaml 里的源话题名要和上面这个一致")
        print("   • outer_domain 是不是 6")
        return 1

    if not src_ok and dst_ok:
        print("⚠️  共享域有图但内部域没有 —— 话题名可能对不上，")
        print("    或者有人直接在共享域上发图（绕过了桥）。")
        return 1

    print("✅ 三段全通。")
    if delays:
        delays.sort()
        p50 = delays[len(delays) // 2] * 1000
        p95 = delays[int(len(delays) * 0.95)] * 1000
        print(f"   桥转发延迟 中位数 {p50:.1f} ms / p95 {p95:.1f} ms")
        if p95 > 200:
            print("   ⚠️  p95 偏高，多半是带宽不够在排队："
                  "调小 debug_image_max_hz 或 debug_image_max_kbps")
    return 0


def main() -> int:
    p = argparse.ArgumentParser(description="跨域视频链路自测")
    p.add_argument("--src-domain", type=int, default=26, help="飞机内部域")
    p.add_argument("--dst-domain", type=int, default=6, help="共享域（消防车所在域）")
    p.add_argument("--src-topic", default=DEFAULT_SRC_TOPIC)
    p.add_argument("--dst-topic", default=DEFAULT_DST_TOPIC)
    p.add_argument("--duration", type=float, default=10.0, help="体检时长秒")
    p.add_argument("--publish", action="store_true", help="改为在源域发假图")
    p.add_argument("--rate", type=float, default=10.0, help="--publish 的发送频率")
    p.add_argument("--dst-only", action="store_true", help="只测共享域（车端用）")
    p.add_argument("--no-port-check", action="store_true", help="跳过 UDP 控制口检查")
    args = p.parse_args()

    if args.src_domain == args.dst_domain and not args.publish and not args.dst_only:
        print(f"[err] 两个域都是 {args.src_domain}，测不出桥的效果")
        return 2

    if args.publish:
        return run_publisher(args)

    print("=" * 62)
    print("视频链路体检")
    print(f"  内部域 {args.src_domain}: {args.src_topic}")
    print(f"  共享域 {args.dst_domain}: {args.dst_topic}")
    print("=" * 62)

    if not args.no_port_check:
        print("\n控制面（优先级最高，视频绝不能挤掉它）：")
        for line in check_udp_ports():
            print(line)

    src = None if args.dst_only else Probe("src", args.src_domain, args.src_topic, None)
    dst = Probe("dst", args.dst_domain, args.dst_topic, None)
    for pr in (src, dst):
        if pr:
            pr.start()

    delays: list[float] = []
    print(f"\n采样 {args.duration:.0f} 秒…\n")
    print(f"{'时间':>6} | {'内部域 Hz':>10} {'KB/s':>8} | {'共享域 Hz':>10} {'KB/s':>8}")
    print("-" * 62)

    t0 = time.monotonic()
    try:
        while time.monotonic() - t0 < args.duration:
            time.sleep(1.0)
            sc, sb = src.snapshot() if src else (0, 0)
            dc, db = dst.snapshot()
            print(f"{time.monotonic() - t0:5.0f}s | {sc:10.1f} {sb / 1024:8.0f} |"
                  f" {dc:10.1f} {db / 1024:8.0f}")

            # 同一帧 header.stamp 在两侧的到达时间差 = 桥的转发延迟
            if src:
                with src.lock, dst.lock:
                    for k, t_dst in dst.stamps.items():
                        t_src = src.stamps.get(k)
                        if t_src is not None and t_dst >= t_src:
                            delays.append(t_dst - t_src)
                    dst.stamps.clear()
    except KeyboardInterrupt:
        print("\n[中断]")

    rc = diagnose(src, dst, delays)

    for pr in (src, dst):
        if pr:
            pr.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
