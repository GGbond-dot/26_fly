#!/usr/bin/env python3
"""fire_link 链路自测 —— 站在**消防车**的位置收发包（2023 电赛 G 题）。

用途有两个：
  1. 不用真飞机就能验证机端 fire_link_node 发出来的包对不对；
  2. 它就是车端接收逻辑的参考实现，车端节点照着 struct 格式抄即可。

包格式与 src/fire_link_pkg/include/fire_link_pkg/fire_link_packet.hpp 一致，
小端定长 32 字节。改一处必须同步改另一处。

端口与《G题_机车通信接口约定.md》一致：本脚本 bind 8892 收遥测，向飞机 8893 发启动包。
火源上报走车端已有的 fire_event_bridge（8889 / 0xFC11 / 16 字节），不在本脚本范围内。

用法（在消防车那块板上跑，或本机跑用 --drone-ip 127.0.0.1 自环）：

    # 只收遥测
    ./fire_link_test.py

    # 收遥测，并在 3 秒后向飞机发"按键启动"包
    ./fire_link_test.py --drone-ip 10.42.0.1 --start-after 3

组网：无人机自建热点，机 = 10.42.0.1（网关），车 = 10.42.0.163 接进来。
--drone-ip 填无人机的实际地址；不确定就用 `arp -a` 或看车端 fire_link_bridge 的日志。
"""

import argparse
import socket
import struct
import time

MAGIC = 0xF14E

# <  小端且不做对齐填充，对应 C 侧的 __attribute__((packed))
#   H magic  B type  B phase  H seq  H reserved16  I stamp_ms
#   f x_dm   f y_dm  f distance_dm   f height_dm   I reserved32
FMT = "<HBBHHIffffI"
SIZE = struct.calcsize(FMT)
assert SIZE == 32, f"包长必须是 32 字节，当前 {SIZE}"

TYPE_TELEMETRY = 1
TYPE_CAR_START = 3
# 注：火源上报不走这个包型，它走车端已有的 fire_event_bridge
#（8889 端口 / magic 0xFC11 / 16 字节），本脚本不涉及。

PHASE_NAMES = {
    0: "待命", 1: "起飞", 2: "巡逻", 3: "接近火源", 4: "降高", 5: "悬停",
    6: "抛包", 7: "恢复巡逻", 8: "返航", 9: "降落", 10: "完成", 255: "未知",
}


def make_start_packet(seq):
    return struct.pack(FMT, MAGIC, TYPE_CAR_START, 255, seq, 0,
                       int(time.monotonic() * 1000) & 0xFFFFFFFF,
                       0.0, 0.0, 0.0, 0.0, 0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--listen-port", type=int, default=8892,
                    help="车端收机端遥测的口")
    ap.add_argument("--drone-ip", default="10.42.0.1",
                    help="无人机固定 IP；自环测试填 127.0.0.1")
    ap.add_argument("--drone-port", type=int, default=8893,
                    help="机端收车端启动包的口")
    ap.add_argument("--start-after", type=float, default=None,
                    help="启动后多少秒发一次'按键启动'包；不给则只收不发")
    ap.add_argument("--start-repeat", type=int, default=5,
                    help="启动包连发次数（UDP 丢包冗余）")
    args = ap.parse_args()

    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("0.0.0.0", args.listen_port))
    rx.settimeout(0.2)
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print(f"监听 :{args.listen_port}，启动包将发往 {args.drone_ip}:{args.drone_port}")
    if args.start_after is not None:
        print(f"将在 {args.start_after}s 后发送启动包")

    t0 = time.monotonic()
    start_sent = False

    while True:
        if (args.start_after is not None and not start_sent
                and time.monotonic() - t0 >= args.start_after):
            for i in range(args.start_repeat):
                tx.sendto(make_start_packet(i), (args.drone_ip, args.drone_port))
            start_sent = True
            print(f"[TX] 已发送启动包 ×{args.start_repeat}")

        try:
            data, addr = rx.recvfrom(1024)
        except socket.timeout:
            continue

        if len(data) != SIZE:
            print(f"[!] 来自 {addr} 的包长度异常：{len(data)} 字节，丢弃")
            continue

        (magic, ptype, phase, seq, _r16, stamp_ms,
         x_dm, y_dm, dist_dm, height_dm, _r32) = struct.unpack(FMT, data)

        if magic != MAGIC:
            print(f"[!] magic 不匹配：0x{magic:04X}，丢弃")
            continue

        if ptype == TYPE_TELEMETRY:
            print(f"[遥测 seq={seq:5d}] 位置=({x_dm:5.1f},{y_dm:5.1f})dm "
                  f"高度={height_dm:4.1f}dm 里程={dist_dm:6.1f}dm "
                  f"阶段={PHASE_NAMES.get(phase, phase)}")
        else:
            print(f"[?] 未知包型 type={ptype}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n退出")
