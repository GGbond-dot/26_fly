#!/usr/bin/env python3
"""TOFSense-M 串口解析自测脚本（宿舍用，不依赖 ROS）。

直接读 /dev/ttyUSB0，按 NLink_TOFSense_M 协议拆帧 + 校验和，
复刻香橙派上 laser_array_ground_node.cpp 的地面高度逻辑：
取所有"有效束"距离的最大值当地面（max(valid)）。

用途：
  1) 确认模块是否已切到 4x4（count=16）还是仍是 8x8（count=64）；
  2) 验证帧能否正确解析（校验和通过率）；
  3) 看地面高度 cm 是否合理（贴桌面应该几 cm~几十 cm）。

用法：
  python3 scripts/tofsense_m_test.py                 # 默认 /dev/ttyUSB0 @ 921600
  python3 scripts/tofsense_m_test.py /dev/ttyUSB0 921600
  python3 scripts/tofsense_m_test.py --raw           # 顺便打印每帧 16 束原始距离

若报权限/占用：
  sudo chmod 666 /dev/ttyUSB0          # 临时放权
  sudo systemctl stop ModemManager     # 放开抢占
  关掉 NAssistant（它占着口，串口同一时刻只能一个进程读）
"""

import argparse
import sys
import time

try:
    import serial  # pyserial
except ImportError:
    sys.exit("缺少 pyserial：pip3 install pyserial")

# ── 协议常量（与 laser_array_ground_node.cpp 一致）──
HEADER = bytes([0x57, 0x01, 0xFF, 0x00])
FRONT = 4 + 4 + 1          # 帧头4 + 时间戳4 + 计数1
UNIT = 6                   # 每束：距离3 + 状态1 + 信号强度2
BACK = 6 + 1               # 预留6 + 校验和1
# 整帧长度 = FRONT + count*UNIT + BACK；4x4→112，8x8→400


def packet_size(count: int) -> int:
    return FRONT + count * UNIT + BACK


def decode_distance_m(b: bytes) -> float:
    """3 字节小端有符号 int24，/1e6 得米（与 C++ decodeDistance 一致）。"""
    raw = b[0] | (b[1] << 8) | (b[2] << 16)
    if raw & 0x800000:                # 符号扩展
        raw |= ~0xFFFFFF
    return raw / 1_000_000.0


def checksum_ok(pkt: bytes) -> bool:
    """累加除最后一字节外的所有字节，低 8 位 == 最后一字节。"""
    return (sum(pkt[:-1]) & 0xFF) == pkt[-1]


def parse_frame(pkt: bytes, count: int):
    """返回 (有效束米列表, 总束数)。状态字节==0 才算有效。"""
    valid = []
    for i in range(count):
        off = FRONT + i * UNIT
        unit = pkt[off:off + UNIT]
        if unit[3] == 0:              # dis_status == 0 → 有效
            valid.append(decode_distance_m(unit))
    return valid, count


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", default="/dev/ttyUSB0")
    ap.add_argument("baud", nargs="?", type=int, default=921600)
    ap.add_argument("--raw", action="store_true", help="打印每束原始距离")
    args = ap.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.2)
    except serial.SerialException as e:
        sys.exit(f"打开串口失败：{e}\n（口被 NAssistant 占用？权限？见脚本顶部说明）")

    print(f"已打开 {args.port} @ {args.baud}，按 Ctrl-C 退出\n")

    buf = bytearray()
    frames_ok = 0
    frames_bad = 0
    t0 = time.time()
    last_report = t0

    try:
        while True:
            chunk = ser.read(512)
            if chunk:
                buf.extend(chunk)

            # 找帧头并拆包（仿 drainPackets：先定位 0x57 01 FF 00）
            while True:
                idx = buf.find(HEADER)
                if idx < 0:
                    # 没帧头，丢掉除末尾 3 字节（可能是半个帧头）外的内容
                    if len(buf) > 3:
                        del buf[:-3]
                    break
                if idx > 0:
                    del buf[:idx]      # 丢掉帧头前的杂字节

                if len(buf) < FRONT + 1:
                    break              # 还没收到计数字节，等更多数据

                count = buf[FRONT - 1]         # 计数字节（偏移 8）
                if count not in (16, 64):
                    # 不是合法计数 → 这个帧头是假的，跳过 1 字节继续找
                    del buf[:1]
                    continue

                size = packet_size(count)
                if len(buf) < size:
                    break              # 整帧还没收齐，等

                pkt = bytes(buf[:size])
                del buf[:size]

                if not checksum_ok(pkt):
                    frames_bad += 1
                    continue

                frames_ok += 1
                valid, total = parse_frame(pkt, count)
                mode = "4x4" if count == 16 else "8x8"

                if valid:
                    ground_m = max(valid)        # ← 节点用 max(valid) 当地面
                    min_m = min(valid)
                    ground_cm = round(ground_m * 100)
                    min_cm = round(min_m * 100)
                else:
                    ground_cm = min_cm = None

                now = time.time()
                if now - last_report >= 0.25:     # 控制打印频率
                    last_report = now
                    g = "—" if ground_cm is None else f"{ground_cm}cm"
                    m = "—" if min_cm is None else f"{min_cm}cm"
                    print(f"[{mode}] 有效束 {len(valid)}/{total}  "
                          f"地面(max)={g}  最近(min)={m}  "
                          f"校验通过 {frames_ok} / 失败 {frames_bad}")
                    if args.raw and valid:
                        print("    距离(cm): " +
                              " ".join(f"{round(d*100)}" for d in
                                       (decode_distance_m(pkt[FRONT + i*UNIT:
                                                               FRONT + i*UNIT + UNIT])
                                        for i in range(total))))
    except KeyboardInterrupt:
        dt = time.time() - t0
        rate = frames_ok / dt if dt > 0 else 0
        print(f"\n退出。共 {frames_ok} 帧通过校验，{frames_bad} 帧失败，"
              f"约 {rate:.1f} 帧/秒。")
        if frames_ok:
            print("解析正常 ✅ —— 香橙派上的 laser_array_ground 节点用的是同一套逻辑，"
                  "会照常发布 /laser_array/ground_height。")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
