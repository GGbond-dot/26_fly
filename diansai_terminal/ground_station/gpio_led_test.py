"""物理 LED 自测脚本——不开地面站、不连 ROS，直接验证 LED 接线/权限对不对。

跑法（地面站那块香橙派上）：
    python3 -m ground_station.gpio_led_test                # 默认 pin13/on低，闪 3 次
    python3 -m ground_station.gpio_led_test --count 5      # 闪 5 次
    python3 -m ground_station.gpio_led_test --pin 7        # 换引脚
    python3 -m ground_station.gpio_led_test --on-level 1 --off-level 0   # 灯反了就翻电平

每次「亮 1s → 灭 0.5s」，和盘点一个货位的 LED 亮灭节奏一致（要求 1-2②）。
看不到亮：①gpio 没装（脚本会提示）②权限（加 gpio 组/重登）③接线/电平极性（--on-level 反一下）。
"""

from __future__ import annotations

import argparse
import sys
import time

if __package__ in (None, ""):
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from ground_station.gpio_led import GpioLed, _gpio_available
else:
    from .gpio_led import GpioLed, _gpio_available


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Ground-station physical LED self-test")
    parser.add_argument("--pin", type=int, default=13, help="WiringOP pin (default 13)")
    parser.add_argument("--on-level", type=int, default=0, help="on level (default 0=low)")
    parser.add_argument("--off-level", type=int, default=1, help="off level (default 1)")
    parser.add_argument("--count", type=int, default=3, help="blink times (default 3)")
    args = parser.parse_args(argv if argv is not None else sys.argv[1:])

    if not _gpio_available():
        print("[FAIL] 找不到 `gpio` 命令 → WiringOP 没装。先 `sudo apt install` 装 wiringOP，"
              "`gpio readall` 能看到引脚表再来。")
        return 1

    led = GpioLed(pin=args.pin, on_level=args.on_level, off_level=args.off_level)
    if not led.available:
        print(f"[FAIL] GPIO 初始化失败（pin={args.pin}）→ 多半是权限。把用户加进 gpio 组后"
              "重新登录，或先用 sudo 跑试试。")
        return 1

    print(f"[OK] gpio 就绪，pin={args.pin} on={args.on_level} off={args.off_level}，"
          f"开始闪 {args.count} 次（亮1s 灭0.5s）。看不到灯=接线/电平极性，试 --on-level 反一下。")
    for i in range(1, args.count + 1):
        print(f"  第 {i}/{args.count} 次：亮…", flush=True)
        led.blink(1.0)          # 子线程亮1s灭
        time.sleep(1.5)         # 等亮灭完整跑完 + 间隔
    led.close()
    print("[DONE] 测试结束，灯已灭。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
