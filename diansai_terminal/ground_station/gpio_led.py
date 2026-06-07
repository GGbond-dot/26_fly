"""地面站物理 LED 灯——和飞机激光同一套接口（香橙派 WiringOP `gpio` 命令控脚）。

要求 1-2② 每盘点一个货位 LED 亮灭 1s。地面站屏幕上那个 LED 方块仍保留（闪烁好看），
这里额外驱动一颗**物理 LED**：接法/控制方式和飞机激光完全一致（见开发笔记 §5-3）——
WiringOP `gpio mode <pin> out` + `gpio write <pin> <level>`，on_level=0 亮 / off_level=1 灭，
pin 用 WiringOP 编号（默认 13，同激光）。命令行控脚，**不是** python wiringpi 模块（本机没装）。

`gpio` 找不到（没装 WiringOP / 跑在开发机）或 pin=-1 时静默降级，只闪屏、不碰 GPIO。
"""

from __future__ import annotations

import shutil
import subprocess
import threading


def _gpio_available() -> bool:
    """香橙派 WiringOP 的 `gpio` 命令行控脚（同飞机激光 / magnet_control_pkg）。"""
    return shutil.which("gpio") is not None


def _gpio_run(*args: str) -> None:
    subprocess.run(["gpio", *args], check=True, text=True,
                   capture_output=True, timeout=1.0)


class GpioLed:
    """物理 LED 灯控制，照搬飞机激光 GPIO 逻辑（亮灭 1s 不阻塞 UI）。"""

    def __init__(self, pin: int = 13, on_level: int = 0, off_level: int = 1) -> None:
        self.pin = int(pin)
        self.on_level = int(on_level)
        self.off_level = int(off_level)
        self.available = False
        self._lock = threading.Lock()
        self._init_gpio()

    def _init_gpio(self) -> None:
        if self.pin == -1:
            return
        if not _gpio_available():
            # 没装 WiringOP（开发机/地面站没接灯）→ 静默降级，只闪屏不碰 GPIO。
            return
        try:
            _gpio_run("mode", str(self.pin), "out")
            _gpio_run("write", str(self.pin), str(self.off_level))  # 初始灭
            self.available = True
        except Exception:
            # 权限/无该脚 → 放弃物理 LED，不影响地面站其它功能。
            self.available = False

    def blink(self, duration_s: float = 1.0) -> None:
        """LED 亮 duration_s 再灭，子线程执行不阻塞 Qt 主线程。"""
        if not self.available:
            return
        threading.Thread(target=self._blink_worker, args=(duration_s,),
                         daemon=True).start()

    def _blink_worker(self, duration_s: float) -> None:
        # 串行化：连续两次盘点间隔短时不让亮灭交叠。
        with self._lock:
            try:
                _gpio_run("write", str(self.pin), str(self.on_level))
                threading.Event().wait(duration_s)
                _gpio_run("write", str(self.pin), str(self.off_level))
            except Exception:
                pass

    def close(self) -> None:
        """退出前确保灭灯。"""
        if not self.available or self.pin == -1:
            return
        try:
            _gpio_run("write", str(self.pin), str(self.off_level))
        except Exception:
            pass
