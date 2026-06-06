"""进阶任务规划航线 —— 脱机调试工具（不连飞机、不连 ROS 话题）。

用途：飞机不开机时，单独把地面站对 24 个货位画的「进阶任务规划航线图」逐个调出来看，
反复改 WarehouseMap.route_points_m() 的绕板逻辑后立刻看效果。复用主程序同一个
WarehouseMap 控件和同一份 slot_waypoints.json，所见即主界面所画。

运行：
    cd diansai_terminal
    python3 -m ground_station.route_debug
键盘：← / → 上一个 / 下一个货位；点右侧按钮直接跳。
"""
from __future__ import annotations

import sys
from pathlib import Path

from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (
    QApplication,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from ground_station.main import WarehouseMap
    from ground_station.models import SLOTS, load_waypoint_config, waypoint_map
else:
    from .main import WarehouseMap
    from .models import SLOTS, load_waypoint_config, waypoint_map


class RouteDebugWindow(QWidget):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("进阶任务规划航线 调试")
        self.resize(1100, 720)

        self.config = load_waypoint_config()
        self.waypoints = waypoint_map(self.config)
        self.map = WarehouseMap(self.config, self.waypoints)

        self.slot_buttons: dict[str, QPushButton] = {}
        grid = QGridLayout()
        grid.setSpacing(4)
        for idx, slot in enumerate(SLOTS):
            btn = QPushButton(slot)
            btn.setCheckable(True)
            btn.setMinimumHeight(40)
            btn.clicked.connect(lambda _checked, s=slot: self.show_slot(s))
            self.slot_buttons[slot] = btn
            grid.addWidget(btn, idx // 6, idx % 6)  # 6 列：每面一行 A/B/C/D

        self.info = QLabel()
        self.info.setFont(QFont("Monospace", 11))
        self.info.setWordWrap(True)
        self.info.setAlignment(Qt.AlignTop)
        self.info.setStyleSheet("background:#0f1722;color:#cfe3ff;padding:10px;border-radius:6px;")

        nav = QHBoxLayout()
        prev_btn = QPushButton("← 上一个")
        next_btn = QPushButton("下一个 →")
        prev_btn.clicked.connect(lambda: self.step(-1))
        next_btn.clicked.connect(lambda: self.step(1))
        nav.addWidget(prev_btn)
        nav.addWidget(next_btn)

        side = QVBoxLayout()
        side.addWidget(QLabel("选择货位（A/B/C/D 各一行）"))
        side.addLayout(grid)
        side.addLayout(nav)
        side.addWidget(QLabel("规划航线折线（米，仓库系；红点=货架两端）"))
        side.addWidget(self.info, 1)
        side_widget = QWidget()
        side_widget.setLayout(side)
        side_widget.setFixedWidth(440)

        root = QHBoxLayout(self)
        root.addWidget(self.map, 1)
        root.addWidget(side_widget)

        self.current = SLOTS[0]
        self.show_slot(self.current)

    def show_slot(self, slot: str) -> None:
        self.current = slot
        self.map.set_target_slot(slot)
        for s, btn in self.slot_buttons.items():
            btn.setChecked(s == slot)
        self._update_info(slot)

    def step(self, delta: int) -> None:
        idx = (SLOTS.index(self.current) + delta) % len(SLOTS)
        self.show_slot(SLOTS[idx])

    def _update_info(self, slot: str) -> None:
        wp = self.waypoints[slot]
        pts = self.map.route_points_m(slot)
        racks = self.config["warehouse"].get("racks", [])
        crossings = self._board_crossings(pts, racks)
        lines = [
            f"货位 {slot}: x={wp.x:.2f} y={wp.y:.2f} z={wp.z:.2f} yaw={wp.yaw:.2f}rad",
            "",
            "航线折线（依次经过）:",
        ]
        lines += [f"  {i}. ({x:.2f}, {y:.2f})" for i, (x, y) in enumerate(pts, 1)]
        lines += ["", "穿板检查: " + ("⚠ 有横段穿过货架!" if crossings else "✅ 无横段穿板")]
        for seg in crossings:
            lines.append(f"  段 {seg}")
        self.info.setText("\n".join(lines))

    @staticmethod
    def _board_crossings(pts, racks) -> list[str]:
        """逐段查：水平段(同 y)是否在某块板 x 处、且 y 落在板 [y_min,y_max] 内穿过。"""
        hits = []
        for (x1, y1), (x2, y2) in zip(pts, pts[1:]):
            if abs(y1 - y2) > 1e-6:
                continue  # 只查横段（竖段沿本列 x，本列无板）
            for r in racks:
                bx, ymin, ymax = float(r["x"]), float(r["y_min"]), float(r["y_max"])
                if min(x1, x2) < bx < max(x1, x2) and ymin <= y1 <= ymax:
                    hits.append(f"y={y1:.2f} 横穿 {r.get('name', '?')}板(x={bx})")
        return hits


def main() -> None:
    app = QApplication(sys.argv)
    win = RouteDebugWindow()
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
