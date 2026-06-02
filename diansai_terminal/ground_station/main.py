from __future__ import annotations

import argparse
import os
import sys
from typing import Any

# The ground station is a pure QWidget UI. On Orange Pi/Rockchip desktop
# images, Qt may try GLX first and print noisy libGL rockchip driver warnings.
os.environ.setdefault("QT_XCB_GL_INTEGRATION", "none")
os.environ.setdefault("QT_OPENGL", "software")

from PyQt5.QtCore import QPointF, QRectF, Qt, QTimer
from PyQt5.QtGui import QColor, QFont, QIntValidator, QPainter, QPen
from PyQt5.QtWidgets import (
    QApplication,
    QAbstractItemView,
    QFrame,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSizePolicy,
    QTabWidget,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

if __package__ in (None, ""):
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from ground_station.models import (
        SLOTS,
        InventoryStore,
        Waypoint,
        load_waypoint_config,
        waypoint_map,
    )
    from ground_station.ros_bridge import RosBridge
else:
    from .models import (
        SLOTS,
        InventoryStore,
        Waypoint,
        load_waypoint_config,
        waypoint_map,
    )
    from .ros_bridge import RosBridge


class WarehouseMap(QWidget):
    def __init__(self, config: dict[str, Any], waypoints: dict[str, Waypoint]) -> None:
        super().__init__()
        self.config = config
        self.waypoints = waypoints
        self.results: dict[str, dict[str, Any]] = {}
        self.target_slot: str | None = None
        self.last_scan_slot: str | None = None
        self.setMinimumSize(320, 240)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def set_results(self, results: dict[str, dict[str, Any]]) -> None:
        self.results = dict(results)
        self.update()

    def set_target_slot(self, slot: str | None) -> None:
        self.target_slot = slot if slot in SLOTS else None
        self.update()

    def set_last_scan_slot(self, slot: str | None) -> None:
        self.last_scan_slot = slot if slot in SLOTS else None
        self.update()

    def paintEvent(self, _event: Any) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor("#f6f8fb"))

        plot = self._plot_rect()
        self._draw_warehouse(painter, plot)
        self._draw_route(painter, plot)
        self._draw_slots(painter, plot)

    def _plot_rect(self) -> QRectF:
        warehouse = self.config["warehouse"]
        width_m = float(warehouse["width_m"])
        height_m = float(warehouse["height_m"])
        margin = 30.0
        scale = min(
            max(1.0, (self.width() - margin * 2) / width_m),
            max(1.0, (self.height() - margin * 2) / height_m),
        )
        plot_w = width_m * scale
        plot_h = height_m * scale
        return QRectF(
            (self.width() - plot_w) / 2,
            (self.height() - plot_h) / 2,
            plot_w,
            plot_h,
        )

    def _point(self, plot: QRectF, x: float, y: float) -> QPointF:
        warehouse = self.config["warehouse"]
        width_m = float(warehouse["width_m"])
        height_m = float(warehouse["height_m"])
        return QPointF(
            plot.left() + (x / width_m) * plot.width(),
            plot.top() + ((height_m - y) / height_m) * plot.height(),
        )

    def _draw_warehouse(self, painter: QPainter, plot: QRectF) -> None:
        painter.setPen(QPen(QColor("#263238"), 2))
        painter.setBrush(QColor("#ffffff"))
        painter.drawRect(plot)

        painter.setPen(QPen(QColor("#d5dbe3"), 1))
        for x in range(1, 5):
            p1 = self._point(plot, float(x), 0.0)
            p2 = self._point(plot, float(x), 4.0)
            painter.drawLine(p1, p2)
        for y in range(1, 4):
            p1 = self._point(plot, 0.0, float(y))
            p2 = self._point(plot, 5.0, float(y))
            painter.drawLine(p1, p2)

        self._draw_takeoff_and_landing(painter, plot)
        self._draw_racks(painter, plot)

        painter.setPen(QColor("#54606d"))
        painter.setFont(QFont("Sans Serif", 13))
        painter.drawText(plot.adjusted(8, 6, -8, -6), Qt.AlignTop | Qt.AlignLeft, "500cm x 400cm 仓库俯视图")

    def _draw_takeoff_and_landing(self, painter: QPainter, plot: QRectF) -> None:
        takeoff = self.config["warehouse"]["takeoff"]
        landing = self.config["warehouse"]["landing"]
        scale = plot.width() / float(self.config["warehouse"]["width_m"])

        takeoff_center = self._point(plot, takeoff["x"], takeoff["y"])
        side = 0.5 * scale
        takeoff_rect = QRectF(
            takeoff_center.x() - side / 2,
            takeoff_center.y() - side / 2,
            side,
            side,
        )
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor("#111111"))
        painter.drawRect(takeoff_rect)
        painter.setPen(QColor("#111111"))
        painter.drawText(takeoff_rect.adjusted(-24, 8, 24, 28), Qt.AlignHCenter | Qt.AlignBottom, "起飞")

        landing_center = self._point(plot, landing["x"], landing["y"])
        radius = 0.25 * scale
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor("#111111"))
        painter.drawEllipse(landing_center, radius, radius)
        painter.setPen(QColor("#111111"))
        painter.drawText(
            QRectF(landing_center.x() - 36, landing_center.y() + radius + 4, 72, 22),
            Qt.AlignCenter,
            "降落",
        )

    def _draw_racks(self, painter: QPainter, plot: QRectF) -> None:
        painter.setFont(QFont("Sans Serif", 15, QFont.Bold))
        for rack in self.config["warehouse"]["racks"]:
            x = float(rack["x"])
            y_min = float(rack["y_min"])
            y_max = float(rack["y_max"])
            p1 = self._point(plot, x, y_min)
            p2 = self._point(plot, x, y_max)
            painter.setPen(QPen(QColor("#1f1f1f"), 5, Qt.SolidLine, Qt.RoundCap))
            painter.drawLine(p1, p2)
            painter.setPen(Qt.NoPen)
            painter.setBrush(QColor("#e51c23"))
            painter.drawEllipse(p1, 8, 8)
            painter.drawEllipse(p2, 8, 8)

        label_specs = {
            "A": (1.22, 3.32),
            "B": (1.78, 3.32),
            "C": (3.22, 3.32),
            "D": (3.78, 3.32),
        }
        painter.setPen(QColor("#263238"))
        for face, (x, y) in label_specs.items():
            point = self._point(plot, x, y)
            painter.drawText(QRectF(point.x() - 28, point.y() - 14, 56, 28), Qt.AlignCenter, f"{face}面")

    def _draw_route(self, painter: QPainter, plot: QRectF) -> None:
        if self.target_slot not in self.waypoints:
            return

        takeoff = self.config["warehouse"]["takeoff"]
        landing = self.config["warehouse"]["landing"]
        waypoint = self.waypoints[self.target_slot]
        points = self._orthogonal_route_points(plot, takeoff, landing, waypoint)

        painter.setPen(QPen(QColor("#1976d2"), 4, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
        for p1, p2 in zip(points, points[1:]):
            painter.drawLine(p1, p2)
        painter.setPen(QPen(QColor("#ffffff"), 2, Qt.DashLine))
        for p1, p2 in zip(points, points[1:]):
            painter.drawLine(p1, p2)

    def _orthogonal_route_points(
        self,
        plot: QRectF,
        takeoff: dict[str, float],
        landing: dict[str, float],
        waypoint: Waypoint,
    ) -> list[QPointF]:
        route = [
            (float(takeoff["x"]), float(takeoff["y"])),
            (waypoint.x, float(takeoff["y"])),
            (waypoint.x, waypoint.y),
            (float(landing["x"]), waypoint.y),
            (float(landing["x"]), float(landing["y"])),
        ]
        compact: list[tuple[float, float]] = []
        for point in route:
            if not compact or point != compact[-1]:
                compact.append(point)
        return [self._point(plot, x, y) for x, y in compact]

    def _draw_slots(self, painter: QPainter, plot: QRectF) -> None:
        painter.setFont(QFont("Sans Serif", 11, QFont.Bold))
        for slot in SLOTS:
            point = self._slot_display_point(plot, slot)
            result = self.results.get(slot)
            is_target = slot == self.target_slot
            is_last = slot == self.last_scan_slot

            if result and result.get("ok", True):
                fill = QColor("#2e7d32")
                text_color = QColor("#ffffff")
            elif result:
                fill = QColor("#ef6c00")
                text_color = QColor("#ffffff")
            else:
                fill = QColor("#c62828")
                text_color = QColor("#ffffff")

            if is_target:
                painter.setPen(QPen(QColor("#f9a825"), 5))
                painter.setBrush(Qt.NoBrush)
                painter.drawEllipse(point, 25, 25)
            if is_last:
                painter.setPen(QPen(QColor("#00acc1"), 3))
                painter.setBrush(Qt.NoBrush)
                painter.drawEllipse(point, 20, 20)

            painter.setPen(QPen(QColor("#607d8b"), 1))
            painter.setBrush(fill)
            painter.drawEllipse(point, 17, 17)
            painter.setPen(text_color)
            label = str(result["item_id"]) if result and result.get("item_id") is not None else slot
            painter.drawText(QRectF(point.x() - 22, point.y() - 11, 44, 22), Qt.AlignCenter, label)

    def _slot_display_point(self, plot: QRectF, slot: str) -> QPointF:
        face = slot[0]
        number = int(slot[1])
        rack_x = 1.5 if face in ("A", "B") else 3.5
        side = -1 if face in ("A", "C") else 1
        row_offset = 0.23 if number <= 3 else 0.40
        y_values = (1.25, 2.0, 2.75)
        y = y_values[(number - 1) % 3]
        return self._point(plot, rack_x + side * row_offset, y)


class GroundStationWindow(QMainWindow):
    def __init__(self, start_ros: bool = True, demo: bool = False) -> None:
        super().__init__()
        self.config = load_waypoint_config()
        self.waypoints = waypoint_map(self.config)
        self.store = InventoryStore()
        self.bridge = RosBridge()

        self._build_ui()
        self._connect_bridge()
        self._refresh_all()

        self.ui_timer = QTimer(self)
        self.ui_timer.timeout.connect(self._refresh_status_labels)
        self.ui_timer.start(500)

        if start_ros:
            self.bridge.start()
        else:
            self.ros_state_label.setText("ROS 未启动")

        if demo:
            self._load_demo_data()

    def _build_ui(self) -> None:
        self.setWindowTitle("立体货架盘点无人机地面站")
        central = QWidget()
        central.setObjectName("root")
        self.setCentralWidget(central)

        root = QVBoxLayout(central)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        top_bar = QFrame()
        top_bar.setObjectName("topBar")
        top_layout = QHBoxLayout(top_bar)
        top_layout.setContentsMargins(18, 10, 12, 10)
        top_layout.setSpacing(14)

        title = QLabel("立体货架盘点无人机地面站")
        title.setObjectName("titleLabel")
        top_layout.addWidget(title)
        top_layout.addStretch(1)

        self.ros_state_label = self._status_label("ROS 连接中")
        self.phase_label = self._status_label("任务: 待机")
        self.progress_label = self._status_label("进度: 0/24")
        self.target_label = self._status_label("目标: -")
        for widget in (
            self.ros_state_label,
            self.phase_label,
            self.progress_label,
            self.target_label,
        ):
            top_layout.addWidget(widget)

        self.led_indicator = QLabel("LED")
        self.led_indicator.setObjectName("ledIndicator")
        self.led_indicator.setAlignment(Qt.AlignCenter)
        self.led_indicator.setFixedSize(70, 34)
        top_layout.addWidget(self.led_indicator)

        close_button = QPushButton("×")
        close_button.setObjectName("closeButton")
        close_button.setFixedSize(44, 38)
        close_button.clicked.connect(self.close)
        top_layout.addWidget(close_button)
        root.addWidget(top_bar)

        work_area = QHBoxLayout()
        work_area.setContentsMargins(0, 0, 0, 0)
        work_area.setSpacing(0)

        controls_panel = QFrame()
        controls_panel.setObjectName("controlsPanel")
        controls_panel.setMinimumWidth(210)
        controls_panel.setMaximumWidth(260)
        controls_layout = QVBoxLayout(controls_panel)
        controls_layout.setContentsMargins(12, 12, 12, 12)
        controls_layout.setSpacing(10)

        controls_title = QLabel("定点控制")
        controls_title.setObjectName("sectionTitle")
        controls_layout.addWidget(controls_title)

        query_label = QLabel("货物编号")
        self.query_input = QLineEdit()
        self.query_input.setPlaceholderText("1-24")
        self.query_input.setValidator(QIntValidator(1, 24, self))
        self.query_input.returnPressed.connect(self.query_item)

        query_button = QPushButton("查询坐标")
        query_button.clicked.connect(self.query_item)
        self.send_button = QPushButton("发送库位")
        self.send_button.clicked.connect(self.send_selected_waypoint)
        clear_button = QPushButton("清空本轮")
        clear_button.clicked.connect(self.clear_results)

        for button in (query_button, self.send_button, clear_button):
            button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

        self.query_result_label = QLabel("输入编号后查询；收到目标编号时会自动发送 A1 这类库位信息。")
        self.query_result_label.setObjectName("queryResult")
        self.query_result_label.setWordWrap(True)

        controls_layout.addWidget(query_label)
        controls_layout.addWidget(self.query_input)
        controls_layout.addWidget(query_button)
        controls_layout.addWidget(self.send_button)
        controls_layout.addWidget(clear_button)
        controls_layout.addWidget(self.query_result_label)
        controls_layout.addStretch(1)
        work_area.addWidget(controls_panel)

        self.content_tabs = QTabWidget()
        self.content_tabs.setObjectName("contentTabs")
        self.content_tabs.setDocumentMode(True)

        self.map_widget = WarehouseMap(self.config, self.waypoints)
        self.content_tabs.addTab(self.map_widget, "仓库地图")

        results_page = QWidget()
        results_page.setObjectName("resultsPage")
        results_layout = QVBoxLayout(results_page)
        results_layout.setContentsMargins(14, 12, 14, 14)
        results_layout.setSpacing(10)

        table_title = QLabel("实时盘点结果")
        table_title.setObjectName("sectionTitle")
        results_layout.addWidget(table_title)

        self.result_table = QTableWidget(len(SLOTS), 3)
        self.result_table.setHorizontalHeaderLabels(("坐标", "货物编号", "状态"))
        self.result_table.verticalHeader().setVisible(False)
        self.result_table.verticalHeader().setDefaultSectionSize(36)
        self.result_table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.result_table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.result_table.setSelectionMode(QAbstractItemView.SingleSelection)
        self.result_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.result_table.setAlternatingRowColors(True)
        self.result_table.setShowGrid(False)
        results_layout.addWidget(self.result_table, 1)
        self.content_tabs.addTab(results_page, "盘点表格")

        work_area.addWidget(self.content_tabs, 1)
        root.addLayout(work_area, 1)

        self.setStyleSheet(STYLE_SHEET)

    def _status_label(self, text: str) -> QLabel:
        label = QLabel(text)
        label.setObjectName("statusLabel")
        label.setAlignment(Qt.AlignCenter)
        label.setMinimumWidth(118)
        return label

    def _connect_bridge(self) -> None:
        self.bridge.inventory_result.connect(self.handle_inventory_result)
        self.bridge.target_id.connect(self.handle_target_id)
        self.bridge.target_confirmed.connect(self.handle_target_confirmed)
        self.bridge.led_blink.connect(self.blink_led)
        self.bridge.mission_complete.connect(self.handle_mission_complete)
        self.bridge.connection_state.connect(self.ros_state_label.setText)
        self.bridge.error.connect(self.show_error)

    def _refresh_all(self) -> None:
        self._refresh_table()
        self.map_widget.set_results(self.store.results)
        self.map_widget.set_target_slot(self.store.last_target_slot)
        self._refresh_status_labels()

    def _refresh_table(self) -> None:
        for row, slot in enumerate(SLOTS):
            result = self.store.results.get(slot, {})
            values = (
                slot,
                str(result.get("item_id", "-")),
                "已盘点" if result.get("ok", False) else ("异常" if result else "未盘点"),
            )
            for col, value in enumerate(values):
                item = QTableWidgetItem(value)
                item.setTextAlignment(Qt.AlignCenter)
                if result.get("ok", False):
                    item.setBackground(QColor("#e8f5e9"))
                elif result:
                    item.setBackground(QColor("#fff3e0"))
                else:
                    item.setBackground(QColor("#ffebee"))
                self.result_table.setItem(row, col, item)

    def _refresh_status_labels(self) -> None:
        completed = self.store.completed_count()
        self.progress_label.setText(f"进度: {completed}/24")

        status = self.store.mission_status or {}
        phase = str(status.get("phase", "待机"))
        self.phase_label.setText(f"任务: {phase}")

        if self.store.target_id is None:
            self.target_label.setText("目标: -")
        else:
            slot = self.store.item_to_slot().get(self.store.target_id)
            self.target_label.setText(f"目标: {self.store.target_id}" + (f" -> {slot}" if slot else ""))

    def handle_inventory_result(self, data: dict[str, Any]) -> None:
        result = self.store.apply_inventory_result(data)
        self.store.update_mission_status({"phase": "盘点中"})
        self.map_widget.set_last_scan_slot(result["slot"])
        self._refresh_all()
        # LED 由无人机 /inventory_led 单独驱动，不在结果回调里闪。

    def handle_target_id(self, item_id: int) -> None:
        self.store.update_target_id(item_id)
        self.query_input.setText(str(item_id))
        found = self.store.find_item(item_id)
        if found is None:
            self.query_result_label.setText(f"目标编号 {item_id} 尚未在遍历结果中找到")
            self.map_widget.set_target_slot(None)
            self._refresh_status_labels()
            return
        slot, _result = found
        self._select_slot(slot)
        self.publish_waypoint(item_id, slot)

    def handle_target_confirmed(self, data: dict[str, Any]) -> None:
        # 无人机收到我们下发的货位后回报确认（"目标编号=N,货位=XY"），仅更新显示，不再重发。
        item_id = int(data["item_id"])
        slot = str(data["slot"])
        self.store.update_target_id(item_id)
        self.query_input.setText(str(item_id))
        if slot in SLOTS:
            self._select_slot(slot)
        self.query_result_label.setText(f"无人机已确认目标 货物 {item_id} -> 库位 {slot}，开始直飞盘点")
        self._refresh_status_labels()

    def handle_mission_complete(self) -> None:
        self.store.update_mission_status({"phase": "完成"})
        self.query_result_label.setText("无人机已完成本轮盘点任务。")
        self._refresh_status_labels()

    def query_item(self) -> None:
        item_id = self._current_query_id()
        if item_id is None:
            self.query_result_label.setText("请输入 1-24 的货物编号")
            return

        found = self.store.find_item(item_id)
        if found is None:
            self.query_result_label.setText(f"货物 {item_id} 未找到，请先完成遍历盘点")
            self.map_widget.set_target_slot(None)
            return

        slot, _result = found
        self._select_slot(slot)
        waypoint = self.waypoints[slot]
        self.query_result_label.setText(
            f"货物 {item_id} 位于 {slot}，航点 ({waypoint.x:.2f}, {waypoint.y:.2f}, {waypoint.z:.2f})m"
        )

    def send_selected_waypoint(self) -> None:
        item_id = self._current_query_id()
        if item_id is None and self.store.target_id is not None:
            item_id = self.store.target_id
        if item_id is None:
            self.query_result_label.setText("请输入编号或等待无人机回传目标编号")
            return

        found = self.store.find_item(item_id)
        if found is None:
            self.query_result_label.setText(f"货物 {item_id} 未找到，无法发送库位")
            return
        slot, _result = found
        self._select_slot(slot)
        self.publish_waypoint(item_id, slot)

    def publish_waypoint(self, item_id: int, slot: str) -> None:
        waypoint = self.waypoints[slot]
        sent = self.bridge.publish_target_slot(slot)
        self.store.set_last_target_slot(slot)
        self.map_widget.set_target_slot(slot)
        if sent:
            self.query_result_label.setText(
                f"已发布货物 {item_id} 的目标库位: {slot}；地图显示横平竖直路径"
            )
        else:
            self.query_result_label.setText(
                f"已选中 {slot}，本地航点 ({waypoint.x:.2f}, {waypoint.y:.2f}, {waypoint.z:.2f})m；ROS 未就绪"
            )
        self._refresh_status_labels()

    def _select_slot(self, slot: str) -> None:
        self.store.set_last_target_slot(slot)
        self.map_widget.set_target_slot(slot)
        row = SLOTS.index(slot)
        self.result_table.selectRow(row)

    def _current_query_id(self) -> int | None:
        text = self.query_input.text().strip()
        if not text:
            return None
        try:
            item_id = int(text)
        except ValueError:
            return None
        return item_id if 1 <= item_id <= 24 else None

    def clear_results(self) -> None:
        reply = QMessageBox.question(
            self,
            "清空本轮盘点",
            "确认清空当前盘点结果和目标记录？",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if reply != QMessageBox.Yes:
            return
        self.store.clear_results()
        self.map_widget.set_last_scan_slot(None)
        self.query_input.clear()
        self.query_result_label.setText("已清空，等待新的盘点结果。")
        self._refresh_all()

    def blink_led(self) -> None:
        self.led_indicator.setProperty("active", True)
        self.led_indicator.style().unpolish(self.led_indicator)
        self.led_indicator.style().polish(self.led_indicator)
        QTimer.singleShot(1000, self._reset_led)

    def _reset_led(self) -> None:
        self.led_indicator.setProperty("active", False)
        self.led_indicator.style().unpolish(self.led_indicator)
        self.led_indicator.style().polish(self.led_indicator)

    def show_error(self, text: str) -> None:
        self.query_result_label.setText(text)

    def _load_demo_data(self) -> None:
        for index, slot in enumerate(SLOTS[:8], start=1):
            self.store.apply_inventory_result(
                {
                    "item_id": index,
                    "slot": slot,
                    "mode": "traverse",
                    "ok": True,
                    "time_s": index * 5.4,
                }
            )
        self.handle_target_id(3)
        self._refresh_all()

    def keyPressEvent(self, event: Any) -> None:
        if event.key() == Qt.Key_Escape:
            self.close()
            return
        if event.key() == Qt.Key_F11:
            if self.isFullScreen():
                self.showNormal()
            else:
                self.showFullScreen()
            return
        super().keyPressEvent(event)

    def closeEvent(self, event: Any) -> None:
        self.bridge.stop()
        super().closeEvent(event)


STYLE_SHEET = """
#root {
    background: #edf1f5;
    color: #18212b;
    font-family: "Noto Sans CJK SC", "Microsoft YaHei", "Sans Serif";
    font-size: 18px;
}
#topBar {
    background: #1f2a35;
    color: #ffffff;
}
#titleLabel {
    font-size: 26px;
    font-weight: 700;
    color: #ffffff;
}
#statusLabel {
    background: #314252;
    color: #f8fbff;
    border: 1px solid #4b6175;
    border-radius: 4px;
    padding: 6px 8px;
    font-size: 17px;
}
#closeButton {
    background: #b71c1c;
    color: #ffffff;
    border: none;
    border-radius: 4px;
    font-size: 30px;
    font-weight: 700;
}
#closeButton:hover {
    background: #d32f2f;
}
#ledIndicator {
    background: #47525f;
    color: #e8edf2;
    border: 1px solid #6b7785;
    border-radius: 4px;
    font-weight: 700;
}
#ledIndicator[active="true"] {
    background: #26a65b;
    color: #ffffff;
    border: 1px solid #8ee0ad;
}
#contentTabs {
    background: #edf1f5;
}
QTabWidget::pane {
    background: #ffffff;
    border: 1px solid #cfd8e3;
    border-left: none;
    border-right: none;
    border-bottom: none;
}
QTabBar::tab {
    background: #dfe6ee;
    color: #263238;
    padding: 10px 28px;
    min-width: 116px;
    border: 1px solid #c3ccd6;
    border-bottom: none;
    font-weight: 700;
    font-size: 19px;
}
QTabBar::tab:selected {
    background: #ffffff;
    color: #0d47a1;
}
QTabBar::tab:hover {
    background: #eef3f8;
}
#resultsPage {
    background: #ffffff;
}
#sectionTitle {
    font-size: 22px;
    font-weight: 700;
    color: #263238;
}
QTableWidget {
    background: #ffffff;
    alternate-background-color: #f7f9fb;
    border: 1px solid #d5dce5;
    selection-background-color: #d7ecff;
    selection-color: #18212b;
    gridline-color: #e1e7ee;
    font-size: 18px;
}
QHeaderView::section {
    background: #e9eef4;
    color: #1f2a35;
    border: none;
    border-right: 1px solid #d2dae3;
    padding: 8px 4px;
    font-weight: 700;
    font-size: 18px;
}
#controlsPanel {
    background: #ffffff;
    border-right: 1px solid #cfd8e3;
}
QLineEdit {
    background: #ffffff;
    border: 1px solid #aeb9c5;
    border-radius: 4px;
    padding: 8px 10px;
    min-width: 92px;
    font-size: 18px;
}
QPushButton {
    background: #1976d2;
    color: #ffffff;
    border: none;
    border-radius: 4px;
    padding: 9px 14px;
    font-weight: 700;
    font-size: 18px;
}
QPushButton:hover {
    background: #1565c0;
}
QPushButton:pressed {
    background: #0d47a1;
}
#queryResult {
    color: #263238;
    background: #f4f7fa;
    border: 1px solid #d5dce5;
    border-radius: 4px;
    padding: 8px;
}
"""


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Rack inventory drone ground station")
    parser.add_argument("--windowed", action="store_true", help="Run in a normal window")
    parser.add_argument("--no-ros", action="store_true", help="Start UI without ROS bridge")
    parser.add_argument("--demo", action="store_true", help="Load demo inventory data")
    parser.add_argument("--smoke-test", action="store_true", help="Open briefly and exit")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    app = QApplication(sys.argv[:1])
    window = GroundStationWindow(start_ros=not args.no_ros, demo=args.demo)
    app.aboutToQuit.connect(window.bridge.stop)

    if args.smoke_test:
        QTimer.singleShot(250, window.close)

    if args.windowed or args.smoke_test:
        window.resize(1280, 760)
        window.show()
    else:
        window.setWindowFlag(Qt.FramelessWindowHint, True)
        window.showFullScreen()

    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
