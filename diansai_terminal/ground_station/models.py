from __future__ import annotations

import json
import re
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any


ROOT_DIR = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG_PATH = ROOT_DIR / "config" / "slot_waypoints.json"
DEFAULT_STATE_PATH = ROOT_DIR / "data" / "inventory_state.json"

FACES = ("A", "B", "C", "D")
SLOT_NUMBERS = tuple(range(1, 7))
SLOTS = tuple(f"{face}{number}" for face in FACES for number in SLOT_NUMBERS)


@dataclass(frozen=True)
class Waypoint:
    slot: str
    x: float
    y: float
    z: float
    yaw: float

    @classmethod
    def from_dict(cls, slot: str, raw: dict[str, Any]) -> "Waypoint":
        return cls(
            slot=slot,
            x=float(raw["x"]),
            y=float(raw["y"]),
            z=float(raw["z"]),
            yaw=float(raw.get("yaw", 0.0)),
        )

    def to_dict(self) -> dict[str, float | str]:
        return {
            "slot": self.slot,
            "x": self.x,
            "y": self.y,
            "z": self.z,
            "yaw": self.yaw,
        }


def load_waypoint_config(path: Path = DEFAULT_CONFIG_PATH) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        config = json.load(f)

    missing = [slot for slot in SLOTS if slot not in config.get("slots", {})]
    if missing:
        raise ValueError(f"Missing waypoint config for slots: {', '.join(missing)}")
    return config


def waypoint_map(config: dict[str, Any]) -> dict[str, Waypoint]:
    return {
        slot: Waypoint.from_dict(slot, config["slots"][slot])
        for slot in SLOTS
    }


# 无人机端 inventory_mission_node 发布的字符串格式：
#   /inventory_result  -> "编号=7,货位=B3"
#   /inventory_target  -> "7"  或  "目标编号=7,货位=C5"
_ID_RE = re.compile(r"编号\s*=\s*(\d+)")
_SLOT_RE = re.compile(r"货位\s*=\s*([A-Da-d][1-6])")


def parse_inventory_payload(payload: str) -> dict[str, Any]:
    payload = payload.strip()
    if not payload:
        raise ValueError("empty inventory payload")

    if payload.startswith("{"):
        data = json.loads(payload)
    elif "编号" in payload or "货位" in payload:
        id_match = _ID_RE.search(payload)
        slot_match = _SLOT_RE.search(payload)
        if not id_match or not slot_match:
            raise ValueError(f"无法解析无人机盘点结果: {payload}")
        data = {"item_id": int(id_match.group(1)), "slot": slot_match.group(1)}
    else:
        parts = [part.strip() for part in payload.split(",")]
        if len(parts) < 2:
            raise ValueError("inventory payload must be JSON, '编号=N,货位=XY' or 'item_id,slot'")
        data = {"item_id": int(parts[0]), "slot": parts[1]}

    if "item_id" not in data or "slot" not in data:
        raise ValueError("inventory payload needs item_id and slot")

    item_id = int(data["item_id"])
    slot = str(data["slot"]).upper()
    if item_id < 1 or item_id > 24:
        raise ValueError(f"item_id out of range: {item_id}")
    if slot not in SLOTS:
        raise ValueError(f"unknown slot: {slot}")

    data["item_id"] = item_id
    data["slot"] = slot
    data["ok"] = bool(data.get("ok", True))
    if "time_s" in data and data["time_s"] is not None:
        data["time_s"] = float(data["time_s"])
    return data


def parse_target_payload(payload: str) -> dict[str, Any]:
    """解析无人机 /inventory_target：

    - 起飞前仅报送编号，内容形如 ``"7"`` -> ``{"item_id": 7}``
    - 地面站下发货位后无人机回报确认，形如 ``"目标编号=7,货位=C5"``
      -> ``{"item_id": 7, "slot": "C5"}``
    """
    payload = payload.strip()
    if not payload:
        raise ValueError("empty target payload")

    id_match = _ID_RE.search(payload)
    if id_match:
        item_id = int(id_match.group(1))
    elif payload.isdigit():
        item_id = int(payload)
    else:
        raise ValueError(f"无法解析无人机目标编号: {payload}")

    if item_id < 1 or item_id > 24:
        raise ValueError(f"目标编号超出范围: {item_id}")

    data: dict[str, Any] = {"item_id": item_id}
    slot_match = _SLOT_RE.search(payload)
    if slot_match:
        data["slot"] = slot_match.group(1).upper()
    return data


class InventoryStore:
    def __init__(self, state_path: Path = DEFAULT_STATE_PATH) -> None:
        self.state_path = state_path
        self.results: dict[str, dict[str, Any]] = {}
        self.target_id: int | None = None
        self.mission_status: dict[str, Any] = {}
        self.last_target_slot: str | None = None
        self.load()

    def load(self) -> None:
        if not self.state_path.exists():
            return
        try:
            with self.state_path.open("r", encoding="utf-8") as f:
                raw = json.load(f)
        except (OSError, json.JSONDecodeError):
            return

        results = raw.get("results", {})
        self.results = {
            slot: value
            for slot, value in results.items()
            if slot in SLOTS and isinstance(value, dict)
        }
        target_id = raw.get("target_id")
        self.target_id = int(target_id) if target_id is not None else None
        self.mission_status = raw.get("mission_status", {})
        self.last_target_slot = raw.get("last_target_slot")

    def save(self) -> None:
        self.state_path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "saved_at": datetime.now().isoformat(timespec="seconds"),
            "results": self.results,
            "target_id": self.target_id,
            "mission_status": self.mission_status,
            "last_target_slot": self.last_target_slot,
        }
        tmp_path = self.state_path.with_suffix(".tmp")
        with tmp_path.open("w", encoding="utf-8") as f:
            json.dump(payload, f, ensure_ascii=False, indent=2)
        tmp_path.replace(self.state_path)

    def apply_inventory_result(self, data: dict[str, Any]) -> dict[str, Any]:
        normalized = parse_inventory_payload(json.dumps(data, ensure_ascii=False))
        slot = normalized["slot"]
        normalized["received_at"] = datetime.now().isoformat(timespec="seconds")
        self.results[slot] = normalized
        self.save()
        return normalized

    def update_target_id(self, item_id: int) -> None:
        self.target_id = int(item_id)
        self.save()

    def update_mission_status(self, status: dict[str, Any]) -> None:
        self.mission_status = status
        self.save()

    def set_last_target_slot(self, slot: str | None) -> None:
        self.last_target_slot = slot
        self.save()

    def clear_results(self) -> None:
        self.results = {}
        self.target_id = None
        self.mission_status = {}
        self.last_target_slot = None
        self.save()

    def item_to_slot(self) -> dict[int, str]:
        mapping: dict[int, str] = {}
        for slot, result in self.results.items():
            if result.get("ok", True) and result.get("item_id") is not None:
                mapping[int(result["item_id"])] = slot
        return mapping

    def find_item(self, item_id: int) -> tuple[str, dict[str, Any]] | None:
        slot = self.item_to_slot().get(int(item_id))
        if slot is None:
            return None
        return slot, self.results[slot]

    def completed_count(self) -> int:
        return sum(
            1
            for value in self.results.values()
            if value.get("ok", True) and value.get("item_id") is not None
        )
