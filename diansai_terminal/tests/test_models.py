from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from ground_station.models import (
    SLOTS,
    InventoryStore,
    load_waypoint_config,
    parse_inventory_payload,
    parse_target_payload,
    waypoint_map,
)


class ModelTests(unittest.TestCase):
    def test_waypoint_config_contains_all_slots(self) -> None:
        config = load_waypoint_config()
        waypoints = waypoint_map(config)
        self.assertEqual(set(waypoints), set(SLOTS))
        self.assertEqual(config["warehouse"]["width_m"], 5.0)
        self.assertEqual(config["warehouse"]["height_m"], 4.0)

    def test_parse_inventory_json(self) -> None:
        data = parse_inventory_payload('{"item_id": 7, "slot": "b3", "ok": true, "time_s": 12.5}')
        self.assertEqual(data["item_id"], 7)
        self.assertEqual(data["slot"], "B3")
        self.assertTrue(data["ok"])
        self.assertEqual(data["time_s"], 12.5)

    def test_parse_inventory_csv_fallback(self) -> None:
        data = parse_inventory_payload("12,D6")
        self.assertEqual(data["item_id"], 12)
        self.assertEqual(data["slot"], "D6")

    def test_parse_inventory_drone_format(self) -> None:
        data = parse_inventory_payload("编号=7,货位=B3")
        self.assertEqual(data["item_id"], 7)
        self.assertEqual(data["slot"], "B3")
        self.assertTrue(data["ok"])

    def test_parse_target_id_only(self) -> None:
        data = parse_target_payload("7")
        self.assertEqual(data["item_id"], 7)
        self.assertNotIn("slot", data)

    def test_parse_target_confirmed(self) -> None:
        data = parse_target_payload("目标编号=7,货位=C5")
        self.assertEqual(data["item_id"], 7)
        self.assertEqual(data["slot"], "C5")

    def test_store_persists_and_finds_item(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "state.json"
            store = InventoryStore(path)
            store.apply_inventory_result(
                {
                    "item_id": 5,
                    "slot": "A5",
                    "mode": "traverse",
                    "ok": True,
                    "time_s": 31.0,
                }
            )
            self.assertEqual(store.completed_count(), 1)
            self.assertEqual(store.find_item(5)[0], "A5")

            restored = InventoryStore(path)
            self.assertEqual(restored.find_item(5)[0], "A5")
            self.assertEqual(restored.completed_count(), 1)


if __name__ == "__main__":
    unittest.main()
