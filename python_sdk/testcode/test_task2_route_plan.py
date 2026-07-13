import json
from pathlib import Path
import tempfile
import unittest

from task2_route_plan import load_inventory_map, route_for_location


class Task2RoutePlanTests(unittest.TestCase):
    def test_all_routes_start_north_and_end_at_landing(self):
        for face in "ABCD":
            for index in range(1, 7):
                route = route_for_location(f"{face}{index}")
                self.assertEqual(route.outbound_local[0], (0.0, 0.0))
                self.assertEqual(route.outbound_local[1], (255.0, 0.0))
                self.assertEqual(route.return_local[-1], (250.0, -350.0))

    def test_opposite_faces_reverse_left_to_right_order(self):
        self.assertEqual(route_for_location("A1").outbound_local[-1][0], 175.0)
        self.assertEqual(route_for_location("A3").outbound_local[-1][0], 75.0)
        self.assertEqual(route_for_location("B1").outbound_local[-1][0], 75.0)
        self.assertEqual(route_for_location("B3").outbound_local[-1][0], 175.0)
        self.assertEqual(route_for_location("A1").face_yaw_deg, 270.0)
        self.assertEqual(route_for_location("B1").face_yaw_deg, 90.0)

    def test_row_height_follows_pdf_layout(self):
        self.assertEqual(route_for_location("D2").scan_height_cm, 140.0)
        self.assertEqual(route_for_location("D5").scan_height_cm, 100.0)

    def test_inventory_map_accepts_task1_record_format(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "inventory.json"
            path.write_text(
                json.dumps([
                    {"position": "A1", "qr_number": 17},
                    {"position": "D6", "qr_number": 4},
                ]),
                encoding="utf-8",
            )
            self.assertEqual(load_inventory_map(path), {17: "A1", 4: "D6"})

    def test_inventory_map_rejects_duplicate_positions(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "inventory.json"
            path.write_text(
                json.dumps([
                    {"position": "A1", "qr_number": 1},
                    {"position": "A1", "qr_number": 2},
                ]),
                encoding="utf-8",
            )
            with self.assertRaises(ValueError):
                load_inventory_map(path)


if __name__ == "__main__":
    unittest.main()
