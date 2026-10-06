"""Check actual state decoding and cell-based warnings without hardware imports."""

import ast
import copy
from pathlib import Path
import struct
from threading import Event
from types import SimpleNamespace
from typing import List, Optional
import unittest
from unittest.mock import Mock


SOURCE = Path(__file__).resolve().parents[1] / "FlightController/Base.py"
TREE = ast.parse(SOURCE.read_text(encoding="utf-8"))
CLASSES = [node for node in TREE.body if isinstance(node, ast.ClassDef)
           and node.name in ("Byte_Var", "FC_State_Struct")]
PAYLOAD_FORMAT = "<hhhiihhhiiHBBBBB"


def payload(pack_voltage):
    return struct.pack(PAYLOAD_FORMAT, 125, -250, 900, 123, 456,
                       7, 8, 9, 111, 222, round(pack_voltage * 100),
                       3, 1, 16, 0, 4)


class BatteryWarningTests(unittest.TestCase):
    def setUp(self):
        self.clock = SimpleNamespace(now=20.0)
        self.logger = Mock()
        namespace = dict(copy=copy, struct=struct, Event=Event,
                         Optional=Optional, List=List, logger=self.logger,
                         time=SimpleNamespace(monotonic=lambda: self.clock.now,
                                              perf_counter=lambda: self.clock.now))
        module = ast.Module(body=CLASSES, type_ignores=[])
        exec(compile(ast.fix_missing_locations(module), str(SOURCE), "exec"), namespace)
        self.State = namespace["FC_State_Struct"]
        self.state = self.State()

    def test_preserves_original_3_5_volts_per_cell_floor(self):
        self.assertEqual(4, self.state._battery_cells)
        self.assertEqual(3.5, self.state._low_bat_warn_threshold)
        # Compare to the original 3S policy at the same average cell voltage.
        for cell_voltage in (2.5, 3.0, 3.25, 3.49, 3.5, 3.51, 3.6, 3.69, 3.7, 4.2):
            with self.subTest(cell_voltage=cell_voltage):
                state = self.State()
                self.logger.warning.reset_mock()
                state.update_from_bytes(payload(cell_voltage * 4))
                old_3s_voltage = cell_voltage * 3
                expected = 1 < old_3s_voltage < 3.5 * 3
                self.assertEqual(expected, self.logger.warning.called)

    def test_4s_pack_boundaries_and_previous_false_alarms(self):
        for voltage, expected in ((16.8, False), (14.9, False), (14.8, False),
                                  (14.79, False), (14.76, False), (14.0, False),
                                  (13.99, True), (13.0, True), (10.5, True),
                                  (1.01, True), (1.0, False), (0.0, False)):
            with self.subTest(voltage=voltage):
                state = self.State()
                self.logger.warning.reset_mock()
                state.update_from_bytes(payload(voltage))
                self.assertEqual(expected, self.logger.warning.called)

    def test_warning_rate_limit_and_recovery_are_preserved(self):
        self.state.update_from_bytes(payload(13.9))
        self.logger.warning.assert_called_once_with("[FC] Low battery: 13.9V")
        for timestamp in (20.5, 21.0):
            self.clock.now = timestamp
            self.state.update_from_bytes(payload(13.9))
        self.assertEqual(1, self.logger.warning.call_count)
        self.clock.now = 21.01
        self.state.update_from_bytes(payload(13.9))
        self.assertEqual(2, self.logger.warning.call_count)
        self.clock.now = 23.0
        self.state.update_from_bytes(payload(14.5))
        self.assertEqual(2, self.logger.warning.call_count)
        self.state.update_from_bytes(payload(13.9))
        self.assertEqual(3, self.logger.warning.call_count)

    def test_pack_voltage_units_other_fields_and_freshness_are_preserved(self):
        self.assertEqual(35, self.state._fmt_length)
        self.assertFalse(self.state.is_fresh())
        self.state.update_from_bytes(payload(14.76))
        self.assertEqual(14.76, self.state.bat.value)
        self.assertEqual(1.25, self.state.rol.value)
        self.assertEqual(-2.5, self.state.pit.value)
        self.assertEqual(123, self.state.alt_fused.value)
        self.assertEqual(456, self.state.alt.value)
        self.assertEqual(3, self.state.mode.value)
        self.assertTrue(self.state.unlock.value)
        self.assertEqual((16, 0, 4), self.state.command_now)
        self.assertTrue(self.state.update_event.is_set())
        self.assertTrue(self.state.is_fresh())
        self.clock.now += 0.51
        self.assertFalse(self.state.is_fresh())

    def test_invalid_frame_does_not_update_state_or_warn(self):
        self.state.update_from_bytes(payload(14.5))
        self.state.update_event.clear()
        self.clock.now += 1
        for data in (b"", payload(13.9)[:-1], payload(13.9) + b"\0"):
            with self.subTest(length=len(data)):
                with self.assertRaises(ValueError):
                    self.state.update_from_bytes(data)
                self.assertEqual(14.5, self.state.bat.value)
                self.assertEqual(20.0, self.state.last_update_monotonic)
                self.assertFalse(self.state.update_event.is_set())
        self.logger.warning.assert_not_called()

    def test_state_instances_do_not_share_telemetry_or_warning_timers(self):
        other = self.State()
        self.state.update_from_bytes(payload(13.9))
        self.assertEqual(0.0, other.bat.value)
        self.assertEqual(0, other._low_bat_warn_last_time)
        other.update_from_bytes(payload(14.76))
        self.assertEqual(13.9, self.state.bat.value)
        self.assertEqual(14.76, other.bat.value)
        self.assertEqual(1, self.logger.warning.call_count)


if __name__ == "__main__":
    unittest.main()
