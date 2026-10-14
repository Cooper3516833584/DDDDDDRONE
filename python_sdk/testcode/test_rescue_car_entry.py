"""Execute only the task entry AST with fake interfaces; never import hardware."""
import ast
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


class CarEntryTests(unittest.TestCase):
    def test_network_counts_replace_ledger_before_mission_run(self):
        source = Path(__file__).resolve().parents[1]/"rescue_drop_2026.py"
        tree = ast.parse(source.read_text(encoding="utf8"))
        main = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == "main")
        tree = ast.fix_missing_locations(ast.Module(body=[main], type_ignores=[]))
        fc, relay, mission, navi, node, receiver = [Mock() for _ in range(6)]
        fc.connected = True
        fc.state.is_fresh.return_value = True
        fc.state.unlock.value = False
        relay.detect_channel_count.return_value = 8
        relay.query_status.return_value = dict.fromkeys(range(1, 9), False)
        receiver.wait_for_start.return_value = SimpleNamespace(task_id=42, red=0, blue=3, green=1)
        allocation = dict(red=0, blue=3, green=1)
        ledger = Mock(return_value=allocation)
        def run():
            self.assertEqual(mission.ledger, allocation)
            self.assertEqual(receiver.wait_for_start.call_count, 1)
        mission.run.side_effect = run
        namespace = dict(parse_args=lambda: SimpleNamespace(confirm_flight=True, car_control=True,
            fc_host="127.0.0.1", fc_server_port=5654, relay_port="fake",
            red_count=None, blue_count=None, green_count=None), threading=threading, time=time,
            FREE_COLORS=("red", "blue", "green"), validate_allocation=lambda r,b,g: dict(red=r,blue=b,green=g),
            DropLedger=ledger, VisionInterface=Mock(), ObstaclePlanner2D=Mock(), ObstacleInterface=Mock(),
            require_flight_interfaces=Mock(), FC_Client=Mock(return_value=fc), Navigation=Mock(return_value=navi),
            Mission=Mock(return_value=mission), LCUSRelay=Mock(return_value=relay), RELAY_CHANNEL_COUNT=8,
            TOTAL_DROP_COUNT=5, PAYLOAD_RELAY_CHANNELS=(8,6,4,1,5), PAYLOAD_ANGLES_DEG=(0,72,144,216,288),
            MISSION_TIMEOUT=1200, wait_for_start_command=Mock(), stop_navigation_ros=Mock(),
            emergency_land=Mock(), logger=Mock())
        exec(compile(tree, str(source), "exec"), namespace)
        with patch("fleet_bus.rescue_task.attach_rescue_fleet_node", return_value=(node, receiver)) as attach:
            self.assertEqual(namespace["main"](), 0)
            self.assertTrue(attach.call_args.kwargs["allow_start"])
        ledger.assert_called_once_with(allocation)
        mission.run.assert_called_once()
        namespace["wait_for_start_command"].assert_not_called()
        node.close.assert_called_once()
        fc.close.assert_called_once()

    def test_cli_network_counts_and_flight_confirmation(self):
        source = Path(__file__).resolve().parents[1]/"rescue_drop_2026.py"
        tree = ast.parse(source.read_text(encoding="utf8"))
        parse = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == "parse_args")
        import argparse
        namespace = dict(argparse=argparse, FC_SERVER_HOST="127.0.0.1", FC_SERVER_PORT=5654, FREE_DROP_COUNT=4,
                         validate_allocation=Mock())
        exec(compile(ast.fix_missing_locations(ast.Module(body=[parse], type_ignores=[])), str(source), "exec"), namespace)
        with patch.object(sys, "argv", ["rescue", "--confirm-flight", "--route-direction", "left"]):
            args = namespace["parse_args"]()
            self.assertTrue(args.car_control)
            self.assertEqual(args.route_direction, "left")
        with patch.object(sys, "argv", ["rescue", "--car-control", "--confirm-flight"]):
            args = namespace["parse_args"]()
            self.assertTrue(args.confirm_flight)
            self.assertTrue(args.car_control)
            self.assertIsNone(args.red_count)
        with patch.object(sys, "argv", ["rescue", "--car-control"]):
            self.assertFalse(namespace["parse_args"]().confirm_flight)
        with patch.object(sys, "argv", ["rescue", "--car-control", "--red-count", "2"]):
            with self.assertRaises(SystemExit):
                namespace["parse_args"]()
        namespace["FREE_DROP_COUNT"] = 3
        with patch.object(sys, "argv", ["rescue", "--car-control", "--confirm-flight"]):
            with self.assertRaises(SystemExit):
                namespace["parse_args"]()

    def test_stop_requests_landing_before_receiver_and_vision_cleanup(self):
        source = Path(__file__).resolve().parents[1]/"rescue_drop_2026.py"
        tree = ast.parse(source.read_text(encoding="utf8"))
        main = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == "main")
        tree = ast.fix_missing_locations(ast.Module(body=[main], type_ignores=[]))
        events = []
        fc, relay, mission, navi, node, receiver = [Mock() for _ in range(6)]
        fc.connected = True
        fc.state.is_fresh.return_value = True
        fc.state.unlock.value = False
        relay.detect_channel_count.return_value = 8
        relay.query_status.return_value = dict.fromkeys(range(1, 9), False)
        receiver.wait_for_start.return_value = SimpleNamespace(task_id=42, red=0, blue=3, green=1)
        navi.stop.side_effect = lambda: events.append("navigation_stop")
        node.close.side_effect = lambda: events.append("receiver_close")
        mission.stop.side_effect = lambda: events.append("mission_stop")
        def interrupted():
            fc.state.unlock.value = True
            raise RuntimeError("mission stopped")
        mission.run.side_effect = interrupted
        namespace = dict(parse_args=lambda: SimpleNamespace(confirm_flight=True, car_control=True,
            fc_host="127.0.0.1", fc_server_port=5654, relay_port="fake",
            red_count=None, blue_count=None, green_count=None), threading=threading, time=time,
            FREE_COLORS=("red", "blue", "green"), validate_allocation=lambda r,b,g: dict(red=r,blue=b,green=g),
            DropLedger=Mock(), VisionInterface=Mock(), ObstaclePlanner2D=Mock(), ObstacleInterface=Mock(),
            require_flight_interfaces=Mock(), FC_Client=Mock(return_value=fc), Navigation=Mock(return_value=navi),
            Mission=Mock(return_value=mission), LCUSRelay=Mock(return_value=relay), RELAY_CHANNEL_COUNT=8,
            TOTAL_DROP_COUNT=5, PAYLOAD_RELAY_CHANNELS=(8,6,4,1,5), PAYLOAD_ANGLES_DEG=(0,72,144,216,288),
            MISSION_TIMEOUT=1200, stop_navigation_ros=Mock(),
            emergency_land=Mock(side_effect=lambda _: events.append("landing_request")), logger=Mock())
        exec(compile(tree, str(source), "exec"), namespace)
        with patch("fleet_bus.rescue_task.attach_rescue_fleet_node", return_value=(node, receiver)):
            self.assertEqual(namespace["main"](), 1)
        self.assertEqual(events[:4], ["navigation_stop", "landing_request",
                                     "receiver_close", "mission_stop"])
        namespace["emergency_land"].assert_called_once_with(fc)

    def test_stop_at_takeoff_handoff_blocks_following_flight_commands(self):
        source = Path(__file__).resolve().parents[1]/"rescue_drop_2026.py"
        tree = ast.parse(source.read_text(encoding="utf8"))
        mission_class = next(node for node in tree.body
                             if isinstance(node, ast.ClassDef) and node.name == "Mission")
        run = next(node for node in mission_class.body
                   if isinstance(node, ast.FunctionDef) and node.name == "run")
        tree = ast.fix_missing_locations(ast.Module(body=[run], type_ignores=[]))
        namespace = dict(TAKEOFF_POINT=(0, 0), CRUISE_HEIGHT=100,
                         MissionDeadline=type("MissionDeadline", (RuntimeError,), {}), logger=Mock())
        exec(compile(tree, str(source), "exec"), namespace)
        for checks, should_take_off in (([None, RuntimeError("stopped")], False),
                                        ([None, None, RuntimeError("stopped")], True)):
            navi = Mock()
            mission = SimpleNamespace(_check=Mock(side_effect=checks),
                                      start_vision=Mock(), warmup_vision=Mock(), navi=navi)
            with self.assertRaisesRegex(RuntimeError, "stopped"):
                namespace["run"](mission)
            self.assertEqual(navi.pointing_takeoff.call_count, int(should_take_off))
            navi.wait_for_height.assert_not_called()


if __name__ == "__main__":
    unittest.main()
