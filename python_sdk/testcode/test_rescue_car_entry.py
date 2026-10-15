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
    def test_car_monitor_prepares_navigation_without_servo_commands(self):
        source = Path(__file__).resolve().parents[1]/"rescue_drop_2026.py"
        tree = ast.parse(source.read_text(encoding="utf8"))
        main = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == "main")
        module = ast.fix_missing_locations(ast.Module(body=[main], type_ignores=[]))
        fc, mission, navi, node, receiver = [Mock() for _ in range(5)]
        fc.connected = True
        fc.state.is_fresh.return_value = True
        fc.state.unlock.value = False
        namespace = dict(
            parse_args=lambda: SimpleNamespace(confirm_flight=False, car_control=True,
                fc_host="127.0.0.1", fc_server_port=5654),
            threading=threading, time=time, signal=Mock(),
            FREE_COLORS=("red", "blue", "green"), FC_Client=Mock(return_value=fc),
            DronePayloadServos=Mock(), VisionInterface=Mock(),
            Navigation=Mock(return_value=navi), Mission=Mock(return_value=mission),
            validate_drop_plan=Mock(), stop_navigation_ros=Mock(), logger=Mock())
        exec(compile(module, str(source), "exec"), namespace)
        with patch("fleet_bus.rescue_task.attach_rescue_fleet_node", return_value=(node, receiver)) as attach:
            self.assertEqual(namespace["main"](), 0)
        namespace["DronePayloadServos"].assert_not_called()
        namespace["VisionInterface"].assert_not_called()
        self.assertIsNone(attach.call_args.kwargs["payload"])
        mission.prepare_navigation.assert_called_once_with()
        mission.monitor_pose.assert_called_once_with()

    def test_five_drop_entry_refuses_flight_before_hardware_setup(self):
        source = Path(__file__).resolve().parents[1]/"rescue_drop_2026.py"
        tree = ast.parse(source.read_text(encoding="utf8"))
        main = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == "main")
        module = ast.fix_missing_locations(ast.Module(body=[main], type_ignores=[]))
        namespace = dict(
            parse_args=lambda: SimpleNamespace(confirm_flight=True, car_control=True),
            threading=threading, signal=Mock(), FREE_COLORS=("red", "blue", "green"),
            TOTAL_DROP_COUNT=5, PAYLOAD_COUNT=4, FC_Client=Mock(), VisionInterface=Mock(),
            logger=Mock())
        exec(compile(module, str(source), "exec"), namespace)
        self.assertEqual(namespace["main"](), 1)
        namespace["FC_Client"].assert_not_called()
        namespace["VisionInterface"].assert_not_called()

    def test_network_counts_replace_ledger_before_mission_run(self):
        source = Path(__file__).resolve().parents[1]/"rescue_drop_2026.py"
        tree = ast.parse(source.read_text(encoding="utf8"))
        main = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == "main")
        tree = ast.fix_missing_locations(ast.Module(body=[main], type_ignores=[]))
        fc, payload, mission, navi, node, receiver = [Mock() for _ in range(6)]
        fc.connected = True
        fc.state.is_fresh.return_value = True
        fc.state.unlock.value = False
        vision = Mock()
        fc.connect.side_effect = lambda **kw: vision.prime.assert_called_once()
        receiver.wait_for_start.return_value = SimpleNamespace(task_id=42, red=0, blue=2, green=1)
        receiver.remap_state = "ready"
        allocation = dict(red=0, blue=2, green=1)
        ledger = Mock(return_value=allocation)
        def run():
            self.assertEqual(mission.ledger, allocation)
            self.assertEqual(receiver.wait_for_start.call_count, 1)
        mission.run.side_effect = run
        namespace = dict(parse_args=lambda: SimpleNamespace(confirm_flight=True, car_control=True,
            fc_host="127.0.0.1", fc_server_port=5654, relay_port="fake",
            red_count=None, blue_count=None, green_count=None), threading=threading, time=time, signal=Mock(),
            FREE_COLORS=("red", "blue", "green"), validate_allocation=lambda r,b,g: dict(red=r,blue=b,green=g),
            DropLedger=ledger, VisionInterface=Mock(return_value=vision), ObstaclePlanner2D=Mock(), ObstacleInterface=Mock(),
            require_flight_interfaces=Mock(), FC_Client=Mock(return_value=fc), Navigation=Mock(return_value=navi),
            Mission=Mock(return_value=mission), DronePayloadServos=Mock(return_value=payload),
            validate_drop_plan=Mock(),
            TOTAL_DROP_COUNT=4, PAYLOAD_COUNT=4,
            MISSION_TIMEOUT=1200, wait_for_start_command=Mock(), stop_navigation_ros=Mock(),
            emergency_land=Mock(), logger=Mock())
        exec(compile(tree, str(source), "exec"), namespace)
        with patch("fleet_bus.rescue_task.attach_rescue_fleet_node", return_value=(node, receiver)) as attach:
            attach.side_effect = lambda *a, **kw: (vision.prime.assert_called_once() or (node, receiver))
            self.assertEqual(namespace["main"](), 0)
            self.assertTrue(attach.call_args.kwargs["allow_start"])
        self.assertEqual(payload.open_all.call_count, 2)
        mission.prepare_navigation.assert_called_once_with(remap_ready=True)
        ledger.assert_called_once_with(allocation)
        mission.run.assert_called_once()
        namespace["wait_for_start_command"].assert_not_called()
        node.close.assert_called_once()
        fc.close.assert_called_once()
        vision.prime.assert_called_once()

        # STOP before a matching START re-sends OPEN and never flies; prime stays pre-connect.
        for item in (fc,payload,mission,vision,receiver):
            item.reset_mock()
        receiver.wait_for_start.side_effect=RuntimeError('STOP while waiting')
        with patch('fleet_bus.rescue_task.attach_rescue_fleet_node',return_value=(node,receiver)):
            self.assertEqual(namespace['main'](),1)
        self.assertEqual(payload.open_all.call_count,2)
        mission.run.assert_not_called()
        vision.prime.assert_called_once()

    def test_cli_network_counts_and_flight_confirmation(self):
        source = Path(__file__).resolve().parents[1]/"rescue_drop_2026.py"
        tree = ast.parse(source.read_text(encoding="utf8"))
        parse = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == "parse_args")
        import argparse
        namespace = dict(argparse=argparse, FC_SERVER_HOST="127.0.0.1", FC_SERVER_PORT=5654, FREE_DROP_COUNT=3,
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
        namespace["FREE_DROP_COUNT"] = 4
        with patch.object(sys, "argv", ["rescue", "--car-control", "--confirm-flight"]):
            with self.assertRaises(SystemExit):
                namespace["parse_args"]()

    def test_stop_requests_landing_before_receiver_and_vision_cleanup(self):
        source = Path(__file__).resolve().parents[1]/"rescue_drop_2026.py"
        tree = ast.parse(source.read_text(encoding="utf8"))
        main = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == "main")
        tree = ast.fix_missing_locations(ast.Module(body=[main], type_ignores=[]))
        events = []
        fc, payload, mission, navi, node, receiver = [Mock() for _ in range(6)]
        fc.connected = True
        fc.state.is_fresh.return_value = True
        fc.state.unlock.value = False
        receiver.wait_for_start.return_value = SimpleNamespace(task_id=42, red=0, blue=2, green=1)
        navi.stop.side_effect = lambda: events.append("navigation_stop")
        node.close.side_effect = lambda: events.append("receiver_close")
        mission.stop.side_effect = lambda: events.append("mission_stop")
        def interrupted():
            fc.state.unlock.value = True
            raise RuntimeError("mission stopped")
        mission.run.side_effect = interrupted
        namespace = dict(parse_args=lambda: SimpleNamespace(confirm_flight=True, car_control=True,
            fc_host="127.0.0.1", fc_server_port=5654, relay_port="fake",
            red_count=None, blue_count=None, green_count=None), threading=threading, time=time, signal=Mock(),
            FREE_COLORS=("red", "blue", "green"), validate_allocation=lambda r,b,g: dict(red=r,blue=b,green=g),
            DropLedger=Mock(), VisionInterface=Mock(), ObstaclePlanner2D=Mock(), ObstacleInterface=Mock(),
            require_flight_interfaces=Mock(), FC_Client=Mock(return_value=fc), Navigation=Mock(return_value=navi),
            Mission=Mock(return_value=mission), DronePayloadServos=Mock(return_value=payload), validate_drop_plan=Mock(),
            TOTAL_DROP_COUNT=4, PAYLOAD_COUNT=4,
            MISSION_TIMEOUT=1200, stop_navigation_ros=Mock(),
            emergency_land=Mock(side_effect=lambda _: events.append("landing_request")), logger=Mock())
        exec(compile(tree, str(source), "exec"), namespace)
        with patch("fleet_bus.rescue_task.attach_rescue_fleet_node", return_value=(node, receiver)):
            self.assertEqual(namespace["main"](), 1)
        self.assertEqual(events[:4], ["navigation_stop", "landing_request",
                                     "receiver_close", "mission_stop"])
        namespace["emergency_land"].assert_called_once_with(fc)
        # Landing was requested but lock remained unconfirmed: no midair OPEN.
        self.assertEqual(payload.open_all.call_count,1)

        events.clear()
        for item in (fc, payload, mission, navi, node, receiver):
            item.reset_mock()
        fc.state.unlock.value = False
        payload.open_all.side_effect = lambda: events.append("payload_open")
        def land_and_lock(_):
            events.append("landing_request")
            fc.state.unlock.value = False
        namespace["emergency_land"].side_effect = land_and_lock
        with patch("fleet_bus.rescue_task.attach_rescue_fleet_node", return_value=(node, receiver)):
            self.assertEqual(namespace["main"](), 1)
        self.assertEqual(events[:4], ["payload_open", "navigation_stop",
                                      "landing_request", "payload_open"])
        self.assertEqual(payload.open_all.call_count, 2)

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
