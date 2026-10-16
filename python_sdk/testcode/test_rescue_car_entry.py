"""Offline checks for the independent yellow-only rescue entry; no hardware imports."""
import ast
import argparse
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch


SOURCE = Path(__file__).resolve().parents[1] / "rescue_drop_2026.py"


def load_function(name, namespace):
    tree = ast.parse(SOURCE.read_text(encoding="utf8"))
    function = next(node for node in tree.body
                    if isinstance(node, ast.FunctionDef) and node.name == name)
    module = ast.fix_missing_locations(ast.Module(body=[function], type_ignores=[]))
    exec(compile(module, str(SOURCE), "exec"), namespace)
    return namespace[name]


class MandatoryEntryTests(unittest.TestCase):
    def test_cli_requires_no_car_or_free_quota(self):
        namespace = dict(argparse=argparse, FC_SERVER_HOST="127.0.0.1",
                         FC_SERVER_PORT=5654)
        parse_args = load_function("parse_args", namespace)
        with patch.object(sys, "argv", ["rescue", "--confirm-flight",
                                        "--route-direction", "left"]):
            args = parse_args()
        self.assertTrue(args.confirm_flight)
        self.assertEqual(args.route_direction, "left")
        self.assertFalse(hasattr(args, "car_control"))
        for option in ("--car-control", "--manual-control", "--red-count",
                       "--blue-count", "--green-count"):
            with self.subTest(option=option), patch.object(sys, "argv", ["rescue", option]):
                with self.assertRaises(SystemExit):
                    parse_args()

    def test_monitor_mode_does_not_prepare_payload_or_vision(self):
        fc, mission, navi = Mock(), Mock(), Mock()
        fc.connected = True
        fc.state.is_fresh.return_value = True
        fc.state.unlock.value = False
        namespace = dict(parse_args=lambda: SimpleNamespace(confirm_flight=False,
            fc_host="127.0.0.1", fc_server_port=5654), threading=threading,
            time=time, signal=Mock(), FREE_COLORS=("red", "blue", "green"),
            FC_Client=Mock(return_value=fc), STM32Servos=Mock(),
            VisionInterface=Mock(), Navigation=Mock(return_value=navi),
            Mission=Mock(return_value=mission), stop_navigation_ros=Mock(), logger=Mock())
        main = load_function("main", namespace)
        self.assertEqual(main(), 0)
        namespace["STM32Servos"].assert_not_called()
        namespace["VisionInterface"].assert_not_called()
        mission.prepare_navigation.assert_called_once_with()
        mission.monitor_pose.assert_called_once_with()
        mission.run.assert_not_called()
        mission.log_drop_summary.assert_not_called()

    def _flight_fakes(self):
        fc, payload, mission, navi, vision = [Mock() for _ in range(5)]
        fc.connected = True
        fc.state.is_fresh.return_value = True
        fc.state.unlock.value = False
        payload.connected = True
        payload.get_states.return_value = {channel: False for channel in range(1, 5)}
        events = []
        payload.open_all.side_effect = lambda: events.append("open")
        payload.close_all.side_effect = lambda: events.append("close")
        vision.prime.side_effect = lambda: events.append("prime")
        mission.prepare_navigation.side_effect = lambda **_: events.append("prepare")
        mission.validate_payload_ready.side_effect = lambda: (events.append("validate") or True)
        mission.run.side_effect = lambda: events.append("run")
        namespace = dict(parse_args=lambda: SimpleNamespace(confirm_flight=True,
            fc_host="127.0.0.1", fc_server_port=5654, route_direction="right"),
            threading=threading, time=time, signal=Mock(),
            sys=SimpleNamespace(stdin=SimpleNamespace(isatty=lambda: True)),
            FREE_COLORS=("red", "blue", "green"), TOTAL_DROP_COUNT=1,
            PAYLOAD_COUNT=1, MISSION_TIMEOUT=1200,
            VisionInterface=Mock(return_value=vision), ObstaclePlanner2D=Mock(),
            ObstacleInterface=Mock(), require_flight_interfaces=Mock(),
            FC_Client=Mock(return_value=fc), Navigation=Mock(return_value=navi),
            Mission=Mock(return_value=mission), STM32Servos=Mock(return_value=payload),
            connect_stm32_until_ready=Mock(side_effect=lambda p, *_: p.connect()),
            wait_for_operator_command=Mock(side_effect=lambda word, *_: events.append(word)),
            stop_navigation_ros=Mock(), emergency_land=Mock(), logger=Mock())
        main = load_function("main", namespace)
        return main, namespace, (fc, payload, mission, navi, vision), events

    def test_start_command_precedes_navigation_checks_and_flight(self):
        main, namespace, (fc, payload, mission, navi, vision), events = self._flight_fakes()
        self.assertEqual(main(), 0)
        self.assertEqual(events[:8], ["open", "loaded", "close", "prime",
                                      "start_mission", "prepare", "validate", "run"])
        self.assertEqual([call.args[0] for call in
                          namespace["wait_for_operator_command"].call_args_list],
                         ["loaded", "start_mission"])
        vision.set_allocation.assert_called_once_with(
            {"red": 0, "blue": 0, "green": 0})
        mission.prepare_navigation.assert_called_once_with()
        mission.run.assert_called_once_with()
        mission.log_drop_summary.assert_called_once_with()
        self.assertEqual(payload.open_all.call_count, 2)
        fc.close.assert_called_once_with()

    def test_failed_preflight_blocks_takeoff_and_summary(self):
        main, namespace, (_, payload, mission, _, _), events = self._flight_fakes()
        mission.validate_payload_ready.side_effect = lambda: False
        self.assertEqual(main(), 1)
        mission.run.assert_not_called()
        mission.log_drop_summary.assert_not_called()
        self.assertEqual(payload.open_all.call_count, 1)

    def test_missing_fresh_lio_pose_after_start_blocks_takeoff(self):
        main, namespace, (_, payload, mission, _, _), events = self._flight_fakes()
        mission.prepare_navigation.side_effect = RuntimeError("fresh LIO pose unavailable")
        self.assertEqual(main(), 1)
        mission.validate_payload_ready.assert_not_called()
        mission.run.assert_not_called()
        self.assertEqual(payload.open_all.call_count, 1)

    def test_no_start_command_keeps_navigation_and_flight_stopped(self):
        main, namespace, (_, payload, mission, _, _), events = self._flight_fakes()
        def wait_or_fail(word, *_):
            if word == "start_mission":
                raise RuntimeError("terminal closed")
            events.append(word)
        namespace["wait_for_operator_command"].side_effect = wait_or_fail
        self.assertEqual(main(), 1)
        self.assertEqual(events, ["open", "loaded", "close", "prime"])
        namespace["Navigation"].assert_not_called()
        mission.prepare_navigation.assert_not_called()
        mission.run.assert_not_called()
        self.assertEqual(payload.open_all.call_count, 1)

    def test_failed_mission_requests_landing_before_cleanup(self):
        main, namespace, (fc, payload, mission, navi, _), events = self._flight_fakes()
        def fail_in_flight():
            fc.state.unlock.value = True
            events.append("flight_failed")
            raise RuntimeError("mission stopped")
        mission.run.side_effect = fail_in_flight
        navi.stop.side_effect = lambda: events.append("navigation_stop")
        namespace["emergency_land"].side_effect = lambda _: events.append("landing_request")
        mission.stop.side_effect = lambda: events.append("mission_stop")
        self.assertEqual(main(), 1)
        self.assertLess(events.index("navigation_stop"), events.index("landing_request"))
        self.assertLess(events.index("landing_request"), events.index("mission_stop"))
        namespace["emergency_land"].assert_called_once_with(fc)
        mission.log_drop_summary.assert_called_once_with()
        self.assertEqual(payload.open_all.call_count, 1)

    def test_operator_confirmations_ignore_other_input_and_stop(self):
        terminal = Mock()
        terminal.readline.side_effect = ["load\n", "loaded\n", "start_mission\n", ""]
        namespace = dict(threading=threading,
                         sys=SimpleNamespace(stdin=terminal),
                         select=SimpleNamespace(select=lambda *_: ([terminal], [], [])),
                         logger=Mock())
        wait = load_function("wait_for_operator_command", namespace)
        stopped = threading.Event()
        wait("loaded", stopped)
        wait("start_mission", stopped)
        self.assertEqual(terminal.readline.call_count, 3)
        with self.assertRaisesRegex(RuntimeError, "closed"):
            wait("loaded", stopped)
        stopped.set()
        with self.assertRaisesRegex(RuntimeError, "stopped"):
            wait("loaded", stopped)


if __name__ == "__main__":
    unittest.main()
