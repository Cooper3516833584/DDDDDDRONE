"""Fake-only flight sequencing checks; no ROS, network, services or hardware.

    python -m unittest discover -s python_sdk/testcode -p test_lio_waypoint_flight_logic.py
"""

import importlib.util
from pathlib import Path
import sys
from types import ModuleType
import unittest
from unittest.mock import MagicMock, patch


SDK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SDK))
spec = importlib.util.spec_from_file_location("lio_waypoint_flight_test", SDK / "test_lio_waypoint_flight.py")
task = importlib.util.module_from_spec(spec)
spec.loader.exec_module(task)


class FlightLogicTests(unittest.TestCase):
    def setUp(self):
        self.events = []
        self.fc = MagicMock()
        self.fc.connected = True
        self.fc.PROGRAM_MODE = 0
        self.fc.HOLD_POS_MODE = 1
        self.fc.state.is_fresh.return_value = True
        self.fc.state.update_event.wait.return_value = True
        self.fc.state.unlock.value = False
        self.fc.state.mode.value = 0
        self.fc.wait_for_lock.return_value = True
        self.fc.close.side_effect = lambda: self.events.append("close")
        self.fc.land.side_effect = lambda: self.events.append("emergency_land")
        self.navi = MagicMock()
        self.navi.fc = self.fc
        self.navi.lio_pose.mount_reviewed = True
        self.navi.navigation_flag = False
        self.navi.keep_height_flag = False
        self.navi._velocity_override_active = False
        self.navi.traj_running_event.is_set.return_value = False
        self.navi.pose_is_fresh.return_value = True
        self.navi.wait_for_height.return_value = True
        self.navi.wait_for_waypoint.return_value = True
        self.navi.wait_for_yaw.return_value = True
        self.navi.navigation_to_waypoint.return_value = True
        self.navi.start.side_effect = lambda **kwargs: self.events.append("start")
        self.navi.stop.side_effect = lambda: self.events.append("stop")
        self.navi.pointing_takeoff.side_effect = self.takeoff
        self.navi.pointing_landing.side_effect = self.landing

        fc_module = ModuleType("FlightController")
        fc_module.FC_Client = MagicMock(return_value=self.fc)
        ros_module = ModuleType("FlightController.Components.RosNode")
        self.ros_runner = MagicMock()
        self.ros_runner.stop.side_effect = lambda: self.events.append("ros_stop")
        ros_module.RosNodeRunner = MagicMock(return_value=self.ros_runner)
        navigation_module = ModuleType("FlightController.Solutions.Navigation")
        self.navigation_factory = MagicMock(side_effect=self.create_navigation)
        navigation_module.Navigation = self.navigation_factory
        loguru = ModuleType("loguru")
        loguru.logger = MagicMock()
        self.restart = MagicMock(side_effect=self.restart_localization)
        for context in (
            patch.dict(sys.modules, {"FlightController": fc_module,
                                    "FlightController.Components.RosNode": ros_module,
                                    "FlightController.Solutions.Navigation": navigation_module,
                                    "loguru": loguru}),
            patch.object(task.sys, "platform", "linux"),
            patch.object(task, "restart_localization_for_task", self.restart),
            patch.object(task, "require_local_fc_server"),
            patch.object(task.time, "sleep", lambda seconds: None),
        ):
            context.start()
            self.addCleanup(context.stop)

    def create_navigation(self, fc, stop_event):
        self.assertIs(fc, self.fc)
        self.navi.stop_event = stop_event
        return self.navi

    def restart_localization(self, fc, navi, timeout):
        self.assertFalse(fc.state.unlock.value)
        self.assertFalse(navi.navigation_flag)
        self.assertFalse(navi.keep_height_flag)
        self.events.append("restart_calibrate")
        return {"needs_calibration": False}

    def takeoff(self, point, target_height):
        self.events.append("takeoff")
        self.fc.state.unlock.value = True
        self.fc.state.mode.value = self.fc.HOLD_POS_MODE

    def landing(self, point):
        self.events.append("landing")
        self.fc.state.unlock.value = False
        return True

    def run_flight(self):
        return task.main([])

    def test_successful_route_and_height(self):
        self.assertEqual(self.run_flight(), 0)
        self.assertEqual(self.events, ["start", "restart_calibrate", "takeoff", "landing", "stop", "ros_stop", "close"])
        self.navi.pointing_takeoff.assert_called_once_with((0.0, 0.0), target_height=100.0)
        self.assertEqual([call.args[0] for call in self.navi.navigation_to_waypoint.call_args_list],
                         [(100.0, 100.0), (200.0, 0.0), (100.0, -100.0), (0.0, 0.0)])
        self.assertTrue(all(call.kwargs == {"wait": True}
                            for call in self.navi.navigation_to_waypoint.call_args_list))
        self.navi.pointing_landing.assert_called_once_with((0.0, 0.0))
        self.fc.land.assert_not_called()

    def test_help_never_connects(self):
        with self.assertRaises(SystemExit) as result:
            task.main(["--help"])
        self.assertEqual(result.exception.code, 0)
        self.fc.connect.assert_not_called()
        self.restart.assert_not_called()

    def test_missing_server_never_connects_or_restarts(self):
        task.require_local_fc_server.side_effect = RuntimeError("FC_Server 未监听")
        self.assertEqual(self.run_flight(), 1)
        self.fc.connect.assert_not_called()
        self.restart.assert_not_called()
        self.navigation_factory.assert_not_called()

    def test_prearmed_aircraft_is_not_taken_over(self):
        self.fc.state.unlock.value = True
        self.assertEqual(self.run_flight(), 1)
        self.restart.assert_not_called()
        self.navi.pointing_takeoff.assert_not_called()
        self.fc.land.assert_not_called()

    def test_stale_telemetry_refuses_preparation(self):
        self.fc.state.is_fresh.return_value = False
        self.assertEqual(self.run_flight(), 1)
        self.restart.assert_not_called()

    def test_restart_failure_never_takes_off(self):
        self.restart.side_effect = RuntimeError("calibration timeout")
        self.assertEqual(self.run_flight(), 1)
        self.navi.pointing_takeoff.assert_not_called()
        self.fc.land.assert_not_called()

    def test_invalid_pose_never_takes_off(self):
        self.navi.pose_is_fresh.return_value = False
        self.assertEqual(self.run_flight(), 1)
        self.navi.pointing_takeoff.assert_not_called()

    def test_unreviewed_mount_never_starts_navigation_or_restarts(self):
        self.navi.lio_pose.mount_reviewed = False
        self.assertEqual(self.run_flight(), 1)
        self.navi.start.assert_not_called()
        self.restart.assert_not_called()
        self.navi.pointing_takeoff.assert_not_called()
        self.ros_runner.stop.assert_not_called()

    def test_height_failure_stops_and_lands_before_any_waypoint(self):
        self.navi.wait_for_height.return_value = False
        self.assertEqual(self.run_flight(), 1)
        self.navi.navigation_to_waypoint.assert_not_called()
        self.assertLess(self.events.index("stop"), self.events.index("emergency_land"))

    def test_waypoint_failure_does_not_continue_route(self):
        self.navi.navigation_to_waypoint.return_value = False
        self.assertEqual(self.run_flight(), 1)
        self.assertEqual(self.navi.navigation_to_waypoint.call_count, 1)
        self.navi.pointing_landing.assert_not_called()
        self.fc.land.assert_called_once()

    def test_partial_takeoff_failure_requests_landing(self):
        def fail(point, target_height):
            self.takeoff(point, target_height)
            raise RuntimeError("takeoff failed after unlock")
        self.navi.pointing_takeoff.side_effect = fail
        self.assertEqual(self.run_flight(), 1)
        self.fc.land.assert_called_once()

    def test_interrupt_in_flight_stops_and_lands(self):
        self.navi.navigation_to_waypoint.side_effect = KeyboardInterrupt
        self.assertEqual(self.run_flight(), 130)
        self.assertLess(self.events.index("stop"), self.events.index("emergency_land"))
        self.fc.close.assert_called_once()

    def test_failed_emergency_landing_never_force_locks(self):
        self.navi.navigation_to_waypoint.return_value = False
        self.fc.wait_for_lock.return_value = False
        self.assertEqual(self.run_flight(), 1)
        self.assertEqual(self.fc.land.call_count, 2)
        self.fc.lock.assert_not_called()


class ListenerCheckTests(unittest.TestCase):
    def test_expected_loopback_listener(self):
        with patch.object(task.Path, "read_text", return_value="header\n0: 0100007F:1616 00000000:0000 0A"):
            task.require_local_fc_server("127.0.0.1", 5654)

    def test_wildcard_listener(self):
        with patch.object(task.Path, "read_text", return_value="header\n0: 00000000:1616 00000000:0000 0A"):
            task.require_local_fc_server("localhost", 5654)

    def test_other_port_address_or_connected_socket_is_not_server(self):
        for row in ("0: 0100007F:1617 00000000:0000 0A",
                    "0: 0200007F:1616 00000000:0000 0A",
                    "0: 0100007F:1616 00000000:0000 01"):
            with self.subTest(row=row), patch.object(task.Path, "read_text", return_value="header\n" + row):
                with self.assertRaisesRegex(RuntimeError, "FC_Server 未监听"):
                    task.require_local_fc_server("127.0.0.1", 5654)


if __name__ == "__main__":
    unittest.main()
