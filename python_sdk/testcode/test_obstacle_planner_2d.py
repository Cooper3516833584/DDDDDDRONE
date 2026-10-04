"""Offline planner and wiring checks; no ROS, FC, camera or serial imports.

Run: python -m unittest discover -s python_sdk/testcode -p test_obstacle_planner_2d.py
"""

import ast
import importlib.util
import math
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np


SDK = Path(__file__).resolve().parents[1]
SOURCE = SDK / "FlightController/Components/ObstaclePlanner2D.py"
spec = importlib.util.spec_from_file_location("obstacle_planner_offline", SOURCE)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
ObstaclePlanner2D = module.ObstaclePlanner2D
POSE = (0.0, 0.0, 0.0, True)


def pillar(cx=1.5, cy=0.0):
    points = []
    for offset in np.arange(-0.25, 0.251, 0.05):
        points.extend(((cx + offset, cy - 0.25, 0.0),
                       (cx + offset, cy + 0.25, 0.0),
                       (cx - 0.25, cy + offset, 0.0),
                       (cx + 0.25, cy + offset, 0.0)))
    return points


def load_class(relative_path, class_name, names=None, namespace=None):
    source = SDK / relative_path
    tree = ast.parse(source.read_text(encoding="utf-8"))
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == class_name)
    if names is not None:
        cls.body = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in names]
    future = ast.ImportFrom(module="__future__", names=[ast.alias(name="annotations")], level=0)
    tree = ast.fix_missing_locations(ast.Module(body=[future, cls], type_ignores=[]))
    namespace = {} if namespace is None else namespace
    exec(compile(tree, str(source), "exec"), namespace)
    return namespace[class_name]


class PlannerTests(unittest.TestCase):
    def setUp(self):
        self.clock = patch.object(module.time, "monotonic", return_value=100.0).start()
        self.addCleanup(patch.stopall)
        self.planner = ObstaclePlanner2D()
        self.planner.update_body_points([], POSE)

    def seed(self, points_cm):
        p = self.planner
        p._scores.fill(0)
        for point in points_cm:
            x, y = p._cm_to_cell(point)
            p._scores[y, x] = p.SCORE_MAX
        p._revision += 1

    def test_empty_map_exact_goal_and_velocity(self):
        goal = (303.25, -4.75)
        self.assertEqual(self.planner.safe_waypoint((0, 0), goal), goal)
        self.assertEqual(self.planner.safe_velocity((0, 0), (15, -3)), (15, -3))

    def test_pillar_detour_and_path_outside_inflation(self):
        p = self.planner
        p.update_body_points(pillar(), POSE)
        waypoint = p.safe_waypoint((0, 0), (300, 0))
        self.assertIsNotNone(waypoint)
        self.assertGreater(abs(waypoint[1]), 1)
        self.assertLessEqual(math.hypot(*waypoint), 80.0001)
        path = p.plan_path((0, 0), (300, 0))
        self.assertIsNotNone(path)
        blocked, _ = p._fresh_inflated_snapshot()
        for start, goal in zip(path, path[1:]):
            self.assertTrue(p._grid_line_free(blocked, p._cm_to_cell(start), p._cm_to_cell(goal)))

    def test_subgoal_stays_identical_until_reached(self):
        p = self.planner
        p.update_body_points(pillar(), POSE)
        waypoint = p.safe_waypoint((0, 0), (300, 0))
        for current in ((2, 0), (5, 0), (10, 0)):
            self.assertIs(p.safe_waypoint(current, (300, 0)), waypoint)
        self.assertNotEqual(p.safe_waypoint(waypoint, (300, 0)), waypoint)

    def test_new_obstacle_replans_cached_direct_goal(self):
        p = self.planner
        self.assertEqual(p.safe_waypoint((0, 0), (300, 0)), (300, 0))
        p.update_body_points(pillar(), POSE)
        self.assertNotEqual(p.safe_waypoint((0, 0), (300, 0)), (300, 0))

    def test_new_obstacle_blocks_cached_subgoal(self):
        p = self.planner
        p.update_body_points(pillar(), POSE)
        first = p.safe_waypoint((0, 0), (300, 0))
        cell = p._cm_to_cell(first)
        p._scores[cell[1], cell[0]] = p.SCORE_MAX
        p._revision += 1
        self.assertNotEqual(p.safe_waypoint((0, 0), (300, 0)), first)

    def test_blocked_formal_goal_fails_even_with_cached_subgoal(self):
        p = self.planner
        p.update_body_points(pillar(), POSE)
        self.assertIsNotNone(p.safe_waypoint((0, 0), (300, 0)))
        x, y = p._cm_to_cell((300, 0))
        p._scores[y, x] = p.SCORE_MAX
        p._revision += 1
        self.assertIsNone(p.safe_waypoint((0, 0), (300, 0)))
        self.assertIsNone(p.plan_path((0, 0), (300, 0)))

    def test_corner_cutting_rejected_by_astar_and_simplifier(self):
        p = self.planner
        blocked = np.ones_like(p._scores, dtype=bool)
        start, goal = (120, 120), (121, 121)
        blocked[120, 120] = blocked[121, 121] = False
        self.assertFalse(p._grid_line_free(blocked, start, goal))
        self.assertIsNone(p._plan_cells(blocked, start, goal, snap_goal=False))

    def test_yaw_and_translation_conventions(self):
        for yaw, point, expected in (
            (0, (1, 0, 0), (130, 40)),
            (90, (1, 0, 0), (30, -60)),
            (90, (0, 1, 0), (130, 40)),
            (-90, (1, 0, 0), (30, 140)),
        ):
            with self.subTest(yaw=yaw, point=point):
                p = ObstaclePlanner2D()
                p.update_body_points([point], (30, 40, yaw, True))
                x, y = p._cm_to_cell(expected)
                self.assertGreaterEqual(p._scores[y, x], p.OCCUPIED_THRESHOLD)

    def test_slice_range_and_nonfinite_points_filtered(self):
        count = self.planner.update_body_points(
            [(1, 0, 0), (1, 0, 1), (1, 0, -1), (0.1, 0, 0),
             (7, 0, 0), (float("nan"), 0, 0), (1, 0, 0)], POSE)
        self.assertEqual(count, 1)
        state = self.planner.get_debug_state()
        self.assertEqual(state["occupied_cells"], 1)
        self.assertGreater(state["inflated_cells"], state["occupied_cells"])

    def test_ray_free_scores_and_saturation(self):
        p = self.planner
        for _ in range(10):
            p.update_body_points([(1, 0, 0)], POSE)
        hit = p._cm_to_cell((100, 0))
        self.assertEqual(p._scores[hit[1], hit[0]], p.SCORE_MAX)
        for _ in range(12):
            p.update_body_points([(2, 0, 0)], POSE)
        self.assertEqual(p._scores[hit[1], hit[0]], p.SCORE_MIN)

    def test_stale_and_missing_fail_closed_for_all_velocities(self):
        p = self.planner
        for planner in (p, ObstaclePlanner2D()):
            self.clock.return_value = 100.61
            with self.assertRaises(RuntimeError):
                planner.safe_waypoint((0, 0), (100, 0))
            for velocity in ((15, 0), (0.1, 0), (0, 0)):
                with self.assertRaises(RuntimeError):
                    planner.safe_velocity((0, 0), velocity)

    def test_invalid_pose_does_not_refresh_cloud(self):
        self.clock.return_value = 101.0
        self.assertEqual(self.planner.update_body_points([(1, 0, 0)], (0, 0, 0, False)), 0)
        self.assertFalse(self.planner.ready())

    def test_velocity_redirects_preserving_speed(self):
        self.seed([(80, 0)])
        velocity = self.planner.safe_velocity((0, 0), (15, 0))
        self.assertGreater(abs(velocity[1]), 1)
        self.assertAlmostEqual(math.hypot(*velocity), 15)
        p = self.planner
        blocked, _ = p._fresh_inflated_snapshot()
        step = (velocity[0] / 15 * 30, velocity[1] / 15 * 30)
        self.assertTrue(p._grid_line_free(blocked, p._cm_to_cell((0, 0)), p._cm_to_cell(step)))

    def test_no_path_returns_none_or_zero(self):
        p = self.planner
        p._scores.fill(p.SCORE_MAX)
        p._revision += 1
        self.assertIsNone(p.safe_waypoint((0, 0), (300, 0)))
        self.assertEqual(p.safe_velocity((0, 0), (15, 0)), (0, 0))

    def test_out_of_map_and_invalid_inputs(self):
        p = self.planner
        self.assertIsNone(p.safe_waypoint((0, 0), (2000, 0)))
        self.assertIsNone(p.plan_path((2000, 0), (0, 0)))
        self.assertEqual(p.safe_velocity((2000, 0), (15, 0)), (0, 0))
        with self.assertRaises(ValueError):
            p.safe_waypoint((float("nan"), 0), (0, 0))

    def test_reset_clears_evidence_and_readiness(self):
        p = self.planner
        p.update_body_points(pillar(), POSE)
        p.safe_waypoint((0, 0), (300, 0))
        p.reset()
        self.assertFalse(p.ready())
        self.assertFalse(p._scores.any())
        self.assertIsNone(p._cached_subgoal)

    def test_ros_decoding_throttle_and_failure_expiry(self):
        p = self.planner
        getter = Mock(return_value=POSE)
        p.bind_pose_getter(getter)
        decoder = Mock(return_value=np.array([(1, 0, 0)]))
        ros = ModuleType("sensor_msgs_py")
        ros.point_cloud2 = SimpleNamespace(read_points_numpy=decoder)
        with patch.dict(sys.modules, {"sensor_msgs_py": ros}):
            self.clock.return_value = 100.21
            p.on_pointcloud(object())
            self.assertTrue(p.ready())
            self.clock.return_value = 100.3
            p.on_pointcloud(object())
            self.assertEqual(decoder.call_count, 1)
            decoder.side_effect = ValueError("invalid cloud")
            self.clock.return_value = 101.0
            p.on_pointcloud(object())
            self.assertFalse(p.ready())


class WiringTests(unittest.TestCase):
    def test_optional_lio_subscription_uses_existing_node(self):
        class Node:
            def __init__(self, name):
                self.subscriptions = []

            def create_subscription(self, msg_type, topic, callback, qos):
                sub = (msg_type, topic, callback, qos)
                self.subscriptions.append(sub)
                return sub

        fast = ModuleType("fast_lio.msg")
        fast.LioHealth = object()
        sensor = ModuleType("sensor_msgs.msg")
        sensor.Imu = object()
        namespace = {"Node": Node, "Odometry": object(), "pc2": object(),
                     "qos_profile_sensor_data": object(), "_nodes_to_run": [], "logger": Mock()}
        listener = load_class("FlightController/Components/RosNode.py", "LioListenNode",
                              namespace=namespace)
        with patch.dict(sys.modules, {"fast_lio.msg": fast, "sensor_msgs.msg": sensor}):
            normal = listener(Mock(), Mock(), Mock())
            callback = Mock()
            obstacle = listener(Mock(), Mock(), Mock(), pointcloud_callback=callback)
        self.assertIsNone(normal.cloud_sub)
        self.assertEqual(len(normal.subscriptions), 3)
        self.assertEqual(len(obstacle.subscriptions), 4)
        self.assertEqual(obstacle.cloud_sub[1], "/cloud_registered_body")
        self.assertIs(obstacle.cloud_sub[2], callback)
        self.assertIs(obstacle.cloud_sub[3], namespace["qos_profile_sensor_data"])
        self.assertEqual(namespace["_nodes_to_run"], [normal, obstacle])

    def test_navigation_binds_optional_pose_getter(self):
        namespace = {"PID": Mock(), "np": np, "threading": module.threading,
                     "LioPoseProvider": Mock()}
        navigation = load_class("FlightController/Solutions/Navigation.py", "Navigation",
                                names={"__init__"}, namespace=namespace)
        pose = SimpleNamespace(get_pose=Mock(return_value=POSE))
        normal = navigation(fc=Mock(), lio_pose_provider=pose)
        self.assertIsNone(normal.obstacle_planner)
        planner = ObstaclePlanner2D()
        nav = navigation(fc=Mock(), lio_pose_provider=pose, obstacle_planner=planner)
        self.assertIs(nav.obstacle_planner, planner)
        self.assertIs(planner._pose_getter, pose.get_pose)

    def test_navigation_start_forwards_callback_to_original_runner(self):
        for planner in (None, Mock()):
            with self.subTest(planner=planner):
                ros = ModuleType("FlightController.Components.RosNode")
                ros.LioListenNode, ros.RosNodeRunner = Mock(), Mock()
                threads = SimpleNamespace(Thread=Mock(), Event=Mock())
                navigation = load_class("FlightController/Solutions/Navigation.py", "Navigation",
                                        names={"start"}, namespace={"threading": threads, "logger": Mock()})
                nav = navigation()
                nav.running, nav._legacy_mode_warned = False, False
                nav._lio_listener, nav._thread_list = None, []
                nav.lio_pose, nav.obstacle_planner = Mock(), planner
                nav.update_realtime_control = Mock()
                nav._navigation_thread_entry = Mock()
                nav._keep_height_task = Mock()
                nav._velocity_override_watchdog_task = Mock()
                with patch.dict(sys.modules, {ros.__name__: ros}):
                    nav.start()
                ros.LioListenNode.assert_called_once_with(
                    nav.lio_pose.on_odometry, nav.lio_pose.on_health, nav.lio_pose.on_imu,
                    pointcloud_callback=None if planner is None else planner.on_pointcloud)
                ros.RosNodeRunner.assert_called_once_with()
                ros.RosNodeRunner.return_value.add_nodes.return_value.run.assert_called_once_with()

    def test_adapter_preserves_failure_and_return_values(self):
        adapter = load_class("rescue_drop_2026.py", "ObstacleInterface")
        planner = Mock()
        obstacle = adapter(planner)
        self.assertTrue(obstacle.IMPLEMENTED)
        planner.safe_waypoint.return_value = None
        self.assertIsNone(obstacle.safe_waypoint((0, 0), (300, 0)))
        planner.safe_velocity.side_effect = RuntimeError("stale")
        with self.assertRaisesRegex(RuntimeError, "stale"):
            obstacle.safe_velocity((0, 0), (15, 0))

    def test_main_shares_one_planner_and_monitor_has_none(self):
        source = SDK / "rescue_drop_2026.py"
        tree = ast.parse(source.read_text(encoding="utf-8"))
        main = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "main")
        tree = ast.fix_missing_locations(ast.Module(body=[main], type_ignores=[]))
        for flight in (False, True):
            with self.subTest(flight=flight):
                fc, relay, mission = Mock(), Mock(), Mock()
                fc.state.unlock.value = False
                relay.detect_channel_count.return_value = 8
                relay.query_status.return_value = dict.fromkeys(range(1, 9), False)
                planner = Mock()
                namespace = {"parse_args": lambda: SimpleNamespace(confirm_flight=flight,
                             red_count=2, blue_count=1, green_count=1, fc_port="fake", relay_port="fake"),
                             "threading": module.threading, "FREE_COLORS": ("red", "blue", "green"),
                             "validate_allocation": Mock(return_value={"red": 2, "blue": 1, "green": 1}),
                             "VisionInterface": Mock(), "ObstaclePlanner2D": Mock(return_value=planner),
                             "ObstacleInterface": Mock(), "require_flight_interfaces": Mock(),
                             "FC_Controller": Mock(return_value=fc), "Navigation": Mock(),
                             "Mission": Mock(return_value=mission), "LCUSRelay": Mock(return_value=relay),
                             "RELAY_CHANNEL_COUNT": 8, "TOTAL_DROP_COUNT": 5, "MISSION_TIMEOUT": 1200,
                             "wait_for_start_command": Mock(), "time": module.time, "logger": Mock()}
                exec(compile(tree, str(source), "exec"), namespace)
                self.assertEqual(namespace["main"](), 0)
                self.assertEqual(namespace["ObstaclePlanner2D"].call_count, int(flight))
                self.assertIs(namespace["Navigation"].call_args.kwargs["obstacle_planner"],
                              planner if flight else None)
                if flight:
                    namespace["ObstacleInterface"].assert_called_once_with(planner)
                    mission.run.assert_called_once()
                else:
                    mission.monitor_pose.assert_called_once()
                    namespace["LCUSRelay"].assert_not_called()


if __name__ == "__main__":
    unittest.main()
