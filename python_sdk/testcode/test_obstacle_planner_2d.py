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


def background_points():
    """Five distinct endpoints behind the aircraft, clear of the test corridor."""
    return [(-4.0, y, 0.0) for y in (-1.0, -0.5, 0.0, 0.5, 1.0)]


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
        self.planner.update_body_points(background_points(), POSE)

    def seed(self, points_cm):
        p = self.planner
        p._scores.fill(0)
        for point in points_cm:
            x, y = p._cm_to_cell(point)
            p._scores[y, x] = p.SCORE_MAX
        p._revision += 1

    def test_clear_corridor_exact_goal_and_velocity(self):
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
                p.update_body_points([point] + background_points(), (30, 40, yaw, True))
                x, y = p._cm_to_cell(expected)
                self.assertGreaterEqual(p._scores[y, x], p.OCCUPIED_THRESHOLD)

    def test_slice_range_and_nonfinite_points_filtered(self):
        count = self.planner.update_body_points(
            [(1, 0, 0), (1, 0, 1), (1, 0, -1), (0.1, 0, 0),
             (7, 0, 0), (float("nan"), 0, 0), (1, 0, 0)] + background_points(), POSE)
        self.assertEqual(count, 6)
        state = self.planner.get_debug_state()
        self.assertEqual(state["occupied_cells"], 6)
        self.assertGreater(state["inflated_cells"], state["occupied_cells"])

    def test_ray_free_scores_and_saturation(self):
        p = self.planner
        for _ in range(10):
            p.update_body_points([(1, 0, 0)] + background_points(), POSE)
        hit = p._cm_to_cell((100, 0))
        self.assertEqual(p._scores[hit[1], hit[0]], p.SCORE_MAX)
        for _ in range(12):
            p.update_body_points([(2, 0, 0)] + background_points(), POSE)
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

    def test_center_window_keeps_three_exact_anchors_without_obstacles(self):
        points = [(200, -105), (130, -105), (130, -35), (200, -35)]
        plan = self.planner.plan_route_window((200, -175), points)
        self.assertEqual(plan.anchors, tuple(enumerate(points[:3])))
        self.assertEqual(plan.waypoints, tuple(points[:3]))
        self.assertEqual(plan.skipped_offsets, ())
        self.assertTrue(self.planner.path_is_free([(200, -175)] + list(plan.waypoints)))

    def test_center_window_skips_blocked_corner_and_keeps_searching(self):
        points = [(200, -105), (130, -105), (130, -35), (200, -35),
                  (200, 35), (130, 35), (130, 105)]
        self.seed([(160, -70), (170, -70)])
        plan = self.planner.plan_route_window((200, -175), points)
        self.assertIsNotNone(plan)
        self.assertEqual(plan.skipped_offsets, (0, 1, 2, 3))
        self.assertEqual(plan.anchors[0], (4, points[4]))
        self.assertTrue(self.planner.path_is_free([(200, -175)] + list(plan.waypoints)))

    def test_path_checks_segments_and_only_remaining_part(self):
        self.seed([(100, 0)])
        self.assertFalse(self.planner.path_is_free([(0, 0), (200, 0)]))
        self.assertTrue(self.planner.path_is_free([(0, 100), (200, 100)]))
        self.assertTrue(self.planner.path_is_free([(200, 100), (300, 100)]))
        self.seed([(50, 100)])
        self.assertTrue(self.planner.path_is_free([(200, 100), (300, 100)]))

    def test_real_spline_can_leave_a_clear_polyline(self):
        source = SDK / "FlightController/Solutions/SmoothTrajectory.py"
        spec = importlib.util.spec_from_file_location("smooth_trajectory_offline", source)
        smooth = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, {spec.name: smooth}):
            spec.loader.exec_module(smooth)
        polyline = [(0, 0), (100, 0), (100, 100)]
        trajectory = smooth.SplineTrajectoryGenerator(
            polyline, 150, smooth.SplineTrajectoryConfig(navi_speed=15)).generate_traj_list()
        self.seed([(60, -60)])
        self.assertTrue(self.planner.path_is_free(polyline))
        self.assertFalse(self.planner.path_is_free([(point[0], point[1])
                                                    for point in trajectory]))

    def test_mandatory_43cm_admission_does_not_change_50cm_motion(self):
        self.seed([(100, 0)])
        self.assertFalse(self.planner.mandatory_drop_pose_is_clear((142, 0)))
        self.assertTrue(self.planner.mandatory_drop_pose_is_clear((145, 0)))
        self.assertIsNone(self.planner.safe_waypoint((0, 0), (145, 0)))
        self.clock.return_value = 100.61
        with self.assertRaisesRegex(RuntimeError, "missing or stale"):
            self.planner.mandatory_drop_pose_is_clear((200, 0))

    def test_reset_clears_evidence_and_readiness(self):
        p = self.planner
        p.update_body_points(pillar(), POSE)
        p.safe_waypoint((0, 0), (300, 0))
        p.reset()
        self.assertFalse(p.ready())
        self.assertFalse(p._scores.any())
        self.assertIsNone(p._cached_subgoal)
        state = p.get_debug_state()
        self.assertEqual(state["last_endpoint_cells"], 0)
        self.assertEqual(state["invalid_cloud_streak"], 0)

    def test_ros_decoding_throttle_and_failure_expiry(self):
        p = self.planner
        getter = Mock(return_value=POSE)
        p.bind_pose_getter(getter)
        decoder = Mock(return_value=np.array(background_points()))
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

    def test_empty_cloud_is_not_a_valid_map(self):
        p = ObstaclePlanner2D()
        self.assertEqual(p.update_body_points([], POSE), 0)
        state = p.get_debug_state()
        self.assertFalse(state["ready"])
        self.assertEqual(state["last_endpoint_cells"], 0)
        self.assertEqual(state["invalid_cloud_streak"], 1)
        self.assertEqual(state["min_valid_endpoint_cells"], 5)
        with self.assertRaises(RuntimeError):
            p.safe_waypoint((0, 0), (100, 0))
        with self.assertRaises(RuntimeError):
            p.safe_velocity((0, 0), (15, 0))

    def test_filtered_cloud_does_not_refresh_freshness(self):
        for points in (
            [(1, i * 0.2, 1) for i in range(6)],
            [(7, i * 0.2, 0) for i in range(6)],
            [(float("nan"), i * 0.2, 0) for i in range(6)],
            background_points(),
        ):
            with self.subTest(points=points):
                p = ObstaclePlanner2D()
                pose = (2000, 2000, 0, True) if points == background_points() else POSE
                self.assertEqual(p.update_body_points(points, pose), 0)
                self.assertFalse(p.ready())
                self.assertEqual(p._last_cloud_at, 0)
                self.assertEqual(p.get_debug_state()["last_endpoint_cells"], 0)

    def test_endpoint_threshold_uses_unique_cells(self):
        p = ObstaclePlanner2D()
        points = background_points()[:p.MIN_VALID_ENDPOINT_CELLS - 1]
        self.assertEqual(p.update_body_points(points * 50, POSE), 4)
        self.assertEqual(p._last_cloud_at, 0)
        self.assertEqual(p.get_debug_state()["invalid_cloud_streak"], 1)
        self.assertEqual(p._revision, 0)
        self.assertFalse(p._scores.any())
        self.assertEqual(p.update_body_points(background_points(), POSE), 5)
        self.assertTrue(p.ready())
        self.assertEqual(p.get_debug_state()["invalid_cloud_streak"], 0)

    def test_invalid_frames_preserve_map_but_do_not_extend_ttl(self):
        p = ObstaclePlanner2D()
        p.update_body_points(pillar(), POSE)
        scores, revision = p._scores.copy(), p._revision
        for now in (100.20, 100.40):
            self.clock.return_value = now
            p.update_body_points([], POSE)
            self.assertTrue(p.ready())
            self.assertEqual(p._last_cloud_at, 100.0)
            self.assertEqual(p._last_processed_at, now)
            self.assertEqual(p._revision, revision)
            np.testing.assert_array_equal(p._scores, scores)
            self.assertIsNotNone(p.safe_waypoint((0, 0), (300, 0)))
        self.assertEqual(p.get_debug_state()["invalid_cloud_streak"], 2)
        self.clock.return_value = 100.61
        self.assertFalse(p.ready())
        with self.assertRaises(RuntimeError):
            p.safe_waypoint((0, 0), (300, 0))
        with self.assertRaises(RuntimeError):
            p.safe_velocity((0, 0), (15, 0))

    def test_invalid_cloud_attempts_are_still_throttled(self):
        for decode_error in (False, True):
            with self.subTest(decode_error=decode_error):
                p = ObstaclePlanner2D(pose_getter=lambda: POSE)
                decoder = Mock(return_value=np.empty((0, 3)),
                               side_effect=ValueError("decode failure") if decode_error else None)
                ros = ModuleType("sensor_msgs_py")
                ros.point_cloud2 = SimpleNamespace(read_points_numpy=decoder)
                with patch.dict(sys.modules, {"sensor_msgs_py": ros}):
                    self.clock.return_value = 100.0
                    p.on_pointcloud(object())
                    self.assertEqual(p._last_processed_at, 100.0)
                    self.clock.return_value = 100.1
                    p.on_pointcloud(object())
                    self.assertEqual(decoder.call_count, 1)
                    self.clock.return_value = 100.21
                    p.on_pointcloud(object())
                    self.assertEqual(decoder.call_count, 2)
                    self.assertFalse(p.ready())


class MissionVelocityTests(unittest.TestCase):
    def setUp(self):
        source = SDK / "rescue_drop_2026.py"
        tree = ast.parse(source.read_text(encoding="utf-8"))
        helpers = [n for n in tree.body if isinstance(n, ast.FunctionDef)
                   and n.name in {"body_to_world_velocity", "world_to_body_velocity",
                                  "_is_new_visual_frame", "_is_consecutive_visual_frame"}]
        helpers += [n for n in tree.body if isinstance(n, ast.ClassDef)
                    and n.name == "_VisualJumpGuard"]
        future = ast.ImportFrom(module="__future__", names=[ast.alias(name="annotations")], level=0)
        tree = ast.fix_missing_locations(ast.Module(body=[future] + helpers, type_ignores=[]))
        self.namespace = {"math": math, "VISUAL_APPROACH_SPEED": 15.0,
                          "VISUAL_NEAR_CENTER_RADIUS_PX": 100.0,
                          "VISUAL_JUMP_THRESHOLD_PX": 45.0,
                          "VISUAL_JUMP_CONFIRM_MAX_GAP_S": 0.65,
                          "LOW_CALIBRATION_CENTER_FRAMES": 2}
        exec(compile(tree, str(source), "exec"), self.namespace)
        cls = load_class("rescue_drop_2026.py", "Mission",
                         names={"_move_translation_only", "_move_toward", "_position",
                                "_calibrate_low", "_set_height", "_navigate_center_exit"},
                         namespace=self.namespace)
        self.mission = cls()
        self.mission.navi = Mock(current_x=30.0, current_y=40.0, current_yaw=90.0)
        self.mission.obstacle = Mock()
        self.observation = SimpleNamespace(offset_x_px=100.0, offset_y_px=0.0)

    def test_yaw_directions_and_round_trip(self):
        to_world = self.namespace["body_to_world_velocity"]
        to_body = self.namespace["world_to_body_velocity"]
        for yaw, expected in ((0, (15, 0)), (90, (0, -15)), (-90, (0, 15))):
            np.testing.assert_allclose(to_world(15, 0, yaw), expected, atol=1e-12)
        for yaw in (0, 37, 90, -90, 179):
            for vector in ((15, 0), (0, 15), (10, -7)):
                with self.subTest(yaw=yaw, vector=vector):
                    np.testing.assert_allclose(to_body(*to_world(*vector, yaw), yaw), vector, atol=1e-12)

    def test_protected_move_passes_world_to_planner_and_body_to_navigation(self):
        mission = self.mission
        mission.obstacle.safe_velocity.return_value = (15, 0)
        mission._move_toward(self.observation, protected=True)
        position, velocity = mission.obstacle.safe_velocity.call_args.args
        self.assertEqual(position, (30, 40))
        np.testing.assert_allclose(velocity, (0, -15), atol=1e-12)
        mission.navi.update_realtime_control.assert_called_once_with(
            vel_x=0, vel_y=15, yaw=0)
        self.assertFalse(mission.navi.navigation_flag)
        mission.navi.move_by_direction.assert_not_called()
        mission.navi.stop_move.assert_not_called()

    def test_unprotected_move_keeps_body_direction(self):
        self.mission._move_toward(self.observation, protected=False)
        self.mission.obstacle.safe_velocity.assert_not_called()
        self.mission.navi.move_by_direction.assert_called_once_with(speed=15.0, direction_deg=0.0)

    def test_protected_zero_velocity_clears_yaw_and_invalid_results_raise(self):
        self.mission.obstacle.safe_velocity.return_value = (0, 0)
        self.mission._move_toward(self.observation, protected=True)
        self.mission.navi.update_realtime_control.assert_called_once_with(
            vel_x=0, vel_y=0, yaw=0)
        self.mission.navi.stop_move.assert_not_called()
        self.mission.navi.move_by_direction.assert_not_called()
        for result in (None, (float("nan"), 0), (0, float("inf"))):
            with self.subTest(result=result):
                self.mission.obstacle.safe_velocity.return_value = result
                with self.assertRaisesRegex(RuntimeError, "velocity invalid"):
                    self.mission._move_toward(self.observation, protected=True)
        self.mission.navi.move_by_direction.assert_not_called()

    def test_planner_failure_propagates_without_movement(self):
        self.mission.obstacle.safe_velocity.side_effect = RuntimeError("stale")
        with self.assertRaisesRegex(RuntimeError, "stale"):
            self.mission._move_toward(self.observation, protected=True)
        self.mission.navi.move_by_direction.assert_not_called()

    def test_mandatory_height_keeps_yaw_zero_then_exit_restores_navigation(self):
        mission = self.mission
        self.namespace["logger"] = Mock()
        mission._check = Mock()
        mission.navi.wait_for_height.return_value = True
        mission._set_height(100.0, translation_only=True)
        mission.navi.update_realtime_control.assert_called_once_with(
            vel_x=0, vel_y=0, yaw=0)
        mission.navi.stop_move.assert_not_called()
        mission.navi.set_height.assert_called_once_with(100.0)

        mission.route_2 = [(200.0, 160.0)]
        mission._navigate_leg = Mock()
        mission._navigate_center_exit()
        mission.navi.stop_move.assert_called_once()
        mission._navigate_leg.assert_called_once_with((200.0, 160.0), protected=True)

    def test_consecutive_mandatory_calibration_timeouts_never_restore_yaw_pid(self):
        clock = SimpleNamespace(now=0.0)

        def wait(period):
            clock.now += period

        self.namespace.update({
            "time": SimpleNamespace(monotonic=lambda: clock.now),
            "logger": Mock(), "MANDATORY_COLOR": "yellow",
            "MANDATORY_DROP_HEIGHT": 100.0,
            "MANDATORY_TARGET_GROUND_HEIGHT_CM": 30.0,
            "payload_target_offset_px": Mock(return_value=(0.0, 0.0)),
            "LOW_CALIBRATION_TIMEOUT": 0.3,
            "LOW_CALIBRATION_MAX_FRAME_AGE_S": 0.45,
            "LOW_CALIBRATION_HOLD_AFTER_LOSS_S": 0.1,
            "LOW_CALIBRATION_VELOCITY_MAX_GAP_S": 1.0,
            "LOW_CALIBRATION_SETTLE_MAX_GAP_S": 0.65,
            "LOW_CALIBRATION_LOG_PERIOD_S": 0.5,
            "VISUAL_PERIOD": 0.1,
        })
        mission = self.mission
        mission.vision = SimpleNamespace(frame_size=(640, 480))
        mission._check = Mock()
        mission._observation = Mock(return_value=None)
        mission.stop_event = SimpleNamespace(wait=wait)
        target = SimpleNamespace(color="yellow", target_id="yellow-3")
        self.assertFalse(mission._calibrate_low(target, protected=True, drop_number=2))
        self.assertFalse(mission._calibrate_low(target, protected=True, drop_number=3))
        self.assertFalse(mission.navi.navigation_flag)
        mission.navi.stop_move.assert_not_called()
        self.assertGreaterEqual(mission.navi.update_realtime_control.call_count, 4)
        for call in mission.navi.update_realtime_control.call_args_list:
            self.assertEqual(call.kwargs, {"vel_x": 0, "vel_y": 0, "yaw": 0})

    def test_yellow_identity_rebind_requires_two_nearby_frames_and_clear_pose(self):
        clock = SimpleNamespace(now=0.0)
        self.namespace.update({
            "time": SimpleNamespace(monotonic=lambda: clock.now),
            "logger": Mock(), "MANDATORY_COLOR": "yellow",
            "MANDATORY_DROP_HEIGHT": 80.0,
            "MANDATORY_TARGET_GROUND_HEIGHT_CM": 30.0,
            "payload_target_offset_px": Mock(return_value=(10.0, 5.0)),
            "low_calibration_command": lambda error, velocity: (0.0, error),
            "LOW_CALIBRATION_TIMEOUT": 2.0,
            "LOW_CALIBRATION_MAX_FRAME_AGE_S": 0.45,
            "LOW_CALIBRATION_HOLD_AFTER_LOSS_S": 1.0,
            "LOW_CALIBRATION_VELOCITY_MAX_GAP_S": 1.0,
            "LOW_CALIBRATION_SETTLE_MAX_GAP_S": 0.65,
            "LOW_CALIBRATION_THRESHOLD_PX": 20.0,
            "LOW_CALIBRATION_MAX_PIXEL_SPEED": 120.0,
            "LOW_CALIBRATION_LOG_PERIOD_S": 0.5,
            "MANDATORY_REBIND_MAX_DISTANCE_CM": 30.0,
            "MANDATORY_REBIND_MIN_FRAMES": 2,
            "VISUAL_PERIOD": 0.1,
        })
        mission = self.mission
        mission.vision = SimpleNamespace(frame_size=(640, 480))
        mission._check = Mock()
        mission._move_translation_only = Mock()
        mission._estimate_target_world_xy = Mock(return_value=(10.0, 0.0))
        mission._desired_aircraft_drop_pose = Mock(return_value=(9.0, 0.0))
        mission.obstacle.mandatory_drop_pose_is_clear.return_value = True
        mission.stop_event = SimpleNamespace(
            wait=lambda period: setattr(clock, "now", round(clock.now + period, 2)))
        old = SimpleNamespace(color="yellow", target_id="yellow-3")
        mission.target = old

        def observation(color=None, target_id=None):
            if target_id == "yellow-3":
                return None
            captured_at = max((at for at in (0.1, 0.5, 0.9)
                               if at <= clock.now), default=None)
            if captured_at is None:
                return None
            return SimpleNamespace(color="yellow", target_id="yellow-4",
                                   offset_x_px=10.0, offset_y_px=5.0,
                                   captured_at=captured_at)

        mission._observation = observation
        self.assertTrue(mission._calibrate_low(
            old, protected=True, drop_number=2, target_world=(0.0, 0.0)))
        self.assertEqual(mission.target.target_id, "yellow-4")
        mission.obstacle.mandatory_drop_pose_is_clear.assert_called_once_with((9.0, 0.0))
        self.assertGreaterEqual(clock.now, 0.9)

        clock.now = 0.0
        mission.target = old
        mission.obstacle.mandatory_drop_pose_is_clear.reset_mock()
        mission._estimate_target_world_xy.return_value = (50.0, 0.0)
        self.namespace["LOW_CALIBRATION_TIMEOUT"] = 0.6
        self.assertFalse(mission._calibrate_low(
            old, protected=True, drop_number=2, target_world=(0.0, 0.0)))
        self.assertIs(mission.target, old)
        mission.obstacle.mandatory_drop_pose_is_clear.assert_not_called()

        clock.now = 0.0
        mission._estimate_target_world_xy.return_value = (10.0, 0.0)
        self.namespace["LOW_CALIBRATION_TIMEOUT"] = 1.3

        def interrupted_observation(color=None, target_id=None):
            if target_id == "yellow-3":
                return None
            captured_at = 1.0 if clock.now >= 1.0 else 0.1
            return SimpleNamespace(color="yellow", target_id="yellow-4",
                                   offset_x_px=10.0, offset_y_px=5.0,
                                   captured_at=captured_at)

        mission._observation = interrupted_observation
        self.assertFalse(mission._calibrate_low(
            old, protected=True, drop_number=2, target_world=(0.0, 0.0)))
        self.assertIs(mission.target, old)
        mission.obstacle.mandatory_drop_pose_is_clear.assert_not_called()

        clock.now = 0.0
        mission._observation = observation
        mission.obstacle.mandatory_drop_pose_is_clear.return_value = False
        with self.assertRaisesRegex(RuntimeError, "drop pose is blocked"):
            mission._calibrate_low(
                old, protected=True, drop_number=2, target_world=(0.0, 0.0))
        self.assertIs(mission.target, old)


class VisualApproachTests(unittest.TestCase):
    def setUp(self):
        source = SDK / "rescue_drop_2026.py"
        tree = ast.parse(source.read_text(encoding="utf-8"))
        names = {"VISUAL_CENTER_THRESHOLD_PX", "VISUAL_APPROACH_SPEED",
                 "VISUAL_APPROACH_MIN_SPEED", "VISUAL_APPROACH_SPEED_PER_PX",
                 "VISUAL_APPROACH_LOOKAHEAD_S", "VISUAL_APPROACH_CENTER_FRAMES",
                 "VISUAL_APPROACH_SETTLE_MAX_GAP_S", "VISUAL_APPROACH_LOG_PERIOD_S",
                 "VISUAL_PERIOD", "TARGET_LOSS_WAIT", "VISUAL_NEAR_CENTER_RADIUS_PX",
                 "VISUAL_JUMP_THRESHOLD_PX", "VISUAL_JUMP_CONFIRM_MAX_GAP_S",
                 "LOW_CALIBRATION_MAX_PIXEL_SPEED"}
        selected = [n for n in tree.body if isinstance(n, ast.Assign)
                    and any(isinstance(t, ast.Name) and t.id in names for t in n.targets)]
        selected += [n for n in tree.body if isinstance(n, ast.FunctionDef)
                     and n.name in {"visual_approach_command", "_is_new_visual_frame",
                                    "_is_consecutive_visual_frame"}]
        selected += [n for n in tree.body if isinstance(n, ast.ClassDef)
                     and n.name == "_VisualJumpGuard"]
        future = ast.ImportFrom(module="__future__", names=[ast.alias(name="annotations")], level=0)
        module = ast.fix_missing_locations(ast.Module(body=[future] + selected, type_ignores=[]))
        self.namespace = {"math": math, "logger": Mock()}
        exec(compile(module, str(source), "exec"), self.namespace)

    def test_approach_slows_near_center_and_brakes_ahead(self):
        command = self.namespace["visual_approach_command"]
        self.assertEqual(command((300.0, 0.0), (0.0, 0.0))[0], 12.0)
        self.assertEqual(command((50.0, 0.0), (0.0, 0.0))[0], 2.0)
        self.assertEqual(command((50.0, 0.0), (-80.0, 0.0))[0], 0.0)

    def test_single_frame_center_jump_requires_next_frame_confirmation(self):
        guard = self.namespace["_VisualJumpGuard"]()

        def frame(seq, offset):
            return SimpleNamespace(target_id="red-1", frame_seq=seq,
                                   offset_x_px=offset, offset_y_px=0.0,
                                   captured_at=seq * 0.4)

        self.assertTrue(guard.accept(frame(1, 80.0))[0])
        accepted, previous, reason = guard.accept(frame(2, 5.0))
        self.assertFalse(accepted)
        self.assertEqual(previous.frame_seq, 1)
        self.assertEqual(reason, "jump-pending")
        self.assertEqual(guard.accept(frame(3, 78.0))[2], "jump-reverted")
        self.assertFalse(guard.accept(frame(4, 5.0))[0])
        self.assertEqual(guard.accept(frame(5, 7.0))[2], "jump-confirmed")

    def test_approach_waits_for_new_centered_frames_before_descent(self):
        clock = SimpleNamespace(now=0.0)
        self.namespace["time"] = SimpleNamespace(monotonic=lambda: clock.now)
        cls = load_class("rescue_drop_2026.py", "Mission",
                         names={"_approach_target", "_position"},
                         namespace=self.namespace)
        mission = cls()
        mission.navi = Mock(current_x=0.0, current_y=0.0, current_yaw=0.0)
        mission._check = Mock()
        mission._move_translation_only = Mock()
        mission._move_toward = Mock()
        mission.stop_event = SimpleNamespace(
            wait=lambda period: setattr(clock, "now", round(clock.now + period, 2)))
        frames = [(0.0, 100.0), (0.4, 25.0), (0.8, 25.0),
                  (1.2, 25.0), (1.6, 25.0)]

        def observation(target_id):
            frame_seq, (captured_at, offset) = max(
                ((index + 1, frame) for index, frame in enumerate(frames)
                 if frame[0] <= clock.now), default=(1, frames[0]))
            return SimpleNamespace(target_id=target_id, offset_x_px=offset,
                                   offset_y_px=0.0, captured_at=captured_at,
                                   frame_seq=frame_seq)

        mission._observation = observation
        target = SimpleNamespace(target_id="yellow-3")
        result = mission._approach_target(target, protected=True)
        self.assertEqual(result.target_id, "yellow-3")
        self.assertGreaterEqual(clock.now, 1.6)
        self.assertLessEqual(clock.now, 1.7)
        mission._move_translation_only.assert_called_with(0.0, 0.0)
        self.assertTrue(mission._move_toward.called)
        mission.navi.stop_move.assert_called_once()
        mission.navi.set_yaw.assert_called_once_with(0.0)

    def test_poll_stamps_frame_before_inference_and_increments_sequence(self):
        clock = iter((10.0, 10.25, 10.4, 10.7))
        namespace = {"time": SimpleNamespace(monotonic=lambda: next(clock),
                                                time=lambda: 1234.0),
                     "TARGET_CAMERA_READ_FAILURES": 10}
        cls = load_class("rescue_drop_2026.py", "VisionInterface",
                         names={"__init__", "poll"}, namespace=namespace)
        vision = cls()
        vision._camera = Mock()
        vision._camera.read.return_value = (True, object())
        vision._detector = Mock()
        vision._detector.detect.return_value = ()
        vision._track = Mock(return_value=())
        vision.poll()
        vision.poll()
        calls = vision._track.call_args_list
        self.assertEqual([call.args[2] for call in calls], [1, 2])
        self.assertEqual([call.args[1] for call in calls], [10.0, 10.4])
        self.assertEqual([round(call.args[4]) for call in calls], [250, 300])
        self.assertEqual([call.args[3] for call in calls], [1234.0, 1234.0])

    def test_low_calibration_uses_two_distinct_centered_frames(self):
        clock = SimpleNamespace(now=0.0)
        self.namespace.update({
            "time": SimpleNamespace(monotonic=lambda: clock.now),
            "MANDATORY_COLOR": "yellow", "FREE_DROP_HEIGHT": 60.0,
            "FREE_TARGET_GROUND_HEIGHT_CM": 0.0,
            "payload_target_offset_px": Mock(return_value=(0.0, 0.0)),
            "low_calibration_command": lambda error, velocity: (0.0, error),
            "LOW_CALIBRATION_TIMEOUT": 1.0,
            "LOW_CALIBRATION_MAX_FRAME_AGE_S": 0.45,
            "LOW_CALIBRATION_HOLD_AFTER_LOSS_S": 1.0,
            "LOW_CALIBRATION_VELOCITY_MAX_GAP_S": 1.0,
            "LOW_CALIBRATION_SETTLE_MAX_GAP_S": 0.65,
            "LOW_CALIBRATION_MAX_PIXEL_SPEED": 120.0,
            "LOW_CALIBRATION_THRESHOLD_PX": 20.0,
            "LOW_CALIBRATION_CENTER_FRAMES": 2,
            "LOW_CALIBRATION_LOG_PERIOD_S": 0.5,
        })
        cls = load_class("rescue_drop_2026.py", "Mission",
                         names={"_calibrate_low"}, namespace=self.namespace)
        mission = cls()
        mission.vision = SimpleNamespace(frame_size=(640, 480))
        mission.navi = Mock(current_yaw=0.0)
        mission._check = Mock()
        mission._position = Mock(return_value=(0.0, 0.0))
        mission._move_translation_only = Mock()
        mission.stop_event = SimpleNamespace(
            wait=lambda period: setattr(clock, "now", round(clock.now + period, 2)))

        def observation(target_id):
            seq, at = (2, 0.4) if clock.now >= 0.4 else (1, 0.1)
            return SimpleNamespace(target_id=target_id, color="red",
                                   offset_x_px=10.0, offset_y_px=0.0,
                                   captured_at=at, frame_seq=seq)

        mission._observation = observation
        target = SimpleNamespace(target_id="red-1", color="red")
        self.assertTrue(mission._calibrate_low(target, False, 1))
        self.assertGreaterEqual(clock.now, 0.4)
        self.assertLess(clock.now, 0.5)
        mission._move_translation_only.assert_called_once_with(0.0, 0.0)
        mission.navi.stop_move.assert_not_called()

    def test_jump_diagnostics_include_previous_pose_and_command(self):
        cls = load_class("rescue_drop_2026.py", "Mission",
                         names={"_vision_diagnostic"}, namespace=self.namespace)
        mission = cls()
        mission.vision_diagnostics = True
        mission._vision_diag_recent = []
        mission.navi = Mock(current_yaw=0.0)
        mission._position = Mock(side_effect=[(1.0, 2.0), (3.0, 4.0)])
        previous = SimpleNamespace(target_id="red-1", frame_seq=1,
                                   captured_at=0.1, offset_x_px=70.0,
                                   offset_y_px=0.0)
        current = SimpleNamespace(target_id="red-1", frame_seq=2,
                                  captured_at=0.5, offset_x_px=5.0,
                                  offset_y_px=0.0)
        mission._vision_diagnostic("approach", previous, "move", (5, 0))
        mission._vision_diagnostic("approach", current, "jump-pending",
                                   (0, 0), previous)
        prior = self.namespace["logger"].debug.call_args.args[3]
        self.assertEqual(prior[1], (1.0, 2.0))
        self.assertEqual(prior[3:], ((5, 0), "move"))


class CenterMissionOfflineTests(unittest.TestCase):
    def setUp(self):
        source = SDK / "rescue_drop_2026.py"
        tree = ast.parse(source.read_text(encoding="utf-8"))
        helpers = [node for node in tree.body if isinstance(node, ast.FunctionDef)
                   and node.name in {"body_to_world_velocity", "pixel_offset_to_ground_cm",
                                     "payload_offset_body_cm", "sample_polyline",
                                     "segment_distance_to_point"}]
        future = ast.ImportFrom(module="__future__", names=[ast.alias(name="annotations")], level=0)
        helper_tree = ast.fix_missing_locations(ast.Module(body=[future] + helpers, type_ignores=[]))
        self.namespace = {
            "math": math, "logger": Mock(), "MANDATORY_COLOR": "yellow",
            "MANDATORY_DROP_HEIGHT": 100.0, "CRUISE_HEIGHT": 150.0,
            "MANDATORY_TARGET_GROUND_HEIGHT_CM": 30.0,
            "CAMERA_VIEW_LONG_EDGE_CM_AT_REFERENCE_HEIGHT": 300.0,
            "CAMERA_VIEW_REFERENCE_HEIGHT_CM": 160.0,
            "PAYLOAD_ANGLES_DEG": (0.0, 72.0, 144.0, 216.0, 288.0),
            "PAYLOAD_RADIUS_CM": 7.2, "TOTAL_DROP_COUNT": 5,
            "VISUAL_PERIOD": 0.1,
        }
        exec(compile(helper_tree, str(source), "exec"), self.namespace)
        mission_class = load_class("rescue_drop_2026.py", "Mission", names={
            "_position", "_estimate_target_world_xy", "_desired_aircraft_drop_pose",
            "_mandatory_drop", "_mark_mandatory_pose_rejected",
            "_mandatory_observation_allowed", "_start_center_path_with_fallback",
            "_at_waypoint", "_execute_center_route", "_update_center_route_progress",
        }, namespace=self.namespace)
        self.mission = mission_class()
        self.mission.navi = Mock(current_x=200.0, current_y=0.0,
                                 current_yaw=0.0, current_height=150.0)
        self.mission.navi.height_pid.setpoint = 150.0
        self.mission.vision = Mock(frame_size=(640, 480))
        self.mission.obstacle = Mock()
        self.mission.target = SimpleNamespace(color="yellow", target_id="yellow-1")
        self.mission.ledger = SimpleNamespace(next_drop_number=1)
        self.mission._check = Mock()
        self.mission._approach_target = Mock(return_value=SimpleNamespace(
            offset_x_px=0.0, offset_y_px=0.0))
        self.mission._resume_center_route = Mock()
        self.mission._set_height = Mock()
        self.mission._calibrate_low = Mock(return_value=False)
        self.mission._drop = Mock()
        self.mission._navigate_center_exit = Mock()

    def test_blocked_drop_pose_never_descends_or_drops(self):
        mission = self.mission
        mission.obstacle.mandatory_drop_pose_is_clear.return_value = False
        self.assertFalse(mission._mandatory_drop())
        self.assertEqual(mission.obstacle.mandatory_drop_pose_is_clear.call_args.args[0],
                         (192.8, 0.0))
        mission._set_height.assert_not_called()
        mission._calibrate_low.assert_not_called()
        mission._drop.assert_not_called()
        mission._resume_center_route.assert_called_once()
        self.assertFalse(mission._mandatory_observation_allowed(
            SimpleNamespace(target_id="yellow-1")))
        mission.navi.current_x = 281.0
        self.assertTrue(mission._mandatory_observation_allowed(
            SimpleNamespace(target_id="yellow-1")))

    def test_clear_drop_pose_still_drops_after_failed_low_calibration(self):
        mission = self.mission
        mission.obstacle.mandatory_drop_pose_is_clear.return_value = True
        self.assertTrue(mission._mandatory_drop())
        self.assertEqual([call.args[0] for call in mission._set_height.call_args_list],
                         [100.0, 150.0])
        self.assertEqual(mission._set_height.call_args_list[0].kwargs,
                         {"preserve_horizontal_hold": True})
        self.assertEqual(mission._set_height.call_args_list[1].kwargs,
                         {"translation_only": True})
        mission._calibrate_low.assert_called_once_with(
            mission.target, protected=True, drop_number=1,
            target_world=(200.0, 0.0))
        mission._drop.assert_called_once_with("yellow", "yellow-1", False)
        mission._navigate_center_exit.assert_called_once()

    def test_payload_sequence_and_yaw_change_desired_pose(self):
        mission = self.mission
        mission.navi.current_yaw = 90.0
        for number, angle in enumerate(self.namespace["PAYLOAD_ANGLES_DEG"], 1):
            with self.subTest(drop_number=number):
                pose = mission._desired_aircraft_drop_pose((200.0, 0.0), number)
                expected = self.namespace["body_to_world_velocity"](
                    7.2 * math.cos(math.radians(angle)),
                    7.2 * math.sin(math.radians(angle)), 90.0)
                np.testing.assert_allclose(pose, (200.0 - expected[0], -expected[1]))

    def test_spline_rejection_uses_checked_linear_fallback(self):
        mission = self.mission
        worker = Mock()
        mission.navi._thread_list = []
        mission.obstacle.path_is_free.side_effect = [False, True]

        def reject_spline(*args, **kwargs):
            self.assertFalse(kwargs["trajectory_validator"]([(200, 0, 150),
                                                               (260, 0, 150)]))
            return False

        def start_linear(points, **kwargs):
            self.assertEqual(points[-1][:2], (260.0, 0.0))
            mission.navi._thread_list.append(worker)
            return True

        mission.navi.navigation_follow_waypoints.side_effect = reject_spline
        mission.navi.navigation_follow_trajectory.side_effect = start_linear
        self.assertIs(mission._start_center_path_with_fallback([(260, 0)]), worker)
        self.assertEqual(mission.obstacle.path_is_free.call_count, 2)

    def test_blocked_remaining_trajectory_stops_then_replans_from_current_pose(self):
        mission = self.mission
        center = object()
        self.namespace["MissionState"] = SimpleNamespace(CENTER=center)
        mission.route_3 = [(0, 0), (100, 0)]
        mission.route_index = {center: 1}
        mission.navi.current_x = 0.0
        mission.navi.traj_running_event.is_set.return_value = False
        mission.stop_event = Mock()
        plan = SimpleNamespace(waypoints=((100, 0),), anchors=((0, (100, 0)),),
                               revision=1, skipped_offsets=())
        mission.obstacle.plan_route_window.return_value = plan
        first, second = Mock(), Mock()
        first.is_alive.return_value = True
        second.is_alive.return_value = False

        def start(*args):
            if mission.obstacle.plan_route_window.call_count == 1:
                return first
            mission.navi.current_x = 100.0
            return second

        mission._start_center_path_with_fallback = Mock(side_effect=start)
        mission._center_remaining_trajectory_safe = Mock(return_value=False)
        mission._stop_center_worker = Mock(side_effect=lambda worker: setattr(
            mission.navi, "current_x", 40.0))
        self.assertIsNone(mission._execute_center_route(detect_mandatory=False))
        self.assertEqual(mission.obstacle.plan_route_window.call_count, 2)
        self.assertEqual(mission.obstacle.plan_route_window.call_args_list[1].args[0],
                         (40.0, 0.0))
        mission._stop_center_worker.assert_called_once_with(first)
        self.assertEqual(mission.route_index[center], 2)


class WiringTests(unittest.TestCase):
    def test_navigation_validator_rejects_actual_generated_trajectory(self):
        navigation = load_class("FlightController/Solutions/Navigation.py", "Navigation",
                                names={"navigation_follow_waypoints"},
                                namespace={"np": np, "logger": Mock()})
        nav = navigation()
        nav.current_x, nav.current_y, nav.current_height = 0.0, 0.0, 150.0
        nav.create_smooth_traj_list = Mock(return_value=[(0.0, 0.0, 150.0),
                                                        (50.0, 20.0, 150.0),
                                                        (100.0, 0.0, 150.0)])
        nav.navigation_follow_trajectory = Mock()
        seen = []
        result = nav.navigation_follow_waypoints(
            [(100.0, 0.0)], wait=False,
            trajectory_validator=lambda points: seen.extend(points) or False)
        self.assertFalse(result)
        self.assertEqual(seen, [(0.0, 0.0, 150.0), (50.0, 20.0, 150.0),
                                (100.0, 0.0, 150.0)])
        nav.navigation_follow_trajectory.assert_not_called()

    def test_active_trajectory_clears_after_worker_error(self):
        navigation = load_class("FlightController/Solutions/Navigation.py", "Navigation",
                                names={"_trajectory_task", "active_trajectory_remaining"},
                                namespace={})
        nav = navigation()
        nav._control_lock = module.threading.Lock()
        nav._active_traj_list, nav._active_traj_index = (), 0

        def fail_after_progress(*args, **kwargs):
            nav._active_traj_index = 2
            self.assertEqual(nav.active_trajectory_remaining(),
                             [(1.0, 0.0, 150.0), (2.0, 0.0, 150.0)])
            raise RuntimeError("simulated worker failure")

        nav._trajectory_task_inner = fail_after_progress
        with self.assertRaisesRegex(RuntimeError, "simulated worker failure"):
            nav._trajectory_task([(0, 0, 150), (1, 0, 150), (2, 0, 150)])
        self.assertEqual(nav.active_trajectory_remaining(), [])

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
                fc, relay, mission, vision = Mock(), Mock(), Mock(), Mock()
                fc.connect.side_effect = lambda **kwargs: self.assertEqual(
                    vision.prime.call_count, int(flight))
                fc.connected = True
                fc.state.is_fresh.return_value = True
                fc.state.unlock.value = False
                relay.detect_channel_count.return_value = 8
                relay.query_status.return_value = dict.fromkeys(range(1, 9), False)
                planner = Mock()
                namespace = {"parse_args": lambda: SimpleNamespace(confirm_flight=flight,
                             red_count=2, blue_count=1, green_count=1,
                             fc_host="127.0.0.1", fc_server_port=5654, relay_port="fake"),
                             "threading": module.threading, "FREE_COLORS": ("red", "blue", "green"),
                             "validate_allocation": Mock(return_value={"red": 2, "blue": 1, "green": 1}),
                             "VisionInterface": Mock(return_value=vision),
                             "ObstaclePlanner2D": Mock(return_value=planner),
                             "ObstacleInterface": Mock(), "require_flight_interfaces": Mock(),
                             "FC_Client": Mock(return_value=fc), "Navigation": Mock(),
                             "Mission": Mock(return_value=mission), "LCUSRelay": Mock(return_value=relay),
                             "RELAY_CHANNEL_COUNT": 8, "TOTAL_DROP_COUNT": 5, "MISSION_TIMEOUT": 1200,
                             "PAYLOAD_RELAY_CHANNELS": (8, 6, 4, 1, 5),
                             "PAYLOAD_ANGLES_DEG": (0.0, 72.0, 144.0, 216.0, 288.0),
                             "wait_for_start_command": Mock(), "time": module.time, "logger": Mock(),
                             "stop_navigation_ros": Mock()}
                exec(compile(tree, str(source), "exec"), namespace)
                self.assertEqual(namespace["main"](), 0)
                fc.connect.assert_called_once_with(
                    host="127.0.0.1", port=5654, authkey=b"fc",
                    print_state=False, block=True, timeout=10)
                fc.start_listen_serial.assert_not_called()
                self.assertEqual(vision.prime.call_count, int(flight))
                self.assertEqual(namespace["ObstaclePlanner2D"].call_count, int(flight))
                self.assertIs(namespace["Navigation"].call_args.kwargs["obstacle_planner"],
                              planner if flight else None)
                namespace["stop_navigation_ros"].assert_called_once_with(namespace["Navigation"].return_value)
                if flight:
                    namespace["ObstacleInterface"].assert_called_once_with(planner)
                    mission.run.assert_called_once()
                else:
                    mission.monitor_pose.assert_called_once()
                    namespace["LCUSRelay"].assert_not_called()


if __name__ == "__main__":
    unittest.main()
