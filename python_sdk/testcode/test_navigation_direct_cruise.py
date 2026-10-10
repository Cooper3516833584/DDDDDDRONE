"""Offline cruise regressions using real PID and extracted production methods.

No FlightController package import, ROS, serial, camera or flight entry is run.
"""

import ast
from dataclasses import dataclass
from enum import Enum
import math
from pathlib import Path
import threading
import time
from types import SimpleNamespace
import typing
from unittest.mock import Mock

import numpy as np
import pytest
from simple_pid import PID


SDK = Path(__file__).resolve().parents[1]


def load_methods(path, class_name, methods, namespace, helpers=()):
    tree = ast.parse(path.read_text(encoding="utf-8"))
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == class_name)
    cls.body = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in methods]
    functions = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name in helpers]
    future = ast.ImportFrom(module="__future__", names=[ast.alias(name="annotations")], level=0)
    module = ast.fix_missing_locations(ast.Module(body=[future] + functions + [cls], type_ignores=[]))
    exec(compile(module, str(path), "exec"), namespace)
    return namespace[class_name]


@pytest.fixture
def nav_context():
    namespace = {
        "np": np, "PID": PID, "threading": threading,
        "time": SimpleNamespace(sleep=time.sleep, monotonic=time.monotonic,
                                perf_counter=time.perf_counter),
        "logger": Mock(), "logger_dbg": Mock(), "POSE_STALE_TIMEOUT": 0.30,
        "NAVIGATION_LOOP_INTERVAL": 0.005, "NAVIGATION_CONTROL_STALE_TIMEOUT": 0.30,
        "YAW_AMBIGUITY_BAND_DEG": 2.0,
    }
    cls = load_methods(SDK / "FlightController/Solutions/Navigation.py", "Navigation", {
        "__init__", "navigation_to_waypoint_direct", "set_navigation_speed",
        "_wait_for_waypoint_direct",
        "_navigation_task", "update_realtime_control", "pose_is_fresh",
        "wait_for_waypoint", "_reached_waypoint", "_waypoint_param_switch",
        "navigation_stop_here", "navigation_target",
    }, namespace, helpers={"_shortest_yaw_error", "_world_to_body_velocity"})
    fc = SimpleNamespace(HOLD_POS_MODE=1, sent=[], state=SimpleNamespace(
        mode=SimpleNamespace(value=1), unlock=SimpleNamespace(value=True)))
    fc.send_realtime_control_data = lambda *values: fc.sent.append(values)
    provider = SimpleNamespace(get_pose=lambda: (nav.current_x, nav.current_y, nav.current_yaw, True))
    nav = cls(fc=fc, lio_pose_provider=provider)
    nav.running = nav.navigation_flag = True
    nav._last_pose_update = time.monotonic()
    # Keep pose freshness valid during thread lifecycle tests without ROS updates.
    nav.pose_is_fresh = Mock(return_value=True)
    return nav, namespace


def send_one_frame(nav, namespace, monkeypatch):
    monkeypatch.setattr(namespace["time"], "sleep", lambda _: None)

    def send(*values):
        nav.fc.sent.append(values)
        nav.running = False

    nav.fc.send_realtime_control_data = send
    nav._navigation_task()
    assert len(nav.fc.sent) == 1
    return nav.fc.sent[0]


@pytest.mark.parametrize("output,expected", [
    ((20, 20), (14, 14)), ((30, 0), (20, 0)),
    ((0, -30), (0, -20)), ((10, 0), (10, 0)),
])
def test_horizontal_vector_clamp(nav_context, monkeypatch, output, expected):
    nav, namespace = nav_context
    nav.set_navigation_speed(20)
    nav.navi_x_pid = Mock(return_value=output[0])
    nav.navi_y_pid = Mock(return_value=output[1])
    rotation = Mock(wraps=namespace["_world_to_body_velocity"])
    monkeypatch.setitem(namespace, "_world_to_body_velocity", rotation)
    vx, vy, _, _ = send_one_frame(nav, namespace, monkeypatch)
    assert (vx, vy) == expected
    assert math.hypot(vx, vy) <= 20.5
    world_x, world_y, _ = rotation.call_args.args
    assert math.hypot(world_x, world_y) <= 20.0 + 1e-12
    if output == (20, 20):
        assert (world_x, world_y) == pytest.approx((math.sqrt(200), math.sqrt(200)))


@pytest.mark.parametrize("yaw", [37, 90, -90, 179])
def test_rotated_commands_keep_horizontal_limit(nav_context, monkeypatch, yaw):
    nav, namespace = nav_context
    nav.current_yaw = yaw
    nav.set_navigation_speed(20)
    nav.navi_x_pid = Mock(return_value=20)
    nav.navi_y_pid = Mock(return_value=20)
    vx, vy, _, _ = send_one_frame(nav, namespace, monkeypatch)
    assert math.hypot(vx, vy) <= 21.0  # Integer rounding tolerance after rotation.


@pytest.mark.parametrize("wait,result", [(True, True), (True, False), (False, True)])
def test_direct_sets_exact_horizontal_target_without_trajectory(nav_context, monkeypatch, wait, result):
    nav, namespace = nav_context
    nav.set_navigation_speed(30)
    nav.height_pid.setpoint = 150
    nav.yaw_target = 42
    nav._waypoint_param_switch()
    nav._wait_for_waypoint_direct = Mock(return_value=result)
    nav.wait_for_waypoint = Mock(side_effect=AssertionError("legacy waiter called"))
    nav.navigation_follow_trajectory = Mock(side_effect=AssertionError("trajectory called"))
    generator = Mock(side_effect=AssertionError("trajectory generated"))
    monkeypatch.setitem(namespace, "TrajectoryGenerator", generator)
    assert nav.navigation_to_waypoint_direct((200, 0), wait=wait) is result
    if not wait:
        assert len(nav._thread_list) == 1
        worker = nav._thread_list[-1]
        worker.join(timeout=2.0)
        assert not worker.is_alive()
        assert worker.daemon and worker.name == "navigation_direct_waypoint"
    assert (nav.navi_x_pid.setpoint, nav.navi_y_pid.setpoint) == (200, 0)
    assert nav.height_pid.setpoint == 150 and nav.yaw_target == 42
    assert nav.navi_x_pid.tunings == nav.navi_y_pid.tunings == nav.pid_tunings["navi"]
    assert nav.navi_x_pid.output_limits == nav.navi_y_pid.output_limits == (-30, 30)
    nav._wait_for_waypoint_direct.assert_called_once_with(
        200.0, 0.0, time_thres=0.1, pos_thres=10.0, timeout=25.0)
    nav.wait_for_waypoint.assert_not_called()
    generator.assert_not_called()
    nav.navigation_follow_trajectory.assert_not_called()


@pytest.mark.parametrize("speed", [10, 30])
def test_direct_far_target_reaches_real_pid_speed_limit(nav_context, monkeypatch, speed):
    nav, namespace = nav_context
    nav.set_navigation_speed(speed)
    nav._wait_for_waypoint_direct = Mock(return_value=True)
    assert nav.navigation_to_waypoint_direct((200, 0))
    vx, vy, _, _ = send_one_frame(nav, namespace, monkeypatch)
    assert (vx, vy) == (speed, 0)


def test_direct_pid_slows_near_target(nav_context, monkeypatch):
    nav, namespace = nav_context
    nav.current_x = 195
    nav.set_navigation_speed(30)
    nav._wait_for_waypoint_direct = Mock(return_value=True)
    nav.navigation_to_waypoint_direct((200, 0))
    vx, vy, _, _ = send_one_frame(nav, namespace, monkeypatch)
    assert (vx, vy) == (7, 0)


@pytest.mark.parametrize("waypoint", [(1,), (1, 2, 3), (float("nan"), 0), (0, float("inf"))])
def test_direct_rejects_invalid_coordinates(nav_context, waypoint):
    nav, _ = nav_context
    with pytest.raises(ValueError):
        nav.navigation_to_waypoint_direct(waypoint, wait=False)
    assert nav._thread_list == []
    assert nav.navigation_target.tolist() == [0, 0]


@pytest.mark.parametrize("failure", ["stopped", "external_stop", "stale"])
def test_direct_refuses_unavailable_navigation(nav_context, failure):
    nav, _ = nav_context
    if failure == "stopped":
        nav.running = False
    elif failure == "external_stop":
        nav.stop_event = threading.Event()
        nav.stop_event.set()
    else:
        nav.pose_is_fresh.return_value = False
    assert nav.navigation_to_waypoint_direct((200, 0), wait=False) is False
    assert nav._thread_list == []
    assert nav.navigation_target.tolist() == [0, 0]


def test_direct_completion_preserves_exact_target(nav_context):
    nav, _ = nav_context
    nav.current_x = 195
    assert nav.navigation_to_waypoint_direct((200, 0))
    assert nav.navigation_target.tolist() == [200, 0]
    assert nav.navigation_flag
    assert not nav.fc.sent


@pytest.fixture
def direct_clock(nav_context, monkeypatch):
    nav, namespace = nav_context
    clock = SimpleNamespace(now=0.0)
    monkeypatch.setattr(namespace["time"], "perf_counter", lambda: clock.now)
    monkeypatch.setattr(namespace["time"], "sleep",
                        lambda seconds: setattr(clock, "now", clock.now + seconds))
    return nav, namespace, clock


@pytest.mark.parametrize("current,expected", [
    ((0, 0), True), ((9, 0), True), ((0, 9), True), ((7, 7), True),
    ((9, 9), False), ((10, 0), True), ((11, 0), False),
])
def test_direct_waypoint_uses_radial_distance_threshold(direct_clock, current, expected):
    nav, _, clock = direct_clock
    nav.current_x, nav.current_y = current
    nav._waypoint_param_switch = Mock(wraps=nav._waypoint_param_switch)
    assert load_mission()._at_waypoint(current, (0, 0)) is expected
    assert nav._wait_for_waypoint_direct(0, 0, pos_thres=10, timeout=0.2) is expected
    if expected:
        assert clock.now == pytest.approx(0.1)
        nav._waypoint_param_switch.assert_called_once_with()
        assert nav.navi_x_pid.tunings == nav.navi_y_pid.tunings == nav.pid_tunings["hover"]
    else:
        assert clock.now > 0.2
        nav._waypoint_param_switch.assert_not_called()
    assert nav.navigation_target.tolist() == [0, 0]


def test_direct_wait_resets_settle_time_outside_radial_threshold(direct_clock, monkeypatch):
    nav, namespace, clock = direct_clock
    positions = iter([(7, 7), (9, 9), (7, 7), (7, 7)])

    def sleep(seconds):
        clock.now += seconds
        nav.current_x, nav.current_y = next(positions)

    monkeypatch.setattr(namespace["time"], "sleep", sleep)
    assert nav._wait_for_waypoint_direct(0, 0, timeout=0.3)
    assert clock.now == pytest.approx(0.2)


@pytest.mark.parametrize("failure", ["stopped", "stale"])
def test_direct_wait_does_not_confirm_arrival_without_valid_pose(direct_clock, failure):
    nav, _, clock = direct_clock
    if failure == "stopped":
        nav.running = False
    else:
        nav.pose_is_fresh.return_value = False
    assert nav._wait_for_waypoint_direct(0, 0, timeout=0.2) is False
    assert clock.now > 0.2


def test_legacy_rectangular_waypoint_check_is_unchanged(nav_context):
    nav, _ = nav_context
    nav.current_x = nav.current_y = 9
    assert nav._reached_waypoint(10)
    nav.current_x, nav.current_y = 10, 0
    assert not nav._reached_waypoint(10)


def test_direct_timeout_keeps_target_and_reports_failure(nav_context, monkeypatch):
    nav, namespace = nav_context
    clock = SimpleNamespace(now=0.0)
    monkeypatch.setattr(namespace["time"], "perf_counter", lambda: clock.now)
    monkeypatch.setattr(namespace["time"], "sleep",
                        lambda seconds: setattr(clock, "now", clock.now + seconds))
    nav.set_navigation_speed(30)
    assert nav.navigation_to_waypoint_direct((200, 0)) is False
    assert 25.0 < clock.now < 25.1
    assert nav.navigation_target.tolist() == [200, 0]
    assert not nav.fc.sent


def test_external_stop_exits_running_direct_worker(nav_context):
    nav, _ = nav_context
    nav.stop_event = threading.Event()
    assert nav.navigation_to_waypoint_direct((200, 0), wait=False)
    worker = nav._thread_list[-1]
    nav.stop_event.set()
    worker.join(timeout=2.0)
    assert not worker.is_alive()


def test_stop_here_allows_mission_join_within_two_seconds(nav_context):
    nav, _ = nav_context
    entered = threading.Event()
    wait_for_waypoint = nav._wait_for_waypoint_direct
    results = []

    def wait(*args, **kwargs):
        entered.set()
        result = wait_for_waypoint(*args, **kwargs)
        results.append(result)
        return result

    nav._wait_for_waypoint_direct = wait
    assert nav.navigation_to_waypoint_direct((200, 0), wait=False)
    worker = nav._thread_list[-1]
    try:
        assert entered.wait(timeout=1.0)
        assert worker.is_alive()
        mission = load_mission()(fc=None, navi=nav, relay=None, vision=None, obstacle=None,
                                 allocation={"red": 4, "blue": 0, "green": 0},
                                 stop_event=threading.Event())
        started = time.monotonic()
        mission._stop_leg_worker(worker)
        assert time.monotonic() - started < 2.0
        assert not worker.is_alive()
        assert results == [False]  # Cancellation must not report arrival at the original target.
        assert nav.navigation_target.tolist() == [0, 0]
    finally:
        nav.stop_event = threading.Event()
        nav.stop_event.set()
        worker.join(timeout=2.0)


@pytest.mark.parametrize("stop_first", [True, False])
def test_new_direct_target_replaces_old_worker(nav_context, stop_first):
    nav, _ = nav_context
    waiter = nav._wait_for_waypoint_direct
    entered = {(200, 0): threading.Event(), (200, 200): threading.Event()}
    results = {}

    def wait(x, y, **kwargs):
        entered[(x, y)].set()
        result = waiter(x, y, **kwargs)
        results[(x, y)] = result
        return result

    nav._wait_for_waypoint_direct = wait
    try:
        assert nav.navigation_to_waypoint_direct((200, 0), wait=False)
        old_worker = nav._thread_list[-1]
        assert entered[(200, 0)].wait(timeout=1.0)
        assert old_worker.is_alive()
        if stop_first:
            nav.navigation_stop_here()
            old_worker.join(timeout=2.0)
            assert not old_worker.is_alive()
            assert results[(200, 0)] is False

        assert nav.navigation_to_waypoint_direct((200, 200), wait=False)
        new_worker = nav._thread_list[-1]
        assert entered[(200, 200)].wait(timeout=1.0)
        old_worker.join(timeout=2.0)
        assert not old_worker.is_alive()
        assert results[(200, 0)] is False
        assert new_worker is not old_worker and new_worker.is_alive()
        assert nav.navigation_target.tolist() == [200, 200]
        assert nav.navi_x_pid.tunings == nav.navi_y_pid.tunings == nav.pid_tunings["navi"]
    finally:
        nav.navigation_stop_here()
        for worker in nav._thread_list:
            worker.join(timeout=2.0)
            assert not worker.is_alive()


def load_mission():
    path = SDK / "rescue_drop_2026.py"
    tree = ast.parse(path.read_text(encoding="utf-8"))
    selected = [n for n in tree.body if isinstance(n, ast.Assign)
                or (isinstance(n, ast.ClassDef) and n.name in {
                    "Mission", "MissionState", "DropLedger", "TargetObservation"})
                or (isinstance(n, ast.FunctionDef) and n.name in {
                    "build_routes", "body_to_world_velocity", "world_to_body_velocity"})]
    future = ast.ImportFrom(module="__future__", names=[ast.alias(name="annotations")], level=0)
    module = ast.fix_missing_locations(ast.Module(body=[future] + selected, type_ignores=[]))
    namespace = dict(vars(typing), __name__=__name__, dataclass=dataclass, Enum=Enum,
                     math=math, threading=threading, time=time, logger=Mock())
    exec(compile(module, str(path), "exec"), namespace)
    return namespace["Mission"]


@pytest.fixture
def mission():
    cls = load_mission()
    navi = SimpleNamespace(current_x=0, current_y=0, current_yaw=0, current_height=150,
                           _thread_list=[], traj_running_event=threading.Event(),
                           navigation_to_waypoint=Mock(side_effect=AssertionError("legacy API called")),
                           pointing_landing=Mock(return_value=True), move_by_direction=Mock(),
                           update_realtime_control=Mock(), stop_move=Mock())

    def direct(waypoint, **kwargs):
        navi.current_x, navi.current_y = waypoint
        navi._thread_list.append(SimpleNamespace(is_alive=lambda: False, join=Mock()))
        return True

    navi.navigation_to_waypoint_direct = Mock(side_effect=direct)
    obstacle = SimpleNamespace(safe_waypoint=Mock(side_effect=lambda current, goal: goal),
                               safe_velocity=Mock(side_effect=lambda current, velocity: velocity))
    result = cls(None, navi, None, None, obstacle,
                 {"red": 4, "blue": 0, "green": 0}, threading.Event())
    result._check = Mock()
    result._observation = Mock(return_value=None)
    return result


@pytest.mark.parametrize("state_name,route_name", [
    ("ROUTE_1", "route_1"), ("CENTER", "route_3"), ("ROUTE_2", "route_2"),
])
def test_rescue_routes_use_direct_api(mission, state_name, route_name):
    state = mission.__init__.__globals__["MissionState"][state_name]
    assert mission._follow_route(state) is None
    waypoints = getattr(mission, route_name)[1:]
    assert [c.args[0] for c in mission.navi.navigation_to_waypoint_direct.call_args_list] == waypoints
    assert all(c.kwargs == {"wait": False, "pos_thres": 10.0}
               for c in mission.navi.navigation_to_waypoint_direct.call_args_list)
    mission.navi.navigation_to_waypoint.assert_not_called()
    assert mission.obstacle.safe_waypoint.called is (state_name == "CENTER")


def test_rescue_return_home_uses_direct_api(mission):
    mission.navi.current_x = 200
    mission._return_and_land()
    mission.navi.navigation_to_waypoint_direct.assert_called_once_with(
        (0.0, 0.0), wait=False, pos_thres=10.0)
    mission.navi.navigation_to_waypoint.assert_not_called()
    assert mission.landed


def test_protected_leg_uses_planner_intermediate_waypoint(mission):
    mission.obstacle.safe_waypoint.side_effect = [(80, 20), (200, 0)]
    mission._navigate_leg((200, 0), protected=True)
    assert [c.args[0] for c in mission.navi.navigation_to_waypoint_direct.call_args_list] == [
        (80, 20), (200, 0)]


@pytest.mark.parametrize("protected", [False, True])
def test_visual_approach_speed_is_independent_from_cruise(mission, protected):
    namespace = mission._move_toward.__globals__
    namespace["CRUISE_SPEED"] = 30.0
    observation = SimpleNamespace(offset_x_px=100.0, offset_y_px=0.0)
    mission._move_toward(observation, protected=protected)
    if protected:
        mission.navi.update_realtime_control.assert_called_once_with(
            vel_x=10, vel_y=0, yaw=0)
        mission.navi.move_by_direction.assert_not_called()
    else:
        mission.navi.move_by_direction.assert_called_once_with(
            speed=namespace["VISUAL_APPROACH_SPEED"], direction_deg=0.0)
    assert namespace["VISUAL_APPROACH_SPEED"] == 10.0
    mission.navi.navigation_to_waypoint_direct.assert_not_called()


def test_rescue_leg_start_failure_is_reported(mission):
    mission.navi.navigation_to_waypoint_direct.side_effect = None
    mission.navi.navigation_to_waypoint_direct.return_value = False
    with pytest.raises(RuntimeError, match="could not start"):
        mission._start_leg_worker((200, 0))


def test_rescue_already_at_waypoint_does_not_start_worker(mission):
    assert mission._start_leg_worker((5, 0)) is None
    mission.navi.navigation_to_waypoint_direct.assert_not_called()
