"""Offline height-source checks; production modules are parsed, not imported."""

import ast
import threading
import time
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock

import numpy as np
import pytest


SDK = Path(__file__).resolve().parents[1]
NAV_SOURCE = SDK / "FlightController/Solutions/Navigation.py"
RESCUE_SOURCE = SDK / "rescue_drop_2026.py"


class StubPID:
    def __init__(self, *_args, setpoint=0, output_limits=None, auto_mode=False):
        self.setpoint = setpoint
        self.output_limits = output_limits
        self.auto_mode = auto_mode

    def set_auto_mode(self, enabled, **_kwargs):
        self.auto_mode = enabled


def extracted_class(path, class_name, names, namespace):
    tree = ast.parse(path.read_text(encoding="utf-8"))
    source_class = next(node for node in tree.body
                        if isinstance(node, ast.ClassDef) and node.name == class_name)
    selected = ast.ClassDef(
        name=class_name, bases=[], keywords=[],
        body=[node for node in source_class.body
              if isinstance(node, ast.FunctionDef) and node.name in names],
        decorator_list=[],
    )
    future = ast.ImportFrom(module="__future__", names=[ast.alias(name="annotations")], level=0)
    module = ast.fix_missing_locations(ast.Module(body=[future, selected], type_ignores=[]))
    exec(compile(module, str(path), "exec"), namespace)
    return namespace[class_name]


NAV_METHODS = {
    "__init__", "_get_lio_height", "get_agl_height", "_get_control_height",
    "height_is_fresh", "_flight_state_is_fresh", "_keep_height_task",
    "_navigation_task", "pointing_takeoff",
    "update_realtime_control", "wait_for_height", "set_height",
    "calibrate_basepoint", "_stop_velocity_override", "pointing_landing",
}
NAV_NAMESPACE = {
    "np": np, "PID": StubPID, "threading": threading, "time": time,
    "logger": Mock(), "logger_dbg": Mock(), "POSE_STALE_TIMEOUT": 0.30,
    "LIO_CALIBRATION_WAIT_SECONDS": 3.0,
    "NAVIGATION_CONTROL_STALE_TIMEOUT": 0.30,
    "NAVIGATION_LOOP_INTERVAL": 0.005,
    "VELOCITY_OVERRIDE_ZERO_FLUSH_FRAMES": 3,
    "VELOCITY_OVERRIDE_ZERO_FLUSH_INTERVAL": 0.05,
}
Navigation = extracted_class(NAV_SOURCE, "Navigation", NAV_METHODS, NAV_NAMESPACE)


class Provider:
    def __init__(self, snapshots):
        self.snapshots = list(snapshots)

    def get_snapshot(self):
        if len(self.snapshots) > 1:
            return self.snapshots.pop(0)
        return self.snapshots[0]

    def get_pose(self):
        return (0.0, 0.0, 0.0, True)

    def calibrate_basepoint(self, *, disarmed):
        assert disarmed


def snapshot(z_m):
    return {"position_m": (0.0, 0.0, z_m)}


def make_nav(source="lio", snapshots=None, agl=120.0):
    fc = NS(HOLD_POS_MODE=1, PROGRAM_MODE=2, sent=[])
    fc.state = NS(mode=NS(value=1), unlock=NS(value=True), alt_add=NS(value=agl),
                  update_event=NS(wait=lambda _timeout: True, clear=lambda: None),
                  is_fresh=lambda _age: True)
    fc.send_realtime_control_data = lambda *values: fc.sent.append(values)
    nav = Navigation(fc=fc, height_source=source,
                     lio_pose_provider=Provider(snapshots or [snapshot(1.5)]))
    nav.running = True
    nav.keep_height_flag = True
    return nav, fc


class RecordingPID:
    def __init__(self, output):
        self.output = output
        self.setpoint = 150.0
        self.inputs = []
        self.modes = []

    def set_auto_mode(self, enabled, **_kwargs):
        self.modes.append(enabled)

    def __call__(self, value):
        self.inputs.append(value)
        return self.output


def test_default_source_and_invalid_source():
    _, fc = make_nav(source="fc_laser")
    nav = Navigation(fc=fc, lio_pose_provider=Provider([snapshot(1.5)]))
    assert nav.height_source == "fc_laser"
    assert nav._get_control_height() == 120.0
    assert nav.current_height_agl == 120.0
    with pytest.raises(ValueError, match="height_source"):
        Navigation(fc=fc, height_source="unknown",
                   lio_pose_provider=Provider([snapshot(0.0)]))


def test_lio_pid_ignores_agl_step_and_zeros_on_lio_loss():
    nav, fc = make_nav(snapshots=[snapshot(1.5), None], agl=120.0)
    pid = RecordingPID(11)
    nav.height_pid = pid
    fc.send_realtime_control_data = lambda *values: (
        fc.sent.append(values), setattr(nav, "running", len(fc.sent) < 2))
    nav._keep_height_task()
    assert pid.inputs == [150.0]
    assert nav.current_height == nav.current_height_lio == 150.0
    assert nav.current_height_agl == 120.0
    assert [frame[2] for frame in fc.sent] == [11, 0]
    assert pid.modes == [True, False]
    assert nav._height_updated_at == 0.0


def test_default_laser_pid_uses_agl():
    nav, fc = make_nav(source="fc_laser", snapshots=[None], agl=80.0)
    pid = RecordingPID(7)
    nav.height_pid = pid
    fc.send_realtime_control_data = lambda *values: (
        fc.sent.append(values), setattr(nav, "running", False))
    nav._keep_height_task()
    assert pid.inputs == [80.0]
    assert nav.current_height == 80.0
    assert fc.sent[0][2] == 7


def test_fc_telemetry_timeout_disables_height_pid_and_zeros_z():
    nav, fc = make_nav()
    pid = RecordingPID(7)
    nav.height_pid = pid
    fc.state.update_event.wait = lambda _timeout: False
    fc.send_realtime_control_data = lambda *values: (
        fc.sent.append(values), setattr(nav, "running", False))
    nav._keep_height_task()
    assert pid.modes == [False]
    assert fc.sent[0][2] == 0


def test_navigation_pose_loss_clears_previous_lio_vertical_command():
    nav, fc = make_nav()
    nav.lio_pose.get_pose = lambda: None
    nav.navigation_flag = True
    nav._realtime_control_data_in_xyzYaw = [0, 0, 12, 0]
    fc.send_realtime_control_data = lambda *values: (
        fc.sent.append(values), setattr(nav, "running", len(fc.sent) < 2))
    nav._navigation_task()
    assert fc.sent[0][2] == 0
    assert nav._realtime_control_data_in_xyzYaw[2] == 0
    assert nav._height_updated_at == 0.0


def test_stale_lio_cannot_satisfy_height_wait():
    nav, _ = make_nav(snapshots=[None])
    nav.current_height = nav.height_pid.setpoint = 150.0
    nav._height_updated_at = time.monotonic()
    assert nav.wait_for_height(timeout=0) is False


def test_latest_lio_z_must_also_match_height_target():
    nav, _ = make_nav(snapshots=[snapshot(1.2)])
    nav.current_height = nav.height_pid.setpoint = 150.0
    nav._height_updated_at = time.monotonic()
    assert nav.wait_for_height(timeout=0) is False


def test_calibration_initializes_startup_local_z():
    nav, fc = make_nav(snapshots=[snapshot(0.0)])
    fc.state.unlock.value = False
    nav.calibrate_basepoint(wait=False)
    assert nav.current_height_lio == nav.current_height == 0.0
    assert nav.height_is_fresh()


def test_restore_hover_uses_lio_z_instead_of_agl():
    nav, fc = make_nav(agl=70.0)
    nav.pose_is_fresh = Mock(return_value=True)
    nav.direct_set_waypoint = Mock()
    nav.switch_pid = Mock()
    assert nav._stop_velocity_override(restore_hover=True, zero_flush_frames=1)
    assert nav.height_pid.setpoint == 150.0
    assert nav.current_height_lio == 150.0
    assert nav.current_height_agl == 0.0


def test_lio_first_takeoff_height_failure_stops_before_navigation(monkeypatch):
    nav, fc = make_nav()
    nav.navigation_to_waypoint = Mock()
    nav.wait_for_height = Mock(return_value=False)
    fc.set_flight_mode = Mock()
    fc.unlock = Mock()
    fc.take_off = Mock()
    fc.wait_for_takeoff_done = Mock(return_value=True)
    monkeypatch.setitem(NAV_NAMESPACE, "time", NS(
        sleep=lambda _seconds: None, monotonic=time.monotonic,
        perf_counter=time.perf_counter))
    with pytest.raises(RuntimeError, match="initial takeoff height"):
        nav.pointing_takeoff((0, 0), target_height=150)
    nav.navigation_to_waypoint.assert_not_called()


def test_landing_approach_uses_control_target_but_touchdown_uses_agl(monkeypatch):
    nav, fc = make_nav(agl=5.0)
    nav.navigation_to_waypoint = Mock()
    nav.wait_for_waypoint = Mock(return_value=True)
    nav.wait_for_height = Mock(return_value=True)
    nav.pose_is_fresh = Mock(return_value=True)
    nav.direct_set_waypoint = Mock()
    nav.set_height = Mock()
    nav.switch_pid = Mock()
    fc.set_flight_mode = Mock()
    fc.stablize = Mock()
    fc.land = Mock()
    fc.wait_for_lock = Mock(return_value=False)
    fc.lock = Mock()
    monkeypatch.setitem(NAV_NAMESPACE, "time", NS(
        sleep=lambda _seconds: None, perf_counter=time.perf_counter,
        monotonic=time.monotonic))
    assert nav.pointing_landing((0, 0))
    nav.set_height.assert_called_once_with(35.0)
    fc.lock.assert_called_once()


def test_landing_refuses_force_lock_without_fresh_agl(monkeypatch):
    nav, fc = make_nav(agl=5.0)
    nav.navigation_to_waypoint = Mock()
    nav.wait_for_waypoint = Mock(return_value=True)
    nav.wait_for_height = Mock(return_value=True)
    nav.pose_is_fresh = Mock(return_value=True)
    nav.direct_set_waypoint = Mock()
    nav.set_height = Mock()
    nav.switch_pid = Mock()
    fc.set_flight_mode = Mock()
    fc.stablize = Mock()
    fc.land = Mock()
    fc.lock = Mock()
    fc.state.is_fresh = lambda _age: False
    clock = NS(now=0.0)

    def sleep(seconds):
        clock.now += seconds

    monkeypatch.setitem(NAV_NAMESPACE, "time", NS(
        sleep=sleep, perf_counter=lambda: clock.now, monotonic=lambda: clock.now))
    assert not nav.pointing_landing((0, 0), touchdown_timeout=1)
    fc.lock.assert_not_called()


def test_visual_offset_uses_fixed_target_ground_heights():
    heights = []
    namespace = {
        "time": time, "logger": Mock(), "MANDATORY_COLOR": "yellow",
        "MANDATORY_DROP_HEIGHT": 100.0, "MANDATORY_TARGET_GROUND_HEIGHT_CM": 30.0,
        "FREE_DROP_HEIGHT": 80.0, "FREE_TARGET_GROUND_HEIGHT_CM": 0.0,
        "LOW_CALIBRATION_TIMEOUT": 0.0, "LOW_CALIBRATION_LOG_PERIOD_S": 0.5,
        "payload_target_offset_px": lambda _number, height, _frame: (
            heights.append(height) or (0.0, 0.0)),
    }
    Mission = extracted_class(RESCUE_SOURCE, "Mission", {"_calibrate_low"}, namespace)
    mission = object.__new__(Mission)
    mission.vision = NS(frame_size=(640, 480))
    mission.navi = NS(current_height=999.0, stop_move=Mock())
    assert not mission._calibrate_low(NS(color="red", target_id="free"), False, 1)
    assert not mission._calibrate_low(NS(color="yellow", target_id="mandatory"), False, 2)
    assert heights == [80.0, 70.0]
