"""Exercise Navigation's LIO control methods without importing hardware modules."""

import ast
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest


SOURCE = Path(__file__).resolve().parents[1] / "FlightController/Solutions/Navigation.py"
tree = ast.parse(SOURCE.read_text(encoding="utf-8"))
source_class = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "Navigation")
methods = {"calibrate_basepoint", "set_navigation_state", "switch_navigation_mode",
           "update_realtime_control", "_navigation_task"}
selected = ast.ClassDef(name="Navigation", bases=[], keywords=[],
                        body=[node for node in source_class.body
                              if isinstance(node, ast.FunctionDef) and node.name in methods],
                        decorator_list=[])
helpers = [node for node in tree.body if isinstance(node, ast.FunctionDef)
           and node.name in {"_shortest_yaw_error", "_world_to_body_velocity"}]
future_annotations = ast.ImportFrom(module="__future__", names=[ast.alias(name="annotations")], level=0)
module = ast.fix_missing_locations(ast.Module(body=[future_annotations] + helpers + [selected], type_ignores=[]))


class Logger:
    def __getattr__(self, name):
        return lambda *args, **kwargs: None


namespace = {"np": np, "time": time, "logger": Logger(), "logger_dbg": Logger(),
             "NAVIGATION_CONTROL_STALE_TIMEOUT": 0.30,
             "NAVIGATION_LOOP_INTERVAL": 0.005,
             "LIO_CALIBRATION_WAIT_SECONDS": 3.0}
exec(compile(module, str(SOURCE), "exec"), namespace)
Navigation = namespace["Navigation"]


class Provider:
    def __init__(self, poses):
        self.poses = list(poses)
        self.calibrations = 0

    def get_pose(self):
        if len(self.poses) > 1:
            return self.poses.pop(0)
        return self.poses[0]

    def calibrate_basepoint(self, *, disarmed):
        assert disarmed
        self.calibrations += 1


class PID:
    def __init__(self, output):
        self.output = output
        self.auto_mode = False
        self.mode_changes = []

    def __call__(self, value):
        return self.output if self.auto_mode else None

    def set_auto_mode(self, *args, **kwargs):
        self.auto_mode = args[0]
        self.mode_changes.append(args[0])


def navigation(provider):
    nav = object.__new__(Navigation)
    nav.lio_pose = provider
    nav.navigation_flag = False
    nav.keep_height_flag = False
    nav.running = True
    nav.stop_event = None
    nav._control_lock = threading.Lock()
    nav.traj_running_event = threading.Event()
    nav._realtime_control_data_in_xyzYaw = [0, 0, 0, 0]
    nav._velocity_override_active = False
    nav._navigation_control_updated_at = time.monotonic()
    nav._legacy_mode_warned = False
    nav.fc = SimpleNamespace(HOLD_POS_MODE=1,
                             state=SimpleNamespace(mode=SimpleNamespace(value=1),
                                                   unlock=SimpleNamespace(value=True)))
    nav.fc.sent = []
    nav.fc.send_realtime_control_data = lambda *control: nav.fc.sent.append(control)
    nav.navi_x_pid = PID(12)
    nav.navi_y_pid = PID(-7)
    nav.yaw_pid = PID(4)
    nav.yaw_target = 10.0
    nav._yaw_direction_hint = 0
    return nav


def test_missing_fresh_pose_rejects_navigation():
    nav = navigation(Provider([None]))
    with pytest.raises(RuntimeError, match="fresh LIO pose"):
        nav.set_navigation_state(True)
    assert nav.navigation_flag is False
    assert nav.fc.sent == []


def test_basepoint_calibration_delegates_to_lio():
    provider = Provider([(0, 0, 0, True)])
    nav = navigation(provider)
    nav.fc.state.unlock.value = False
    assert np.array_equal(nav.calibrate_basepoint(wait=False), [0, 0])
    assert provider.calibrations == 1
    assert nav.lio_pose.get_pose() is not None


def test_waited_calibration_allows_new_two_second_stationary_window(monkeypatch):
    class Clock:
        value = 0.0

        def monotonic(self):
            return self.value

        def sleep(self, duration):
            self.value += duration

    clock = Clock()

    class DelayedProvider(Provider):
        def calibrate_basepoint(self, *, disarmed):
            if clock.value < 2.1:
                raise RuntimeError("Stationary window incomplete")
            super().calibrate_basepoint(disarmed=disarmed)

    nav = navigation(DelayedProvider([(0, 0, 0, True)]))
    nav.fc.state.unlock.value = False
    monkeypatch.setitem(namespace, "time", clock)
    assert np.array_equal(nav.calibrate_basepoint(wait=True), [0, 0])
    assert 2.1 <= clock.value < 3.0


def test_stale_pose_zeros_previous_horizontal_and_yaw_commands(monkeypatch):
    nav = navigation(Provider([(0, 0, 0, True), None]))
    nav.navigation_flag = True
    monkeypatch.setattr(namespace["time"], "sleep",
                        lambda seconds: setattr(nav, "running", False) if seconds == 0.05 else None)
    nav._navigation_task()
    assert any(x != 0 or y != 0 or yaw != 0 for x, y, _, yaw in nav.fc.sent[:-1])
    assert nav.fc.sent[-1][0] == 0
    assert nav.fc.sent[-1][1] == 0
    assert nav.fc.sent[-1][3] == 0
    assert nav._realtime_control_data_in_xyzYaw[0] == 0
    assert nav._realtime_control_data_in_xyzYaw[1] == 0
    assert nav._realtime_control_data_in_xyzYaw[3] == 0


def test_initial_valid_pose_enables_initially_disabled_pids(monkeypatch):
    nav = navigation(Provider([(0, 0, 0, True)]))
    nav.navigation_flag = True
    monkeypatch.setattr(namespace["time"], "sleep", lambda seconds: None)
    nav.fc.send_realtime_control_data = lambda *control: (
        nav.fc.sent.append(control), setattr(nav, "running", False))
    nav._navigation_task()
    assert nav.navi_x_pid.auto_mode and nav.navi_y_pid.auto_mode and nav.yaw_pid.auto_mode
    assert nav.fc.sent[-1] == (12, -7, 0, 4)


def test_stale_recovery_reenables_pids_only_after_explicit_navigation_enable(monkeypatch):
    nav = navigation(Provider([(0, 0, 0, True), None, (0, 0, 0, True)]))
    nav.navigation_flag = True
    def sleep(seconds):
        if seconds == 0.05:
            assert not nav.navigation_flag
            assert not nav.navi_x_pid.auto_mode
            nav.set_navigation_state(True)
    monkeypatch.setattr(namespace["time"], "sleep", sleep)
    def send(*control):
        nav.fc.sent.append(control)
        if len(nav.fc.sent) == 3:
            nav.running = False
    nav.fc.send_realtime_control_data = send
    nav._navigation_task()
    assert nav.navi_x_pid.mode_changes == [True, False, True]
    assert nav.fc.sent[1] == (0, 0, 0, 0)
    assert nav.fc.sent[-1] == (12, -7, 0, 4)


def test_paused_navigation_yields_and_does_not_enable_pids(monkeypatch):
    nav = navigation(Provider([(0, 0, 0, True)]))
    sleeps = []
    def sleep(seconds):
        sleeps.append(seconds)
        if len(sleeps) == 3:
            nav.running = False
    monkeypatch.setattr(namespace["time"], "sleep", sleep)
    nav._navigation_task()
    assert sleeps == [0.005, 0.005, 0.005]
    assert not nav.navi_x_pid.auto_mode
    assert not nav.fc.sent


def test_legacy_mode_cannot_switch_backend():
    nav = navigation(Provider([(0, 0, 0, True)]))
    nav.switch_navigation_mode("fusion")
    assert nav._navigation_mode == "lio"
