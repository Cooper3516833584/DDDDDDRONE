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
             "NAVIGATION_CONTROL_STALE_TIMEOUT": 0.30}
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

    def calibrate_basepoint(self):
        self.calibrations += 1


class PID:
    def __init__(self, output):
        self.output = output

    def __call__(self, value):
        return self.output

    def set_auto_mode(self, *args, **kwargs):
        pass


def navigation(provider):
    nav = object.__new__(Navigation)
    nav.lio_pose = provider
    nav.navigation_flag = False
    nav.keep_height_flag = False
    nav.running = True
    nav.stop_event = None
    nav._control_lock = threading.Lock()
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
    assert np.array_equal(nav.calibrate_basepoint(wait=False), [0, 0])
    assert provider.calibrations == 1
    assert nav.lio_pose.get_pose() is not None


def test_stale_pose_zeros_previous_horizontal_and_yaw_commands(monkeypatch):
    nav = navigation(Provider([(0, 0, 0, True), None]))
    nav.navigation_flag = True
    monkeypatch.setattr(namespace["time"], "sleep", lambda seconds: setattr(nav, "running", False))
    nav._navigation_task()
    assert any(x != 0 or y != 0 or yaw != 0 for x, y, _, yaw in nav.fc.sent[:-1])
    assert nav.fc.sent[-1][0] == 0
    assert nav.fc.sent[-1][1] == 0
    assert nav.fc.sent[-1][3] == 0
    assert nav._realtime_control_data_in_xyzYaw[0] == 0
    assert nav._realtime_control_data_in_xyzYaw[1] == 0
    assert nav._realtime_control_data_in_xyzYaw[3] == 0


def test_legacy_mode_cannot_switch_backend():
    nav = navigation(Provider([(0, 0, 0, True)]))
    nav.switch_navigation_mode("fusion")
    assert nav._navigation_mode == "lio"
