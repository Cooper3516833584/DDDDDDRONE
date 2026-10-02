"""Service restart tests with stub FC/navigation/systemctl; no hardware."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

source = Path(__file__).resolve().parents[1] / "fastlio_control.py"
spec = importlib.util.spec_from_file_location("fastlio_control_test", source)
control = importlib.util.module_from_spec(spec)
spec.loader.exec_module(control)


def setup_task(monkeypatch):
    monkeypatch.setattr(control.sys, "platform", "linux")
    events = []
    fc = NS(connected=True, state=NS(is_fresh=lambda age: True, unlock=NS(value=False)))
    pose = NS(mount_reviewed=True, invalidate_for_restart=lambda: events.append("invalidate"),
              get_snapshot=lambda: {"stamp_ns": 1001})
    nav = NS(fc=fc, lio_pose=pose, navigation_flag=False, keep_height_flag=False,
             traj_running_event=NS(is_set=lambda: False), stop_event=None, _lio_listener=object(),
             calibrate_basepoint=lambda wait: events.append("calibrate"))
    identity = iter([dict(ActiveState="active", MainPID="1", InvocationID="old"),
                     dict(ActiveState="active", MainPID="2", InvocationID="new")])
    monkeypatch.setattr(control, "_service_identity", lambda: next(identity))
    monkeypatch.setattr(control, "_systemctl", lambda *args, **kw: events.append(args))
    monkeypatch.setattr(control.time, "time_ns", lambda: 1000)
    return fc, nav, events


@pytest.mark.parametrize("failure", ["armed", "stale", "disconnected", "navigation", "trajectory", "stopped"])
def test_unsafe_restart_runs_no_commands(monkeypatch, failure):
    fc, nav, events = setup_task(monkeypatch)
    if failure == "armed": fc.state.unlock.value = True
    if failure == "stale": fc.state.is_fresh = lambda age: False
    if failure == "disconnected": fc.connected = False
    if failure == "navigation": nav.navigation_flag = True
    if failure == "trajectory": nav.traj_running_event.is_set = lambda: True
    if failure == "stopped": nav.stop_event = NS(is_set=lambda: True)
    with pytest.raises(RuntimeError):
        control.restart_fastlio_for_task(fc, nav)
    assert events == []


def test_restart_invalidates_before_only_fastlio_restart(monkeypatch):
    fc, nav, events = setup_task(monkeypatch)
    result = control.restart_fastlio_for_task(fc, nav)
    assert events == [("is-active", "--quiet", control.DRIVER_SERVICE), "invalidate",
                      ("restart", control.FASTLIO_SERVICE)]
    assert result["needs_calibration"] and result["invocation_id"] == "new"


def test_command_failure_keeps_pose_invalid_and_releases_lock(monkeypatch):
    fc, nav, events = setup_task(monkeypatch)
    def systemctl(*args, **kw):
        if args[0] == "restart": raise RuntimeError("permission denied")
    monkeypatch.setattr(control, "_systemctl", systemctl)
    with pytest.raises(RuntimeError, match="permission denied"):
        control.restart_fastlio_for_task(fc, nav)
    assert events == ["invalidate"]
    assert control._restart_lock.acquire(blocking=False)
    control._restart_lock.release()


def test_calibration_requires_new_map_source_stamp(monkeypatch):
    fc, nav, events = setup_task(monkeypatch)
    snapshots = iter([{"stamp_ns": 999}, {"stamp_ns": 1001}])
    nav.lio_pose.get_snapshot = lambda: next(snapshots)
    monkeypatch.setattr(control.time, "sleep", lambda seconds: None)
    result = control.restart_and_calibrate_fastlio(fc, nav)
    assert not result["needs_calibration"] and result["pose"]["stamp_ns"] == 1001
    assert events.count("calibrate") == 2


def test_timeout_revokes_any_partial_calibration(monkeypatch):
    fc, nav, events = setup_task(monkeypatch)
    nav.lio_pose.get_snapshot = lambda: None
    clocks = iter([0, 1])
    monkeypatch.setattr(control.time, "monotonic", lambda: next(clocks))
    with pytest.raises(RuntimeError, match="timed out"):
        control.restart_and_calibrate_fastlio(fc, nav, timeout=0.1)
    assert events[-1] == "invalidate"
