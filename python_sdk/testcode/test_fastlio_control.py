"""Service restart tests with stub FC/navigation/systemctl; no hardware."""
import importlib.util
import json
from contextlib import nullcontext
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


def setup_dds_audit(monkeypatch, report):
    clock = NS(now=0.0)
    commands = []
    monkeypatch.setattr(control.os, "geteuid", lambda: 0, raising=False)
    monkeypatch.setattr(control.time, "monotonic", lambda: clock.now)
    monkeypatch.setattr(control.time, "sleep", lambda seconds: setattr(clock, "now", clock.now + seconds))

    def run(command):
        commands.append(command)
        return json.dumps(report())

    monkeypatch.setattr(control, "_run_recovery_command", run)
    return clock, commands


def test_dds_wait_allows_exiting_owner_but_requires_empty_audit(monkeypatch):
    reports = iter([
        {"owners": [{"pid": 1395, "name": "python3"}], "unreadable": []},
        {"owners": [{"pid": 1395, "name": "python3"}], "unreadable": []},
        {"owners": [], "unreadable": []},
    ])
    clock, commands = setup_dds_audit(monkeypatch, lambda: next(reports))
    guard_calls = []
    control._assert_dds_released(timeout=5, ground_check=lambda: guard_calls.append(clock.now))
    assert len(commands) == len(guard_calls) == 3
    assert clock.now == pytest.approx(0.2)


@pytest.mark.parametrize("report", [
    {"owners": [{"pid": 1395, "name": "python3"}], "unreadable": []},
    {"owners": [], "unreadable": [1395]},
])
def test_dds_wait_persistent_or_unreadable_owner_still_fails(monkeypatch, report):
    clock, commands = setup_dds_audit(monkeypatch, lambda: report)
    with pytest.raises(RuntimeError, match="DDS users remain.*1395"):
        control._assert_dds_released(timeout=0.25)
    assert clock.now == pytest.approx(0.25)
    assert len(commands) == 4


def test_default_dds_audit_remains_immediate(monkeypatch):
    clock, commands = setup_dds_audit(
        monkeypatch, lambda: {"owners": [{"pid": 1395}], "unreadable": []})
    with pytest.raises(RuntimeError, match="DDS users remain"):
        control._assert_dds_released()
    assert clock.now == 0 and len(commands) == 1


@pytest.mark.parametrize("failure", ["armed", "stale", "disconnected", "stopped"])
def test_dds_wait_aborts_if_ground_guard_changes(monkeypatch, failure):
    fc, nav, _ = setup_task(monkeypatch)
    _, commands = setup_dds_audit(
        monkeypatch, lambda: {"owners": [{"pid": 1395}], "unreadable": []})
    checks = []

    def guard():
        if checks:
            if failure == "armed": fc.state.unlock.value = True
            if failure == "stale": fc.state.is_fresh = lambda age: False
            if failure == "disconnected": fc.connected = False
            if failure == "stopped": nav.stop_event = NS(is_set=lambda: True)
        checks.append(True)
        control.require_ground_restart(fc, nav)

    with pytest.raises(RuntimeError):
        control._assert_dds_released(timeout=5, ground_check=guard)
    assert len(checks) == 2 and len(commands) == 1


def test_dds_audit_command_failure_is_not_retried(monkeypatch):
    _, commands = setup_dds_audit(monkeypatch, lambda: {})

    def fail(command):
        commands.append(command)
        raise RuntimeError("audit permission denied")

    monkeypatch.setattr(control, "_run_recovery_command", fail)
    with pytest.raises(RuntimeError, match="audit permission denied"):
        control._assert_dds_released(timeout=5)
    assert len(commands) == 1


@pytest.mark.parametrize("persistent", [False, True])
def test_full_recovery_cannot_clean_or_start_before_dds_release(monkeypatch, persistent):
    fc, nav, events = setup_task(monkeypatch)
    count = NS(audits=0, stopped=False)

    def report():
        count.audits += 1
        blocked = persistent or count.audits == 1
        return {"owners": [{"pid": 1395}] if blocked else [], "unreadable": []}

    _, commands = setup_dds_audit(monkeypatch, report)
    audit_run = control._run_recovery_command

    def recovery_command(command):
        if "--released-task-pid" in command:
            return audit_run(command)
        events.append(tuple(command))
        return ""

    def systemctl(*args, **kwargs):
        events.append(args)
        if args[0] == "stop": count.stopped = True

    def identity(service):
        started = ("start", service) in events
        active = not count.stopped or started
        return dict(ActiveState="active" if active else "inactive",
                    MainPID="2" if started else "1" if active else "0",
                    InvocationID="new" if started else "old")

    runner = NS(validate_localization_recovery=lambda listener: None,
                release_localization_context=lambda listener: events.append("release"),
                restore_localization_context=lambda *callbacks: events.append("restore") or object())
    nav.lio_pose.on_odometry = nav.lio_pose.on_health = nav.lio_pose.on_imu = lambda msg: None
    monkeypatch.setattr(control, "_dds_recovery_lock", nullcontext)
    monkeypatch.setattr(control, "_localization_runner", lambda: runner)
    monkeypatch.setattr(control, "_service_identity", identity)
    monkeypatch.setattr(control, "_systemctl", systemctl)
    monkeypatch.setattr(control, "_run_recovery_command", recovery_command)
    monkeypatch.setattr(control.shutil, "which", lambda tool: "/usr/bin/" + tool)
    if persistent:
        with pytest.raises(RuntimeError, match="DDS users remain"):
            control.restart_localization_for_task(fc, nav)
        assert not any(isinstance(e, tuple) and e[0] in ("start", "bash") for e in events)
        assert "calibrate" not in events and events[-2:] == ["invalidate", "restore"]
    else:
        result = control.restart_localization_for_task(fc, nav)
        assert not result["needs_calibration"] and result["dds_rebuilt"]
        assert len(commands) == 3  # Two before SHM clean, one after.
        cleaner = ("bash", "/usr/bin/fastdds", "shm", "clean")
        assert events.index(cleaner) < events.index(("start", control.DRIVER_SERVICE))
        assert events.index(("start", control.FASTLIO_SERVICE)) < events.index("calibrate")
