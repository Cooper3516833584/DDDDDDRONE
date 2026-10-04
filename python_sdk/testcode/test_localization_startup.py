"""Pure fakes and /proc fixtures; no server import, ROS context or hardware."""

import importlib.util
from contextlib import nullcontext
from pathlib import Path
from types import SimpleNamespace as NS
import sys

import pytest

SDK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SDK))
import localization_startup as startup


def fixture_startup(monkeypatch, reports):
    fc = NS(connected=True, state=NS(is_fresh=lambda age: True, unlock=NS(value=False)))
    calls = []
    monkeypatch.setattr(startup.sys, "platform", "linux")
    monkeypatch.setattr(startup, "ipc_cleanup_disabled", lambda: True)
    reports = iter(reports)
    monkeypatch.setattr(startup, "probe_localization", lambda: next(reports))
    monkeypatch.setattr(startup.control, "_dds_recovery_lock", nullcontext)
    monkeypatch.setattr(startup, "audit_startup_owners", lambda: {
        "owners": [{"deleted_shm": ["/dev/shm/fastrtps_test"]}], "unreadable": []})
    monkeypatch.setattr(startup.control, "_service_identity", lambda service: {})

    def rebuild(before, guard):
        guard()
        calls.append("rebuild")
        return {}

    monkeypatch.setattr(startup.control, "_rebuild_localization_services", rebuild)
    return fc, calls


def test_healthy_start_does_not_restart_services(monkeypatch):
    fc, calls = fixture_startup(monkeypatch, [{"ready": True}])
    assert startup.prepare_localization_at_startup(fc, lambda: False)["repaired"] is False
    assert calls == []


@pytest.mark.parametrize("failure", ["armed", "stale", "disconnected", "mission", "ipc"])
def test_unsafe_start_cannot_probe_or_rebuild(monkeypatch, failure):
    fc, calls = fixture_startup(monkeypatch, [])
    if failure == "armed": fc.state.unlock.value = True
    if failure == "stale": fc.state.is_fresh = lambda age: False
    if failure == "disconnected": fc.connected = False
    if failure == "ipc": monkeypatch.setattr(startup, "ipc_cleanup_disabled", lambda: False)
    with pytest.raises(RuntimeError):
        startup.prepare_localization_at_startup(fc, lambda: failure == "mission")
    assert calls == []


@pytest.mark.parametrize("ready", [True, False])
def test_confirmed_shm_loss_rebuilds_once_and_requires_new_flow(monkeypatch, ready):
    fc, calls = fixture_startup(monkeypatch, [{"ready": False}, {"ready": ready}])
    if ready:
        assert startup.prepare_localization_at_startup(fc, lambda: False)["repaired"] is True
    else:
        with pytest.raises(RuntimeError, match="after one recovery"):
            startup.prepare_localization_at_startup(fc, lambda: False)
    assert calls == ["rebuild"]


def test_unexplained_no_data_cannot_trigger_blind_restart(monkeypatch):
    fc, calls = fixture_startup(monkeypatch, [{"ready": False}])
    monkeypatch.setattr(startup, "audit_startup_owners", lambda: {
        "owners": [{"deleted_shm": []}], "unreadable": []})
    with pytest.raises(RuntimeError, match="without deleted DDS"):
        startup.prepare_localization_at_startup(fc, lambda: False)
    assert calls == []


def test_arm_change_during_probe_stops_recovery(monkeypatch):
    fc, calls = fixture_startup(monkeypatch, [])

    def probe():
        fc.state.unlock.value = True
        return {"ready": False}

    monkeypatch.setattr(startup, "probe_localization", probe)
    with pytest.raises(RuntimeError, match="armed"):
        startup.prepare_localization_at_startup(fc, lambda: False)
    assert calls == []


@pytest.mark.parametrize("foreign,unreadable", [(False, False), (True, False), (False, True)])
def test_foreign_or_unreadable_dds_owner_is_rejected_before_stop(monkeypatch, foreign, unreadable):
    import json
    group = "/system.slice/mid360s-driver.service"
    report = {"owners": [{"pid": 100, "name": "node", "deleted_shm": ["x"],
                          "cgroups": ["/user.slice/mission" if foreign else group]}],
              "unreadable": [200] if unreadable else []}
    monkeypatch.setattr(startup.os, "geteuid", lambda: 0, raising=False)
    monkeypatch.setattr(startup.control, "_run_recovery_command", lambda cmd: json.dumps(report))
    monkeypatch.setattr(startup.control, "_systemctl", lambda *args: group + "\n")
    if foreign or unreadable:
        with pytest.raises(RuntimeError, match="other ROS users"):
            startup.audit_startup_owners()
    else:
        assert startup.audit_startup_owners() == report


@pytest.mark.parametrize("text,expected", [
    ("[Login]\n#RemoveIPC=yes\n", False),
    ("[Login]\nRemoveIPC=yes\n[Login]\nRemoveIPC=no\n", True),
    ("[Login]\nRemoveIPC=no\n[Login]\nRemoveIPC=yes\n", False),
    ("[Other]\nRemoveIPC=no\n", False),
])
def test_ipc_policy_uses_effective_section_and_last_setting(monkeypatch, text, expected):
    monkeypatch.setattr(startup.control, "_run_recovery_command", lambda cmd: text)
    assert startup.ipc_cleanup_disabled() is expected


def test_audit_details_identify_deleted_dds_without_changing_default_schema(tmp_path):
    spec = importlib.util.spec_from_file_location("dds_audit_test", SDK / "localization_dds_audit.py")
    audit = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(audit)
    proc = tmp_path / "123"
    proc.mkdir()
    (proc / "maps").write_text("000-fff rw-s 0 00:00 1 /dev/shm/fastrtps_test (deleted)\n")
    (proc / "comm").write_text("node\n")
    (proc / "cgroup").write_text("0::/system.slice/mid360s-driver.service\n")
    assert audit.inspect_dds_owners(999, tmp_path) == {
        "owners": [{"pid": 123, "name": "node"}], "unreadable": []}
    report = audit.inspect_dds_owners(999, tmp_path, details=True)
    assert report["owners"][0]["deleted_shm"] == ["/dev/shm/fastrtps_test"]
    assert report["owners"][0]["cgroups"] == ["/system.slice/mid360s-driver.service"]


@pytest.mark.parametrize("setup_failure", [False, True])
def test_passive_child_requires_real_flow_and_releases_context(monkeypatch, setup_failure):
    callbacks, events = {}, []
    clock = NS(now=0.0)

    class FakeNode:
        def __init__(self, name):
            events.append("node")

        def create_subscription(self, kind, topic, callback, qos, **kw):
            if setup_failure:
                raise RuntimeError("subscription failed")
            assert kw == ({"raw": True} if topic == "/livox/lidar" else {})
            callbacks[topic] = callback
            return object()

        def destroy_node(self):
            events.append("destroy")

    def spin(node, timeout_sec):
        clock.now += 0.02
        for callback in callbacks.values():
            callback(object())

    modules = {
        "rclpy": NS(init=lambda: events.append("init"), spin_once=spin,
                    shutdown=lambda: events.append("shutdown")),
        "rclpy.node": NS(Node=FakeNode),
        "rclpy.qos": NS(qos_profile_sensor_data=object()),
        "livox_ros_driver2.msg": NS(CustomMsg=object),
        "nav_msgs.msg": NS(Odometry=object),
        "sensor_msgs.msg": NS(Imu=object),
        "fast_lio.msg": NS(LioHealth=object),
    }
    for key, module in modules.items():
        monkeypatch.setitem(sys.modules, key, module)
    pose = NS(on_imu=lambda msg: None, on_health=lambda msg: None,
              on_odometry=lambda msg: None, _healthy=lambda now: True)
    monkeypatch.setattr(startup.importlib.util, "spec_from_file_location",
                        lambda *args: NS(loader=NS(exec_module=lambda module: None)))
    monkeypatch.setattr(startup.importlib.util, "module_from_spec",
                        lambda spec: NS(LioPoseProvider=lambda: pose))
    monkeypatch.setattr(startup.time, "monotonic", lambda: clock.now)
    if setup_failure:
        with pytest.raises(RuntimeError, match="subscription failed"):
            startup.passive_probe(1)
    else:
        report = startup.passive_probe(1)
        assert report["ready"] and list(report["counts"].values()) == [3] * 5
    assert events == ["init", "node", "destroy", "shutdown"]


def test_server_startup_recovery_precedes_client_and_mission_acceptance():
    import ast
    tree = ast.parse((SDK / "server_ros.py").read_text(encoding="utf-8"))
    lines = {}
    for node in ast.walk(tree):
        if isinstance(node, ast.Call):
            name = (node.func.id if isinstance(node.func, ast.Name) else
                    node.func.attr if isinstance(node.func, ast.Attribute) else "")
            if name in ("start_listen_serial", "prepare_localization_at_startup",
                        "register_report_callback", "serve_forever"):
                lines[name] = node.lineno
    assert lines["start_listen_serial"] < lines["prepare_localization_at_startup"]
    assert lines["prepare_localization_at_startup"] < lines["register_report_callback"]
    assert lines["prepare_localization_at_startup"] < lines["serve_forever"]
