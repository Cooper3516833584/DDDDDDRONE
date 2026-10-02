"""Execute server callbacks with fakes, without importing hardware startup."""

import ast
import subprocess
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest


SOURCE = Path(__file__).resolve().parents[1] / "server_ros.py"
TREE = ast.parse(SOURCE.read_text(encoding="utf-8"))


def server_namespace(active=True, topics=None, mount=True):
    functions = {"systemd_service_active", "selected_localization_services",
                 "report_localization_status", "localization_service_log", "callback"}
    nodes = [node for node in TREE.body if
             isinstance(node, ast.FunctionDef) and node.name in functions or
             isinstance(node, ast.Assign) and any(
                 isinstance(target, ast.Name) and target.id in
                 ("LOCALIZATION_SERVICES", "REQUIRED_LIO_TOPICS") for target in node.targets)]
    rm = Mock()
    rm.get_running_topics.return_value = topics if topics is not None else [
        "/livox/lidar", "/livox/imu", "/Odometry", "/Odometry_highrate", "/LioHealth"]
    process = Mock()
    process.run.return_value = SimpleNamespace(returncode=0 if active else 3)
    process.TimeoutExpired = subprocess.TimeoutExpired

    def config_check(*, require_mount=False):
        if require_mount and not mount:
            raise RuntimeError("Reviewed mount required")

    ns = {"subprocess": process, "logger": Mock(), "scr": Mock(), "rm": rm,
          "require_production_localization": Mock(side_effect=config_check),
          "mis_tmux": Mock(session_running=False), "PATH": "/sdk",
          "os": SimpleNamespace(path=SimpleNamespace(exists=lambda path: True)),
          "time": Mock(), "sending_log": False, "mis_num": 0,
          "PYTHON_EXCUTEABLE": "python3"}
    exec(compile(ast.fix_missing_locations(ast.Module(body=nodes, type_ignores=[])),
                 str(SOURCE), "exec"), ns)
    ns["missing_lio_topics"] = lambda available: [
        topic for topic in ns["REQUIRED_LIO_TOPICS"] if topic not in available]
    return ns


@pytest.mark.parametrize("active", [True, False])
@pytest.mark.parametrize("index", [0, 1, 2])
def test_ros_boot_never_launches_a_second_owner(active, index):
    ns = server_namespace(active=active, mount=False)
    ns["callback"](f"ros_boot={index}")
    ns["rm"].launch_package.assert_not_called()
    ns["rm"].run_package.assert_not_called()
    calls = ns["subprocess"].run.call_args_list
    assert calls
    assert all(call.args[0][:3] == ["systemctl", "is-active", "--quiet"] for call in calls)
    assert ns["scr"].set_widget_value.called
    ns["logger"].exception.assert_not_called()
    if index != 1:
        ns["require_production_localization"].assert_called_once_with()


def test_inactive_service_reports_failure_without_sudo_or_fallback():
    ns = server_namespace(active=False)
    ns["callback"]("ros_boot=0")
    ns["scr"].set_widget_value.assert_called_with("main_info.txt", '"定位服务未启动"')
    ns["rm"].get_running_topics.assert_not_called()


def test_service_query_timeout_fails_closed():
    ns = server_namespace()
    ns["subprocess"].run.side_effect = subprocess.TimeoutExpired("systemctl", 2)
    assert not ns["systemd_service_active"]("mid360s-driver.service")


def test_active_services_with_missing_topics_are_not_reported_ready():
    ns = server_namespace(topics=["/livox/lidar"])
    ns["callback"]("ros_boot=0")
    ns["scr"].set_widget_value.assert_called_with("main_info.txt", '"定位话题未就绪"')


@pytest.mark.parametrize("mount,topics", [(False, None), (True, [])])
def test_mission_still_requires_reviewed_mount_and_required_topics(mount, topics):
    ns = server_namespace(mount=mount, topics=topics)
    ns["callback"]("mis_boot=0")
    ns["require_production_localization"].assert_called_once_with(require_mount=True)
    ns["mis_tmux"].new_session.assert_not_called()


def test_screen_cannot_stop_systemd_localization():
    ns = server_namespace()
    ns["callback"]("ros_kill=0")
    ns["rm"].kill_package.assert_not_called()
    ns["subprocess"].run.assert_not_called()


def test_mission_refuses_inactive_owner_even_with_advertised_topics():
    ns = server_namespace(active=False)
    ns["callback"]("mis_boot=0")
    ns["mis_tmux"].new_session.assert_not_called()
    ns["scr"].set_widget_value.assert_called_with("main_info.txt", '"定位服务未启动"')


def test_production_server_has_no_rosmanager_lifecycle_calls():
    forbidden = {"launch_package", "run_package", "kill_package", "get_log"}
    assert not [node for node in ast.walk(TREE) if isinstance(node, ast.Call)
                and isinstance(node.func, ast.Attribute) and node.func.attr in forbidden
                and isinstance(node.func.value, ast.Name) and node.func.value.id == "rm"]
