"""No ROS or GUI startup: check diagnostic frame transforms and pairing."""

import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace as NS

import pytest


source = Path(__file__).resolve().parents[1] / "lio_trajectory_viewer.py"
spec = importlib.util.spec_from_file_location("trajectory_viewer_test", source)
viewer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(viewer)


def messages(ns, xyz=(0, 0, 0), yaw=0, state=1, epoch=1):
    stamp = NS(sec=100, nanosec=ns)
    odom = NS(header=NS(stamp=stamp, frame_id="camera_init"), child_frame_id="imu",
              pose=NS(pose=NS(position=NS(x=xyz[0], y=xyz[1], z=xyz[2]),
                             orientation=NS(x=0, y=0, z=math.sin(yaw / 2), w=math.cos(yaw / 2)))))
    health = NS(state_stamp=stamp, epoch=epoch, state=state, gravity_o=NS(x=0, y=0, z=-9.81))
    return odom, health


def deliver(model, pair):
    model.receive(0, pair[0])
    model.receive(1, pair[1])


def test_startup_heading_and_three_dimensional_displacement(tmp_path, monkeypatch):
    monkeypatch.setattr(viewer.time, "time", lambda: 100.0)
    model = viewer.TrajectoryModel(tmp_path / "missing.json")
    deliver(model, messages(0, (10, 20, 3), yaw=math.pi / 2))
    assert model.snapshot()[1] == pytest.approx((0, 0, 0))
    deliver(model, messages(1000000, (9.5, 21.2, 3.3), yaw=math.pi / 2))
    assert model.snapshot()[1] == pytest.approx((1.2, 0.5, 0.3))
    assert model.snapshot()[3] == "LIDAR ORIGIN"


def test_exact_pairing_degraded_freeze_and_epoch_reset(tmp_path, monkeypatch):
    monkeypatch.setattr(viewer.time, "time", lambda: 100.0)
    model = viewer.TrajectoryModel(tmp_path / "missing.json")
    first = messages(0)
    model.receive(0, first[0])
    model.receive(1, messages(1000000)[1])
    assert model.snapshot()[1] is None
    model.receive(1, first[1])
    assert model.snapshot()[1] == pytest.approx((0, 0, 0))
    deliver(model, messages(2000000, (2, 0, 0), state=2))
    assert model.snapshot()[1] == pytest.approx((0, 0, 0))
    assert model.snapshot()[2] == "DEGRADED"
    deliver(model, messages(3000000, (10, 0, 0), epoch=2))
    assert model.snapshot()[1] == pytest.approx((0, 0, 0))
    assert len(model.snapshot()[0]) == 1


def test_stale_pair_and_manual_reset_do_not_reuse_old_origin(tmp_path, monkeypatch):
    monkeypatch.setattr(viewer.time, "time", lambda: 100.0)
    model = viewer.TrajectoryModel(tmp_path / "missing.json")
    deliver(model, messages(0))
    model.reset_origin()
    assert model.snapshot()[1] is None
    deliver(model, messages(1000000, (5, 6, 7)))
    assert model.snapshot()[1] == pytest.approx((0, 0, 0))
    monkeypatch.setattr(viewer.time, "time", lambda: 101.0)
    deliver(model, messages(2000000, (6, 6, 7)))
    assert model.snapshot()[2] == "STALE / CLOCK MISMATCH"
    assert model.snapshot()[1] == pytest.approx((0, 0, 0))


def test_vscode_bootstrap_sources_ros_and_uses_system_python(monkeypatch):
    monkeypatch.setattr(viewer.sys, "platform", "linux")
    monkeypatch.delenv("LIO_VIEWER_ROS_BOOTSTRAPPED", raising=False)
    monkeypatch.setattr(viewer.Path, "is_file", lambda path: True)
    calls = []

    def run(command, **kwargs):
        calls.append(command)
        return NS(stdout=b"ROS_DISTRO=humble\0PYTHONPATH=/ros/python\0")

    monkeypatch.setattr(viewer.subprocess, "run", run)
    executed = []
    monkeypatch.setattr(viewer.os, "execve", lambda executable, args, env: executed.append((executable, args, env)))
    viewer.prepare_ros_environment()
    assert calls[0][-2].replace("\\", "/") == "/opt/ros/humble/setup.bash"
    assert calls[0][-1].replace("\\", "/").endswith("ros2_ws/install/setup.bash")
    assert executed[0][0] == "/usr/bin/python3"
    assert executed[0][2]["ROS_DISTRO"] == "humble"
    assert executed[0][2]["LIO_VIEWER_ROS_BOOTSTRAPPED"] == "1"


def test_local_stream_preserves_gap_bounded_history_and_epoch_reset(tmp_path, monkeypatch):
    model = viewer.TrajectoryModel(tmp_path / "missing.json", max_points=2)
    packet = dict(generation=0, points=[(1, [0, 0, 0]), (2, None), (3, [1, 2, 3])],
                  position=[1, 2, 3], status="DEGRADED", reference="LIDAR ORIGIN")
    model.receive_packet(packet)
    points, position, status, _ = model.snapshot()
    assert len(points) == 2 and all(math.isnan(value) for value in points[0][1])
    assert position == [1, 2, 3] and status == "DEGRADED"
    monkeypatch.setattr(viewer.time, "monotonic", lambda: model.received + 1)
    assert model.snapshot()[2] == "STALE / NO FRESH PAIRED DATA"
    packet.update(generation=1, points=[(4, [0, 0, 0])], position=[0, 0, 0])
    model.receive_packet(packet)
    assert model.snapshot()[0] == [(4, (0, 0, 0))]


def test_wsl_translation_bypasses_shell_backslash_processing(monkeypatch):
    calls = []

    def output(command, **kwargs):
        calls.append(command)
        return "/mnt/c/viewer.py\n"

    monkeypatch.setattr(viewer.subprocess, "check_output", output)
    assert viewer.LocalWslConnection.translate_path(source) == "/mnt/c/viewer.py"
    assert calls[0][:4] == ["wsl.exe", "--exec", "wslpath", "-a"]
    assert "\\" not in calls[0][-1]


def test_lost_reason_is_visible_without_creating_an_origin(tmp_path, monkeypatch):
    monkeypatch.setattr(viewer.time, "time", lambda: 100.0)
    model = viewer.TrajectoryModel(tmp_path / "missing.json")
    pair = messages(0, state=3)
    pair[1].reason = "laser_correction_stale"
    deliver(model, pair)
    assert model.snapshot()[1] is None
    assert model.snapshot()[2] == "LOST - laser_correction_stale"


def test_session_shutdown_escalates_only_owned_process_group(monkeypatch):
    sent = []
    monkeypatch.setattr(viewer.os, "killpg", lambda pid, sig: sent.append((pid, sig)), raising=False)
    monkeypatch.setattr(viewer.signal, "SIGKILL", 9, raising=False)
    waits = []

    def wait(timeout):
        waits.append(timeout)
        if len(waits) == 1:
            raise viewer.subprocess.TimeoutExpired("owned-node", timeout)

    closed = []
    process = NS(pid=123, poll=lambda: None, wait=wait, stdout=NS(close=lambda: closed.append(True)))
    session = viewer.LocalLocalizationSession()
    session.children.append((process, [], NS(join=lambda timeout: None)))
    session.close()
    session.close()
    assert sent == [(123, viewer.signal.SIGINT), (123, viewer.signal.SIGTERM)]
    assert waits == [4, 2] and closed == [True]


def test_session_start_failure_still_closes_owned_nodes(monkeypatch, tmp_path):
    closed = []
    class Session:
        def start(self):
            raise RuntimeError("existing FAST-LIO")
        def close(self):
            closed.append(True)
    class Thread:
        ident = None
        def __init__(self, **kwargs):
            pass
        def start(self):
            pass
    monkeypatch.setattr(viewer, "LocalLocalizationSession", Session)
    monkeypatch.setattr(viewer.threading, "Thread", Thread)
    with pytest.raises(RuntimeError, match="existing FAST-LIO"):
        viewer.stream_local_ros(viewer.TrajectoryModel(tmp_path / "missing.json"), new_map=True)
    assert closed == [True]


def test_actual_3d_draw_handles_nan_gaps_empty_reset_and_clipping():
    import numpy as np
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    figure = Figure()
    canvas = FigureCanvasAgg(figure)
    axes = figure.add_subplot(projection="3d")
    line, = axes.plot([], [], [])
    marker, = axes.plot([], [], [], "o")
    axes.set_xlim(-1, 1)
    for points in ([(0, 0, 0), (np.nan,) * 3, (2, 1, 1)], [], [(0.2, 0.1, 0.3)]):
        viewer.set_line_xyz(line, points)
        viewer.set_line_xyz(marker, points[-1:] if points else [])
        assert all(isinstance(values, np.ndarray) for values in line.get_data_3d())
        for azimuth in (0, 45, 90):
            axes.view_init(azim=azimuth)
            canvas.draw()


def test_service_restart_sequence_rewind_resets_display_origin(tmp_path, monkeypatch):
    monkeypatch.setattr(viewer.time, "time", lambda: 100.0)
    model = viewer.TrajectoryModel(tmp_path / "missing.json")
    old = messages(0, (1, 2, 3))
    old[1].correction_seq = 100
    deliver(model, old)
    fresh = messages(1000000, (20, 30, 40))
    fresh[1].correction_seq = 1
    deliver(model, fresh)
    assert model.snapshot()[1] == pytest.approx((0, 0, 0))
    assert model.generation == 1
