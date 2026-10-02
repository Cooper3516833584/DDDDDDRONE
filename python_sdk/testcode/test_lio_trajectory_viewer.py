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
