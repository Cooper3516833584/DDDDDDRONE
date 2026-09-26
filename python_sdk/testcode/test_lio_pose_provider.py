"""Pure transform and failure tests; no ROS or flight-controller imports."""

import importlib.util
import json
import math
from pathlib import Path
from types import SimpleNamespace

import pytest


SOURCE = Path(__file__).resolve().parents[1] / "FlightController/Components/LioPoseProvider.py"
spec = importlib.util.spec_from_file_location("lio_pose_provider_test", SOURCE)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
LioPoseProvider = module.LioPoseProvider


def odom(stamp, xyz=(0, 0, 0), yaw_deg=0, frame="camera_init"):
    half = math.radians(yaw_deg) / 2
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=int(stamp), nanosec=int((stamp % 1) * 1e9)),
            frame_id=frame,
        ),
        child_frame_id="body",
        pose=SimpleNamespace(pose=SimpleNamespace(
            position=SimpleNamespace(x=xyz[0], y=xyz[1], z=xyz[2]),
            orientation=SimpleNamespace(x=0, y=0, z=math.sin(half), w=math.cos(half)),
        )),
    )


def provider(tmp_path, translation=(0, 0, 0)):
    mount = tmp_path / "mount.json"
    mount.write_text(json.dumps({"T_I_B": {
        "translation_m": translation,
        "quaternion_xyzw": [0, 0, 0, 1],
    }}), encoding="utf-8")
    return LioPoseProvider(mount)


def test_mount_and_startup_local_coordinates(tmp_path):
    p = provider(tmp_path, (1, 0, 0))
    p.on_odometry(odom(1))
    p.calibrate_basepoint()
    assert p.get_pose() == pytest.approx((0, 0, 0, 1))

    p.on_odometry(odom(1.01, (1, 0, 0)))
    assert p.get_pose() == pytest.approx((100, 0, 0, 1))

    p.on_odometry(odom(1.02, (1, 0, 0), -90))
    assert p.get_pose() == pytest.approx((0, -100, 90, 1))


def test_stale_and_timestamp_rewind_invalidate_origin(tmp_path):
    p = provider(tmp_path)
    p.on_odometry(odom(2))
    p.calibrate_basepoint()
    p._received_at -= 1.0
    assert p.get_pose() is None
    p.on_odometry(odom(2.01))
    assert p.get_pose() is None
    p.calibrate_basepoint()
    p.on_odometry(odom(1.0))
    assert p.get_pose() is None


def test_missing_mount_and_wrong_frame_fail_closed(tmp_path):
    with pytest.raises(RuntimeError):
        LioPoseProvider(tmp_path / "missing.json")
    p = provider(tmp_path)
    p.on_odometry(odom(1, frame="unexpected"))
    with pytest.raises(RuntimeError):
        p.calibrate_basepoint()
