"""Pure transform and failure tests; no ROS or flight-controller imports."""

import importlib.util
import json
import math
import time
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
        child_frame_id="imu",
        pose=SimpleNamespace(covariance=[1 if i % 7 == 0 else 0 for i in range(36)], pose=SimpleNamespace(
            position=SimpleNamespace(x=xyz[0], y=xyz[1], z=xyz[2]),
            orientation=SimpleNamespace(x=0, y=0, z=math.sin(half), w=math.cos(half)),
        )),
        twist=SimpleNamespace(covariance=[1 if i % 7 == 0 else 0 for i in range(36)], twist=SimpleNamespace(
            linear=SimpleNamespace(x=0, y=0, z=0),
            angular=SimpleNamespace(x=0, y=0, z=0),
        )),
    )


def provider(tmp_path, translation=(0, 0, 0)):
    mount = tmp_path / "mount.json"
    mount.write_text(json.dumps({"T_I_B": {
        "translation_m": translation,
        "quaternion_xyzw": [0, 0, 0, 1],
    }}), encoding="utf-8")
    return LioPoseProvider(mount, require_health=False)


def test_mount_and_startup_local_coordinates(tmp_path):
    p = provider(tmp_path, (1, 0, 0))
    p.on_odometry(odom(1))
    p.calibrate_basepoint(disarmed=True)
    assert p.get_pose() == pytest.approx((0, 0, 0, 1))

    p.on_odometry(odom(1.01, (1, 0, 0)))
    assert p.get_pose() == pytest.approx((100, 0, 0, 1))

    p.on_odometry(odom(1.02, (1, 0, 0), -90))
    assert p.get_pose() == pytest.approx((0, -100, 90, 1))


def test_stale_and_timestamp_rewind_invalidate_origin(tmp_path):
    p = provider(tmp_path)
    p.on_odometry(odom(2))
    p.calibrate_basepoint(disarmed=True)
    p._received_at -= 1.0
    assert p.get_pose() is None
    p.on_odometry(odom(2.01))
    assert p.get_pose() is None
    p.calibrate_basepoint(disarmed=True)
    p.on_odometry(odom(1.0))
    assert p.get_pose() is None


def test_missing_mount_and_wrong_frame_fail_closed(tmp_path):
    with pytest.raises(RuntimeError):
        LioPoseProvider(tmp_path / "missing.json")
    p = provider(tmp_path)
    p.on_odometry(odom(1, frame="unexpected"))
    with pytest.raises(RuntimeError):
        p.calibrate_basepoint(disarmed=True)


def test_health_requires_stable_corrections_and_watchdog(tmp_path):
    mount = tmp_path / "reviewed.json"
    mount.write_text(json.dumps({"reviewed": True, "T_I_B": {
        "translation_m": [1, 0, 0], "quaternion_xyzw": [0, 0, 0, 1],
    }}), encoding="utf-8")
    p = LioPoseProvider(mount)
    with pytest.raises(RuntimeError):
        p.calibrate_basepoint(disarmed=True)

    def stamp(ns):
        return SimpleNamespace(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)

    now_ns = time.time_ns()
    for i in range(401):
        imu_ns = now_ns - 2_050_000_000 + i * 5_000_000
        p.on_imu(SimpleNamespace(
            header=SimpleNamespace(stamp=stamp(imu_ns)),
            angular_velocity=SimpleNamespace(x=0.001, y=0, z=0),
            linear_acceleration=SimpleNamespace(x=0, y=0, z=1),
        ))
    assert p._stationary_ready
    for seq in range(1, 11):
        state_ns = now_ns - 20_000_000 + seq * 1_000_000
        p.on_health(SimpleNamespace(
            epoch=1, correction_seq=seq,
            state_stamp=stamp(state_ns), anchor_stamp=stamp(state_ns - 1_000_000),
            state=1, matched_points=250, residual_rms_m=0.05,
            geometry_ratio=0.01,
            gravity_o=SimpleNamespace(x=0, y=0, z=-9.81),
        ))
    final_ns = now_ns - 10_000_000
    final_odom = odom(final_ns * 1e-9)
    final_odom.header.stamp = stamp(final_ns)
    p.on_odometry(final_odom)
    with pytest.raises(RuntimeError):
        p.calibrate_basepoint(disarmed=False)
    p.calibrate_basepoint(disarmed=True)
    assert p.get_pose() is not None
    next_ns = final_ns + 1_000_000
    p.on_health(SimpleNamespace(
        epoch=1, correction_seq=10, state_stamp=stamp(next_ns),
        anchor_stamp=stamp(final_ns - 1_000_000), state=1,
        matched_points=250, residual_rms_m=0.05, geometry_ratio=0.01,
        gravity_o=SimpleNamespace(x=0, y=0, z=-9.81),
    ))
    turn_odom = odom(next_ns * 1e-9, yaw_deg=90)
    turn_odom.header.stamp = stamp(next_ns)
    turn_odom.twist.twist.angular.z = 1.0
    p.on_odometry(turn_odom)
    assert p.get_pose() == pytest.approx((-100, 100, -90, True))
    snapshot = p.get_snapshot()
    assert snapshot["position_m"] == pytest.approx((-1, 1, 0))
    assert snapshot["velocity_w_mps"] == pytest.approx((-1, 0, 0))
    assert snapshot["angular_velocity_b_radps"] == pytest.approx((0, 0, 1))
    p._correction_received_at -= 0.3
    assert p.get_pose() is None
    assert p._lost_latched
