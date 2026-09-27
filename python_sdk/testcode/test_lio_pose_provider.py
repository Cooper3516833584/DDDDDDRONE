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
    assert p.get_pose() is not None
    p.on_odometry(odom(1.0))
    assert p.get_pose() is None


def test_optional_mount_and_wrong_frame_fail_closed(tmp_path):
    default = LioPoseProvider(tmp_path / "missing.json", require_health=False)
    assert default._reference_frame == "lidar"
    assert default._mount == (LioPoseProvider.LIDAR_ORIGIN_IN_IMU_M,
                              LioPoseProvider.ALIGNED_QUATERNION)
    default.on_odometry(odom(1))
    default.calibrate_basepoint(disarmed=True)
    assert default.get_snapshot()["reference_frame"] == "lidar"
    p = provider(tmp_path)
    p.on_odometry(odom(1, frame="unexpected"))
    with pytest.raises(RuntimeError):
        p.calibrate_basepoint(disarmed=True)


def test_optional_radar_height_and_on_demand_ceiling(tmp_path):
    mount = tmp_path / "height.json"
    mount.write_text(json.dumps({"radar_height_above_body_origin_m": 0.25}),
                     encoding="utf-8")
    p = LioPoseProvider(mount, require_health=False)
    assert p._reference_frame == "body"
    assert p._mount[0] == pytest.approx((-0.011, -0.02329, 0.04412 - 0.25))
    assert p._mount[1] == (0, 0, 0, 1)
    calls = []

    def estimator(points):
        calls.append(points)
        return 1.4

    assert p.estimate_ceiling_clearance_m([]) is None
    p.set_ceiling_clearance_estimator(estimator)
    assert calls == []
    assert p.estimate_ceiling_clearance_m([(0, 0, 1.4)]) == pytest.approx(1.4)
    assert len(calls) == 1
    p.set_ceiling_clearance_estimator(None)
    assert p.estimate_ceiling_clearance_m([]) is None
    assert len(calls) == 1


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
    p._valid_correction_received_at -= 0.3
    assert p.get_pose() is None
    assert p._lost_latched


def stamp_ns(ns):
    return SimpleNamespace(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)


def imu_ns(ns, *, angular=0.001, acceleration=1.0):
    return SimpleNamespace(
        header=SimpleNamespace(stamp=stamp_ns(ns)),
        angular_velocity=SimpleNamespace(x=angular, y=0, z=0),
        linear_acceleration=SimpleNamespace(x=0, y=0, z=acceleration),
    )


def health_ns(state_ns, seq, anchor_ns, *, state=1):
    return SimpleNamespace(
        epoch=1, correction_seq=seq, state_stamp=stamp_ns(state_ns),
        anchor_stamp=stamp_ns(anchor_ns), state=state,
        matched_points=250 if state == 1 else 0,
        residual_rms_m=0.05 if state == 1 else 0.0,
        geometry_ratio=0.01 if state == 1 else 0.0,
        gravity_o=SimpleNamespace(x=0, y=0, z=-9.81),
    )


def odom_ns(ns, xyz=(0, 0, 0)):
    message = odom(ns * 1e-9, xyz)
    message.header.stamp = stamp_ns(ns)
    return message


def reviewed_provider(tmp_path):
    mount = tmp_path / "reviewed_body.json"
    mount.write_text(json.dumps({"reviewed": True, "T_I_B": {
        "translation_m": [0, 0, 0], "quaternion_xyzw": [0, 0, 0, 1],
    }}), encoding="utf-8")
    return LioPoseProvider(mount)


def stationary_window(p, first_ns=1_000_000_000):
    for i in range(401):
        p.on_imu(imu_ns(first_ns + i * 5_000_000))


def tracking_provider(tmp_path, *, anchor_offset_ns=30_000_000):
    p = reviewed_provider(tmp_path)
    stationary_window(p)
    now_ns = time.time_ns()
    for seq in range(1, 11):
        state_ns = now_ns - 30_000_000 + seq * 1_000_000
        anchor_ns = now_ns - anchor_offset_ns - (10 - seq) * 1_000_000
        p.on_health(health_ns(state_ns, seq, anchor_ns))
    last_ns = now_ns - 20_000_000
    p.on_odometry(odom_ns(last_ns))
    p.calibrate_basepoint(disarmed=True)
    assert p.get_pose() is not None
    return p, now_ns


def test_degraded_keeps_valid_receive_time_and_base_then_lost(tmp_path):
    p, now_ns = tracking_provider(tmp_path)
    valid_time = p._valid_correction_received_at
    base, basis = p._base, p._world_basis
    for offset in (-19_000_000, -18_000_000):
        state_ns = now_ns + offset
        p.on_health(health_ns(state_ns, 10, p._anchor_ns, state=2))
        p.on_odometry(odom_ns(state_ns))
    assert p._valid_correction_received_at == valid_time
    assert p.get_pose() is None
    assert not p._lost_latched
    assert p._base == base and p._world_basis == basis
    p._valid_correction_received_at -= 0.26
    assert p.get_pose() is None
    assert p._lost_latched and p._base is None and p._world_basis is None


def test_degraded_recovers_after_ten_distinct_valid_corrections(tmp_path):
    p, now_ns = tracking_provider(tmp_path)
    base, basis = p._base, p._world_basis
    p.on_health(health_ns(now_ns - 19_000_000, 10, p._anchor_ns, state=2))
    p.on_odometry(odom_ns(now_ns - 19_000_000))
    assert p.get_pose() is None
    assert p._base == base and p._world_basis == basis
    for seq in range(11, 21):
        state_ns = now_ns - 19_000_000 + (seq - 10) * 1_000_000
        p.on_health(health_ns(state_ns, seq, state_ns - 1_000_000))
        p.on_odometry(odom_ns(state_ns))
        if seq < 20:
            assert p.get_pose() is None
    assert p._good_corrections == 10
    assert p.get_pose() is not None
    assert p._base == base and p._world_basis == basis


def test_repeated_health_does_not_count_or_refresh_valid_correction(tmp_path):
    p, now_ns = tracking_provider(tmp_path)
    valid_time = p._valid_correction_received_at
    p.on_health(health_ns(now_ns - 19_000_000, 10, p._anchor_ns))
    assert p._good_corrections == 10
    assert p._valid_correction_received_at == valid_time


def test_fresh_bad_stream_loses_after_old_source_anchor(tmp_path):
    p, now_ns = tracking_provider(tmp_path, anchor_offset_ns=260_000_000)
    for state_ns in (now_ns - 19_000_000, now_ns - 1_000_000):
        p.on_health(health_ns(state_ns, 10, p._anchor_ns, state=2))
        p.on_odometry(odom_ns(state_ns))
    assert p._health_received_at >= p._valid_correction_received_at
    assert p._received_at >= p._valid_correction_received_at
    assert p.get_pose() is None
    assert p._lost_latched


def test_new_valid_correction_refreshes_age_but_needs_ten_to_recover(tmp_path):
    p, now_ns = tracking_provider(tmp_path)
    p.on_health(health_ns(now_ns - 19_000_000, 10, p._anchor_ns, state=2))
    old_time = p._valid_correction_received_at
    p._valid_correction_received_at -= 0.15
    state_ns = now_ns - 18_000_000
    p.on_health(health_ns(state_ns, 11, state_ns - 1_000_000))
    p.on_odometry(odom_ns(state_ns))
    assert p._valid_correction_received_at > old_time - 0.15
    assert p._good_corrections == 1
    assert not p._lost_latched
    assert p.get_pose() is None


def test_stationarity_revoked_by_motion_and_requires_new_two_seconds(tmp_path):
    p = reviewed_provider(tmp_path)
    stationary_window(p)
    assert p._stationary_ready
    p.on_imu(imu_ns(3_005_000_000, angular=0.03))
    assert not p._stationary_ready and not p._imu_window
    for i in range(380):
        p.on_imu(imu_ns(3_010_000_000 + i * 5_000_000))
    assert not p._stationary_ready
    for i in range(380, 401):
        p.on_imu(imu_ns(3_010_000_000 + i * 5_000_000))
    assert p._stationary_ready


def test_imu_gap_and_rewind_reset_stationarity(tmp_path):
    p = reviewed_provider(tmp_path)
    stationary_window(p)
    p.on_imu(imu_ns(3_100_000_000))
    assert not p._stationary_ready
    assert len(p._imu_window) == 1
    stationary_window(p, 3_105_000_000)
    assert p._stationary_ready
    p.on_imu(imu_ns(3_000_000_000))
    assert not p._stationary_ready and not p._imu_window


def test_ground_reset_clears_imu_and_message_history(tmp_path):
    p, _ = tracking_provider(tmp_path)
    assert p.reset_ground(disarmed=True) is False
    assert p._stationary_ready
    p._lost_latched = True
    with pytest.raises(RuntimeError, match="disarmed"):
        p.reset_ground(disarmed=False)
    assert p.reset_ground(disarmed=True) is True
    assert not p._stationary_ready and not p._imu_window and p._last_imu_ns == 0
    assert p._latest is None and p._health is None and p._base is None
    with pytest.raises(RuntimeError):
        p.calibrate_basepoint(disarmed=True)
    stationary_window(p, 10_000_000_000)
    assert p._stationary_ready
    now_ns = time.time_ns()
    for seq in range(1, 11):
        state_ns = now_ns - 30_000_000 + seq * 1_000_000
        p.on_health(health_ns(state_ns, seq, state_ns - 1_000_000))
    p.on_odometry(odom_ns(now_ns - 20_000_000))
    p.calibrate_basepoint(disarmed=True)
    assert p.get_pose() is not None


def test_mount_review_gate_and_invalid_configs(tmp_path):
    missing = LioPoseProvider(tmp_path / "missing.json")
    assert missing._reference_frame == "lidar" and not missing.mount_reviewed
    with pytest.raises(RuntimeError):
        missing.calibrate_basepoint(disarmed=True)

    mount = tmp_path / "mount.json"
    mount.write_text(json.dumps({"reviewed": True,
                                 "radar_height_above_body_origin_m": 0.25}), encoding="utf-8")
    height = LioPoseProvider(mount)
    assert height.mount_reviewed and height._reference_frame == "body"
    assert height._mount[0] == pytest.approx((-0.011, -0.02329, -0.20588))

    mount.write_text(json.dumps({"reviewed": False, "T_I_B": {
        "translation_m": [1, 0, 0], "quaternion_xyzw": [0, 0, 0, 1]}}), encoding="utf-8")
    unreviewed = LioPoseProvider(mount)
    assert not unreviewed.mount_reviewed
    with pytest.raises(RuntimeError):
        unreviewed.calibrate_basepoint(disarmed=True)

    invalid = [
        {"reviewed": True, "radar_height_above_body_origin_m": -0.1},
        {"reviewed": True, "radar_height_above_body_origin_m": float("nan")},
        {"reviewed": True, "T_I_B": {"translation_m": [1, 2],
                                      "quaternion_xyzw": [0, 0, 0, 1]}},
        {"reviewed": True, "T_I_B": {"translation_m": [1, 2, 3],
                                      "quaternion_xyzw": [0, 0, 0, 0]}},
        {"reviewed": True, "radar_height_above_body_origin_m": 0.2,
         "T_I_B": {"translation_m": [1, 2, 3],
                   "quaternion_xyzw": [0, 0, 0, 1]}},
        [],
    ]
    for data in invalid:
        mount.write_text(json.dumps(data), encoding="utf-8")
        with pytest.raises(RuntimeError, match="Invalid MID360S mount"):
            LioPoseProvider(mount)


def test_reviewed_full_mount_rotates_body_lever_arm(tmp_path):
    mount = tmp_path / "rotated.json"
    half = math.sqrt(0.5)
    mount.write_text(json.dumps({"reviewed": True, "T_I_B": {
        "translation_m": [1, 0, 0], "quaternion_xyzw": [0, 0, half, half]}}),
        encoding="utf-8")
    p = LioPoseProvider(mount, require_health=False)
    assert p.mount_reviewed and p._reference_frame == "body"
    p.on_odometry(odom(1))
    assert p._reference_pose()[0] == pytest.approx((1, 0, 0))
    p.on_odometry(odom(1.01, yaw_deg=90))
    assert p._reference_pose()[0] == pytest.approx((0, 1, 0))
