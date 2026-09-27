"""Convert FAST-LIO IMU odometry to the startup-local aircraft reference."""

import json
import math
import threading
import time
from collections import deque
from pathlib import Path


def _unit(q):
    if len(q) != 4 or not all(math.isfinite(v) for v in q):
        raise ValueError("Invalid LIO quaternion")
    norm = math.sqrt(sum(v * v for v in q))
    if norm < 1e-9:
        raise ValueError("Zero LIO quaternion")
    return tuple(v / norm for v in q)


def _mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return _unit((aw * bx + ax * bw + ay * bz - az * by,
                  aw * by - ax * bz + ay * bw + az * bx,
                  aw * bz + ax * by - ay * bx + az * bw,
                  aw * bw - ax * bx - ay * by - az * bz))


def _rotate(q, v):
    x, y, z, w = q
    vx, vy, vz = v
    tx, ty, tz = (2 * (y * vz - z * vy),
                  2 * (z * vx - x * vz),
                  2 * (x * vy - y * vx))
    return (vx + w * tx + y * tz - z * ty,
            vy + w * ty + z * tx - x * tz,
            vz + w * tz + x * ty - y * tx)


def _relative(base, current):
    bp, bq = base
    cp, cq = current
    inv = (-bq[0], -bq[1], -bq[2], bq[3])
    return _rotate(inv, tuple(cp[i] - bp[i] for i in range(3))), _mul(inv, cq)


def _dot(a, b):
    return sum(a[i] * b[i] for i in range(3))


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def _normalize(v):
    n = math.sqrt(_dot(v, v))
    if not math.isfinite(n) or n < 1e-6:
        raise ValueError("Invalid gravity or heading")
    return tuple(x / n for x in v)


class LioPoseProvider:
    STALE_SECONDS = 0.05
    CORRECTION_STALE_SECONDS = 0.25
    # Manufacturer T_I_L. User confirmed L and aircraft axes are aligned.
    LIDAR_ORIGIN_IN_IMU_M = (-0.011, -0.02329, 0.04412)
    ALIGNED_QUATERNION = (0.0, 0.0, 0.0, 1.0)

    def __init__(self, mount_path=None, *, require_health=True):
        if mount_path is None:
            mount_path = Path(__file__).resolve().parents[2] / "config" / "mid360s_mount.json"
        p, q = self.LIDAR_ORIGIN_IN_IMU_M, self.ALIGNED_QUATERNION
        self._reference_frame = "lidar"
        self._mount_reviewed = False
        mount_file = Path(mount_path)
        if mount_file.exists():
            try:
                data = json.loads(mount_file.read_text(encoding="utf-8"))
                if "T_I_B" in data and data.get("radar_height_above_body_origin_m") is not None:
                    raise ValueError("Conflicting body mount definitions")
                if "T_I_B" in data:
                    mount = data["T_I_B"]
                    p = tuple(float(v) for v in mount["translation_m"])
                    q = _unit(tuple(float(v) for v in mount["quaternion_xyzw"]))
                    self._reference_frame = "body"
                elif data.get("radar_height_above_body_origin_m") is not None:
                    height = float(data["radar_height_above_body_origin_m"])
                    if not math.isfinite(height) or height < 0:
                        raise ValueError("Invalid radar height")
                    p = (p[0], p[1], p[2] - height)
                    self._reference_frame = "body"
                if len(p) != 3 or not all(math.isfinite(v) for v in p):
                    raise ValueError("Invalid mount translation")
                self._mount_reviewed = data.get("reviewed") is True and self._reference_frame == "body"
            except (OSError, AttributeError, KeyError, TypeError, ValueError) as exc:
                raise RuntimeError("Invalid MID360S mount configuration") from exc
        self._mount = (p, q)
        self._ceiling_estimator = None
        self._require_health = require_health
        self._lock = threading.Lock()
        self._latest = None
        self._latest_twist = None
        self._received_at = 0.0
        self._stamp = None
        self._stamp_ns_value = None
        self._base = None
        self._world_basis = None
        self._health = None
        self._health_received_at = 0.0
        self._valid_correction_received_at = 0.0
        self._health_epoch = None
        self._correction_seq = 0
        self._anchor_ns = 0
        self._state_ns = 0
        self._good_corrections = 0
        self._lost_latched = False
        self._imu_window = deque()
        self._stationary_ready = False
        self._last_imu_ns = 0

    @staticmethod
    def _stamp_ns(stamp):
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    @property
    def mount_reviewed(self):
        return self._mount_reviewed

    def set_ceiling_clearance_estimator(self, estimator):
        """Register an on-demand LiDAR point-cloud estimator; no background work."""
        if estimator is not None and not callable(estimator):
            raise TypeError("Ceiling estimator must be callable or None")
        self._ceiling_estimator = estimator

    def estimate_ceiling_clearance_m(self, lidar_points_xyz):
        """Return LiDAR-to-ceiling distance only when explicitly requested."""
        if self._ceiling_estimator is None:
            return None
        distance = self._ceiling_estimator(lidar_points_xyz)
        if distance is None:
            return None
        distance = float(distance)
        if not math.isfinite(distance) or distance <= 0:
            raise ValueError("Invalid ceiling clearance")
        return distance

    def on_imu(self, imu):
        stamp_ns = self._stamp_ns(imu.header.stamp)
        gyro = imu.angular_velocity
        acc = imu.linear_acceleration
        g = math.sqrt(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z)
        w = math.sqrt(gyro.x * gyro.x + gyro.y * gyro.y + gyro.z * gyro.z)
        with self._lock:
            monotonic = stamp_ns > self._last_imu_ns
            gap_ok = not self._last_imu_ns or stamp_ns - self._last_imu_ns <= 20_000_000
            stationary_sample = (math.isfinite(g) and math.isfinite(w) and
                                 0.8 <= g <= 1.2 and w <= 0.02)
            if not monotonic or not gap_ok or not stationary_sample:
                self._imu_window.clear()
                self._stationary_ready = False
            self._last_imu_ns = stamp_ns
            if monotonic and stationary_sample:
                self._imu_window.append((stamp_ns, g))
            while self._imu_window and stamp_ns - self._imu_window[0][0] > 2_500_000_000:
                self._imu_window.popleft()
            self._stationary_ready = bool(
                self._imu_window and
                stamp_ns - self._imu_window[0][0] >= 2_000_000_000 and
                max(item[1] for item in self._imu_window) -
                min(item[1] for item in self._imu_window) <= 0.05)

    def on_health(self, health):
        state_ns = self._stamp_ns(health.state_stamp)
        anchor_ns = self._stamp_ns(health.anchor_stamp)
        epoch = int(health.epoch)
        seq = int(health.correction_seq)
        now = time.monotonic()
        with self._lock:
            if (epoch < 1 or seq < 1 or state_ns < anchor_ns or anchor_ns <= 0 or
                    (self._health_epoch is not None and epoch != self._health_epoch) or
                    state_ns <= self._state_ns or seq < self._correction_seq or
                    (seq == self._correction_seq and anchor_ns != self._anchor_ns) or
                    (seq > self._correction_seq and anchor_ns <= self._anchor_ns)):
                self._lost_latched = True
                self._base = None
                self._world_basis = None
                return
            new_correction = seq > self._correction_seq
            self._health_epoch = epoch
            self._correction_seq = seq
            self._anchor_ns = anchor_ns
            self._state_ns = state_ns
            self._health = health
            self._health_received_at = now
            gravity = health.gravity_o
            gravity_values = (float(gravity.x), float(gravity.y), float(gravity.z))
            gravity_norm = math.sqrt(sum(v * v for v in gravity_values))
            good = (int(health.state) == 1 and int(health.matched_points) >= 200 and
                    math.isfinite(health.residual_rms_m) and
                    0 <= health.residual_rms_m <= 0.15 and
                    math.isfinite(health.geometry_ratio) and
                    1e-4 < health.geometry_ratio <= 1.000001 and
                    all(math.isfinite(v) for v in gravity_values) and
                    5.0 <= gravity_norm <= 15.0)
            if not good:
                self._good_corrections = 0
            elif new_correction:
                self._valid_correction_received_at = now
                self._good_corrections += 1
            if int(health.state) == 3:
                self._lost_latched = True
                self._base = None
                self._world_basis = None

    def reset_ground(self, *, disarmed):
        if not disarmed:
            raise RuntimeError("Ground reset requires verified disarmed state")
        with self._lock:
            if not self._lost_latched:
                return False
            self._latest = None
            self._latest_twist = None
            self._received_at = 0.0
            self._stamp = None
            self._stamp_ns_value = None
            self._health = None
            self._health_received_at = 0.0
            self._health_epoch = None
            self._correction_seq = 0
            self._anchor_ns = 0
            self._state_ns = 0
            self._good_corrections = 0
            self._valid_correction_received_at = 0.0
            self._lost_latched = False
            self._base = None
            self._world_basis = None
            self._imu_window.clear()
            self._stationary_ready = False
            self._last_imu_ns = 0
            return True

    def _healthy(self, now):
        if not self._require_health:
            return self._latest is not None and now - self._received_at <= self.STALE_SECONDS
        if not self._mount_reviewed or self._reference_frame != "body":
            return False
        if self._lost_latched or self._health is None or self._latest is None:
            return False
        if self._stamp_ns_value is None or self._state_ns != self._stamp_ns_value:
            return False
        source_age = time.time() - self._stamp
        if source_age < -0.01 or source_age > self.STALE_SECONDS:
            self._lost_latched = True
            self._base = None
            self._world_basis = None
            return False
        if (now - self._received_at > self.STALE_SECONDS or
                now - self._health_received_at > self.STALE_SECONDS or
                now - self._valid_correction_received_at > self.CORRECTION_STALE_SECONDS or
                (self._state_ns - self._anchor_ns) * 1e-9 > self.CORRECTION_STALE_SECONDS or
                self._good_corrections < 10 or not self._stationary_ready):
            if now - self._valid_correction_received_at > self.CORRECTION_STALE_SECONDS or (
                    self._state_ns - self._anchor_ns) * 1e-9 > self.CORRECTION_STALE_SECONDS:
                self._lost_latched = True
                self._base = None
                self._world_basis = None
            return False
        return int(self._health.state) == 1

    def on_odometry(self, odom):
        if odom.header.frame_id != "camera_init" or odom.child_frame_id != "imu":
            return
        stamp_ns = self._stamp_ns(odom.header.stamp)
        stamp = stamp_ns * 1e-9
        position = odom.pose.pose.position
        rotation = odom.pose.pose.orientation
        p = (float(position.x), float(position.y), float(position.z))
        if not math.isfinite(stamp) or not all(math.isfinite(v) for v in p):
            return
        try:
            q = _unit((rotation.x, rotation.y, rotation.z, rotation.w))
        except ValueError:
            return
        try:
            linear = odom.twist.twist.linear
            angular = odom.twist.twist.angular
            v_imu = (float(linear.x), float(linear.y), float(linear.z))
            omega_imu = (float(angular.x), float(angular.y), float(angular.z))
            if not all(math.isfinite(v) for v in v_imu + omega_imu):
                return
        except AttributeError:
            if self._require_health:
                return
            v_imu = omega_imu = (0.0, 0.0, 0.0)
        if self._require_health:
            covariance = odom.pose.covariance
            twist_covariance = odom.twist.covariance
            if any(not math.isfinite(covariance[7 * i]) or covariance[7 * i] <= 0
                   or not math.isfinite(twist_covariance[7 * i]) or
                   twist_covariance[7 * i] <= 0 for i in range(6)):
                return
        now = time.monotonic()
        with self._lock:
            if self._stamp is not None and stamp <= self._stamp:
                self._latest = None
                self._base = None
                self._world_basis = None
                self._stamp = None
                self._stamp_ns_value = None
                self._lost_latched = True
                return
            self._latest = (p, q)
            self._latest_twist = (v_imu, omega_imu)
            self._stamp = stamp
            self._stamp_ns_value = stamp_ns
            self._received_at = now

    def _reference_pose(self):
        p, q = self._latest
        mp, mq = self._mount
        offset = _rotate(q, mp)
        return (tuple(p[i] + offset[i] for i in range(3)), _mul(q, mq))

    def get_snapshot(self):
        """Return one validated reference-point snapshot in startup-local W."""
        with self._lock:
            if self._base is None or not self._healthy(time.monotonic()):
                return None
            reference_p, reference_q = self._reference_pose()
            delta = tuple(reference_p[i] - self._base[0][i] for i in range(3))
            axes = self._world_basis
            if axes is None:
                base_q = self._base[1]
                axes = tuple(_rotate(base_q, axis) for axis in
                             ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)))
            v_imu, omega_imu = self._latest_twist
            lever = _cross(omega_imu, self._mount[0])
            velocity_o = _rotate(self._latest[1], tuple(v_imu[i] + lever[i]
                                                          for i in range(3)))
            mount_q = self._mount[1]
            omega_reference = _rotate((-mount_q[0], -mount_q[1], -mount_q[2], mount_q[3]),
                                      omega_imu)
            basis = tuple(tuple(_dot(row, _rotate(reference_q, axis)) for axis in
                                ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)))
                          for row in axes)
            return {
                "stamp_ns": self._stamp_ns_value,
                "epoch": self._health_epoch,
                "correction_seq": self._correction_seq,
                "reference_frame": self._reference_frame,
                "position_m": tuple(_dot(axis, delta) for axis in axes),
                "rotation_w_b": basis,
                "velocity_w_mps": tuple(_dot(axis, velocity_o) for axis in axes),
                "angular_velocity_b_radps": omega_reference,
            }

    def calibrate_basepoint(self, *, disarmed=False):
        with self._lock:
            if not disarmed or not self._healthy(time.monotonic()):
                raise RuntimeError("Disarmed, stationary and healthy LIO are required")
            self._base = self._reference_pose()
            if self._require_health:
                try:
                    gravity = self._health.gravity_o
                    z_axis = _normalize((-gravity.x, -gravity.y, -gravity.z))
                    forward = _rotate(self._base[1], (1.0, 0.0, 0.0))
                    horizontal = tuple(forward[i] - _dot(forward, z_axis) * z_axis[i]
                                       for i in range(3))
                    x_axis = _normalize(horizontal)
                    y_axis = _normalize(_cross(z_axis, x_axis))
                except (AttributeError, ValueError) as exc:
                    self._base = None
                    raise RuntimeError("Valid gravity and heading are required") from exc
                self._world_basis = (x_axis, y_axis, z_axis)

    def get_pose(self):
        with self._lock:
            if self._base is None or not self._healthy(time.monotonic()):
                return None
            if self._world_basis is None:
                position, q = _relative(self._base, self._reference_pose())
                yaw_ccw = math.degrees(math.atan2(2 * (q[3] * q[2] + q[0] * q[1]),
                                                   1 - 2 * (q[1] ** 2 + q[2] ** 2)))
            else:
                reference_p, reference_q = self._reference_pose()
                delta = tuple(reference_p[i] - self._base[0][i] for i in range(3))
                position = tuple(_dot(axis, delta) for axis in self._world_basis)
                forward = _rotate(reference_q, (1.0, 0.0, 0.0))
                yaw_ccw = math.degrees(math.atan2(_dot(forward, self._world_basis[1]),
                                                   _dot(forward, self._world_basis[0])))
        return position[0] * 100, position[1] * 100, -yaw_ccw, True
