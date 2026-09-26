"""Convert FAST-LIO IMU odometry to the Navigation startup-local body pose."""

import json
import math
import threading
import time
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


class LioPoseProvider:
    STALE_SECONDS = 0.30

    def __init__(self, mount_path=None):
        if mount_path is None:
            mount_path = Path(__file__).resolve().parents[2] / "config" / "mid360s_mount.json"
        try:
            data = json.loads(Path(mount_path).read_text(encoding="utf-8"))
            mount = data["T_I_B"]
            p = tuple(float(v) for v in mount["translation_m"])
            q = _unit(tuple(float(v) for v in mount["quaternion_xyzw"]))
            if len(p) != 3 or not all(math.isfinite(v) for v in p):
                raise ValueError("Invalid mount translation")
        except (OSError, KeyError, TypeError, ValueError) as exc:
            raise RuntimeError("Measured MID360S IMU-to-body mount is required") from exc
        self._mount = (p, q)
        self._lock = threading.Lock()
        self._latest = None
        self._received_at = 0.0
        self._stamp = None
        self._base = None

    def on_odometry(self, odom):
        if odom.header.frame_id != "camera_init" or odom.child_frame_id != "body":
            return
        stamp = odom.header.stamp.sec + odom.header.stamp.nanosec * 1e-9
        position = odom.pose.pose.position
        rotation = odom.pose.pose.orientation
        p = (float(position.x), float(position.y), float(position.z))
        if not math.isfinite(stamp) or not all(math.isfinite(v) for v in p):
            return
        try:
            q = _unit((rotation.x, rotation.y, rotation.z, rotation.w))
        except ValueError:
            return
        now = time.monotonic()
        with self._lock:
            if self._stamp is not None and stamp <= self._stamp:
                self._latest = None
                self._base = None
                self._stamp = None
                return
            if self._received_at and now - self._received_at > self.STALE_SECONDS:
                self._base = None
            self._latest = (p, q)
            self._stamp = stamp
            self._received_at = now

    def _body_pose(self):
        p, q = self._latest
        mp, mq = self._mount
        offset = _rotate(q, mp)
        return (tuple(p[i] + offset[i] for i in range(3)), _mul(q, mq))

    def calibrate_basepoint(self):
        with self._lock:
            if self._latest is None or time.monotonic() - self._received_at > self.STALE_SECONDS:
                raise RuntimeError("Fresh /Odometry_highrate is required")
            self._base = self._body_pose()

    def get_pose(self):
        with self._lock:
            if (self._latest is None or self._base is None or
                    time.monotonic() - self._received_at > self.STALE_SECONDS):
                return None
            position, q = _relative(self._base, self._body_pose())
        yaw_ccw = math.degrees(math.atan2(2 * (q[3] * q[2] + q[0] * q[1]),
                                           1 - 2 * (q[1] ** 2 + q[2] ** 2)))
        return position[0] * 100, position[1] * 100, -yaw_ccw, True
