"""Lightweight 2-D obstacle map + A* planner for DDDDDDRONE.

The component is intentionally small:
- input: FAST-LIO ``/cloud_registered_body`` point cloud + current startup-local pose
- map: fixed 2-D occupancy score grid in Navigation's startup-local frame
- safety: occupied-cell inflation
- planning: 8-neighbour A* with diagonal corner-cut prevention
- outputs: one stable safe waypoint, or a safe velocity direction

Units at the public API follow Navigation.py:
- positions: cm
- velocity: cm/s
- yaw: clockwise-positive degrees
Point-cloud coordinates are metres in the aircraft/IMU-aligned body frame.

This is an original, simplified implementation inspired by the architecture of
ROG-Map (ray-based occupancy update, inflation, grid search). It is not a copy
of ROG-Map source code.
"""

from __future__ import annotations

import heapq
import math
import threading
import time
from typing import Callable, Iterable, List, Optional, Sequence, Tuple

import numpy as np

Point = Tuple[float, float]
PoseGetter = Callable[[], Optional[Tuple[float, float, float, bool]]]


class ObstaclePlanner2D:
    """Small 2-D occupancy/A* component for the RoboCup field.

    The map is fixed around the startup-local origin.  Unknown cells are treated
    as traversable because the competition field is known to be open except for
    a few obstacles, and the aircraft must be able to move before every cell has
    been observed.
    """

    # ---- deliberately few constants: tune these first on the real aircraft ----
    MAP_SIZE_M = 24.0
    RESOLUTION_M = 0.10

    # Use a horizontal slice around the LiDAR/body origin. The 2 m pillars cross
    # this slice; floor/ceiling points are excluded.
    Z_MIN_M = -0.35
    Z_MAX_M = 0.35
    MIN_RANGE_M = 0.35
    MAX_RANGE_M = 6.0

    # Physical safety envelope in the XY plane. 0.60 m is intentionally larger
    # than the nominal aircraft radius to cover prop guards + localization error.
    INFLATION_RADIUS_M = 0.60
    SELF_CLEAR_RADIUS_M = 0.25

    # ROG-like evidence update, simplified to a signed integer score.
    SCORE_MIN = -6
    SCORE_MAX = 6
    HIT_DELTA = 3
    MISS_DELTA = 1
    OCCUPIED_THRESHOLD = 2

    # Point cloud can arrive faster than this component needs.
    PROCESS_PERIOD_S = 0.20
    CLOUD_STALE_S = 0.60
    MAX_ENDPOINT_CELLS_PER_UPDATE = 1200

    # Stable subgoal behaviour is important because rescue_drop_2026.py stops
    # and restarts its navigation worker whenever safe_waypoint() changes.
    SUBGOAL_REACHED_CM = 15.0
    MAX_SUBGOAL_DISTANCE_CM = 80.0
    GOAL_SAME_EPS_CM = 5.0

    # safe_velocity() looks this far ahead in the desired direction.
    VELOCITY_LOOKAHEAD_CM = 80.0

    def __init__(self, pose_getter: Optional[PoseGetter] = None) -> None:
        self._pose_getter = pose_getter
        self._lock = threading.RLock()

        self._cell_count = int(round(self.MAP_SIZE_M / self.RESOLUTION_M)) + 1
        self._origin_m = -0.5 * self.MAP_SIZE_M
        self._scores = np.zeros((self._cell_count, self._cell_count), dtype=np.int8)

        self._inflation_offsets = self._disk_offsets(self.INFLATION_RADIUS_M)
        self._self_clear_offsets = self._disk_offsets(self.SELF_CLEAR_RADIUS_M)

        self._revision = 0
        self._inflated_revision = -1
        self._inflated_cache: Optional[np.ndarray] = None
        self._last_cloud_at = 0.0
        self._last_processed_at = 0.0

        self._cached_goal: Optional[Point] = None
        self._cached_subgoal: Optional[Point] = None
        self._cached_revision = -1

    # ------------------------------------------------------------------
    # Public lifecycle / data input
    # ------------------------------------------------------------------
    def bind_pose_getter(self, pose_getter: PoseGetter) -> None:
        """Bind ``LioPoseProvider.get_pose`` (or a compatible callable)."""
        if not callable(pose_getter):
            raise TypeError("pose_getter must be callable")
        with self._lock:
            self._pose_getter = pose_getter

    def reset(self) -> None:
        """Clear map evidence and cached planning state."""
        with self._lock:
            self._scores.fill(0)
            self._revision += 1
            self._inflated_revision = -1
            self._inflated_cache = None
            self._last_cloud_at = 0.0
            self._last_processed_at = 0.0
            self._cached_goal = None
            self._cached_subgoal = None
            self._cached_revision = -1

    def ready(self, now: Optional[float] = None) -> bool:
        """Return True when a recently processed point cloud is available."""
        now = time.monotonic() if now is None else float(now)
        with self._lock:
            return self._last_cloud_at > 0.0 and now - self._last_cloud_at <= self.CLOUD_STALE_S

    def on_pointcloud(self, msg) -> None:
        """ROS callback for FAST-LIO ``/cloud_registered_body``.

        ``sensor_msgs_py`` is imported lazily so the component can be imported
        and unit-tested on a non-ROS development machine.
        """
        now = time.monotonic()
        with self._lock:
            if now - self._last_processed_at < self.PROCESS_PERIOD_S:
                return
            pose_getter = self._pose_getter
        if pose_getter is None:
            return
        pose = pose_getter()
        if pose is None or len(pose) < 3 or (len(pose) >= 4 and not bool(pose[3])):
            return

        try:
            from sensor_msgs_py import point_cloud2

            if hasattr(point_cloud2, "read_points_numpy"):
                points = point_cloud2.read_points_numpy(
                    msg, field_names=("x", "y", "z"), skip_nans=True
                )
                points = np.asarray(points)
                if points.dtype.names:
                    points = np.column_stack(
                        [points[name].astype(float, copy=False) for name in ("x", "y", "z")]
                    )
                else:
                    points = np.asarray(points, dtype=float).reshape(-1, 3)
            else:
                points = np.asarray(
                    list(point_cloud2.read_points(
                        msg, field_names=("x", "y", "z"), skip_nans=True
                    )),
                    dtype=float,
                ).reshape(-1, 3)
        except Exception:
            # Let the ROS executor stay alive. Freshness will expire and motion
            # planning will fail closed if point-cloud decoding keeps failing.
            return

        self.update_body_points(points, pose, received_at=now)

    def update_body_points(
        self,
        points_xyz_m: Sequence[Sequence[float]],
        pose: Sequence[float],
        *,
        received_at: Optional[float] = None,
    ) -> int:
        """Update the map from body-frame XYZ points.

        Args:
            points_xyz_m: Nx3 points in metres, +X forward, +Y left, +Z up.
            pose: Navigation startup-local pose ``(x_cm, y_cm, yaw_cw_deg, ...)``.
            received_at: monotonic timestamp, mainly for offline tests.

        Returns:
            Number of unique endpoint cells integrated.
        """
        now = time.monotonic() if received_at is None else float(received_at)
        if len(pose) < 3:
            raise ValueError("pose must contain x_cm, y_cm, yaw_cw_deg")
        if len(pose) >= 4 and not bool(pose[3]):
            return 0
        x0_cm, y0_cm, yaw_cw_deg = map(float, pose[:3])
        if not all(math.isfinite(v) for v in (x0_cm, y0_cm, yaw_cw_deg)):
            raise ValueError("pose contains non-finite values")

        pts = np.asarray(points_xyz_m, dtype=float)
        if pts.size == 0:
            with self._lock:
                self._last_cloud_at = now
                self._last_processed_at = now
            return 0
        pts = pts.reshape(-1, 3)
        finite = np.isfinite(pts).all(axis=1)
        horizontal_sq = pts[:, 0] * pts[:, 0] + pts[:, 1] * pts[:, 1]
        keep = (
            finite
            & (pts[:, 2] >= self.Z_MIN_M)
            & (pts[:, 2] <= self.Z_MAX_M)
            & (horizontal_sq >= self.MIN_RANGE_M * self.MIN_RANGE_M)
            & (horizontal_sq <= self.MAX_RANGE_M * self.MAX_RANGE_M)
        )
        pts = pts[keep]

        # Current Navigation coordinates are startup-local. Yaw is clockwise.
        # This is the inverse of Navigation._world_to_body_velocity().
        x0_m = x0_cm / 100.0
        y0_m = y0_cm / 100.0
        yaw = math.radians(yaw_cw_deg)
        c = math.cos(yaw)
        s = math.sin(yaw)

        if len(pts):
            xw = x0_m + c * pts[:, 0] + s * pts[:, 1]
            yw = y0_m - s * pts[:, 0] + c * pts[:, 1]
            cells = np.column_stack(
                (
                    np.rint((xw - self._origin_m) / self.RESOLUTION_M),
                    np.rint((yw - self._origin_m) / self.RESOLUTION_M),
                )
            ).astype(np.int32)
            inside = (
                (cells[:, 0] >= 0)
                & (cells[:, 0] < self._cell_count)
                & (cells[:, 1] >= 0)
                & (cells[:, 1] < self._cell_count)
            )
            cells = cells[inside]
            if len(cells):
                cells = np.unique(cells, axis=0)
                if len(cells) > self.MAX_ENDPOINT_CELLS_PER_UPDATE:
                    step = int(math.ceil(len(cells) / self.MAX_ENDPOINT_CELLS_PER_UPDATE))
                    cells = cells[::step]
        else:
            cells = np.empty((0, 2), dtype=np.int32)

        start = self._world_to_cell(x0_m, y0_m)
        with self._lock:
            if start is not None:
                sx, sy = start
                for endpoint in cells:
                    ex, ey = int(endpoint[0]), int(endpoint[1])
                    ray = self._bresenham((sx, sy), (ex, ey))
                    # Free-space evidence along the beam; endpoint is hit evidence.
                    for cx, cy in ray[:-1]:
                        value = int(self._scores[cy, cx]) - self.MISS_DELTA
                        self._scores[cy, cx] = max(self.SCORE_MIN, value)
                    if ray:
                        cx, cy = ray[-1]
                        value = int(self._scores[cy, cx]) + self.HIT_DELTA
                        self._scores[cy, cx] = min(self.SCORE_MAX, value)

                # Do not allow returns from the aircraft/prop guard itself to make
                # the aircraft immediately trapped in its own inflated obstacle.
                for dx, dy in self._self_clear_offsets:
                    cx, cy = sx + dx, sy + dy
                    if 0 <= cx < self._cell_count and 0 <= cy < self._cell_count:
                        self._scores[cy, cx] = self.SCORE_MIN

            self._last_cloud_at = now
            self._last_processed_at = now
            self._revision += 1
            self._inflated_revision = -1
            self._inflated_cache = None
        return int(len(cells))

    # ------------------------------------------------------------------
    # Public planning API used by rescue_drop_2026.ObstacleInterface
    # ------------------------------------------------------------------
    def safe_waypoint(self, current_cm: Point, goal_cm: Point) -> Optional[Point]:
        """Return a stable intermediate waypoint toward ``goal_cm``.

        The same subgoal is returned repeatedly until it is reached or its direct
        segment becomes blocked. This avoids restarting the mission navigation
        worker on every 10 Hz check.
        """
        current = self._validate_point(current_cm, "current")
        goal = self._validate_point(goal_cm, "goal")
        blocked, revision = self._fresh_inflated_snapshot()

        start_cell = self._cm_to_cell(current)
        goal_cell = self._cm_to_cell(goal)
        if start_cell is None or goal_cell is None:
            return None
        if blocked[goal_cell[1], goal_cell[0]]:
            return None
        blocked = blocked.copy()
        self._clear_start_in_snapshot(blocked, start_cell)

        if self._grid_line_free(blocked, start_cell, goal_cell):
            with self._lock:
                self._cached_goal = goal
                self._cached_subgoal = goal
                self._cached_revision = revision
            return goal

        with self._lock:
            cached_goal = self._cached_goal
            cached_subgoal = self._cached_subgoal

        same_goal = cached_goal is not None and self._distance_cm(cached_goal, goal) <= self.GOAL_SAME_EPS_CM
        if same_goal and cached_subgoal is not None:
            if self._distance_cm(current, cached_subgoal) > self.SUBGOAL_REACHED_CM:
                sub_cell = self._cm_to_cell(cached_subgoal)
                if sub_cell is not None and self._grid_line_free(blocked, start_cell, sub_cell):
                    return cached_subgoal

        path = self._plan_cells(blocked, start_cell, goal_cell, snap_goal=False)
        if not path:
            with self._lock:
                self._cached_goal = goal
                self._cached_subgoal = None
                self._cached_revision = revision
            return None

        simplified = self._simplify_cells(blocked, path)
        if len(simplified) <= 1:
            subgoal = goal
        else:
            raw_subgoal = self._cell_to_cm(simplified[1])
            subgoal = self._cap_subgoal(current, raw_subgoal, self.MAX_SUBGOAL_DISTANCE_CM)
            # If the path directly reaches the real goal, return the exact user
            # coordinate rather than the centre of the goal grid cell.
            if len(simplified) == 2 and simplified[-1] == goal_cell:
                subgoal = self._cap_subgoal(current, goal, self.MAX_SUBGOAL_DISTANCE_CM)

        with self._lock:
            self._cached_goal = goal
            self._cached_subgoal = subgoal
            self._cached_revision = revision
        return subgoal

    def safe_velocity(self, current_cm: Point, velocity_cm_s: Point) -> Point:
        """Keep/redirect a visual-approach velocity using the current map.

        This intentionally does not share the stable waypoint cache because the
        camera approach direction changes continuously. If no local path exists,
        return zero horizontal velocity so the caller hovers instead of charging
        an obstacle.
        """
        current = self._validate_point(current_cm, "current")
        velocity = self._validate_point(velocity_cm_s, "velocity")
        blocked, _ = self._fresh_inflated_snapshot()
        speed = math.hypot(velocity[0], velocity[1])
        if speed < 1.0:
            return velocity

        start_cell = self._cm_to_cell(current)
        if start_cell is None:
            return (0.0, 0.0)

        ux, uy = velocity[0] / speed, velocity[1] / speed
        desired = (
            current[0] + ux * self.VELOCITY_LOOKAHEAD_CM,
            current[1] + uy * self.VELOCITY_LOOKAHEAD_CM,
        )
        goal_cell = self._nearest_inside_cell(current, desired)
        if goal_cell is None:
            return (0.0, 0.0)
        if blocked[goal_cell[1], goal_cell[0]]:
            # Find a real free endpoint before carving out the aircraft's start
            # area; that temporary clearance must never masquerade as an exit.
            goal_cell = self._nearest_free_cell(blocked, goal_cell, max_radius_cells=6)
            if goal_cell is None:
                return (0.0, 0.0)
            redirected = True
        else:
            redirected = False

        blocked = blocked.copy()
        self._clear_start_in_snapshot(blocked, start_cell)

        if not redirected and self._grid_line_free(blocked, start_cell, goal_cell):
            return velocity

        path = self._plan_cells(blocked, start_cell, goal_cell, snap_goal=False)
        if not path:
            return (0.0, 0.0)
        simplified = self._simplify_cells(blocked, path)
        target_cell = simplified[1] if len(simplified) > 1 else simplified[0]
        target = self._cell_to_cm(target_cell)
        dx, dy = target[0] - current[0], target[1] - current[1]
        norm = math.hypot(dx, dy)
        if norm < 1e-6:
            return (0.0, 0.0)
        return (speed * dx / norm, speed * dy / norm)

    def plan_path(self, start_cm: Point, goal_cm: Point) -> Optional[List[Point]]:
        """Offline/debug helper returning a simplified path in cm."""
        start = self._validate_point(start_cm, "start")
        goal = self._validate_point(goal_cm, "goal")
        blocked, _ = self._fresh_inflated_snapshot()
        start_cell, goal_cell = self._cm_to_cell(start), self._cm_to_cell(goal)
        if start_cell is None or goal_cell is None:
            return None
        if blocked[goal_cell[1], goal_cell[0]]:
            return None
        blocked = blocked.copy()
        self._clear_start_in_snapshot(blocked, start_cell)
        cells = self._plan_cells(blocked, start_cell, goal_cell, snap_goal=False)
        if not cells:
            return None
        cells = self._simplify_cells(blocked, cells)
        points = [self._cell_to_cm(cell) for cell in cells]
        points[0] = start
        points[-1] = goal
        return points

    def get_debug_state(self) -> dict:
        """Small diagnostic snapshot; safe to print from bench scripts."""
        now = time.monotonic()
        with self._lock:
            occupied = self._scores >= self.OCCUPIED_THRESHOLD
            age = float("inf") if self._last_cloud_at <= 0 else now - self._last_cloud_at
            revision = self._revision
            last_cloud_at = self._last_cloud_at
        inflated = self._inflate(occupied)
        return {
            "ready": bool(last_cloud_at > 0 and age <= self.CLOUD_STALE_S),
            "cloud_age_s": age,
            "revision": revision,
            "occupied_cells": int(np.count_nonzero(occupied)),
            "inflated_cells": int(np.count_nonzero(inflated)),
            "resolution_m": self.RESOLUTION_M,
            "map_size_m": self.MAP_SIZE_M,
        }

    # ------------------------------------------------------------------
    # Occupancy helpers
    # ------------------------------------------------------------------
    def _fresh_inflated_snapshot(self) -> Tuple[np.ndarray, int]:
        now = time.monotonic()
        with self._lock:
            if self._last_cloud_at <= 0.0 or now - self._last_cloud_at > self.CLOUD_STALE_S:
                raise RuntimeError("2-D obstacle point cloud is missing or stale")
            revision = self._revision
            if self._inflated_cache is None or self._inflated_revision != revision:
                occupied = self._scores >= self.OCCUPIED_THRESHOLD
                self._inflated_cache = self._inflate(occupied)
                self._inflated_revision = revision
            return self._inflated_cache.copy(), revision

    def _inflate(self, occupied: np.ndarray) -> np.ndarray:
        out = np.zeros_like(occupied, dtype=bool)
        ys, xs = np.nonzero(occupied)
        if len(xs) == 0:
            return out
        for dx, dy in self._inflation_offsets:
            nx = xs + dx
            ny = ys + dy
            valid = (nx >= 0) & (nx < self._cell_count) & (ny >= 0) & (ny < self._cell_count)
            out[ny[valid], nx[valid]] = True
        return out

    def _disk_offsets(self, radius_m: float) -> List[Tuple[int, int]]:
        radius_cells = int(math.ceil(radius_m / self.RESOLUTION_M))
        radius_sq = (radius_m / self.RESOLUTION_M) ** 2
        offsets = []
        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                if dx * dx + dy * dy <= radius_sq + 1e-9:
                    offsets.append((dx, dy))
        return offsets

    def _clear_start_in_snapshot(self, blocked: np.ndarray, start: Tuple[int, int]) -> None:
        sx, sy = start
        for dx, dy in self._self_clear_offsets:
            x, y = sx + dx, sy + dy
            if 0 <= x < self._cell_count and 0 <= y < self._cell_count:
                blocked[y, x] = False

    # ------------------------------------------------------------------
    # Grid geometry
    # ------------------------------------------------------------------
    def _world_to_cell(self, x_m: float, y_m: float) -> Optional[Tuple[int, int]]:
        ix = int(round((float(x_m) - self._origin_m) / self.RESOLUTION_M))
        iy = int(round((float(y_m) - self._origin_m) / self.RESOLUTION_M))
        if 0 <= ix < self._cell_count and 0 <= iy < self._cell_count:
            return ix, iy
        return None

    def _cm_to_cell(self, point_cm: Point) -> Optional[Tuple[int, int]]:
        return self._world_to_cell(point_cm[0] / 100.0, point_cm[1] / 100.0)

    def _cell_to_cm(self, cell: Tuple[int, int]) -> Point:
        ix, iy = cell
        return (
            (self._origin_m + ix * self.RESOLUTION_M) * 100.0,
            (self._origin_m + iy * self.RESOLUTION_M) * 100.0,
        )

    @staticmethod
    def _bresenham(start: Tuple[int, int], end: Tuple[int, int]) -> List[Tuple[int, int]]:
        x0, y0 = start
        x1, y1 = end
        points: List[Tuple[int, int]] = []
        dx = abs(x1 - x0)
        sx = 1 if x0 < x1 else -1
        dy = -abs(y1 - y0)
        sy = 1 if y0 < y1 else -1
        err = dx + dy
        while True:
            points.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x0 += sx
            if e2 <= dx:
                err += dx
                y0 += sy
        return points

    def _grid_line_free(
        self,
        blocked: np.ndarray,
        start: Tuple[int, int],
        end: Tuple[int, int],
    ) -> bool:
        line = self._bresenham(start, end)
        previous = line[0]
        for cell in line[1:]:
            x, y = cell
            if blocked[y, x]:
                return False
            px, py = previous
            dx, dy = x - px, y - py
            if dx != 0 and dy != 0:
                # Do not squeeze diagonally between two touching inflated cells.
                if blocked[py, x] or blocked[y, px]:
                    return False
            previous = cell
        return True

    # ------------------------------------------------------------------
    # A* + line-of-sight simplification
    # ------------------------------------------------------------------
    def _plan_cells(
        self,
        blocked: np.ndarray,
        start: Tuple[int, int],
        goal: Tuple[int, int],
        *,
        snap_goal: bool,
    ) -> Optional[List[Tuple[int, int]]]:
        if blocked[goal[1], goal[0]]:
            if not snap_goal:
                return None
            replacement = self._nearest_free_cell(blocked, goal, max_radius_cells=6)
            if replacement is None:
                return None
            goal = replacement
        if start == goal:
            return [start]

        n = self._cell_count
        g_score = np.full((n, n), np.inf, dtype=np.float32)
        closed = np.zeros((n, n), dtype=bool)
        parent_x = np.full((n, n), -1, dtype=np.int16)
        parent_y = np.full((n, n), -1, dtype=np.int16)

        sx, sy = start
        gx, gy = goal
        g_score[sy, sx] = 0.0
        heap: List[Tuple[float, float, int, int]] = [
            (math.hypot(gx - sx, gy - sy), 0.0, sx, sy)
        ]
        neighbours = (
            (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
            (1, 1, math.sqrt(2.0)), (1, -1, math.sqrt(2.0)),
            (-1, 1, math.sqrt(2.0)), (-1, -1, math.sqrt(2.0)),
        )

        found = False
        while heap:
            _, g_here, x, y = heapq.heappop(heap)
            if closed[y, x]:
                continue
            closed[y, x] = True
            if (x, y) == goal:
                found = True
                break

            for dx, dy, step_cost in neighbours:
                nx, ny = x + dx, y + dy
                if nx < 0 or nx >= n or ny < 0 or ny >= n:
                    continue
                if blocked[ny, nx] or closed[ny, nx]:
                    continue
                if dx != 0 and dy != 0:
                    if blocked[y, nx] or blocked[ny, x]:
                        continue
                candidate = g_here + step_cost
                if candidate + 1e-6 >= float(g_score[ny, nx]):
                    continue
                g_score[ny, nx] = candidate
                parent_x[ny, nx] = x
                parent_y[ny, nx] = y
                h = math.hypot(gx - nx, gy - ny)
                heapq.heappush(heap, (candidate + h, candidate, nx, ny))

        if not found:
            return None

        path = [goal]
        x, y = goal
        while (x, y) != start:
            px, py = int(parent_x[y, x]), int(parent_y[y, x])
            if px < 0 or py < 0:
                return None
            x, y = px, py
            path.append((x, y))
        path.reverse()
        return path

    def _simplify_cells(
        self, blocked: np.ndarray, path: List[Tuple[int, int]]
    ) -> List[Tuple[int, int]]:
        if len(path) <= 2:
            return path
        simplified = [path[0]]
        i = 0
        while i < len(path) - 1:
            chosen = i + 1
            for j in range(len(path) - 1, i, -1):
                if self._grid_line_free(blocked, path[i], path[j]):
                    chosen = j
                    break
            simplified.append(path[chosen])
            i = chosen
        return simplified

    def _nearest_free_cell(
        self,
        blocked: np.ndarray,
        origin: Tuple[int, int],
        *,
        max_radius_cells: int,
    ) -> Optional[Tuple[int, int]]:
        ox, oy = origin
        candidates: List[Tuple[float, int, int]] = []
        for radius in range(1, max_radius_cells + 1):
            for dy in range(-radius, radius + 1):
                for dx in range(-radius, radius + 1):
                    if max(abs(dx), abs(dy)) != radius:
                        continue
                    x, y = ox + dx, oy + dy
                    if 0 <= x < self._cell_count and 0 <= y < self._cell_count and not blocked[y, x]:
                        candidates.append((dx * dx + dy * dy, x, y))
            if candidates:
                _, x, y = min(candidates)
                return x, y
        return None

    def _nearest_inside_cell(self, start_cm: Point, desired_cm: Point) -> Optional[Tuple[int, int]]:
        cell = self._cm_to_cell(desired_cm)
        if cell is not None:
            return cell
        # Shorten the look-ahead instead of clipping each axis independently.
        for factor in (0.8, 0.6, 0.4, 0.2):
            candidate = (
                start_cm[0] + (desired_cm[0] - start_cm[0]) * factor,
                start_cm[1] + (desired_cm[1] - start_cm[1]) * factor,
            )
            cell = self._cm_to_cell(candidate)
            if cell is not None:
                return cell
        return None

    # ------------------------------------------------------------------
    # Misc helpers
    # ------------------------------------------------------------------
    @staticmethod
    def _validate_point(value: Sequence[float], name: str) -> Point:
        if len(value) < 2:
            raise ValueError(f"{name} must contain x and y")
        point = (float(value[0]), float(value[1]))
        if not all(math.isfinite(v) for v in point):
            raise ValueError(f"{name} contains non-finite values")
        return point

    @staticmethod
    def _distance_cm(a: Point, b: Point) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])

    @staticmethod
    def _cap_subgoal(current: Point, target: Point, max_distance_cm: float) -> Point:
        dx, dy = target[0] - current[0], target[1] - current[1]
        distance = math.hypot(dx, dy)
        if distance <= max_distance_cm or distance < 1e-9:
            return (float(target[0]), float(target[1]))
        scale = max_distance_cm / distance
        return (current[0] + dx * scale, current[1] + dy * scale)
