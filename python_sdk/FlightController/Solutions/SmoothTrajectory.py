"""二维平滑轨迹生成器。

该模块只负责将一组有序二维航点转换为可供 ``Navigation`` 使用的
``traj_list``，不启动飞控、传感器或任何硬件线程。

几何路径使用弦长参数化的二维三次插值样条；速度规划使用弧长网格、
曲率速度包络、横向加速度限制和前向/反向切向加速度约束。生成器保留
所有原始航点，并可同时输出旧版 ``(x, y, z)`` 格式与带时间/速度信息
的扩展格式。
"""

from dataclasses import dataclass, replace
from typing import List, Optional, Sequence, Tuple

import numpy as np
from scipy.interpolate import CubicSpline


@dataclass(frozen=True)
class SplineTrajectoryConfig:
    """平滑轨迹参数，长度单位为 cm，时间单位为 s。"""

    navi_speed: float
    control_dt: float = 0.1
    geometry_sample_ds: float = 1.0
    curve_speed_ratio: float = 0.6
    turn_radius_ref: float = 80.0
    curvature_power: float = 2.0
    max_tangential_acc: float = 20.0
    max_tangential_decel: float = 25.0
    max_lateral_acc: float = 30.0
    duplicate_tolerance: float = 1e-3
    max_geometry_samples: int = 100000

    def validated(self) -> "SplineTrajectoryConfig":
        positive_values = {
            "navi_speed": self.navi_speed,
            "control_dt": self.control_dt,
            "geometry_sample_ds": self.geometry_sample_ds,
            "turn_radius_ref": self.turn_radius_ref,
            "curvature_power": self.curvature_power,
            "max_tangential_acc": self.max_tangential_acc,
            "max_tangential_decel": self.max_tangential_decel,
            "max_lateral_acc": self.max_lateral_acc,
        }
        for name, value in positive_values.items():
            if not np.isfinite(value) or value <= 0:
                raise ValueError("{} must be a finite positive number".format(name))
        if not np.isfinite(self.curve_speed_ratio) or not 0 < self.curve_speed_ratio <= 1:
            raise ValueError("curve_speed_ratio must be in (0, 1]")
        if not np.isfinite(self.duplicate_tolerance) or self.duplicate_tolerance < 0:
            raise ValueError("duplicate_tolerance must be a finite non-negative number")
        if self.max_geometry_samples < 2:
            raise ValueError("max_geometry_samples must be at least 2")
        return self

    def with_navi_speed(self, speed: float) -> "SplineTrajectoryConfig":
        """返回使用指定巡航速度的配置副本。"""

        return replace(self, navi_speed=float(speed)).validated()


@dataclass(frozen=True)
class TrajectorySample:
    """一条带时间和速度规划信息的二维轨迹样本。"""

    x: float
    y: float
    z: float
    t: float
    vx: float
    vy: float
    speed_limit: float
    curvature: float
    arc_length: float
    waypoint_index: int = -1

    def as_legacy_point(self) -> Tuple[float, float, float]:
        return (self.x, self.y, self.z)

    def as_extended_point(self) -> Tuple[float, float, float, float, float, float, float]:
        return (self.x, self.y, self.z, self.t, self.vx, self.vy, self.speed_limit)


class SplineTrajectoryGenerator:
    """从二维航点生成弧长重参数化的 C2 平滑轨迹。"""

    _NUMERIC_EPS = 1e-9

    def __init__(
        self,
        waypoints: Sequence[Sequence[float]],
        altitude: float,
        config: SplineTrajectoryConfig,
    ) -> None:
        self.config = config.validated()
        if not np.isfinite(altitude):
            raise ValueError("altitude must be finite")
        self.altitude = float(altitude)
        self.waypoints = self._prepare_waypoints(waypoints)
        self._samples: Optional[List[TrajectorySample]] = None

    def _prepare_waypoints(self, waypoints: Sequence[Sequence[float]]) -> np.ndarray:
        points = np.asarray(waypoints, dtype=float)
        if points.ndim != 2 or points.shape[1] < 2:
            raise ValueError("waypoints must have shape (N, 2) or more columns")
        if points.shape[0] == 0:
            raise ValueError("waypoints must contain at least one point")
        points = points[:, :2]
        if not np.all(np.isfinite(points)):
            raise ValueError("waypoints must contain only finite coordinates")

        kept = [points[0]]
        for point in points[1:]:
            if float(np.linalg.norm(point - kept[-1])) > self.config.duplicate_tolerance:
                kept.append(point)
        return np.asarray(kept, dtype=float)

    def _build_geometry(self):
        points = self.waypoints
        if len(points) == 1:
            zeros = np.array([0.0])
            return None, zeros, zeros, zeros, zeros

        chord_lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
        parameters = np.concatenate(([0.0], np.cumsum(chord_lengths)))

        if len(points) == 2:
            spline = None
        else:
            start_tangent = (points[1] - points[0]) / chord_lengths[0]
            end_tangent = (points[-1] - points[-2]) / chord_lengths[-1]
            spline = CubicSpline(
                parameters,
                points,
                axis=0,
                bc_type=((1, start_tangent), (1, end_tangent)),
                extrapolate=False,
            )

        parameter_chunks = []
        estimated_count = 1
        for index, segment_length in enumerate(chord_lengths):
            segment_count = max(
                5,
                int(np.ceil(segment_length / self.config.geometry_sample_ds)) + 1,
            )
            estimated_count += segment_count - 1
            if estimated_count > self.config.max_geometry_samples:
                raise ValueError(
                    "geometry sampling exceeds max_geometry_samples; "
                    "increase geometry_sample_ds or the configured limit"
                )
            parameter_chunks.append(
                np.linspace(
                    parameters[index],
                    parameters[index + 1],
                    segment_count,
                    endpoint=index == len(chord_lengths) - 1,
                )
            )

        parameter_grid = np.concatenate(parameter_chunks)
        positions = self._evaluate(parameter_grid, spline, parameters, derivative=0)
        arc_steps = np.linalg.norm(np.diff(positions, axis=0), axis=1)
        arc_grid = np.concatenate(([0.0], np.cumsum(arc_steps)))

        strictly_increasing = np.concatenate(
            ([True], np.diff(arc_grid) > self._NUMERIC_EPS)
        )
        parameter_grid = parameter_grid[strictly_increasing]
        arc_grid = arc_grid[strictly_increasing]
        if len(arc_grid) < 2 or arc_grid[-1] <= self._NUMERIC_EPS:
            zeros = np.array([0.0])
            return None, zeros, zeros, zeros, zeros

        waypoint_arc = np.interp(parameters, parameter_grid, arc_grid)
        return spline, parameters, parameter_grid, arc_grid, waypoint_arc

    def _evaluate(self, values, spline, parameters, derivative=0) -> np.ndarray:
        values = np.asarray(values, dtype=float)
        if spline is not None:
            return np.asarray(spline(values, derivative), dtype=float)

        start = self.waypoints[0]
        end = self.waypoints[-1]
        length = float(parameters[-1] - parameters[0])
        if derivative == 0:
            ratio = ((values - parameters[0]) / length)[..., np.newaxis]
            return start + ratio * (end - start)
        if derivative == 1:
            tangent = (end - start) / length
            return np.broadcast_to(tangent, values.shape + (2,)).copy()
        return np.zeros(values.shape + (2,), dtype=float)

    def _curvature(self, parameters, spline, spline_parameters) -> np.ndarray:
        first = self._evaluate(
            parameters, spline, spline_parameters, derivative=1
        )
        second = self._evaluate(
            parameters, spline, spline_parameters, derivative=2
        )
        numerator = np.abs(first[:, 0] * second[:, 1] - first[:, 1] * second[:, 0])
        denominator = np.power(np.sum(first * first, axis=1), 1.5)
        curvature = np.empty_like(numerator)
        regular = denominator > self._NUMERIC_EPS
        curvature[regular] = numerator[regular] / denominator[regular]
        curvature[~regular] = 1.0 / self._NUMERIC_EPS
        return curvature

    def _plan_speed(self, arc_grid: np.ndarray, curvature: np.ndarray) -> np.ndarray:
        config = self.config
        curvature_ref = 1.0 / config.turn_radius_ref
        curvature_ratio = np.abs(curvature) / curvature_ref
        curve_limit = config.navi_speed * (
            config.curve_speed_ratio
            + (1.0 - config.curve_speed_ratio)
            / (1.0 + np.power(curvature_ratio, config.curvature_power))
        )
        lateral_limit = np.sqrt(
            config.max_lateral_acc
            / np.maximum(np.abs(curvature), self._NUMERIC_EPS)
        )
        speed = np.minimum(
            np.minimum(curve_limit, lateral_limit),
            config.navi_speed,
        )
        speed[0] = 0.0
        speed[-1] = 0.0

        for index in range(1, len(speed)):
            ds = arc_grid[index] - arc_grid[index - 1]
            reachable = np.sqrt(
                max(0.0, speed[index - 1] ** 2 + 2.0 * config.max_tangential_acc * ds)
            )
            speed[index] = min(speed[index], reachable)

        for index in range(len(speed) - 2, -1, -1):
            ds = arc_grid[index + 1] - arc_grid[index]
            reachable = np.sqrt(
                max(0.0, speed[index + 1] ** 2 + 2.0 * config.max_tangential_decel * ds)
            )
            speed[index] = min(speed[index], reachable)
        return speed

    def _build_time_grid(self, arc_grid: np.ndarray, speed: np.ndarray) -> np.ndarray:
        segment_distance = np.diff(arc_grid)
        speed_sum = speed[:-1] + speed[1:]
        segment_time = np.empty_like(segment_distance)
        regular = speed_sum > self._NUMERIC_EPS
        segment_time[regular] = 2.0 * segment_distance[regular] / speed_sum[regular]

        fallback_acc = min(
            self.config.max_tangential_acc,
            self.config.max_tangential_decel,
        )
        segment_time[~regular] = 2.0 * np.sqrt(
            segment_distance[~regular] / fallback_acc
        )
        return np.concatenate(([0.0], np.cumsum(segment_time)))

    def generate(self) -> List[TrajectorySample]:
        """生成带时间、速度、曲率和弧长信息的轨迹样本。"""

        if self._samples is not None:
            return list(self._samples)

        if len(self.waypoints) == 1:
            point = self.waypoints[0]
            self._samples = [
                TrajectorySample(
                    x=float(point[0]),
                    y=float(point[1]),
                    z=self.altitude,
                    t=0.0,
                    vx=0.0,
                    vy=0.0,
                    speed_limit=0.0,
                    curvature=0.0,
                    arc_length=0.0,
                    waypoint_index=0,
                )
            ]
            return list(self._samples)

        geometry = self._build_geometry()
        spline, spline_parameters, parameter_grid, arc_grid, waypoint_arc = geometry
        curvature_grid = self._curvature(
            parameter_grid, spline, spline_parameters
        )
        speed_grid = self._plan_speed(arc_grid, curvature_grid)
        time_grid = self._build_time_grid(arc_grid, speed_grid)
        waypoint_time = np.interp(waypoint_arc, arc_grid, time_grid)

        regular_time = np.arange(
            0.0,
            time_grid[-1],
            self.config.control_dt,
            dtype=float,
        )
        if len(regular_time) == 1 and time_grid[-1] > self._NUMERIC_EPS:
            regular_time = np.array([0.0, 0.5 * time_grid[-1]])
        sample_time = np.unique(
            np.round(
                np.concatenate((regular_time, waypoint_time, [time_grid[-1]])),
                decimals=12,
            )
        )
        sample_arc = np.interp(sample_time, time_grid, arc_grid)
        sample_parameters = np.interp(sample_arc, arc_grid, parameter_grid)
        sample_position = self._evaluate(
            sample_parameters, spline, spline_parameters, derivative=0
        )
        sample_first = self._evaluate(
            sample_parameters, spline, spline_parameters, derivative=1
        )
        tangent_norm = np.linalg.norm(sample_first, axis=1)
        tangent = np.zeros_like(sample_first)
        regular_tangent = tangent_norm > self._NUMERIC_EPS
        tangent[regular_tangent] = (
            sample_first[regular_tangent]
            / tangent_norm[regular_tangent, np.newaxis]
        )
        sample_speed = np.interp(sample_arc, arc_grid, speed_grid)
        sample_velocity = tangent * sample_speed[:, np.newaxis]
        sample_curvature = np.interp(sample_arc, arc_grid, curvature_grid)

        waypoint_by_sample = {}
        for waypoint_index, passage_time in enumerate(waypoint_time):
            sample_index = int(np.argmin(np.abs(sample_time - passage_time)))
            waypoint_by_sample[sample_index] = waypoint_index
            sample_position[sample_index] = self.waypoints[waypoint_index]

        samples = []
        for index in range(len(sample_time)):
            samples.append(
                TrajectorySample(
                    x=float(sample_position[index, 0]),
                    y=float(sample_position[index, 1]),
                    z=self.altitude,
                    t=float(sample_time[index]),
                    vx=float(sample_velocity[index, 0]),
                    vy=float(sample_velocity[index, 1]),
                    speed_limit=float(sample_speed[index]),
                    curvature=float(sample_curvature[index]),
                    arc_length=float(sample_arc[index]),
                    waypoint_index=waypoint_by_sample.get(index, -1),
                )
            )

        first = samples[0]
        last = samples[-1]
        samples[0] = replace(
            first,
            x=float(self.waypoints[0, 0]),
            y=float(self.waypoints[0, 1]),
            vx=0.0,
            vy=0.0,
            speed_limit=0.0,
            waypoint_index=0,
        )
        samples[-1] = replace(
            last,
            x=float(self.waypoints[-1, 0]),
            y=float(self.waypoints[-1, 1]),
            vx=0.0,
            vy=0.0,
            speed_limit=0.0,
            waypoint_index=len(self.waypoints) - 1,
        )
        self._samples = samples
        return list(self._samples)

    def generate_traj_list(self, extended: bool = False) -> List[Tuple[float, ...]]:
        """输出可传给 ``navigation_follow_trajectory`` 的轨迹列表。

        ``extended=False`` 返回完全兼容现有执行器的 ``(x, y, z)``。
        ``extended=True`` 返回 ``(x, y, z, t, vx, vy, speed_limit)``；当前
        执行器会安全地忽略前三项以外的字段，时间/速度字段主要供后续
        按时间跟踪与离线验收使用。
        """

        samples = self.generate()
        if extended:
            return [sample.as_extended_point() for sample in samples]
        return [sample.as_legacy_point() for sample in samples]


__all__ = [
    "SplineTrajectoryConfig",
    "SplineTrajectoryGenerator",
    "TrajectorySample",
]
