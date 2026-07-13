"""Warehouse-specific radar localization from LD radar polar metadata.

This module intentionally does not call the legacy Radar_SLAM/Hough flow and
does not send any flight-control command.  It reads a copied snapshot from
``radar.map`` and estimates only the observable pose components requested by
the upper task state machine.
"""

from __future__ import annotations

import math
import os
import time
from collections import defaultdict, deque
from dataclasses import dataclass
from enum import Enum, auto
from threading import RLock
from typing import Any, Deque

import numpy as np

try:
    from loguru import logger
except ImportError:  # pragma: no cover - the project normally provides loguru.
    import logging

    logger = logging.getLogger(__name__)  # type: ignore[assignment]


class SurfaceID(Enum):
    """Known finite surfaces in the warehouse inventory field."""

    WEST_NET = auto()
    EAST_NET = auto()
    SOUTH_NET = auto()
    NORTH_NET = auto()
    SHELF_AB = auto()
    SHELF_CD = auto()


class LocalizationMode(Enum):
    """Pose components requested from the radar localizer."""

    ABSOLUTE_ANCHOR = auto()
    CORRIDOR_TRACK = auto()
    YAW_ONLY = auto()
    DETECTION_ONLY = auto()


@dataclass(frozen=True)
class WarehouseMapConfig:
    """Warehouse field geometry and radar installation offset, in centimeters."""

    field_x_min_cm: float = -75.0
    field_x_max_cm: float = 425.0
    field_y_min_cm: float = -75.0
    field_y_max_cm: float = 325.0

    shelf_ab_x_cm: float = 75.0
    shelf_cd_x_cm: float = 275.0
    shelf_y_min_cm: float = 25.0
    shelf_y_max_cm: float = 225.0

    radar_offset_x_cm: float = 0.0
    radar_offset_y_cm: float = 0.0
    radar_yaw_offset_deg: float = 0.0


@dataclass(frozen=True)
class LineSegment2D:
    """Finite line segment in the fixed field frame."""

    surface_id: SurfaceID
    p1: tuple[float, float]
    p2: tuple[float, float]


@dataclass
class Pose2D:
    """2D drone-center pose in the field frame."""

    x_cm: float
    y_cm: float
    yaw_deg: float
    timestamp_s: float | None = None


@dataclass
class LocalizationRequest:
    """Localization query constrained by trusted surfaces from the task state."""

    mode: LocalizationMode
    trusted_surfaces: tuple[SurfaceID, ...]
    prior_pose: Pose2D
    max_position_deviation_cm: float = 35.0
    max_yaw_deviation_deg: float = 12.0
    allow_partial_result: bool = True


@dataclass
class SurfaceMatch:
    """Single known-surface observation result."""

    surface_id: SurfaceID
    valid: bool

    line_point_body_cm: tuple[float, float] | None
    line_direction_body: tuple[float, float] | None
    line_normal_body: tuple[float, float] | None

    signed_distance_cm: float | None
    absolute_distance_cm: float | None
    line_angle_body_deg: float | None

    inlier_count: int
    total_candidate_count: int
    inlier_ratio: float
    residual_rms_cm: float | None
    support_length_cm: float | None

    expected_distance_cm: float | None
    expected_angle_body_deg: float | None
    distance_error_cm: float | None
    angle_error_deg: float | None

    confidence: float
    timestamp_s: float
    reason: str


@dataclass
class RadarLocalizationResult:
    """Final radar localization result with explicit observable components."""

    x_cm: float | None
    y_cm: float | None
    yaw_deg: float | None

    valid_x: bool
    valid_y: bool
    valid_yaw: bool

    matched_surfaces: tuple[SurfaceID, ...]
    surface_matches: dict[SurfaceID, SurfaceMatch]

    position_std_cm: float | None
    yaw_std_deg: float | None
    residual_cm: float | None
    confidence: float

    absolute_fix: bool
    partial_fix: bool
    reason: str
    timestamp_s: float


@dataclass(frozen=True)
class RawRadarPoint:
    """Raw polar radar point copied from the latest radar map snapshot."""

    angle_deg: float
    distance_mm: float
    confidence: float
    timestamp_s: float


@dataclass
class ExpectedSurfaceObservation:
    """Predicted finite-surface observation in the body frame."""

    surface_id: SurfaceID
    expected_distance_cm: float
    expected_line_angle_body_deg: float
    expected_normal_body_deg: float
    segment_body_cm: np.ndarray
    projection_inside_segment: bool


@dataclass
class LineFitResult:
    """RANSAC and TLS line fit result."""

    valid: bool
    point: np.ndarray | None
    direction: np.ndarray | None
    normal: np.ndarray | None
    inlier_mask: np.ndarray | None
    inlier_count: int
    residual_rms_cm: float | None
    support_length_cm: float | None
    reason: str


@dataclass
class EndpointDetection:
    """Shelf endpoint observation for task-state assistance only."""

    detected: bool
    shelf_id: SurfaceID
    endpoint: str
    estimated_position_body_cm: tuple[float, float] | None
    confidence: float
    reason: str


@dataclass
class SectorDistanceStats:
    """Robust distance statistics in a radar angle sector."""

    valid_count: int
    min_cm: float | None
    median_cm: float | None
    percentile_20_cm: float | None
    confidence_mean: float | None


@dataclass
class DebugOptions:
    """Optional debug visualization settings.  Drawing is off by default."""

    enabled: bool = False
    show_window: bool = False
    save_directory: str | None = None
    save_every_n_frames: int = 10


@dataclass
class RadarAlgorithmConfig:
    """Centralized thresholds for warehouse radar localization."""

    min_distance_cm: float = 10.0
    max_distance_cm: float = 650.0
    min_confidence: float = 30.0
    max_point_age_s: float = 0.35

    expected_angle_gate_deg: float = 25.0
    expected_distance_gate_cm: float = 45.0
    segment_extension_cm: float = 25.0

    ransac_distance_threshold_cm: float = 4.0
    ransac_iterations: int = 120
    min_inlier_count: int = 12
    min_inlier_ratio: float = 0.25
    min_support_length_cm: float = 35.0
    max_residual_rms_cm: float = 5.0

    max_surface_angle_error_deg: float = 10.0
    max_surface_distance_error_cm: float = 30.0

    corridor_width_cm: float = 200.0
    corridor_width_tolerance_cm: float = 15.0

    temporal_confirm_frames: int = 3
    temporal_history_size: int = 5
    temporal_distance_std_cm: float = 5.0
    temporal_angle_std_deg: float = 3.0

    parallel_surface_consistency_cm: float = 15.0
    endpoint_roi_radius_cm: float = 18.0
    endpoint_min_points: int = 4


@dataclass
class _PreparedPoints:
    body_xy_cm: np.ndarray
    confidence: np.ndarray
    raw_angle_deg: np.ndarray
    raw_distance_cm: np.ndarray
    timestamp_s: np.ndarray

    @property
    def count(self) -> int:
        return int(self.body_xy_cm.shape[0])


@dataclass
class _SurfaceDiagnostics:
    expected: ExpectedSurfaceObservation | None
    candidate_points: np.ndarray
    inlier_points: np.ndarray


def normalize_angle_deg(angle_deg: float) -> float:
    """Normalize an angle to [-180, 180) degrees."""

    return (float(angle_deg) + 180.0) % 360.0 - 180.0


def angle_diff_deg(angle_a_deg: float, angle_b_deg: float) -> float:
    """Return signed ``angle_a - angle_b`` in [-180, 180) degrees."""

    return normalize_angle_deg(angle_a_deg - angle_b_deg)


def undirected_angle_diff_deg(angle_a_deg: float, angle_b_deg: float) -> float:
    """Smallest difference between two unoriented line angles."""

    diff = abs(angle_diff_deg(angle_a_deg, angle_b_deg))
    return min(diff, 180.0 - diff)


def normalize_line_angle_deg(angle_deg: float) -> float:
    """Normalize an unoriented line angle to [-90, 90) degrees."""

    angle = normalize_angle_deg(angle_deg)
    if angle >= 90.0:
        angle -= 180.0
    if angle < -90.0:
        angle += 180.0
    return angle


def vector_angle_clockwise_deg(vector_xy: np.ndarray) -> float:
    """Angle of a vector, where +X is 0 deg and clockwise is positive."""

    return normalize_angle_deg(math.degrees(math.atan2(-float(vector_xy[1]), float(vector_xy[0]))))


def circular_mean_deg(values: list[float], weights: list[float]) -> float:
    """Weighted circular mean of directed angles in degrees."""

    if not values:
        raise ValueError("circular_mean_deg requires at least one value")
    weights_array = np.asarray(weights, dtype=float)
    if np.all(weights_array <= 0.0):
        weights_array = np.ones_like(weights_array)
    radians = np.deg2rad(np.asarray(values, dtype=float))
    sin_sum = float(np.sum(np.sin(radians) * weights_array))
    cos_sum = float(np.sum(np.cos(radians) * weights_array))
    return normalize_angle_deg(math.degrees(math.atan2(sin_sum, cos_sum)))


def circular_std_deg(values: list[float], center_deg: float) -> float:
    """Standard deviation of angles around an already selected center."""

    if len(values) <= 1:
        return 0.0
    diffs = np.asarray([angle_diff_deg(value, center_deg) for value in values], dtype=float)
    return float(np.std(diffs))


def body_to_world(points_body_cm: np.ndarray, pose: Pose2D) -> np.ndarray:
    """Transform body-frame points to the fixed field frame.

    Yaw is clockwise-positive.  With yaw=90 deg, body +X maps to world -Y and
    body +Y maps to world +X.
    """

    points = np.asarray(points_body_cm, dtype=float)
    original_shape = points.shape
    points_2d = points.reshape(-1, 2)
    yaw_rad = math.radians(pose.yaw_deg)
    cos_yaw = math.cos(yaw_rad)
    sin_yaw = math.sin(yaw_rad)
    rotation = np.array([[cos_yaw, sin_yaw], [-sin_yaw, cos_yaw]], dtype=float)
    translated = points_2d @ rotation.T + np.array([pose.x_cm, pose.y_cm], dtype=float)
    return translated.reshape(original_shape)


def world_to_body(points_world_cm: np.ndarray, pose: Pose2D) -> np.ndarray:
    """Transform field-frame points to drone body-center coordinates."""

    points = np.asarray(points_world_cm, dtype=float)
    original_shape = points.shape
    points_2d = points.reshape(-1, 2)
    delta = points_2d - np.array([pose.x_cm, pose.y_cm], dtype=float)
    yaw_rad = math.radians(pose.yaw_deg)
    cos_yaw = math.cos(yaw_rad)
    sin_yaw = math.sin(yaw_rad)
    rotation_inverse = np.array([[cos_yaw, -sin_yaw], [sin_yaw, cos_yaw]], dtype=float)
    body = delta @ rotation_inverse.T
    return body.reshape(original_shape)


def rotate_body_vectors_to_world(vectors_body: np.ndarray, yaw_deg: float) -> np.ndarray:
    """Rotate body-frame vectors into the field frame without translation."""

    vectors = np.asarray(vectors_body, dtype=float)
    original_shape = vectors.shape
    vectors_2d = vectors.reshape(-1, 2)
    yaw_rad = math.radians(yaw_deg)
    cos_yaw = math.cos(yaw_rad)
    sin_yaw = math.sin(yaw_rad)
    rotation = np.array([[cos_yaw, sin_yaw], [-sin_yaw, cos_yaw]], dtype=float)
    return (vectors_2d @ rotation.T).reshape(original_shape)


def point_to_segment_distance(points: np.ndarray, p1: np.ndarray, p2: np.ndarray) -> np.ndarray:
    """Vectorized Euclidean distance from points to a finite line segment."""

    pts = np.asarray(points, dtype=float).reshape(-1, 2)
    start = np.asarray(p1, dtype=float).reshape(2)
    end = np.asarray(p2, dtype=float).reshape(2)
    seg = end - start
    denom = float(np.dot(seg, seg))
    if denom <= 1e-12:
        return np.linalg.norm(pts - start, axis=1)
    t = np.clip(((pts - start) @ seg) / denom, 0.0, 1.0)
    nearest = start + t[:, None] * seg
    return np.linalg.norm(pts - nearest, axis=1)


def build_default_surface_map(config: WarehouseMapConfig) -> dict[SurfaceID, LineSegment2D]:
    """Build finite field/shelf surface segments from the map configuration."""

    return {
        SurfaceID.WEST_NET: LineSegment2D(
            SurfaceID.WEST_NET,
            (config.field_x_min_cm, config.field_y_min_cm),
            (config.field_x_min_cm, config.field_y_max_cm),
        ),
        SurfaceID.EAST_NET: LineSegment2D(
            SurfaceID.EAST_NET,
            (config.field_x_max_cm, config.field_y_min_cm),
            (config.field_x_max_cm, config.field_y_max_cm),
        ),
        SurfaceID.SOUTH_NET: LineSegment2D(
            SurfaceID.SOUTH_NET,
            (config.field_x_min_cm, config.field_y_min_cm),
            (config.field_x_max_cm, config.field_y_min_cm),
        ),
        SurfaceID.NORTH_NET: LineSegment2D(
            SurfaceID.NORTH_NET,
            (config.field_x_min_cm, config.field_y_max_cm),
            (config.field_x_max_cm, config.field_y_max_cm),
        ),
        SurfaceID.SHELF_AB: LineSegment2D(
            SurfaceID.SHELF_AB,
            (config.shelf_ab_x_cm, config.shelf_y_min_cm),
            (config.shelf_ab_x_cm, config.shelf_y_max_cm),
        ),
        SurfaceID.SHELF_CD: LineSegment2D(
            SurfaceID.SHELF_CD,
            (config.shelf_cd_x_cm, config.shelf_y_min_cm),
            (config.shelf_cd_x_cm, config.shelf_y_max_cm),
        ),
    }


class RadarPointSource:
    """Read-only adapter over ``LD_Radar.map`` snapshots.

    ``Map_Circle`` keeps distances, angles and update timestamps but not the
    original per-point confidence.  When no confidence array is exposed by the
    map, this adapter assigns a uniform valid confidence of 255.
    """

    _DEFAULT_CONFIDENCE = 255.0

    def __init__(self, radar: Any):
        self.radar = radar

    def snapshot(self) -> list[RawRadarPoint]:
        """Return a copied, non-blocking snapshot of valid radar-map bins."""

        radar_map = getattr(self.radar, "map", None)
        if radar_map is None:
            return []

        before_count = getattr(radar_map, "update_count", None)
        data = np.array(getattr(radar_map, "data", []), copy=True)
        deg_arr = np.array(getattr(radar_map, "_deg_arr", []), dtype=float, copy=True)
        timestamp_arr = np.array(getattr(radar_map, "time_stamp", []), dtype=float, copy=True)
        confidence_arr = self._copy_confidence_array(radar_map, data.shape)
        after_count = getattr(radar_map, "update_count", None)

        if before_count != after_count:
            data = np.array(getattr(radar_map, "data", []), copy=True)
            deg_arr = np.array(getattr(radar_map, "_deg_arr", []), dtype=float, copy=True)
            timestamp_arr = np.array(getattr(radar_map, "time_stamp", []), dtype=float, copy=True)
            confidence_arr = self._copy_confidence_array(radar_map, data.shape)

        if data.ndim != 1 or data.size == 0:
            return []
        if deg_arr.shape != data.shape:
            deg_arr = np.arange(data.size, dtype=float) * (360.0 / float(data.size))
        if timestamp_arr.shape != data.shape:
            timestamp_arr = np.zeros_like(data, dtype=float)
        if confidence_arr.shape != data.shape:
            confidence_arr = np.full(data.shape, self._DEFAULT_CONFIDENCE, dtype=float)

        valid = np.asarray(data, dtype=float) >= 0.0
        if not np.any(valid):
            return []
        return [
            RawRadarPoint(
                angle_deg=float(angle_deg),
                distance_mm=float(distance_mm),
                confidence=float(confidence),
                timestamp_s=float(timestamp_s),
            )
            for angle_deg, distance_mm, confidence, timestamp_s in zip(
                deg_arr[valid],
                np.asarray(data, dtype=float)[valid],
                confidence_arr[valid],
                timestamp_arr[valid],
            )
        ]

    def _copy_confidence_array(self, radar_map: Any, shape: tuple[int, ...]) -> np.ndarray:
        for attr_name in ("confidence_data", "confidences", "confidence"):
            if not hasattr(radar_map, attr_name):
                continue
            values = getattr(radar_map, attr_name)
            try:
                array = np.array(values, dtype=float, copy=True)
            except (TypeError, ValueError):
                continue
            if array.shape == shape:
                return array
        return np.full(shape, self._DEFAULT_CONFIDENCE, dtype=float)


def fit_line_ransac(
    points_xy: np.ndarray,
    config: RadarAlgorithmConfig,
    rng: np.random.Generator,
) -> LineFitResult:
    """Fit a 2D line using RANSAC followed by TLS/SVD refinement."""

    points = np.asarray(points_xy, dtype=float).reshape(-1, 2)
    point_count = int(points.shape[0])
    if point_count < 2:
        return LineFitResult(False, None, None, None, None, 0, None, None, "not_enough_points")
    if point_count < config.min_inlier_count:
        return LineFitResult(False, None, None, None, None, 0, None, None, "not_enough_candidates")

    best_mask: np.ndarray | None = None
    best_count = 0
    best_rms = math.inf
    iterations = max(1, int(config.ransac_iterations))

    for _ in range(iterations):
        idx_a, idx_b = rng.choice(point_count, size=2, replace=False)
        p1 = points[idx_a]
        p2 = points[idx_b]
        direction = p2 - p1
        norm = float(np.linalg.norm(direction))
        if norm <= 1e-9:
            continue
        direction /= norm
        normal = np.array([-direction[1], direction[0]], dtype=float)
        distances = np.abs((points - p1) @ normal)
        mask = distances <= config.ransac_distance_threshold_cm
        count = int(np.count_nonzero(mask))
        if count == 0:
            continue
        rms = float(np.sqrt(np.mean(distances[mask] ** 2)))
        if count > best_count or (count == best_count and rms < best_rms):
            best_mask = mask
            best_count = count
            best_rms = rms

    if best_mask is None or best_count < config.min_inlier_count:
        return LineFitResult(False, None, None, None, best_mask, best_count, None, None, "ransac_no_model")

    inliers = points[best_mask]
    inlier_ratio = best_count / float(point_count)
    if inlier_ratio < config.min_inlier_ratio:
        return LineFitResult(False, None, None, None, best_mask, best_count, None, None, "inlier_ratio_low")

    centroid = inliers.mean(axis=0)
    _, _, vh = np.linalg.svd(inliers - centroid, full_matrices=False)
    direction = np.asarray(vh[0], dtype=float)
    norm = float(np.linalg.norm(direction))
    if norm <= 1e-9:
        return LineFitResult(False, None, None, None, best_mask, best_count, None, None, "tls_degenerate")
    direction /= norm
    if direction[0] < 0.0 or (abs(direction[0]) <= 1e-12 and direction[1] < 0.0):
        direction = -direction
    normal = np.array([-direction[1], direction[0]], dtype=float)
    residuals = np.abs((inliers - centroid) @ normal)
    residual_rms = float(np.sqrt(np.mean(residuals**2)))
    projections = (inliers - centroid) @ direction
    support_length = float(projections.max() - projections.min()) if projections.size else 0.0

    if support_length < config.min_support_length_cm:
        return LineFitResult(
            False,
            centroid,
            direction,
            normal,
            best_mask,
            best_count,
            residual_rms,
            support_length,
            "support_length_low",
        )
    if residual_rms > config.max_residual_rms_cm:
        return LineFitResult(
            False,
            centroid,
            direction,
            normal,
            best_mask,
            best_count,
            residual_rms,
            support_length,
            "residual_high",
        )

    return LineFitResult(
        True,
        centroid,
        direction,
        normal,
        best_mask,
        best_count,
        residual_rms,
        support_length,
        "ok",
    )


def estimate_yaw_from_surface(
    surface: LineSegment2D,
    match: SurfaceMatch,
    prior_yaw_deg: float,
) -> float:
    """Estimate drone yaw from one known surface and one fitted body line."""

    if match.line_angle_body_deg is None:
        raise ValueError("match has no body line angle")
    p1 = np.asarray(surface.p1, dtype=float)
    p2 = np.asarray(surface.p2, dtype=float)
    world_direction = p2 - p1
    world_angle = normalize_line_angle_deg(vector_angle_clockwise_deg(world_direction))
    body_angle = normalize_line_angle_deg(match.line_angle_body_deg)
    candidate = normalize_angle_deg(world_angle - body_angle)
    options = [normalize_angle_deg(candidate + 180.0 * k) for k in range(-2, 3)]
    return min(options, key=lambda value: abs(angle_diff_deg(value, prior_yaw_deg)))


def _surface_constant(surface: LineSegment2D) -> tuple[str, float]:
    p1 = surface.p1
    p2 = surface.p2
    if abs(p1[0] - p2[0]) <= abs(p1[1] - p2[1]):
        return "x", float(p1[0])
    return "y", float(p1[1])


def _radar_origin_body(config: WarehouseMapConfig) -> np.ndarray:
    return np.array([config.radar_offset_x_cm, config.radar_offset_y_cm], dtype=float)


def _rotate_radar_to_body(points_radar_cm: np.ndarray, config: WarehouseMapConfig) -> np.ndarray:
    yaw_rad = math.radians(config.radar_yaw_offset_deg)
    cos_yaw = math.cos(yaw_rad)
    sin_yaw = math.sin(yaw_rad)
    rotation = np.array([[cos_yaw, sin_yaw], [-sin_yaw, cos_yaw]], dtype=float)
    return points_radar_cm @ rotation.T + _radar_origin_body(config)


def _rotate_body_to_radar(points_body_cm: np.ndarray, config: WarehouseMapConfig) -> np.ndarray:
    shifted = np.asarray(points_body_cm, dtype=float).reshape(-1, 2) - _radar_origin_body(config)
    yaw_rad = math.radians(config.radar_yaw_offset_deg)
    cos_yaw = math.cos(yaw_rad)
    sin_yaw = math.sin(yaw_rad)
    rotation_inverse = np.array([[cos_yaw, -sin_yaw], [sin_yaw, cos_yaw]], dtype=float)
    return shifted @ rotation_inverse.T


def _bearing_from_xy_clockwise(points_xy: np.ndarray) -> np.ndarray:
    points = np.asarray(points_xy, dtype=float).reshape(-1, 2)
    return np.array(
        [normalize_angle_deg(math.degrees(math.atan2(-point[1], point[0]))) for point in points],
        dtype=float,
    )


def _angles_within_minor_arc(
    angles_deg: np.ndarray,
    start_deg: float,
    end_deg: float,
    margin_deg: float,
) -> np.ndarray:
    start = normalize_angle_deg(start_deg)
    end = normalize_angle_deg(end_deg)
    span = (end - start) % 360.0
    angles = np.asarray([normalize_angle_deg(angle) for angle in angles_deg], dtype=float)
    if span <= 180.0:
        forward = (angles - start) % 360.0
        return (forward <= span + margin_deg) | (forward >= 360.0 - margin_deg)
    reverse_span = 360.0 - span
    reverse = (angles - end) % 360.0
    return (reverse <= reverse_span + margin_deg) | (reverse >= 360.0 - margin_deg)


def _weighted_average(values: list[float], weights: list[float]) -> float:
    weights_array = np.asarray(weights, dtype=float)
    if np.all(weights_array <= 0.0):
        weights_array = np.ones_like(weights_array)
    return float(np.average(np.asarray(values, dtype=float), weights=weights_array))


def _score_descending(value: float, good_at_or_below: float) -> float:
    if good_at_or_below <= 0.0:
        return 0.0
    return float(np.clip(1.0 - value / good_at_or_below, 0.0, 1.0))


def _surface_match_invalid(
    surface_id: SurfaceID,
    reason: str,
    timestamp_s: float,
    expected: ExpectedSurfaceObservation | None = None,
    total_candidate_count: int = 0,
    fit: LineFitResult | None = None,
) -> SurfaceMatch:
    return SurfaceMatch(
        surface_id=surface_id,
        valid=False,
        line_point_body_cm=None if fit is None or fit.point is None else (float(fit.point[0]), float(fit.point[1])),
        line_direction_body=None
        if fit is None or fit.direction is None
        else (float(fit.direction[0]), float(fit.direction[1])),
        line_normal_body=None if fit is None or fit.normal is None else (float(fit.normal[0]), float(fit.normal[1])),
        signed_distance_cm=None,
        absolute_distance_cm=None,
        line_angle_body_deg=None,
        inlier_count=0 if fit is None else int(fit.inlier_count),
        total_candidate_count=int(total_candidate_count),
        inlier_ratio=0.0 if total_candidate_count <= 0 else (0 if fit is None else fit.inlier_count / total_candidate_count),
        residual_rms_cm=None if fit is None else fit.residual_rms_cm,
        support_length_cm=None if fit is None else fit.support_length_cm,
        expected_distance_cm=None if expected is None else expected.expected_distance_cm,
        expected_angle_body_deg=None if expected is None else expected.expected_line_angle_body_deg,
        distance_error_cm=None,
        angle_error_deg=None,
        confidence=0.0,
        timestamp_s=timestamp_s,
        reason=reason,
    )


class WarehouseRadarLocalizer:
    """Warehouse radar localization component.

    The class observes radar data and computes pose components; it never starts
    the radar, controls T265, or sends flight-control commands.
    """

    def __init__(
        self,
        radar: Any,
        map_config: WarehouseMapConfig | None = None,
        algorithm_config: RadarAlgorithmConfig | None = None,
        debug_options: DebugOptions | None = None,
    ):
        self.radar = radar
        self.map_config = map_config or WarehouseMapConfig()
        self.algorithm_config = algorithm_config or RadarAlgorithmConfig()
        self.debug_options = debug_options or DebugOptions()
        self.surface_map = build_default_surface_map(self.map_config)
        self.point_source = RadarPointSource(radar)
        self._rng = np.random.default_rng(20240713)
        self._lock = RLock()
        self._match_history: dict[SurfaceID, Deque[SurfaceMatch]] = {
            surface_id: deque(maxlen=self.algorithm_config.temporal_history_size) for surface_id in SurfaceID
        }
        self._endpoint_history: dict[tuple[SurfaceID, str], Deque[bool]] = defaultdict(
            lambda: deque(maxlen=self.algorithm_config.temporal_history_size)
        )
        self._debug_frame_index = 0
        self._last_debug: dict[SurfaceID, _SurfaceDiagnostics] = {}
        logger.info("[WAREHOUSE_RADAR] localizer initialized")

    def localize(self, request: LocalizationRequest) -> RadarLocalizationResult:
        """Localize according to a trusted-surface request."""

        now = time.perf_counter()
        if not request.trusted_surfaces:
            return self._invalid_result("trusted_surfaces_empty", now)
        if request.prior_pose is None:
            return self._invalid_result("invalid_prior", now)

        with self._lock:
            points = self._collect_points(now)
            if points.count == 0:
                return self._invalid_result("point_cloud_empty_or_expired", now)
            matches = self._match_surfaces_with_points(request, points, now)

            if request.mode == LocalizationMode.DETECTION_ONLY:
                valid_matches = tuple(surface_id for surface_id, match in matches.items() if match.valid)
                return RadarLocalizationResult(
                    x_cm=None,
                    y_cm=None,
                    yaw_deg=None,
                    valid_x=False,
                    valid_y=False,
                    valid_yaw=False,
                    matched_surfaces=valid_matches,
                    surface_matches=matches,
                    position_std_cm=None,
                    yaw_std_deg=None,
                    residual_cm=self._match_residual(matches),
                    confidence=self._mean_confidence(matches),
                    absolute_fix=False,
                    partial_fix=bool(valid_matches),
                    reason="detection_only",
                    timestamp_s=now,
                )

            result = self._solve_pose_from_matches(request, matches, now)
            self._render_debug_if_needed(points, request, result)
            return result

    def match_surfaces(self, request: LocalizationRequest) -> dict[SurfaceID, SurfaceMatch]:
        """Match only the trusted surfaces in the request."""

        now = time.perf_counter()
        with self._lock:
            points = self._collect_points(now)
            if points.count == 0:
                return {
                    surface_id: _surface_match_invalid(surface_id, "point_cloud_empty_or_expired", now)
                    for surface_id in request.trusted_surfaces
                }
            return self._match_surfaces_with_points(request, points, now)

    def get_latest_raw_points(self) -> np.ndarray:
        """Return N x 4 ``[x_body_cm, y_body_cm, confidence, angle_deg]``."""

        points = self._collect_points(time.perf_counter())
        if points.count == 0:
            return np.empty((0, 4), dtype=float)
        return np.column_stack((points.body_xy_cm, points.confidence, points.raw_angle_deg))

    def detect_surface_presence(
        self,
        surface_id: SurfaceID,
        prior_pose: Pose2D,
    ) -> SurfaceMatch:
        """Detect whether one trusted surface is present near its prediction."""

        request = LocalizationRequest(
            mode=LocalizationMode.DETECTION_ONLY,
            trusted_surfaces=(surface_id,),
            prior_pose=prior_pose,
        )
        return self.match_surfaces(request).get(
            surface_id,
            _surface_match_invalid(surface_id, "surface_not_requested", time.perf_counter()),
        )

    def detect_shelf_endpoint(
        self,
        shelf_id: SurfaceID,
        prior_pose: Pose2D,
        endpoint: str,
    ) -> EndpointDetection:
        """Detect a shelf north/south endpoint near its predicted body position."""

        if shelf_id not in (SurfaceID.SHELF_AB, SurfaceID.SHELF_CD):
            raise ValueError("shelf_id must be SHELF_AB or SHELF_CD")
        endpoint_key = endpoint.lower()
        if endpoint_key not in ("north", "south"):
            raise ValueError("endpoint must be 'north' or 'south'")

        now = time.perf_counter()
        points = self._collect_points(now)
        if points.count == 0:
            return EndpointDetection(False, shelf_id, endpoint_key, None, 0.0, "point_cloud_empty_or_expired")

        shelf_match = self.detect_surface_presence(shelf_id, prior_pose)
        if not shelf_match.valid:
            self._endpoint_history[(shelf_id, endpoint_key)].append(False)
            return EndpointDetection(False, shelf_id, endpoint_key, None, 0.0, "shelf_not_matched")

        segment = self.surface_map[shelf_id]
        endpoint_world = np.asarray(
            [
                segment.p1[0],
                self.map_config.shelf_y_max_cm if endpoint_key == "north" else self.map_config.shelf_y_min_cm,
            ],
            dtype=float,
        )
        endpoint_body = world_to_body(endpoint_world, prior_pose).reshape(2)
        distances = np.linalg.norm(points.body_xy_cm - endpoint_body, axis=1)
        roi_mask = distances <= self.algorithm_config.endpoint_roi_radius_cm
        roi_count = int(np.count_nonzero(roi_mask))
        detected_current = roi_count >= self.algorithm_config.endpoint_min_points
        history = self._endpoint_history[(shelf_id, endpoint_key)]
        history.append(detected_current)
        required = min(self.algorithm_config.temporal_confirm_frames, len(history))
        detected = sum(1 for item in history if item) >= required and detected_current

        if roi_count > 0:
            estimated = points.body_xy_cm[roi_mask].mean(axis=0)
            estimated_tuple = (float(estimated[0]), float(estimated[1]))
        else:
            estimated_tuple = None
        count_score = min(1.0, roi_count / max(1.0, float(self.algorithm_config.endpoint_min_points)))
        confidence = float(np.clip(0.5 * shelf_match.confidence + 0.5 * count_score, 0.0, 1.0))
        reason = "ok" if detected else "endpoint_unconfirmed"
        logger.debug("[WAREHOUSE_RADAR] endpoint {} {} count={} reason={}", shelf_id.name, endpoint_key, roi_count, reason)
        return EndpointDetection(detected, shelf_id, endpoint_key, estimated_tuple, confidence if detected else 0.0, reason)

    def get_sector_distance_stats(
        self,
        center_angle_deg: float,
        half_width_deg: float,
    ) -> SectorDistanceStats:
        """Return robust raw-distance statistics for a radar angle sector."""

        now = time.perf_counter()
        raw_points = self.point_source.snapshot()
        values_cm: list[float] = []
        confidences: list[float] = []
        for point in raw_points:
            if now - point.timestamp_s > self.algorithm_config.max_point_age_s:
                continue
            distance_cm = point.distance_mm / 10.0
            if distance_cm < self.algorithm_config.min_distance_cm or distance_cm > self.algorithm_config.max_distance_cm:
                continue
            if point.confidence < self.algorithm_config.min_confidence:
                continue
            if abs(angle_diff_deg(point.angle_deg, center_angle_deg)) <= half_width_deg:
                values_cm.append(distance_cm)
                confidences.append(point.confidence)
        if not values_cm:
            return SectorDistanceStats(0, None, None, None, None)
        array = np.asarray(values_cm, dtype=float)
        return SectorDistanceStats(
            valid_count=int(array.size),
            min_cm=float(np.min(array)),
            median_cm=float(np.median(array)),
            percentile_20_cm=float(np.percentile(array, 20)),
            confidence_mean=float(np.mean(confidences)) if confidences else None,
        )

    def _collect_points(self, now_s: float) -> _PreparedPoints:
        raw_points = self.point_source.snapshot()
        if not raw_points:
            return _PreparedPoints(
                np.empty((0, 2), dtype=float),
                np.empty(0, dtype=float),
                np.empty(0, dtype=float),
                np.empty(0, dtype=float),
                np.empty(0, dtype=float),
            )

        raw_angle = np.asarray([point.angle_deg for point in raw_points], dtype=float)
        distance_cm = np.asarray([point.distance_mm / 10.0 for point in raw_points], dtype=float)
        confidence = np.asarray([point.confidence for point in raw_points], dtype=float)
        timestamp = np.asarray([point.timestamp_s for point in raw_points], dtype=float)
        age = now_s - timestamp
        valid = (
            (age >= 0.0)
            & (age <= self.algorithm_config.max_point_age_s)
            & (distance_cm >= self.algorithm_config.min_distance_cm)
            & (distance_cm <= self.algorithm_config.max_distance_cm)
            & (confidence >= self.algorithm_config.min_confidence)
        )
        if not np.any(valid):
            return _PreparedPoints(
                np.empty((0, 2), dtype=float),
                np.empty(0, dtype=float),
                np.empty(0, dtype=float),
                np.empty(0, dtype=float),
                np.empty(0, dtype=float),
            )

        raw_angle = raw_angle[valid]
        distance_cm = distance_cm[valid]
        confidence = confidence[valid]
        timestamp = timestamp[valid]
        angle_rad = np.deg2rad(raw_angle)
        points_radar = np.column_stack((distance_cm * np.cos(angle_rad), -distance_cm * np.sin(angle_rad)))
        points_body = _rotate_radar_to_body(points_radar, self.map_config)
        logger.debug("[WAREHOUSE_RADAR] raw_points={} filtered_points={}", len(raw_points), points_body.shape[0])
        return _PreparedPoints(points_body, confidence, raw_angle, distance_cm, timestamp)

    def _match_surfaces_with_points(
        self,
        request: LocalizationRequest,
        points: _PreparedPoints,
        now_s: float,
    ) -> dict[SurfaceID, SurfaceMatch]:
        matches: dict[SurfaceID, SurfaceMatch] = {}
        self._last_debug = {}
        for surface_id in request.trusted_surfaces:
            surface = self.surface_map.get(surface_id)
            if surface is None:
                matches[surface_id] = _surface_match_invalid(surface_id, "unknown_surface", now_s)
                continue
            match, diagnostics = self._match_single_surface(surface, request.prior_pose, points, now_s)
            matches[surface_id] = match
            self._last_debug[surface_id] = diagnostics
            logger.debug(
                "[WAREHOUSE_RADAR] surface={} valid={} candidates={} inliers={} reason={}",
                surface_id.name,
                match.valid,
                match.total_candidate_count,
                match.inlier_count,
                match.reason,
            )
        return matches

    def _match_single_surface(
        self,
        surface: LineSegment2D,
        prior_pose: Pose2D,
        points: _PreparedPoints,
        now_s: float,
    ) -> tuple[SurfaceMatch, _SurfaceDiagnostics]:
        expected = self._predict_surface_observation(surface, prior_pose)
        if expected is None:
            match = _surface_match_invalid(surface.surface_id, "surface_not_observable", now_s)
            return match, _SurfaceDiagnostics(None, np.empty((0, 2), dtype=float), np.empty((0, 2), dtype=float))

        candidate_mask = self._candidate_mask(points, expected, surface, prior_pose)
        candidate_points = points.body_xy_cm[candidate_mask]
        total_candidate_count = int(candidate_points.shape[0])
        if total_candidate_count < self.algorithm_config.min_inlier_count:
            match = _surface_match_invalid(
                surface.surface_id,
                "not_enough_candidates",
                now_s,
                expected=expected,
                total_candidate_count=total_candidate_count,
            )
            return match, _SurfaceDiagnostics(expected, candidate_points, np.empty((0, 2), dtype=float))

        fit = fit_line_ransac(candidate_points, self.algorithm_config, self._rng)
        if fit.inlier_mask is not None and fit.inlier_mask.shape[0] == candidate_points.shape[0]:
            inlier_points = candidate_points[fit.inlier_mask]
        else:
            inlier_points = np.empty((0, 2), dtype=float)
        if not fit.valid or fit.point is None or fit.direction is None or fit.normal is None:
            match = _surface_match_invalid(
                surface.surface_id,
                fit.reason,
                now_s,
                expected=expected,
                total_candidate_count=total_candidate_count,
                fit=fit,
            )
            return match, _SurfaceDiagnostics(expected, candidate_points, inlier_points)

        expected_dir, expected_normal, expected_signed = self._expected_line_vectors(expected)
        direction = np.array(fit.direction, dtype=float)
        if float(np.dot(direction, expected_dir)) < 0.0:
            direction = -direction
        normal = np.array([-direction[1], direction[0]], dtype=float)
        if float(np.dot(normal, expected_normal)) < 0.0:
            normal = -normal

        line_point = np.array(fit.point, dtype=float)
        radar_origin = _radar_origin_body(self.map_config)
        signed_distance = float((radar_origin - line_point) @ normal)
        absolute_distance = abs(signed_distance)
        line_angle = normalize_line_angle_deg(vector_angle_clockwise_deg(direction))
        angle_error = undirected_angle_diff_deg(line_angle, expected.expected_line_angle_body_deg)
        distance_error = abs(signed_distance - expected_signed)

        valid = True
        reason = "ok"
        if angle_error > self.algorithm_config.max_surface_angle_error_deg:
            valid = False
            reason = "surface_angle_error"
        elif distance_error > self.algorithm_config.max_surface_distance_error_cm:
            valid = False
            reason = "surface_distance_error"

        inlier_ratio = fit.inlier_count / float(total_candidate_count) if total_candidate_count else 0.0
        confidence = self._surface_confidence(
            inlier_count=fit.inlier_count,
            total_candidate_count=total_candidate_count,
            residual_rms_cm=fit.residual_rms_cm,
            angle_error_deg=angle_error,
            distance_error_cm=distance_error,
        )

        match = SurfaceMatch(
            surface_id=surface.surface_id,
            valid=valid,
            line_point_body_cm=(float(line_point[0]), float(line_point[1])),
            line_direction_body=(float(direction[0]), float(direction[1])),
            line_normal_body=(float(normal[0]), float(normal[1])),
            signed_distance_cm=signed_distance,
            absolute_distance_cm=absolute_distance,
            line_angle_body_deg=line_angle,
            inlier_count=int(fit.inlier_count),
            total_candidate_count=total_candidate_count,
            inlier_ratio=float(inlier_ratio),
            residual_rms_cm=fit.residual_rms_cm,
            support_length_cm=fit.support_length_cm,
            expected_distance_cm=expected.expected_distance_cm,
            expected_angle_body_deg=expected.expected_line_angle_body_deg,
            distance_error_cm=distance_error,
            angle_error_deg=angle_error,
            confidence=confidence if valid else 0.0,
            timestamp_s=now_s,
            reason=reason,
        )

        if match.valid:
            self._match_history[surface.surface_id].append(match)
            if not self._temporal_consistent(surface.surface_id):
                match.valid = False
                match.confidence = 0.0
                match.reason = "temporal_unstable"
        return match, _SurfaceDiagnostics(expected, candidate_points, inlier_points)

    def _predict_surface_observation(
        self,
        surface: LineSegment2D,
        prior_pose: Pose2D,
    ) -> ExpectedSurfaceObservation | None:
        segment_world = np.asarray([surface.p1, surface.p2], dtype=float)
        segment_body = world_to_body(segment_world, prior_pose).reshape(2, 2)
        segment_vector = segment_body[1] - segment_body[0]
        length = float(np.linalg.norm(segment_vector))
        if length <= 1e-9:
            return None
        direction = segment_vector / length
        normal = np.array([-direction[1], direction[0]], dtype=float)
        radar_origin = _radar_origin_body(self.map_config)
        signed = float((radar_origin - segment_body[0]) @ normal)
        if signed < 0.0:
            normal = -normal
            signed = -signed
        projection = float((radar_origin - segment_body[0]) @ direction)
        projection_inside = 0.0 <= projection <= length
        radar_segment = _rotate_body_to_radar(segment_body, self.map_config)
        endpoint_distances = np.linalg.norm(radar_segment, axis=1)
        if min(endpoint_distances.min(), signed) > self.algorithm_config.max_distance_cm + length:
            return None
        return ExpectedSurfaceObservation(
            surface_id=surface.surface_id,
            expected_distance_cm=signed,
            expected_line_angle_body_deg=normalize_line_angle_deg(vector_angle_clockwise_deg(direction)),
            expected_normal_body_deg=vector_angle_clockwise_deg(normal),
            segment_body_cm=segment_body,
            projection_inside_segment=projection_inside,
        )

    def _expected_line_vectors(
        self,
        expected: ExpectedSurfaceObservation,
    ) -> tuple[np.ndarray, np.ndarray, float]:
        segment = expected.segment_body_cm
        direction = segment[1] - segment[0]
        direction /= max(float(np.linalg.norm(direction)), 1e-9)
        normal = np.array([-direction[1], direction[0]], dtype=float)
        radar_origin = _radar_origin_body(self.map_config)
        signed = float((radar_origin - segment[0]) @ normal)
        if signed < 0.0:
            normal = -normal
            signed = -signed
        return direction, normal, signed

    def _candidate_mask(
        self,
        points: _PreparedPoints,
        expected: ExpectedSurfaceObservation,
        surface: LineSegment2D,
        prior_pose: Pose2D,
    ) -> np.ndarray:
        segment_world = np.asarray([surface.p1, surface.p2], dtype=float)
        segment_vector = segment_world[1] - segment_world[0]
        length = float(np.linalg.norm(segment_vector))
        if length <= 1e-9:
            return np.zeros(points.count, dtype=bool)
        unit = segment_vector / length
        p1_ext = segment_world[0] - unit * self.algorithm_config.segment_extension_cm
        p2_ext = segment_world[1] + unit * self.algorithm_config.segment_extension_cm
        points_world = body_to_world(points.body_xy_cm, prior_pose).reshape(-1, 2)
        spatial_distance = point_to_segment_distance(points_world, p1_ext, p2_ext)
        spatial_mask = spatial_distance <= self.algorithm_config.expected_distance_gate_cm

        segment_radar = _rotate_body_to_radar(expected.segment_body_cm, self.map_config)
        bearings = _bearing_from_xy_clockwise(segment_radar)
        angle_mask = _angles_within_minor_arc(
            points.raw_angle_deg,
            bearings[0],
            bearings[1],
            self.algorithm_config.expected_angle_gate_deg,
        )
        return spatial_mask & angle_mask

    def _surface_confidence(
        self,
        inlier_count: int,
        total_candidate_count: int,
        residual_rms_cm: float | None,
        angle_error_deg: float,
        distance_error_cm: float,
    ) -> float:
        inlier_count_score = min(1.0, inlier_count / max(1.0, float(self.algorithm_config.min_inlier_count * 2)))
        inlier_ratio_score = min(1.0, inlier_count / max(1.0, float(total_candidate_count)))
        residual_score = _score_descending(residual_rms_cm or 0.0, self.algorithm_config.max_residual_rms_cm)
        angle_score = _score_descending(angle_error_deg, self.algorithm_config.max_surface_angle_error_deg)
        distance_score = _score_descending(distance_error_cm, self.algorithm_config.max_surface_distance_error_cm)
        confidence = (
            0.30 * inlier_count_score
            + 0.20 * inlier_ratio_score
            + 0.20 * residual_score
            + 0.15 * angle_score
            + 0.15 * distance_score
        )
        return float(np.clip(confidence, 0.0, 1.0))

    def _temporal_consistent(self, surface_id: SurfaceID) -> bool:
        history = [match for match in self._match_history[surface_id] if match.valid]
        if len(history) < self.algorithm_config.temporal_confirm_frames:
            return True
        recent = history[-self.algorithm_config.temporal_confirm_frames :]
        distances = [match.absolute_distance_cm for match in recent if match.absolute_distance_cm is not None]
        angles = [match.line_angle_body_deg for match in recent if match.line_angle_body_deg is not None]
        if len(distances) >= 2 and float(np.std(distances)) > self.algorithm_config.temporal_distance_std_cm:
            return False
        if len(angles) >= 2:
            center = circular_mean_deg(angles, [1.0] * len(angles))
            if circular_std_deg(angles, center) > self.algorithm_config.temporal_angle_std_deg:
                return False
        return True

    def _solve_pose_from_matches(
        self,
        request: LocalizationRequest,
        matches: dict[SurfaceID, SurfaceMatch],
        now_s: float,
    ) -> RadarLocalizationResult:
        valid_matches = {surface_id: match for surface_id, match in matches.items() if match.valid}
        if not valid_matches:
            return self._invalid_result("no_valid_surface", now_s, matches)

        yaw_values: list[float] = []
        yaw_weights: list[float] = []
        for surface_id, match in valid_matches.items():
            surface = self.surface_map[surface_id]
            try:
                yaw_values.append(estimate_yaw_from_surface(surface, match, request.prior_pose.yaw_deg))
                yaw_weights.append(max(match.confidence, 0.05))
            except ValueError:
                continue

        yaw_deg: float | None = None
        valid_yaw = False
        yaw_std: float | None = None
        if yaw_values:
            yaw_deg = circular_mean_deg(yaw_values, yaw_weights)
            yaw_std = circular_std_deg(yaw_values, yaw_deg)
            valid_yaw = abs(angle_diff_deg(yaw_deg, request.prior_pose.yaw_deg)) <= request.max_yaw_deviation_deg
            if not valid_yaw:
                return self._invalid_result("pose_jump_rejected", now_s, matches)

        if request.mode == LocalizationMode.YAW_ONLY:
            if not valid_yaw or yaw_deg is None:
                return self._invalid_result("yaw_unobservable", now_s, matches)
            return self._result_from_components(
                None,
                None,
                yaw_deg,
                False,
                False,
                True,
                tuple(valid_matches.keys()),
                matches,
                None,
                yaw_std,
                "yaw_only",
                now_s,
                absolute_fix=False,
                partial_fix=True,
            )

        if not valid_yaw or yaw_deg is None:
            return self._invalid_result("yaw_unobservable", now_s, matches)

        if request.mode == LocalizationMode.CORRIDOR_TRACK:
            result = self._solve_corridor(request, valid_matches, matches, yaw_deg, yaw_std, now_s)
        else:
            result = self._solve_absolute(request, valid_matches, matches, yaw_deg, yaw_std, now_s)
        return result

    def _solve_absolute(
        self,
        request: LocalizationRequest,
        valid_matches: dict[SurfaceID, SurfaceMatch],
        all_matches: dict[SurfaceID, SurfaceMatch],
        yaw_deg: float,
        yaw_std_deg: float | None,
        now_s: float,
    ) -> RadarLocalizationResult:
        allowed_x = (SurfaceID.WEST_NET, SurfaceID.EAST_NET)
        allowed_y = (SurfaceID.SOUTH_NET, SurfaceID.NORTH_NET)
        x_estimates = self._axis_estimates(valid_matches, yaw_deg, allowed_x, "x")
        y_estimates = self._axis_estimates(valid_matches, yaw_deg, allowed_y, "y")

        x_cm, valid_x, x_std = self._merge_axis_estimates(x_estimates)
        y_cm, valid_y, y_std = self._merge_axis_estimates(y_estimates)
        if (valid_x and abs(x_cm - request.prior_pose.x_cm) > request.max_position_deviation_cm) or (
            valid_y and abs(y_cm - request.prior_pose.y_cm) > request.max_position_deviation_cm
        ):
            return self._invalid_result("pose_jump_rejected", now_s, all_matches)

        absolute_fix = bool(valid_x and valid_y)
        partial_fix = bool((valid_x or valid_y) and request.allow_partial_result)
        if not absolute_fix and not partial_fix:
            return self._invalid_result("not_enough_non_parallel_surfaces", now_s, all_matches)

        position_std = self._combine_position_std(x_std if valid_x else None, y_std if valid_y else None)
        reason = "absolute_anchor" if absolute_fix else "partial_anchor"
        return self._result_from_components(
            x_cm if valid_x else None,
            y_cm if valid_y else None,
            yaw_deg,
            valid_x,
            valid_y,
            True,
            tuple(valid_matches.keys()),
            all_matches,
            position_std,
            yaw_std_deg,
            reason,
            now_s,
            absolute_fix=absolute_fix,
            partial_fix=partial_fix and not absolute_fix,
        )

    def _solve_corridor(
        self,
        request: LocalizationRequest,
        valid_matches: dict[SurfaceID, SurfaceMatch],
        all_matches: dict[SurfaceID, SurfaceMatch],
        yaw_deg: float,
        yaw_std_deg: float | None,
        now_s: float,
    ) -> RadarLocalizationResult:
        shelf_ids = tuple(surface_id for surface_id in (SurfaceID.SHELF_AB, SurfaceID.SHELF_CD) if surface_id in valid_matches)
        if not shelf_ids:
            return self._invalid_result("no_shelf_surface", now_s, all_matches)
        if len(shelf_ids) == 2:
            ab = valid_matches[SurfaceID.SHELF_AB]
            cd = valid_matches[SurfaceID.SHELF_CD]
            if ab.absolute_distance_cm is None or cd.absolute_distance_cm is None:
                return self._invalid_result("corridor_distance_missing", now_s, all_matches)
            width = ab.absolute_distance_cm + cd.absolute_distance_cm
            if abs(width - self.algorithm_config.corridor_width_cm) > self.algorithm_config.corridor_width_tolerance_cm:
                return self._invalid_result("corridor_width_inconsistent", now_s, all_matches)

        x_estimates = self._axis_estimates(valid_matches, yaw_deg, shelf_ids, "x")
        x_cm, valid_x, x_std = self._merge_axis_estimates(x_estimates)
        if not valid_x:
            return self._invalid_result("corridor_x_unobservable", now_s, all_matches)
        if abs(x_cm - request.prior_pose.x_cm) > request.max_position_deviation_cm:
            return self._invalid_result("pose_jump_rejected", now_s, all_matches)

        return self._result_from_components(
            x_cm,
            None,
            yaw_deg,
            True,
            False,
            True,
            tuple(valid_matches.keys()),
            all_matches,
            x_std,
            yaw_std_deg,
            "corridor_track",
            now_s,
            absolute_fix=False,
            partial_fix=True,
        )

    def _axis_estimates(
        self,
        valid_matches: dict[SurfaceID, SurfaceMatch],
        yaw_deg: float,
        surface_ids: tuple[SurfaceID, ...],
        axis: str,
    ) -> list[tuple[float, float]]:
        estimates: list[tuple[float, float]] = []
        for surface_id in surface_ids:
            match = valid_matches.get(surface_id)
            if match is None or match.line_point_body_cm is None:
                continue
            surface = self.surface_map[surface_id]
            surface_axis, constant = _surface_constant(surface)
            if surface_axis != axis:
                continue
            point_body = np.asarray(match.line_point_body_cm, dtype=float)
            point_world_without_translation = rotate_body_vectors_to_world(point_body, yaw_deg).reshape(2)
            axis_index = 0 if axis == "x" else 1
            estimate = constant - float(point_world_without_translation[axis_index])
            estimates.append((estimate, max(match.confidence, 0.05)))
        return estimates

    def _merge_axis_estimates(self, estimates: list[tuple[float, float]]) -> tuple[float | None, bool, float | None]:
        if not estimates:
            return None, False, None
        values = [value for value, _weight in estimates]
        weights = [weight for _value, weight in estimates]
        if len(values) > 1 and max(values) - min(values) > self.algorithm_config.parallel_surface_consistency_cm:
            return None, False, None
        merged = _weighted_average(values, weights)
        std = float(np.std(values)) if len(values) > 1 else 0.0
        return merged, True, std

    def _combine_position_std(self, x_std: float | None, y_std: float | None) -> float | None:
        values = [value for value in (x_std, y_std) if value is not None]
        if not values:
            return None
        return float(np.linalg.norm(values))

    def _result_from_components(
        self,
        x_cm: float | None,
        y_cm: float | None,
        yaw_deg: float | None,
        valid_x: bool,
        valid_y: bool,
        valid_yaw: bool,
        matched_surfaces: tuple[SurfaceID, ...],
        matches: dict[SurfaceID, SurfaceMatch],
        position_std_cm: float | None,
        yaw_std_deg: float | None,
        reason: str,
        timestamp_s: float,
        absolute_fix: bool,
        partial_fix: bool,
    ) -> RadarLocalizationResult:
        return RadarLocalizationResult(
            x_cm=x_cm,
            y_cm=y_cm,
            yaw_deg=None if yaw_deg is None else normalize_angle_deg(yaw_deg),
            valid_x=valid_x,
            valid_y=valid_y,
            valid_yaw=valid_yaw,
            matched_surfaces=matched_surfaces,
            surface_matches=matches,
            position_std_cm=position_std_cm,
            yaw_std_deg=yaw_std_deg,
            residual_cm=self._match_residual(matches),
            confidence=self._mean_confidence(matches),
            absolute_fix=absolute_fix,
            partial_fix=partial_fix,
            reason=reason,
            timestamp_s=timestamp_s,
        )

    def _invalid_result(
        self,
        reason: str,
        timestamp_s: float,
        matches: dict[SurfaceID, SurfaceMatch] | None = None,
    ) -> RadarLocalizationResult:
        logger.debug("[WAREHOUSE_RADAR] invalid_result reason={}", reason)
        return RadarLocalizationResult(
            x_cm=None,
            y_cm=None,
            yaw_deg=None,
            valid_x=False,
            valid_y=False,
            valid_yaw=False,
            matched_surfaces=(),
            surface_matches={} if matches is None else matches,
            position_std_cm=None,
            yaw_std_deg=None,
            residual_cm=None if matches is None else self._match_residual(matches),
            confidence=0.0,
            absolute_fix=False,
            partial_fix=False,
            reason=reason,
            timestamp_s=timestamp_s,
        )

    def _match_residual(self, matches: dict[SurfaceID, SurfaceMatch]) -> float | None:
        residuals = [match.residual_rms_cm for match in matches.values() if match.valid and match.residual_rms_cm is not None]
        if not residuals:
            return None
        return float(np.mean(residuals))

    def _mean_confidence(self, matches: dict[SurfaceID, SurfaceMatch]) -> float:
        values = [match.confidence for match in matches.values() if match.valid]
        if not values:
            return 0.0
        return float(np.clip(np.mean(values), 0.0, 1.0))

    def _render_debug_if_needed(
        self,
        points: _PreparedPoints,
        request: LocalizationRequest,
        result: RadarLocalizationResult,
    ) -> None:
        options = self.debug_options
        if not options.enabled:
            return
        self._debug_frame_index += 1
        save_due = bool(options.save_directory) and (
            self._debug_frame_index % max(1, options.save_every_n_frames) == 0
        )
        if not options.show_window and not save_due:
            return
        try:
            import matplotlib.pyplot as plt
        except ImportError:
            logger.warning("[WAREHOUSE_RADAR] matplotlib unavailable; debug visualization skipped")
            return

        fig, ax = plt.subplots(figsize=(7, 7))
        if points.count:
            ax.scatter(points.body_xy_cm[:, 0], points.body_xy_cm[:, 1], s=6, c="#999999", label="filtered")
        for surface_id, diagnostics in self._last_debug.items():
            if diagnostics.expected is not None:
                segment = diagnostics.expected.segment_body_cm
                ax.plot(segment[:, 0], segment[:, 1], "--", linewidth=1.0, label=f"{surface_id.name} expected")
            if diagnostics.candidate_points.size:
                ax.scatter(
                    diagnostics.candidate_points[:, 0],
                    diagnostics.candidate_points[:, 1],
                    s=8,
                    label=f"{surface_id.name} candidate",
                )
            if diagnostics.inlier_points.size:
                ax.scatter(
                    diagnostics.inlier_points[:, 0],
                    diagnostics.inlier_points[:, 1],
                    s=14,
                    label=f"{surface_id.name} inlier",
                )
            match = result.surface_matches.get(surface_id)
            if match and match.valid and match.line_point_body_cm and match.line_direction_body:
                p = np.asarray(match.line_point_body_cm)
                d = np.asarray(match.line_direction_body)
                line = np.vstack((p - d * 80.0, p + d * 80.0))
                ax.plot(line[:, 0], line[:, 1], linewidth=2.0, label=f"{surface_id.name} fit")
        ax.scatter([0.0], [0.0], marker="x", c="black", label="prior body")
        title_pose = f"x={result.x_cm} y={result.y_cm} yaw={result.yaw_deg}"
        ax.set_title(f"{request.mode.name} {result.reason} {title_pose}")
        ax.set_xlabel("body x cm")
        ax.set_ylabel("body y cm")
        ax.axis("equal")
        ax.grid(True)
        ax.legend(loc="best", fontsize=7)
        if save_due and options.save_directory:
            os.makedirs(options.save_directory, exist_ok=True)
            filename = os.path.join(options.save_directory, f"warehouse_radar_{self._debug_frame_index:05d}.png")
            fig.savefig(filename, dpi=130)
        if options.show_window:
            plt.show(block=False)
            plt.pause(0.001)
        plt.close(fig)


__all__ = [
    "DebugOptions",
    "EndpointDetection",
    "ExpectedSurfaceObservation",
    "LineFitResult",
    "LineSegment2D",
    "LocalizationMode",
    "LocalizationRequest",
    "Pose2D",
    "RadarAlgorithmConfig",
    "RadarLocalizationResult",
    "RadarPointSource",
    "RawRadarPoint",
    "SectorDistanceStats",
    "SurfaceID",
    "SurfaceMatch",
    "WarehouseMapConfig",
    "WarehouseRadarLocalizer",
    "angle_diff_deg",
    "body_to_world",
    "build_default_surface_map",
    "circular_mean_deg",
    "estimate_yaw_from_surface",
    "fit_line_ransac",
    "normalize_angle_deg",
    "point_to_segment_distance",
    "undirected_angle_diff_deg",
    "world_to_body",
]
