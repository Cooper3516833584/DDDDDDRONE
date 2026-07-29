"""Downward-camera landing-marker detection and pixel-offset tracking."""

from __future__ import annotations

import collections as _collections
import collections.abc as _abc
import math as _math
import sys as _sys
import threading as _threading
import time as _time
from dataclasses import dataclass as _dataclass
from typing import Tuple as _Tuple

import cv2 as _cv2
import numpy as _np

globals().pop("annotations", None)

__all__ = ["track_landing_marker"]

_FRAME_WIDTH = 256
_FRAME_HEIGHT = 256
_FRAME_CENTER_U = (_FRAME_WIDTH - 1) / 2.0
_FRAME_CENTER_V = (_FRAME_HEIGHT - 1) / 2.0
_MAX_READ_FAILURES = 10
_GEOMETRY_MAX_AGE_NS = 250_000_000
_GEOMETRY_EARLY_REFRESH_NS = 180_000_000
_SEARCH_SCORE_THRESHOLD = 0.68
_TRACK_SCORE_THRESHOLD = 0.58
_DEBUG = False
_DEBUG_WINDOW = "Landing marker offset"
_DEBUG_FPS: float | None = None

try:
    _dataclass_slots = _dataclass(slots=True)
except TypeError:
    # Python 3.8 on the flight computer has dataclasses but not slots=True.
    _dataclass_slots = _dataclass

_Ellipse = _Tuple[_Tuple[float, float], _Tuple[float, float], float]
_Roi = _Tuple[int, int, int, int]


@_dataclass_slots
class _Detection:
    center_u: float
    center_v: float
    diameter_px: float
    score: float
    ellipse: _Ellipse
    mask_index: int
    near_cross: bool = False


@_dataclass_slots
class _HoleAnalysis:
    score: float
    centers: _np.ndarray
    areas: _np.ndarray
    weighted_center: tuple[float, float]


@_dataclass_slots
class _Candidate:
    ellipse: _Ellipse
    diameter_px: float
    mask_index: int
    ellipse_score: float
    hole_score: float
    occupancy_score: float
    temporal_score: float
    hole_center: tuple[float, float]
    clipped: bool
    pre_score: float


@_dataclass_slots
class _LineSegment:
    x1: float
    y1: float
    x2: float
    y2: float
    length: float
    angle: float


@_dataclass_slots
class _CrossLine:
    normal_x: float
    normal_y: float
    offset: float
    width: float
    support_score: float


@_dataclass_slots
class _OpticalResult:
    detection: _Detection
    points: _np.ndarray


class _VisionResources:
    """OpenCV objects reused for the lifetime of one tracker."""

    def __init__(self) -> None:
        self.clahe = _cv2.createCLAHE(
            clipLimit=2.0,
            tileGridSize=(8, 8),
        )
        self.blackhat_kernel = _cv2.getStructuringElement(
            _cv2.MORPH_ELLIPSE,
            (31, 31),
        )
        self.small_kernel = _np.ones((3, 3), dtype=_np.uint8)
        self.lsd = None
        if hasattr(_cv2, "createLineSegmentDetector"):
            try:
                self.lsd = _cv2.createLineSegmentDetector(
                    _cv2.LSD_REFINE_STD,
                )
            except _cv2.error:
                self.lsd = None


class _LatestFrameCapture:
    """Read continuously and retain only the newest camera frame."""

    def __init__(self, cap: _cv2.VideoCapture) -> None:
        self._cap = cap
        self._condition = _threading.Condition()
        self._stop_event = _threading.Event()
        self._thread = _threading.Thread(
            target=self._read_loop,
            name="landing-marker-camera",
            daemon=True,
        )
        self._frame: _np.ndarray | None = None
        self._timestamp_ns = 0
        self._sequence = 0
        self._consecutive_failures = 0
        self._read_error: str | None = None

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        with self._condition:
            self._condition.notify_all()

    def join(self, timeout: float = 1.0) -> None:
        if (
            self._thread.is_alive()
            and self._thread is not _threading.current_thread()
        ):
            self._thread.join(timeout=timeout)

    def read_after(
        self,
        sequence: int,
    ) -> tuple[int, _np.ndarray, int]:
        with self._condition:
            while (
                self._sequence <= sequence
                and self._consecutive_failures < _MAX_READ_FAILURES
                and not self._stop_event.is_set()
            ):
                self._condition.wait(timeout=0.25)

            if self._consecutive_failures >= _MAX_READ_FAILURES:
                detail = (
                    f": {self._read_error}"
                    if self._read_error is not None
                    else ""
                )
                raise RuntimeError(
                    f"摄像头连续读取失败 {_MAX_READ_FAILURES} 次{detail}"
                )
            if self._frame is None or self._sequence <= sequence:
                raise RuntimeError("摄像头采集线程已停止")
            return self._sequence, self._frame, self._timestamp_ns

    def _read_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                ok, frame = self._cap.read()
            except Exception as exc:
                with self._condition:
                    self._read_error = f"{type(exc).__name__}: {exc}"
                    self._consecutive_failures = _MAX_READ_FAILURES
                    self._condition.notify_all()
                return

            if not ok or frame is None:
                with self._condition:
                    self._consecutive_failures += 1
                    failed = self._consecutive_failures
                    self._condition.notify_all()
                if failed >= _MAX_READ_FAILURES:
                    return
                self._stop_event.wait(0.01)
                continue

            captured_ns = _time.monotonic_ns()
            with self._condition:
                self._frame = frame
                self._timestamp_ns = captured_ns
                self._sequence += 1
                self._consecutive_failures = 0
                self._read_error = None
                self._condition.notify_all()


def _safe_capture_set(
    cap: _cv2.VideoCapture,
    property_id: int,
    value: float,
) -> None:
    try:
        cap.set(property_id, value)
    except (TypeError, ValueError, _cv2.error):
        return


def _open_camera(camera_index: int) -> _cv2.VideoCapture:
    cap: _cv2.VideoCapture
    if _sys.platform.startswith("linux"):
        cap = _cv2.VideoCapture(camera_index, _cv2.CAP_V4L2)
        if not cap.isOpened():
            cap.release()
            cap = _cv2.VideoCapture(camera_index)
    else:
        cap = _cv2.VideoCapture(camera_index)

    if not cap.isOpened():
        cap.release()
        raise RuntimeError(f"无法打开摄像头索引 {camera_index}")

    if _sys.platform.startswith("linux"):
        fourcc = _cv2.VideoWriter_fourcc(*"MJPG")
        _safe_capture_set(cap, _cv2.CAP_PROP_FOURCC, float(fourcc))
    _safe_capture_set(cap, _cv2.CAP_PROP_FRAME_WIDTH, float(_FRAME_WIDTH))
    _safe_capture_set(cap, _cv2.CAP_PROP_FRAME_HEIGHT, float(_FRAME_HEIGHT))
    _safe_capture_set(cap, _cv2.CAP_PROP_FPS, 60.0)
    _safe_capture_set(cap, _cv2.CAP_PROP_BUFFERSIZE, 1.0)
    return cap


def _warm_up_camera(cap: _cv2.VideoCapture) -> None:
    successful_frames = 0
    consecutive_failures = 0
    while successful_frames < 20:
        try:
            ok, frame = cap.read()
        except Exception as exc:
            raise RuntimeError(
                f"摄像头预热读取失败: {type(exc).__name__}: {exc}"
            ) from exc
        if ok and frame is not None:
            successful_frames += 1
            consecutive_failures = 0
            continue
        consecutive_failures += 1
        if consecutive_failures >= _MAX_READ_FAILURES:
            raise RuntimeError(
                f"摄像头连续读取失败 {_MAX_READ_FAILURES} 次"
            )
        _time.sleep(0.01)


def _build_binary_variants(
    gray: _np.ndarray,
    _resources: _VisionResources | None = None,
) -> list[_np.ndarray]:
    """Build independent dark-pattern masks for one uint8 gray frame."""
    if gray.dtype != _np.uint8 or gray.ndim != 2:
        raise ValueError("gray 必须是 uint8 单通道图像")

    resources = _resources or _VisionResources()
    blurred = _cv2.GaussianBlur(gray, (3, 3), 0)
    p_low, p_high = _np.percentile(blurred, (3, 97))
    if float(p_high - p_low) >= 20.0:
        scale = 255.0 / float(p_high - p_low)
        normalized = _np.clip(
            (blurred.astype(_np.float32) - float(p_low)) * scale,
            0.0,
            255.0,
        ).astype(_np.uint8)
    else:
        normalized = blurred

    enhanced = resources.clahe.apply(normalized)
    adaptive_31 = _cv2.adaptiveThreshold(
        enhanced,
        255,
        _cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        _cv2.THRESH_BINARY_INV,
        31,
        5,
    )
    adaptive_51 = _cv2.adaptiveThreshold(
        enhanced,
        255,
        _cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        _cv2.THRESH_BINARY_INV,
        51,
        7,
    )
    blackhat = _cv2.morphologyEx(
        enhanced,
        _cv2.MORPH_BLACKHAT,
        resources.blackhat_kernel,
    )
    _, blackhat_mask = _cv2.threshold(
        blackhat,
        0,
        255,
        _cv2.THRESH_BINARY | _cv2.THRESH_OTSU,
    )

    masks = [adaptive_31, adaptive_51, blackhat_mask]
    for index, mask in enumerate(masks):
        opened = _cv2.morphologyEx(
            mask,
            _cv2.MORPH_OPEN,
            resources.small_kernel,
            iterations=1,
        )
        masks[index] = _cv2.morphologyEx(
            opened,
            _cv2.MORPH_CLOSE,
            resources.small_kernel,
            iterations=1,
        )
    return masks


def _clip_roi(roi: _Roi, width: int, height: int) -> _Roi:
    x0, y0, x1, y1 = roi
    x0 = max(0, min(width, int(x0)))
    y0 = max(0, min(height, int(y0)))
    x1 = max(x0, min(width, int(x1)))
    y1 = max(y0, min(height, int(y1)))
    return x0, y0, x1, y1


def _square_roi(
    center: tuple[float, float],
    side: float,
    width: int,
    height: int,
) -> _Roi:
    half = max(1.0, side / 2.0)
    return _clip_roi(
        (
            int(_math.floor(center[0] - half)),
            int(_math.floor(center[1] - half)),
            int(_math.ceil(center[0] + half)),
            int(_math.ceil(center[1] + half)),
        ),
        width,
        height,
    )


def _normalize_ellipse(raw_ellipse: tuple) -> _Ellipse | None:
    (cx, cy), (axis_a, axis_b), angle = raw_ellipse
    values = _np.asarray(
        (cx, cy, axis_a, axis_b, angle),
        dtype=_np.float64,
    )
    if not _np.all(_np.isfinite(values)):
        return None
    if axis_a <= 0.0 or axis_b <= 0.0:
        return None
    if axis_a >= axis_b:
        major = float(axis_a)
        minor = float(axis_b)
        major_angle = float(angle) % 180.0
    else:
        major = float(axis_b)
        minor = float(axis_a)
        major_angle = (float(angle) + 90.0) % 180.0
    return (
        (float(cx), float(cy)),
        (major, minor),
        major_angle,
    )


def _ellipse_normalized_points(
    points: _np.ndarray,
    ellipse: _Ellipse,
) -> _np.ndarray:
    (cx, cy), (major, minor), angle = ellipse
    if major <= 0.0 or minor <= 0.0:
        return _np.full((len(points), 2), _np.nan, dtype=_np.float64)
    radians = _math.radians(angle)
    cosine = _math.cos(radians)
    sine = _math.sin(radians)
    shifted = points.astype(_np.float64) - _np.asarray((cx, cy))
    rotated_u = shifted[:, 0] * cosine + shifted[:, 1] * sine
    rotated_v = -shifted[:, 0] * sine + shifted[:, 1] * cosine
    return _np.column_stack(
        (
            rotated_u / (major / 2.0),
            rotated_v / (minor / 2.0),
        )
    )


def _ellipse_bounds(ellipse: _Ellipse) -> tuple[float, float, float, float]:
    (cx, cy), (major, minor), angle = ellipse
    radians = _math.radians(angle)
    cosine = _math.cos(radians)
    sine = _math.sin(radians)
    radius_u = _math.sqrt(
        (major * cosine / 2.0) ** 2
        + (minor * sine / 2.0) ** 2
    )
    radius_v = _math.sqrt(
        (major * sine / 2.0) ** 2
        + (minor * cosine / 2.0) ** 2
    )
    return cx - radius_u, cy - radius_v, cx + radius_u, cy + radius_v


def _descendant_holes(
    hierarchy: _np.ndarray,
    parent_index: int,
) -> list[int]:
    holes: list[int] = []
    first_child = int(hierarchy[parent_index][2])
    stack: list[tuple[int, int]] = []
    child = first_child
    while child != -1:
        stack.append((child, 1))
        child = int(hierarchy[child][0])

    while stack:
        contour_index, depth = stack.pop()
        if depth % 2 == 1:
            holes.append(contour_index)
        child = int(hierarchy[contour_index][2])
        while child != -1:
            stack.append((child, depth + 1))
            child = int(hierarchy[child][0])
    return holes


def _angular_uniformity(angles: _np.ndarray) -> float:
    if len(angles) < 3:
        return 0.0
    ordered = _np.sort(_np.mod(angles, 2.0 * _math.pi))
    wrapped = _np.append(ordered, ordered[0] + 2.0 * _math.pi)
    gaps = _np.diff(wrapped)
    errors = _np.abs(gaps - _math.pi / 2.0)
    scores = _np.exp(-((errors / (_math.pi / 3.0)) ** 2))
    return float(_np.clip(_np.mean(scores), 0.0, 1.0))


def _radial_range_score(value: float, ideal: float, radius: float) -> float:
    if radius <= 0.0:
        return 0.0
    return float(_np.clip(1.0 - abs(value - ideal) / radius, 0.0, 1.0))


def _analyze_holes(
    contours: list[_np.ndarray],
    hierarchy: _np.ndarray,
    parent_index: int,
    ellipse: _Ellipse,
) -> _HoleAnalysis | None:
    (_, _), (major, minor), _ = ellipse
    ellipse_area = _math.pi * major * minor / 4.0
    if ellipse_area <= 0.0:
        return None

    centers: list[tuple[float, float]] = []
    areas: list[float] = []
    for contour_index in _descendant_holes(hierarchy, parent_index):
        area = abs(float(_cv2.contourArea(contours[contour_index])))
        if area < ellipse_area * 0.008 or area > ellipse_area * 0.20:
            continue
        moments = _cv2.moments(contours[contour_index])
        denominator = float(moments["m00"])
        if abs(denominator) < 1e-9:
            continue
        center = (
            float(moments["m10"] / denominator),
            float(moments["m01"] / denominator),
        )
        normalized = _ellipse_normalized_points(
            _np.asarray([center], dtype=_np.float64),
            ellipse,
        )[0]
        if not _np.all(_np.isfinite(normalized)):
            continue
        if float(_np.dot(normalized, normalized)) > 1.0:
            continue
        centers.append(center)
        areas.append(area)

    if not 6 <= len(centers) <= 10:
        return None

    center_array = _np.asarray(centers, dtype=_np.float64)
    area_array = _np.asarray(areas, dtype=_np.float64)
    normalized = _ellipse_normalized_points(center_array, ellipse)
    radii = _np.linalg.norm(normalized, axis=1)
    angles = _np.mod(
        _np.arctan2(normalized[:, 1], normalized[:, 0]),
        2.0 * _math.pi,
    )
    if not _np.all(_np.isfinite(radii)):
        return None

    samples = radii.astype(_np.float32).reshape(-1, 1)
    initial_labels = _np.zeros((len(samples), 1), dtype=_np.int32)
    radius_order = _np.argsort(samples[:, 0])
    initial_labels[radius_order[len(samples) // 2:], 0] = 1
    criteria = (
        _cv2.TERM_CRITERIA_EPS | _cv2.TERM_CRITERIA_COUNT,
        30,
        0.001,
    )
    try:
        compactness, labels, cluster_centers = _cv2.kmeans(
            samples,
            2,
            initial_labels,
            criteria,
            1,
            _cv2.KMEANS_USE_INITIAL_LABELS,
        )
    except _cv2.error:
        return None
    if not _np.isfinite(compactness):
        return None
    if not _np.all(_np.isfinite(cluster_centers)):
        return None

    labels = labels.reshape(-1)
    means = [
        float(_np.mean(radii[labels == cluster]))
        for cluster in (0, 1)
    ]
    order = _np.argsort(means)
    inner_label = int(order[0])
    outer_label = int(order[1])
    inner_indices = labels == inner_label
    outer_indices = labels == outer_label
    if int(_np.count_nonzero(inner_indices)) < 3:
        return None
    if int(_np.count_nonzero(outer_indices)) < 3:
        return None

    inner_mean = means[inner_label]
    outer_mean = means[outer_label]
    if outer_mean - inner_mean < 0.15:
        return None
    if not 0.15 <= inner_mean <= 0.55:
        return None
    if not 0.45 <= outer_mean <= 0.95:
        return None

    quadrant_counts: list[int] = []
    uniformity_scores: list[float] = []
    for indices in (inner_indices, outer_indices):
        cluster_angles = angles[indices]
        quadrants = _np.floor(
            cluster_angles / (_math.pi / 2.0)
        ).astype(_np.int32)
        quadrant_count = len(_np.unique(_np.clip(quadrants, 0, 3)))
        if quadrant_count < 3:
            return None
        quadrant_counts.append(quadrant_count)
        uniformity_scores.append(_angular_uniformity(cluster_angles))

    count_score = float(
        _np.clip(1.0 - abs(len(centers) - 8) / 4.0, 0.0, 1.0)
    )
    separation_score = float(
        _np.clip((outer_mean - inner_mean - 0.15) / 0.30, 0.0, 1.0)
    )
    radial_score = 0.5 * (
        _radial_range_score(inner_mean, 0.32, 0.25)
        + _radial_range_score(outer_mean, 0.72, 0.28)
    )
    quadrant_score = sum(quadrant_counts) / 8.0
    uniformity_score = sum(uniformity_scores) / 2.0
    score = float(
        _np.clip(
            0.18 * count_score
            + 0.20 * separation_score
            + 0.20 * radial_score
            + 0.17 * quadrant_score
            + 0.25 * uniformity_score,
            0.0,
            1.0,
        )
    )
    area_sum = float(_np.sum(area_array))
    if area_sum <= 0.0:
        return None
    weighted = _np.sum(center_array * area_array[:, None], axis=0) / area_sum
    return _HoleAnalysis(
        score=score,
        centers=center_array,
        areas=area_array,
        weighted_center=(float(weighted[0]), float(weighted[1])),
    )


def _ellipse_fit_score(
    contour: _np.ndarray,
    ellipse: _Ellipse,
    image_width: int,
    image_height: int,
    roi_clipped: bool,
) -> tuple[float, bool]:
    (_, _), (major, minor), _ = ellipse
    axis_ratio = minor / major if major > 0.0 else 0.0
    points = contour.reshape(-1, 2).astype(_np.float64)
    normalized = _ellipse_normalized_points(points, ellipse)
    radii = _np.linalg.norm(normalized, axis=1)
    if not _np.all(_np.isfinite(radii)):
        return 0.0, True
    residual = float(_np.median(_np.abs(radii - 1.0)))
    residual_score = float(_np.clip(1.0 - residual / 0.30, 0.0, 1.0))
    ratio_score = float(
        _np.clip((axis_ratio - 0.35) / 0.40, 0.0, 1.0)
    )
    left, top, right, bottom = _ellipse_bounds(ellipse)
    image_clipped = (
        left < 0.0
        or top < 0.0
        or right > image_width - 1
        or bottom > image_height - 1
    )
    clipped = image_clipped or roi_clipped
    edge_score = 0.45 if clipped else 1.0
    score = float(
        _np.clip(
            0.55 * residual_score
            + 0.25 * ratio_score
            + 0.20 * edge_score,
            0.0,
            1.0,
        )
    )
    return score, clipped


def _ellipse_occupancy(
    binary: _np.ndarray,
    ellipse: _Ellipse,
) -> tuple[float, float] | None:
    height, width = binary.shape
    left, top, right, bottom = _ellipse_bounds(ellipse)
    x0, y0, x1, y1 = _clip_roi(
        (
            int(_math.floor(left)) - 2,
            int(_math.floor(top)) - 2,
            int(_math.ceil(right)) + 3,
            int(_math.ceil(bottom)) + 3,
        ),
        width,
        height,
    )
    if x1 <= x0 or y1 <= y0:
        return None
    (cx, cy), (major, minor), angle = ellipse
    ellipse_mask = _np.zeros((y1 - y0, x1 - x0), dtype=_np.uint8)
    _cv2.ellipse(
        ellipse_mask,
        (int(round(cx - x0)), int(round(cy - y0))),
        (
            max(1, int(round(major / 2.0))),
            max(1, int(round(minor / 2.0))),
        ),
        angle,
        0.0,
        360.0,
        255,
        -1,
    )
    inside = ellipse_mask > 0
    pixel_count = int(_np.count_nonzero(inside))
    if pixel_count == 0:
        return None
    # Adaptive thresholds deliberately preserve weak dark edges, which can
    # bloom by roughly one pixel after blur.  Erode only the measurement copy
    # so occupancy describes the dark core without changing contour topology.
    occupancy_binary = _cv2.erode(
        binary[y0:y1, x0:x1],
        None,
        iterations=1,
    )
    occupancy = float(
        _np.count_nonzero(occupancy_binary[inside]) / pixel_count
    )
    if occupancy < 0.08 or occupancy > 0.42:
        return None
    if occupancy <= 0.20:
        score = (occupancy - 0.08) / 0.12
    else:
        score = (0.42 - occupancy) / 0.22
    return occupancy, float(_np.clip(score, 0.0, 1.0))


def _temporal_score(
    center: tuple[float, float],
    diameter: float,
    previous_center: tuple[float, float] | None,
    previous_diameter: float | None,
    search_roi: _Roi | None,
) -> float:
    if previous_center is None or previous_diameter is None:
        return 0.5
    if previous_diameter <= 0.0 or diameter <= 0.0:
        return 0.0
    distance = _math.hypot(
        center[0] - previous_center[0],
        center[1] - previous_center[1],
    )
    distance_score = _math.exp(
        -distance / max(1.0, 0.60 * previous_diameter)
    )
    size_score = _math.exp(
        -abs(_math.log(diameter / previous_diameter)) / 0.40
    )
    roi_score = 0.5
    if search_roi is not None:
        x0, y0, x1, y1 = search_roi
        roi_score = (
            1.0
            if x0 <= center[0] < x1 and y0 <= center[1] < y1
            else 0.0
        )
    return float(
        _np.clip(
            0.55 * distance_score + 0.35 * size_score + 0.10 * roi_score,
            0.0,
            1.0,
        )
    )


def _collect_candidates(
    masks: list[_np.ndarray],
    search_roi: _Roi | None,
    previous_center: tuple[float, float] | None,
    previous_diameter: float | None,
) -> list[_Candidate]:
    if not masks:
        return []
    image_height, image_width = masks[0].shape
    image_area = float(image_width * image_height)
    minimum_area = max(300.0, image_area * 0.0015)
    maximum_area = image_area * 0.65
    maximum_axis = _math.hypot(image_width, image_height) * 1.1
    roi = search_roi or (0, 0, image_width, image_height)
    x0, y0, x1, y1 = _clip_roi(roi, image_width, image_height)
    if x1 <= x0 or y1 <= y0:
        return []

    candidates: list[_Candidate] = []
    for mask_index, mask in enumerate(masks):
        roi_mask = mask[y0:y1, x0:x1].copy()
        contours, raw_hierarchy = _cv2.findContours(
            roi_mask,
            _cv2.RETR_TREE,
            _cv2.CHAIN_APPROX_SIMPLE,
        )
        if raw_hierarchy is None or not contours:
            continue
        hierarchy = raw_hierarchy[0]
        offset = _np.asarray([[[x0, y0]]], dtype=_np.int32)
        global_contours = [contour + offset for contour in contours]

        for contour_index, contour in enumerate(global_contours):
            if len(contour) < 20:
                continue
            area = abs(float(_cv2.contourArea(contour)))
            if area < minimum_area or area > maximum_area:
                continue
            try:
                ellipse = _normalize_ellipse(_cv2.fitEllipse(contour))
            except _cv2.error:
                continue
            if ellipse is None:
                continue
            (cx, cy), (major, minor), _ = ellipse
            if minor < 24.0 or major > maximum_axis:
                continue
            if major <= 0.0 or minor / major < 0.35:
                continue
            if not (-major <= cx <= image_width + major):
                continue
            if not (-major <= cy <= image_height + major):
                continue

            local_x, local_y, local_w, local_h = _cv2.boundingRect(
                contours[contour_index]
            )
            roi_clipped = search_roi is not None and (
                local_x <= 1
                or local_y <= 1
                or local_x + local_w >= (x1 - x0) - 1
                or local_y + local_h >= (y1 - y0) - 1
            )
            ellipse_score, clipped = _ellipse_fit_score(
                contour,
                ellipse,
                image_width,
                image_height,
                roi_clipped,
            )
            if ellipse_score <= 0.20:
                continue
            holes = _analyze_holes(
                global_contours,
                hierarchy,
                contour_index,
                ellipse,
            )
            if holes is None:
                continue
            occupancy = _ellipse_occupancy(mask, ellipse)
            if occupancy is None:
                continue
            _, occupancy_score = occupancy
            diameter = (major + minor) / 2.0
            temporal = _temporal_score(
                (cx, cy),
                diameter,
                previous_center,
                previous_diameter,
                search_roi,
            )
            known_weighted_score = (
                0.15 * ellipse_score
                + 0.35 * holes.score
                + 0.08 * occupancy_score
                + 0.12 * temporal
            )
            candidates.append(
                _Candidate(
                    ellipse=ellipse,
                    diameter_px=diameter,
                    mask_index=mask_index,
                    ellipse_score=ellipse_score,
                    hole_score=holes.score,
                    occupancy_score=occupancy_score,
                    temporal_score=temporal,
                    hole_center=holes.weighted_center,
                    clipped=clipped,
                    pre_score=known_weighted_score / 0.70,
                )
            )
    candidates.sort(key=lambda candidate: candidate.pre_score, reverse=True)
    return candidates[:3]


def _line_angle_difference(first: float, second: float) -> float:
    difference = abs(first - second) % _math.pi
    return min(difference, _math.pi - difference)


def _filter_line_segment(
    coordinates: _np.ndarray,
    ellipse_mask: _np.ndarray,
    center: tuple[float, float],
    diameter: float,
) -> _LineSegment | None:
    x1, y1, x2, y2 = (float(value) for value in coordinates)
    dx = x2 - x1
    dy = y2 - y1
    length = _math.hypot(dx, dy)
    if length < 0.30 * diameter:
        return None
    midpoint = ((x1 + x2) / 2.0, (y1 + y2) / 2.0)
    if (
        _math.hypot(
            midpoint[0] - center[0],
            midpoint[1] - center[1],
        )
        > 0.42 * diameter
    ):
        return None

    samples = _np.linspace(0.0, 1.0, 9)
    sample_x = _np.clip(
        _np.rint(x1 + samples * dx).astype(_np.int32),
        0,
        ellipse_mask.shape[1] - 1,
    )
    sample_y = _np.clip(
        _np.rint(y1 + samples * dy).astype(_np.int32),
        0,
        ellipse_mask.shape[0] - 1,
    )
    if float(_np.mean(ellipse_mask[sample_y, sample_x] > 0)) < 0.67:
        return None
    angle = _math.atan2(dy, dx) % _math.pi
    return _LineSegment(x1, y1, x2, y2, length, angle)


def _extract_line_segments(
    edges: _np.ndarray,
    ellipse_mask: _np.ndarray,
    center: tuple[float, float],
    diameter: float,
    resources: _VisionResources,
) -> list[_LineSegment]:
    raw_lines: list[_np.ndarray] = []
    if resources.lsd is not None:
        try:
            detected = resources.lsd.detect(edges)
        except _cv2.error:
            detected = None
        if detected is not None and detected[0] is not None:
            raw_lines.extend(detected[0].reshape(-1, 4))

    segments = [
        segment
        for raw_line in raw_lines
        if (
            segment := _filter_line_segment(
                raw_line,
                ellipse_mask,
                center,
                diameter,
            )
        )
        is not None
    ]
    if len(segments) >= 4:
        return segments

    hough = _cv2.HoughLinesP(
        edges,
        1.0,
        _math.pi / 180.0,
        threshold=max(18, int(round(0.16 * diameter))),
        minLineLength=max(8, int(round(0.28 * diameter))),
        maxLineGap=max(3, int(round(0.10 * diameter))),
    )
    if hough is None:
        return segments
    for raw_line in hough.reshape(-1, 4):
        segment = _filter_line_segment(
            raw_line,
            ellipse_mask,
            center,
            diameter,
        )
        if segment is not None:
            segments.append(segment)
    return segments


def _refit_edge_offset(
    segments: list[_LineSegment],
    target_signed_offset: float,
    direction: tuple[float, float],
    normal: tuple[float, float],
    center: tuple[float, float],
    edge_points: _np.ndarray,
    diameter: float,
) -> tuple[float, float] | None:
    if not segments or len(edge_points) == 0:
        return None
    projection_values: list[float] = []
    for segment in segments:
        projection_values.extend(
            (
                direction[0] * segment.x1 + direction[1] * segment.y1,
                direction[0] * segment.x2 + direction[1] * segment.y2,
            )
        )
    projection_min = min(projection_values) - 3.0
    projection_max = max(projection_values) + 3.0
    relative = edge_points - _np.asarray(center, dtype=_np.float64)
    signed = relative[:, 0] * normal[0] + relative[:, 1] * normal[1]
    projected = (
        edge_points[:, 0] * direction[0]
        + edge_points[:, 1] * direction[1]
    )
    distance_tolerance = max(1.5, 0.014 * diameter)
    selected = (
        (_np.abs(signed - target_signed_offset) <= distance_tolerance)
        & (projected >= projection_min)
        & (projected <= projection_max)
    )
    selected_points = edge_points[selected]
    if len(selected_points) < max(10, int(round(0.10 * diameter))):
        return None
    try:
        fitted = _cv2.fitLine(
            selected_points.astype(_np.float32).reshape(-1, 1, 2),
            _cv2.DIST_L2,
            0,
            0.01,
            0.01,
        ).reshape(-1)
    except _cv2.error:
        return None
    if len(fitted) != 4 or not _np.all(_np.isfinite(fitted)):
        return None
    vx, vy, point_x, point_y = (float(value) for value in fitted)
    norm = _math.hypot(vx, vy)
    if norm <= 1e-9:
        return None
    fitted_angle = _math.atan2(vy / norm, vx / norm) % _math.pi
    expected_angle = _math.atan2(direction[1], direction[0]) % _math.pi
    if _line_angle_difference(fitted_angle, expected_angle) > _math.radians(15):
        return None
    # Project the fitted point onto the common cluster normal.  Keeping a
    # common normal makes the median of both edge offsets a stable centerline.
    line_offset = normal[0] * point_x + normal[1] * point_y
    support = float(
        _np.clip(len(selected_points) / max(20.0, 0.8 * diameter), 0.0, 1.0)
    )
    return line_offset, support


def _build_cross_center_line(
    segments: list[_LineSegment],
    direction_angle: float,
    edge_points: _np.ndarray,
    center: tuple[float, float],
    diameter: float,
) -> _CrossLine | None:
    direction = (
        _math.cos(direction_angle),
        _math.sin(direction_angle),
    )
    normal = (-direction[1], direction[0])
    if normal[0] < 0.0 or (
        abs(normal[0]) < 1e-9 and normal[1] < 0.0
    ):
        normal = (-normal[0], -normal[1])

    signed_segments: list[tuple[_LineSegment, float]] = []
    center_projection = normal[0] * center[0] + normal[1] * center[1]
    for segment in segments:
        midpoint_x = (segment.x1 + segment.x2) / 2.0
        midpoint_y = (segment.y1 + segment.y2) / 2.0
        signed_offset = (
            normal[0] * midpoint_x
            + normal[1] * midpoint_y
            - center_projection
        )
        signed_segments.append((segment, signed_offset))

    best_pair: tuple[_LineSegment, float, _LineSegment, float] | None = None
    best_pair_score = -_math.inf
    minimum_width = 0.012 * diameter
    maximum_width = 0.12 * diameter
    for first_index, first_item in enumerate(signed_segments):
        for second_item in signed_segments[first_index + 1:]:
            lower, upper = sorted(
                (first_item, second_item),
                key=lambda item: item[1],
            )
            lower_segment, lower_offset = lower
            upper_segment, upper_offset = upper
            width = upper_offset - lower_offset
            if width < minimum_width or width > maximum_width:
                continue
            pair_midpoint = (lower_offset + upper_offset) / 2.0
            if abs(pair_midpoint) > 0.16 * diameter:
                continue
            center_proximity = 1.0 - min(
                1.0,
                abs(pair_midpoint) / max(1e-6, 0.16 * diameter),
            )
            length_score = min(
                1.0,
                (lower_segment.length + upper_segment.length)
                / max(1.0, 1.8 * diameter),
            )
            straddles_center = lower_offset <= 0.0 <= upper_offset
            pair_score = (
                0.25 * center_proximity
                + 0.70 * length_score
                + (0.05 if straddles_center else 0.0)
            )
            if pair_score > best_pair_score:
                best_pair_score = pair_score
                best_pair = (
                    lower_segment,
                    lower_offset,
                    upper_segment,
                    upper_offset,
                )
    if best_pair is None:
        return None

    negative_segment, negative_offset, positive_segment, positive_offset = (
        best_pair
    )
    grouping_tolerance = max(1.5, 0.018 * diameter)
    lower_group = [
        segment
        for segment, offset in signed_segments
        if abs(offset - negative_offset) <= grouping_tolerance
    ]
    upper_group = [
        segment
        for segment, offset in signed_segments
        if abs(offset - positive_offset) <= grouping_tolerance
    ]
    if negative_segment not in lower_group:
        lower_group.append(negative_segment)
    if positive_segment not in upper_group:
        upper_group.append(positive_segment)

    refitted_negative = _refit_edge_offset(
        lower_group,
        negative_offset,
        direction,
        normal,
        center,
        edge_points,
        diameter,
    )
    refitted_positive = _refit_edge_offset(
        upper_group,
        positive_offset,
        direction,
        normal,
        center,
        edge_points,
        diameter,
    )
    if refitted_negative is None or refitted_positive is None:
        return None
    negative_line_offset, negative_support = refitted_negative
    positive_line_offset, positive_support = refitted_positive
    width = positive_line_offset - negative_line_offset
    if width < minimum_width or width > maximum_width:
        return None
    center_offset = (negative_line_offset + positive_line_offset) / 2.0
    support_score = float(
        _np.clip(
            0.5 * (negative_support + positive_support)
            * max(0.0, best_pair_score),
            0.0,
            1.0,
        )
    )
    return _CrossLine(
        normal_x=normal[0],
        normal_y=normal[1],
        offset=center_offset,
        width=width,
        support_score=support_score,
    )


def _refine_center_from_cross(
    gray: _np.ndarray,
    binary: _np.ndarray,
    ellipse: tuple,
    _resources: _VisionResources | None = None,
) -> tuple[float, float, float] | None:
    """Return center_u, center_v and a normalized cross confidence."""
    resources = _resources or _VisionResources()
    normalized_ellipse = _normalize_ellipse(ellipse)
    if normalized_ellipse is None:
        return None
    (cx, cy), (major, minor), angle = normalized_ellipse
    diameter = (major + minor) / 2.0
    if diameter <= 0.0:
        return None
    height, width = binary.shape
    roi = _square_roi(
        (cx, cy),
        1.25 * major,
        width,
        height,
    )
    x0, y0, x1, y1 = roi
    if x1 - x0 < 8 or y1 - y0 < 8:
        return None

    binary_roi = binary[y0:y1, x0:x1]
    gray_roi = gray[y0:y1, x0:x1]
    if binary_roi.shape != gray_roi.shape:
        return None
    edges = _cv2.Canny(binary_roi, 50, 150)
    local_center = (cx - x0, cy - y0)
    local_ellipse: _Ellipse = (
        local_center,
        (major, minor),
        angle,
    )
    ellipse_mask = _np.zeros_like(binary_roi)
    _cv2.ellipse(
        ellipse_mask,
        (int(round(local_center[0])), int(round(local_center[1]))),
        (
            max(1, int(round(major / 2.0 + 2.0))),
            max(1, int(round(minor / 2.0 + 2.0))),
        ),
        angle,
        0.0,
        360.0,
        255,
        -1,
    )
    edges = _cv2.bitwise_and(edges, ellipse_mask)
    segments = _extract_line_segments(
        edges,
        ellipse_mask,
        local_center,
        diameter,
        resources,
    )
    if len(segments) < 4:
        return None

    directions = _np.asarray(
        [
            (
                _math.cos(2.0 * segment.angle),
                _math.sin(2.0 * segment.angle),
            )
            for segment in segments
        ],
        dtype=_np.float32,
    )
    longest_index = int(
        _np.argmax(
            _np.asarray(
                [segment.length for segment in segments],
                dtype=_np.float64,
            )
        )
    )
    reference_direction = directions[longest_index]
    direction_projection = directions @ reference_direction
    initial_labels = (direction_projection < 0.0).astype(
        _np.int32
    ).reshape(-1, 1)
    if len(_np.unique(initial_labels)) < 2:
        projection_order = _np.argsort(direction_projection)
        initial_labels.fill(0)
        initial_labels[projection_order[len(segments) // 2:], 0] = 1
    criteria = (
        _cv2.TERM_CRITERIA_EPS | _cv2.TERM_CRITERIA_COUNT,
        30,
        0.001,
    )
    try:
        compactness, labels, centers = _cv2.kmeans(
            directions,
            2,
            initial_labels,
            criteria,
            1,
            _cv2.KMEANS_USE_INITIAL_LABELS,
        )
    except _cv2.error:
        return None
    if not _np.isfinite(compactness) or not _np.all(_np.isfinite(centers)):
        return None
    labels = labels.reshape(-1)
    if any(int(_np.count_nonzero(labels == label)) < 2 for label in (0, 1)):
        return None

    direction_angles: list[float] = []
    for label, center_vector in enumerate(centers):
        cluster_segments = [
            segment
            for segment, segment_label in zip(segments, labels)
            if int(segment_label) == label
        ]
        longest_length = max(
            segment.length for segment in cluster_segments
        )
        stable_segments = [
            segment
            for segment in cluster_segments
            if segment.length >= 0.65 * longest_length
        ]
        weights = _np.asarray(
            [segment.length ** 2 for segment in stable_segments],
            dtype=_np.float64,
        )
        doubled = _np.asarray(
            [
                (
                    _math.cos(2.0 * segment.angle),
                    _math.sin(2.0 * segment.angle),
                )
                for segment in stable_segments
            ],
            dtype=_np.float64,
        )
        weighted_vector = _np.sum(doubled * weights[:, None], axis=0)
        vector_x, vector_y = (
            float(weighted_vector[0]),
            float(weighted_vector[1]),
        )
        if _math.hypot(vector_x, vector_y) <= 1e-9:
            vector_x, vector_y = (
                float(center_vector[0]),
                float(center_vector[1]),
            )
            if _math.hypot(vector_x, vector_y) <= 1e-9:
                return None
        direction_angles.append(
            (0.5 * _math.atan2(vector_y, vector_x)) % _math.pi
        )
    angle_difference = _line_angle_difference(
        direction_angles[0],
        direction_angles[1],
    )
    angle_degrees = _math.degrees(angle_difference)
    if not 55.0 <= angle_degrees <= 125.0:
        return None

    edge_y, edge_x = _np.nonzero(edges)
    edge_points = _np.column_stack((edge_x, edge_y)).astype(_np.float64)
    cross_lines: list[_CrossLine] = []
    for label in (0, 1):
        cluster = [
            segment
            for segment, segment_label in zip(segments, labels)
            if int(segment_label) == label
        ]
        center_line = _build_cross_center_line(
            cluster,
            direction_angles[label],
            edge_points,
            local_center,
            diameter,
        )
        if center_line is None:
            return None
        cross_lines.append(center_line)

    first, second = cross_lines
    determinant = (
        first.normal_x * second.normal_y
        - first.normal_y * second.normal_x
    )
    if abs(determinant) <= 1e-6:
        return None
    intersection_u = (
        first.offset * second.normal_y
        - first.normal_y * second.offset
    ) / determinant
    intersection_v = (
        first.normal_x * second.offset
        - first.offset * second.normal_x
    ) / determinant
    if not _np.all(_np.isfinite((intersection_u, intersection_v))):
        return None

    normalized_intersection = _ellipse_normalized_points(
        _np.asarray(
            [[intersection_u, intersection_v]],
            dtype=_np.float64,
        ),
        local_ellipse,
    )[0]
    if float(_np.dot(normalized_intersection, normalized_intersection)) > 1.0:
        return None
    center_distance = _math.hypot(
        intersection_u - local_center[0],
        intersection_v - local_center[1],
    )
    if center_distance > 0.16 * diameter:
        return None
    minimum_width = min(first.width, second.width)
    if minimum_width <= 0.0:
        return None
    if max(first.width, second.width) / minimum_width > 2.5:
        return None

    angle_score = float(
        _np.clip(1.0 - abs(angle_degrees - 90.0) / 35.0, 0.0, 1.0)
    )
    mean_width = _math.sqrt(first.width * second.width)
    theoretical_width = max(1e-6, 0.04 * diameter)
    width_score = _math.exp(
        -abs(_math.log(mean_width / theoretical_width)) / _math.log(3.0)
    )
    support_score = 0.5 * (
        first.support_score + second.support_score
    )
    center_score = float(
        _np.clip(
            1.0 - center_distance / max(1e-6, 0.16 * diameter),
            0.0,
            1.0,
        )
    )
    cross_score = float(
        _np.clip(
            0.30 * angle_score
            + 0.25 * width_score
            + 0.25 * support_score
            + 0.20 * center_score,
            0.0,
            1.0,
        )
    )
    return (
        float(intersection_u + x0),
        float(intersection_v + y0),
        cross_score,
    )


def _candidate_to_detection(
    gray: _np.ndarray,
    masks: list[_np.ndarray],
    candidate: _Candidate,
    resources: _VisionResources,
) -> _Detection | None:
    refined = _refine_center_from_cross(
        gray,
        masks[candidate.mask_index],
        candidate.ellipse,
        resources,
    )
    if refined is None:
        if candidate.hole_score < 0.82 or candidate.clipped:
            return None
        (ellipse_u, ellipse_v), _, _ = candidate.ellipse
        center_u = 0.65 * ellipse_u + 0.35 * candidate.hole_center[0]
        center_v = 0.65 * ellipse_v + 0.35 * candidate.hole_center[1]
        cross_score = 0.58
        fallback_penalty = 0.95
    else:
        center_u, center_v, cross_score = refined
        fallback_penalty = 1.0

    score = (
        0.15 * candidate.ellipse_score
        + 0.35 * candidate.hole_score
        + 0.30 * cross_score
        + 0.08 * candidate.occupancy_score
        + 0.12 * candidate.temporal_score
    )
    left, top, right, bottom = _ellipse_bounds(candidate.ellipse)
    height, width = gray.shape
    blur_roi = _clip_roi(
        (
            int(_math.floor(left)),
            int(_math.floor(top)),
            int(_math.ceil(right)) + 1,
            int(_math.ceil(bottom)) + 1,
        ),
        width,
        height,
    )
    x0, y0, x1, y1 = blur_roi
    if x1 > x0 and y1 > y0:
        blur_variance = float(
            _cv2.Laplacian(
                gray[y0:y1, x0:x1],
                _cv2.CV_64F,
            ).var()
        )
        blur_quality = float(
            _np.clip(
                _math.log1p(max(0.0, blur_variance)) / _math.log1p(500.0),
                0.35,
                1.0,
            )
        )
        score *= 0.97 + 0.03 * blur_quality
    score *= fallback_penalty
    score = float(_np.clip(score, 0.0, 1.0))
    if not _np.all(_np.isfinite((center_u, center_v, score))):
        return None
    return _Detection(
        center_u=float(center_u),
        center_v=float(center_v),
        diameter_px=float(candidate.diameter_px),
        score=score,
        ellipse=candidate.ellipse,
        mask_index=candidate.mask_index,
    )


def _find_marker_candidate(
    gray: _np.ndarray,
    masks: list[_np.ndarray],
    search_roi: _Roi | None,
    previous_center: tuple[float, float] | None,
    previous_diameter: float | None,
    resources: _VisionResources,
) -> _Detection | None:
    detections: list[_Detection] = []
    for candidate in _collect_candidates(
        masks,
        search_roi,
        previous_center,
        previous_diameter,
    ):
        detection = _candidate_to_detection(
            gray,
            masks,
            candidate,
            resources,
        )
        if detection is not None:
            detections.append(detection)
    if not detections:
        return None
    return max(detections, key=lambda detection: detection.score)


def _cross_core_is_dark(
    gray: _np.ndarray,
    center: tuple[float, float],
    diameter: float,
    require_bright_background: bool,
) -> bool:
    """Reject thin-line intersections that are not the marker's broad cross."""
    height, width = gray.shape
    center_u = int(round(center[0]))
    center_v = int(round(center[1]))
    outer_radius = max(18, int(round(0.12 * diameter)))
    core_radius = max(6, int(round(0.035 * diameter)))
    outer_roi = _clip_roi(
        (
            center_u - outer_radius,
            center_v - outer_radius,
            center_u + outer_radius + 1,
            center_v + outer_radius + 1,
        ),
        width,
        height,
    )
    core_roi = _clip_roi(
        (
            center_u - core_radius,
            center_v - core_radius,
            center_u + core_radius + 1,
            center_v + core_radius + 1,
        ),
        width,
        height,
    )
    outer_x0, outer_y0, outer_x1, outer_y1 = outer_roi
    core_x0, core_y0, core_x1, core_y1 = core_roi
    if outer_x1 - outer_x0 < 12 or outer_y1 - outer_y0 < 12:
        return False
    if core_x1 - core_x0 < 8 or core_y1 - core_y0 < 8:
        return False
    outer = gray[outer_y0:outer_y1, outer_x0:outer_x1]
    core = gray[core_y0:core_y1, core_x0:core_x1]
    threshold, _ = _cv2.threshold(
        outer,
        0,
        255,
        _cv2.THRESH_BINARY | _cv2.THRESH_OTSU,
    )
    dark_fraction = float(_np.mean(core <= threshold))
    if dark_fraction < 0.50:
        return False
    if not require_bright_background:
        return True

    local_center_u = center[0] - outer_x0
    local_center_v = center[1] - outer_y0
    local_y, local_x = _np.ogrid[:outer.shape[0], :outer.shape[1]]
    local_radius = _np.hypot(
        local_x - local_center_u,
        local_y - local_center_v,
    )
    annulus = outer[
        (local_radius >= 1.6 * core_radius)
        & (local_radius <= outer_radius)
    ]
    if annulus.size < 32:
        return False
    bright_background = float(_np.percentile(annulus, 75))
    dark_cross = float(_np.percentile(core, 25))
    if bright_background - dark_cross < 120.0:
        return False
    bright_regions = _np.where(
        outer >= dark_cross + 100.0,
        255,
        0,
    ).astype(_np.uint8)
    bright_regions = _cv2.morphologyEx(
        bright_regions,
        _cv2.MORPH_OPEN,
        _np.ones((5, 5), dtype=_np.uint8),
    )
    return float(_np.mean(bright_regions > 0)) >= 0.10


def _skeletonize_mask(binary: _np.ndarray) -> _np.ndarray:
    work = _np.where(binary > 0, 255, 0).astype(_np.uint8)
    skeleton = _np.zeros_like(work)
    element = _cv2.getStructuringElement(_cv2.MORPH_CROSS, (3, 3))
    for _ in range(40):
        eroded = _cv2.erode(work, element)
        opened = _cv2.dilate(eroded, element)
        skeleton = _cv2.bitwise_or(
            skeleton,
            _cv2.subtract(work, opened),
        )
        work = eroded
        if _cv2.countNonZero(work) == 0:
            break
    return skeleton


def _segment_intersection(
    first: _LineSegment,
    second: _LineSegment,
) -> tuple[float, float, float] | None:
    first_dx = first.x2 - first.x1
    first_dy = first.y2 - first.y1
    second_dx = second.x2 - second.x1
    second_dy = second.y2 - second.y1
    determinant = first_dx * second_dy - first_dy * second_dx
    if abs(determinant) <= 1e-6:
        return None
    delta_x = second.x1 - first.x1
    delta_y = second.y1 - first.y1
    first_t = (
        delta_x * second_dy - delta_y * second_dx
    ) / determinant
    second_t = (
        delta_x * first_dy - delta_y * first_dx
    ) / determinant
    extension = max(
        0.0,
        -first_t,
        first_t - 1.0,
        -second_t,
        second_t - 1.0,
    )
    if extension > 0.35:
        return None
    return (
        first.x1 + first_t * first_dx,
        first.y1 + first_t * first_dy,
        extension,
    )


def _find_skeleton_cross(
    gray: _np.ndarray,
    binary: _np.ndarray,
    search_diameter: float,
    previous: _Detection | None,
) -> list[tuple[float, float, float]]:
    height, width = binary.shape
    short_side = min(width, height)
    skeleton = _skeletonize_mask(binary)
    raw_lines = _cv2.HoughLinesP(
        skeleton,
        1.0,
        _math.pi / 180.0,
        threshold=max(24, int(round(0.12 * short_side))),
        minLineLength=max(42, int(round(0.22 * short_side))),
        maxLineGap=max(8, int(round(0.08 * short_side))),
    )
    if raw_lines is None:
        return []

    segments: list[_LineSegment] = []
    for raw_line in raw_lines.reshape(-1, 4):
        x1, y1, x2, y2 = (float(value) for value in raw_line)
        if (
            (x1 <= 2.0 and x2 <= 2.0)
            or (x1 >= width - 3.0 and x2 >= width - 3.0)
            or (y1 <= 2.0 and y2 <= 2.0)
            or (y1 >= height - 3.0 and y2 >= height - 3.0)
        ):
            continue
        length = _math.hypot(x2 - x1, y2 - y1)
        if length < 0.22 * short_side:
            continue
        segments.append(
            _LineSegment(
                x1=x1,
                y1=y1,
                x2=x2,
                y2=y2,
                length=length,
                angle=_math.atan2(y2 - y1, x2 - x1) % _math.pi,
            )
        )
    segments.sort(key=lambda segment: segment.length, reverse=True)
    segments = segments[:20]

    candidates: list[tuple[float, float, float]] = []
    for first_index, first in enumerate(segments):
        for second in segments[first_index + 1:]:
            angle = _line_angle_difference(first.angle, second.angle)
            angle_degrees = _math.degrees(angle)
            if not 55.0 <= angle_degrees <= 125.0:
                continue
            intersection = _segment_intersection(first, second)
            if intersection is None:
                continue
            center_u, center_v, extension = intersection
            if not (0.0 <= center_u < width and 0.0 <= center_v < height):
                continue
            if previous is not None:
                displacement = _math.hypot(
                    center_u - previous.center_u,
                    center_v - previous.center_v,
                )
                if displacement > max(24.0, 0.12 * search_diameter):
                    continue
                temporal = _math.exp(
                    -displacement / max(1.0, 0.12 * search_diameter)
                )
            else:
                temporal = 0.5
            angle_score = float(
                _np.clip(
                    1.0 - abs(angle_degrees - 90.0) / 35.0,
                    0.0,
                    1.0,
                )
            )
            length_score = float(
                _np.clip(
                    (first.length + second.length)
                    / max(1.0, 1.35 * short_side),
                    0.0,
                    1.0,
                )
            )
            extension_score = float(
                _np.clip(1.0 - extension / 0.35, 0.0, 1.0)
            )
            if previous is None:
                score = (
                    0.40 * angle_score
                    + 0.36 * length_score
                    + 0.18 * extension_score
                    + 0.06 * temporal
                )
            else:
                score = (
                    0.32 * angle_score
                    + 0.28 * length_score
                    + 0.14 * extension_score
                    + 0.26 * temporal
                )
            candidates.append(
                (float(center_u), float(center_v), float(score))
            )

    candidates.sort(key=lambda candidate: candidate[2], reverse=True)
    distinct: list[tuple[float, float, float]] = []
    checked_centers: list[tuple[float, float]] = []
    minimum_separation = max(7.0, 0.03 * short_side)
    for candidate in candidates:
        if any(
            _math.hypot(
                candidate[0] - center[0],
                candidate[1] - center[1],
            )
            < minimum_separation
            for center in checked_centers
        ):
            continue
        checked_centers.append((candidate[0], candidate[1]))
        if len(checked_centers) > 12:
            break
        if not _cross_core_is_dark(
            gray,
            (candidate[0], candidate[1]),
            search_diameter,
            previous is None,
        ):
            continue
        distinct.append(candidate)
        if len(distinct) >= 8:
            break
    return distinct


def _find_near_cross(
    gray: _np.ndarray,
    masks: list[_np.ndarray],
    previous: _Detection | None,
    _resources: _VisionResources,
) -> _Detection | None:
    """Find a close marker from skeleton centerlines in each mask."""
    if not masks:
        return None

    height, width = gray.shape
    frame_diagonal = _math.hypot(width, height)
    previous_search_diameter = 0.0
    if previous is not None:
        previous_search_diameter = previous.diameter_px
        if not previous.near_cross:
            previous_search_diameter *= 1.20
    search_diameter = min(
        1.35 * frame_diagonal,
        max(
            0.90 * frame_diagonal,
            previous_search_diameter,
        ),
    )
    hits: list[tuple[float, float, float, int]] = []
    for mask_index, mask in enumerate(masks):
        mask_hits = _find_skeleton_cross(
            gray,
            mask,
            search_diameter,
            previous,
        )
        for hit in mask_hits:
            hits.append((hit[0], hit[1], hit[2], mask_index))
    if not hits:
        return None

    cluster_radius = max(8.0, 0.035 * search_diameter)
    best_cluster: list[tuple[float, float, float, int]] | None = None
    best_rank = -_math.inf
    for anchor in hits:
        nearby = [
            hit
            for hit in hits
            if _math.hypot(hit[0] - anchor[0], hit[1] - anchor[1])
            <= cluster_radius
        ]
        cluster_by_mask: dict[
            int,
            tuple[float, float, float, int],
        ] = {}
        for hit in nearby:
            current = cluster_by_mask.get(hit[3])
            if current is None or hit[2] > current[2]:
                cluster_by_mask[hit[3]] = hit
        cluster = list(cluster_by_mask.values())
        if previous is None and len(cluster) < 2:
            continue
        rank = float(_np.mean([hit[2] for hit in cluster]))
        rank += 0.04 * len(cluster)
        if rank > best_rank:
            best_rank = rank
            best_cluster = cluster
    if best_cluster is None:
        return None

    weights = _np.asarray(
        [max(0.05, hit[2]) ** 2 for hit in best_cluster],
        dtype=_np.float64,
    )
    centers = _np.asarray(
        [(hit[0], hit[1]) for hit in best_cluster],
        dtype=_np.float64,
    )
    weighted_center = _np.average(centers, axis=0, weights=weights)
    center_u = float(weighted_center[0])
    center_v = float(weighted_center[1])
    line_score = float(
        _np.average(
            [hit[2] for hit in best_cluster],
            weights=weights,
        )
    )
    temporal = 0.5
    if previous is not None:
        displacement = _math.hypot(
            center_u - previous.center_u,
            center_v - previous.center_v,
        )
        temporal = _math.exp(
            -displacement / max(1.0, 0.12 * search_diameter)
        )
    score = float(
        _np.clip(
            0.46
            + 0.34 * line_score
            + 0.04 * min(1.0, len(best_cluster) / 3.0)
            + 0.10 * temporal,
            0.0,
            0.88,
        )
    )
    mask_index = max(best_cluster, key=lambda hit: hit[2])[3]
    if previous is None:
        axes = (search_diameter, search_diameter)
        angle = 0.0
        diameter_px = search_diameter
    else:
        (_, _), axes, angle = previous.ellipse
        diameter_px = previous.diameter_px
    return _Detection(
        center_u=center_u,
        center_v=center_v,
        diameter_px=float(diameter_px),
        score=score,
        ellipse=((center_u, center_v), axes, angle),
        mask_index=mask_index,
        near_cross=True,
    )


def _detect_marker(
    gray: _np.ndarray,
    masks: list[_np.ndarray],
    search_roi: tuple[int, int, int, int] | None,
    previous_center: tuple[float, float] | None,
    previous_diameter: float | None,
    _resources: _VisionResources | None = None,
) -> _Detection | None:
    resources = _resources or _VisionResources()
    detection = _find_marker_candidate(
        gray,
        masks,
        search_roi,
        previous_center,
        previous_diameter,
        resources,
    )
    threshold = (
        _TRACK_SCORE_THRESHOLD
        if search_roi is not None
        else _SEARCH_SCORE_THRESHOLD
    )
    if detection is None or detection.score < threshold:
        return None
    return detection


def _marker_feature_points(
    gray: _np.ndarray,
    detection: _Detection,
) -> _np.ndarray | None:
    (center_u, center_v), (major, minor), angle = detection.ellipse
    height, width = gray.shape
    roi = _square_roi(
        (center_u, center_v),
        1.15 * major,
        width,
        height,
    )
    x0, y0, x1, y1 = roi
    if x1 - x0 < 8 or y1 - y0 < 8:
        return None
    gray_roi = gray[y0:y1, x0:x1]
    ellipse_mask = _np.zeros_like(gray_roi)
    local_center = (
        int(round(center_u - x0)),
        int(round(center_v - y0)),
    )
    _cv2.ellipse(
        ellipse_mask,
        local_center,
        (
            max(1, int(round(major / 2.0))),
            max(1, int(round(minor / 2.0))),
        ),
        angle,
        0.0,
        360.0,
        255,
        -1,
    )
    min_distance = max(3, int(0.025 * detection.diameter_px))
    points = _cv2.goodFeaturesToTrack(
        gray_roi,
        maxCorners=40,
        qualityLevel=0.01,
        minDistance=min_distance,
        mask=ellipse_mask,
        blockSize=5,
    )
    if points is None:
        return None
    points = points.astype(_np.float32)
    points[:, 0, 0] += float(x0)
    points[:, 0, 1] += float(y0)
    return points


def _track_with_optical_flow(
    previous_gray: _np.ndarray,
    current_gray: _np.ndarray,
    previous_points: _np.ndarray,
    previous_detection: _Detection,
) -> _OpticalResult | None:
    if len(previous_points) < 10:
        return None
    criteria = (
        _cv2.TERM_CRITERIA_EPS | _cv2.TERM_CRITERIA_COUNT,
        20,
        0.01,
    )
    current_points, forward_status, _ = _cv2.calcOpticalFlowPyrLK(
        previous_gray,
        current_gray,
        previous_points,
        None,
        winSize=(21, 21),
        maxLevel=3,
        criteria=criteria,
    )
    if current_points is None or forward_status is None:
        return None
    backward_points, backward_status, _ = _cv2.calcOpticalFlowPyrLK(
        current_gray,
        previous_gray,
        current_points,
        None,
        winSize=(21, 21),
        maxLevel=3,
        criteria=criteria,
    )
    if backward_points is None or backward_status is None:
        return None

    forward_ok = forward_status.reshape(-1) > 0
    backward_ok = backward_status.reshape(-1) > 0
    finite = (
        _np.all(_np.isfinite(current_points.reshape(-1, 2)), axis=1)
        & _np.all(_np.isfinite(backward_points.reshape(-1, 2)), axis=1)
    )
    round_trip = _np.linalg.norm(
        previous_points.reshape(-1, 2)
        - backward_points.reshape(-1, 2),
        axis=1,
    )
    valid = forward_ok & backward_ok & finite & (round_trip < 1.5)
    if int(_np.count_nonzero(valid)) < 10:
        return None
    source = previous_points.reshape(-1, 2)[valid].astype(_np.float32)
    target = current_points.reshape(-1, 2)[valid].astype(_np.float32)
    matrix, inlier_mask = _cv2.estimateAffinePartial2D(
        source,
        target,
        method=_cv2.RANSAC,
        ransacReprojThreshold=2.0,
    )
    if matrix is None or inlier_mask is None:
        return None
    if matrix.shape != (2, 3) or not _np.all(_np.isfinite(matrix)):
        return None
    inliers = inlier_mask.reshape(-1) > 0
    inlier_count = int(_np.count_nonzero(inliers))
    if inlier_count < 8 or inlier_count / len(source) < 0.55:
        return None

    linear = matrix[:, :2].astype(_np.float64)
    determinant = float(_np.linalg.det(linear))
    if not _math.isfinite(determinant) or determinant <= 0.0:
        return None
    scale = _math.sqrt(
        float(linear[0, 0] ** 2 + linear[1, 0] ** 2)
    )
    if not 0.85 <= scale <= 1.18:
        return None
    rotation = _math.atan2(float(linear[1, 0]), float(linear[0, 0]))
    if abs(_math.degrees(rotation)) > 20.0:
        return None

    old_center = _np.asarray(
        [
            previous_detection.center_u,
            previous_detection.center_v,
            1.0,
        ],
        dtype=_np.float64,
    )
    new_center = matrix.astype(_np.float64) @ old_center
    if not _np.all(_np.isfinite(new_center)):
        return None
    displacement = float(
        _np.linalg.norm(
            new_center
            - _np.asarray(
                (
                    previous_detection.center_u,
                    previous_detection.center_v,
                )
            )
        )
    )
    if displacement > 0.35 * previous_detection.diameter_px:
        return None

    (ellipse_u, ellipse_v), (major, minor), ellipse_angle = (
        previous_detection.ellipse
    )
    ellipse_center = matrix.astype(_np.float64) @ _np.asarray(
        (ellipse_u, ellipse_v, 1.0),
        dtype=_np.float64,
    )
    ellipse: _Ellipse = (
        (float(ellipse_center[0]), float(ellipse_center[1])),
        (float(major * scale), float(minor * scale)),
        float((ellipse_angle + _math.degrees(rotation)) % 180.0),
    )
    updated_points = target[inliers].reshape(-1, 1, 2).astype(_np.float32)
    detection = _Detection(
        center_u=float(new_center[0]),
        center_v=float(new_center[1]),
        diameter_px=float(previous_detection.diameter_px * scale),
        score=float(max(0.50, previous_detection.score * 0.985)),
        ellipse=ellipse,
        mask_index=previous_detection.mask_index,
        near_cross=previous_detection.near_cross,
    )
    return _OpticalResult(detection=detection, points=updated_points)


def _is_implausible_jump(
    previous: _Detection,
    current: _Detection,
) -> bool:
    displacement = _math.hypot(
        current.center_u - previous.center_u,
        current.center_v - previous.center_v,
    )
    limit = max(45.0, 0.45 * previous.diameter_px)
    if displacement <= limit or current.score >= 0.88:
        return False
    return current.score <= previous.score + 0.12


def _detections_are_consistent(
    previous: _Detection,
    current: _Detection,
) -> bool:
    if _is_implausible_jump(previous, current):
        return False
    if previous.diameter_px <= 0.0:
        return False
    size_ratio = current.diameter_px / previous.diameter_px
    return 0.65 <= size_ratio <= 1.50


class _MarkerTracker:
    def __init__(self) -> None:
        self._resources = _VisionResources()
        self._state = "SEARCH"
        self._search_history: _collections.deque[_Detection | None] = (
            _collections.deque(maxlen=3)
        )
        self._detection: _Detection | None = None
        self._previous_gray: _np.ndarray | None = None
        self._previous_points: _np.ndarray | None = None
        self._last_geometry_ns = 0
        self._track_frame_count = 0
        self._miss_count = 0
        self._low_score_count = 0

    def process(
        self,
        gray: _np.ndarray,
        timestamp_ns: int,
    ) -> _Detection | None:
        if self._state == "SEARCH":
            return self._process_search(gray, timestamp_ns)
        return self._process_track(gray, timestamp_ns)

    def _run_geometry(
        self,
        gray: _np.ndarray,
        search_roi: _Roi | None,
        previous: _Detection | None,
    ) -> tuple[_Detection | None, float | None]:
        masks = _build_binary_variants(gray, self._resources)
        raw = _find_marker_candidate(
            gray,
            masks,
            search_roi,
            (
                (previous.center_u, previous.center_v)
                if previous is not None
                else None
            ),
            previous.diameter_px if previous is not None else None,
            self._resources,
        )
        threshold = (
            _TRACK_SCORE_THRESHOLD
            if search_roi is not None
            else _SEARCH_SCORE_THRESHOLD
        )
        if raw is not None and raw.score >= threshold:
            return raw, raw.score

        near_cross = _find_near_cross(
            gray,
            masks,
            previous,
            self._resources,
        )
        near_cross_threshold = (
            _TRACK_SCORE_THRESHOLD
            if previous is not None
            else _SEARCH_SCORE_THRESHOLD
        )
        if (
            near_cross is not None
            and near_cross.score >= near_cross_threshold
        ):
            return near_cross, near_cross.score
        if raw is None:
            return None, None
        if raw.score < threshold:
            return None, raw.score
        return raw, raw.score

    def _process_search(
        self,
        gray: _np.ndarray,
        timestamp_ns: int,
    ) -> _Detection | None:
        previous = next(
            (
                detection
                for detection in reversed(self._search_history)
                if detection is not None
            ),
            None,
        )
        detection, _ = self._run_geometry(gray, None, previous)
        if detection is not None and not self._measurement_in_bounds(detection):
            detection = None
        if (
            detection is not None
            and previous is not None
            and not _detections_are_consistent(previous, detection)
        ):
            detection = None
        self._search_history.append(detection)
        if detection is None:
            return None

        recent = [
            item for item in self._search_history if item is not None
        ]
        can_lock = (
            len(self._search_history) == 3
            and len(recent) >= 2
            and detection.score >= _SEARCH_SCORE_THRESHOLD
        )
        if can_lock:
            prior = recent[-2]
            can_lock = _detections_are_consistent(prior, detection)
        if can_lock:
            self._state = "TRACK"
            self._detection = detection
            self._previous_gray = gray
            self._previous_points = _marker_feature_points(gray, detection)
            self._last_geometry_ns = timestamp_ns
            self._track_frame_count = 0
            self._miss_count = 0
            self._low_score_count = 0
        return detection

    def _tracking_roi(self, detection: _Detection) -> _Roi:
        side = max(96.0, 1.6 * detection.diameter_px)
        side = min(
            side,
            float(max(_FRAME_WIDTH, _FRAME_HEIGHT)),
        )
        return _square_roi(
            (detection.center_u, detection.center_v),
            side,
            _FRAME_WIDTH,
            _FRAME_HEIGHT,
        )

    def _process_track(
        self,
        gray: _np.ndarray,
        timestamp_ns: int,
    ) -> _Detection | None:
        previous = self._detection
        if previous is None:
            self._reset_to_search()
            return None
        self._track_frame_count += 1

        optical: _OpticalResult | None = None
        if self._previous_gray is not None and self._previous_points is not None:
            optical = _track_with_optical_flow(
                self._previous_gray,
                gray,
                self._previous_points,
                previous,
            )

        geometry_age = timestamp_ns - self._last_geometry_ns
        full_due = self._track_frame_count % 15 == 0
        roi_due = self._track_frame_count % 3 == 0
        time_due = geometry_age >= _GEOMETRY_EARLY_REFRESH_NS
        geometry_due = full_due or roi_due or time_due or optical is None

        geometry: _Detection | None = None
        raw_score: float | None = None
        if geometry_due:
            search_roi = None if full_due else self._tracking_roi(previous)
            geometry, raw_score = self._run_geometry(
                gray,
                search_roi,
                previous,
            )
            if (
                geometry is None
                and optical is None
                and search_roi is not None
            ):
                geometry, raw_score = self._run_geometry(
                    gray,
                    None,
                    previous,
                )
            if (
                geometry is not None
                and _is_implausible_jump(previous, geometry)
            ):
                geometry = None

        if geometry_due:
            if raw_score is not None and raw_score < 0.50:
                self._low_score_count += 1
            else:
                self._low_score_count = 0
        if self._low_score_count >= 2:
            self._reset_to_search()
            return None

        if geometry is not None:
            if not self._measurement_in_bounds(geometry):
                self._reset_to_search()
                return None
            self._detection = geometry
            self._previous_gray = gray
            self._previous_points = _marker_feature_points(gray, geometry)
            self._last_geometry_ns = timestamp_ns
            self._miss_count = 0
            return geometry

        if geometry_age > _GEOMETRY_MAX_AGE_NS:
            self._reset_to_search()
            return None

        if optical is not None:
            detection = optical.detection
            if not self._measurement_in_bounds(detection):
                self._reset_to_search()
                return None
            self._detection = detection
            self._previous_gray = gray
            self._previous_points = optical.points
            self._miss_count = 0
            return detection

        self._miss_count += 1
        self._previous_gray = None
        self._previous_points = None
        if self._miss_count >= 3:
            self._reset_to_search()
        return None

    @staticmethod
    def _measurement_in_bounds(detection: _Detection) -> bool:
        return (
            0.0 <= detection.center_u < _FRAME_WIDTH
            and 0.0 <= detection.center_v < _FRAME_HEIGHT
            and detection.diameter_px >= 24.0
            and _np.all(
                _np.isfinite(
                    (
                        detection.center_u,
                        detection.center_v,
                        detection.diameter_px,
                    )
                )
            )
        )

    def _reset_to_search(self) -> None:
        self._state = "SEARCH"
        self._search_history.clear()
        self._detection = None
        self._previous_gray = None
        self._previous_points = None
        self._last_geometry_ns = 0
        self._track_frame_count = 0
        self._miss_count = 0
        self._low_score_count = 0


def _prepare_gray_frame(frame: _np.ndarray) -> _np.ndarray | None:
    if not isinstance(frame, _np.ndarray) or frame.size == 0:
        return None
    if frame.shape[:2] != (_FRAME_HEIGHT, _FRAME_WIDTH):
        frame = _cv2.resize(
            frame,
            (_FRAME_WIDTH, _FRAME_HEIGHT),
            interpolation=_cv2.INTER_AREA,
        )
    if frame.ndim == 2:
        gray = frame
    elif frame.ndim == 3 and frame.shape[2] == 3:
        gray = _cv2.cvtColor(frame, _cv2.COLOR_BGR2GRAY)
    elif frame.ndim == 3 and frame.shape[2] == 4:
        gray = _cv2.cvtColor(frame, _cv2.COLOR_BGRA2GRAY)
    else:
        return None
    if gray.dtype != _np.uint8:
        gray = _np.clip(gray, 0, 255).astype(_np.uint8)
    return gray


def _draw_debug(
    frame: _np.ndarray,
    tracker: _MarkerTracker,
    detection: _Detection | None,
    output: tuple[float | None, float | None],
) -> bool:
    if frame.shape[:2] != (_FRAME_HEIGHT, _FRAME_WIDTH):
        display = _cv2.resize(
            frame,
            (_FRAME_WIDTH, _FRAME_HEIGHT),
            interpolation=_cv2.INTER_AREA,
        )
    else:
        display = frame.copy()
    if display.ndim == 2:
        display = _cv2.cvtColor(display, _cv2.COLOR_GRAY2BGR)
    _cv2.drawMarker(
        display,
        (int(round(_FRAME_CENTER_U)), int(round(_FRAME_CENTER_V))),
        (255, 255, 255),
        _cv2.MARKER_CROSS,
        16,
        1,
        _cv2.LINE_AA,
    )
    if detection is not None:
        target_point = (
            int(round(detection.center_u)),
            int(round(detection.center_v)),
        )
        if not detection.near_cross:
            _cv2.ellipse(
                display,
                detection.ellipse,
                (0, 255, 0),
                2,
                _cv2.LINE_AA,
            )
        _cv2.drawMarker(
            display,
            target_point,
            (0, 0, 255),
            _cv2.MARKER_CROSS,
            18,
            2,
            _cv2.LINE_AA,
        )
        _cv2.line(
            display,
            (int(round(_FRAME_CENTER_U)), int(round(_FRAME_CENTER_V))),
            target_point,
            (0, 200, 255),
            2,
            _cv2.LINE_AA,
        )
        mode = " near-cross" if detection.near_cross else ""
        status = f"{tracker._state}{mode} score={detection.score:.2f}"
        offset_text = f"x_px={output[0]:.2f}  y_px={output[1]:.2f}"
    else:
        status = f"{tracker._state} no reliable marker"
        offset_text = "x_px=--  y_px=--"
    _cv2.putText(
        display,
        status,
        (8, 22),
        _cv2.FONT_HERSHEY_SIMPLEX,
        0.52,
        (0, 255, 255),
        1,
        _cv2.LINE_AA,
    )
    _cv2.putText(
        display,
        offset_text,
        (8, 45),
        _cv2.FONT_HERSHEY_SIMPLEX,
        0.52,
        (0, 255, 255),
        1,
        _cv2.LINE_AA,
    )
    fps_text = (
        "FPS: measuring..."
        if _DEBUG_FPS is None
        else f"FPS: {_DEBUG_FPS:.1f}"
    )
    _cv2.putText(
        display,
        fps_text,
        (8, 68),
        _cv2.FONT_HERSHEY_SIMPLEX,
        0.52,
        (0, 255, 0),
        1,
        _cv2.LINE_AA,
    )
    _cv2.putText(
        display,
        "Q: quit",
        (8, _FRAME_HEIGHT - 10),
        _cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (210, 210, 210),
        1,
        _cv2.LINE_AA,
    )
    _cv2.imshow(_DEBUG_WINDOW, display)
    key = _cv2.waitKey(1) & 0xFF
    return key not in (ord("q"), ord("Q"))


def track_landing_marker(
    camera_index: int,
) -> _abc.Iterator[tuple[float | None, float | None]]:
    """Track the landing marker using one downward-facing camera.

    Args:
        camera_index: OpenCV camera index.

    Yields:
        A tuple ``(x_px, y_px)`` for each processed frame. ``x_px`` is
        positive toward the top of the image, and ``y_px`` is positive
        toward the left of the image. If the marker is not reliably
        detected in the current frame, yields ``(None, None)``.

    The return value is a generator. The camera is opened once, processes
    256 x 256 frames, and is released when the generator is closed or exits.
    """
    cap = _open_camera(camera_index)
    capture: _LatestFrameCapture | None = None
    try:
        _warm_up_camera(cap)
        capture = _LatestFrameCapture(cap)
        capture.start()
        tracker = _MarkerTracker()
        sequence = 0

        while True:
            sequence, frame, timestamp_ns = capture.read_after(sequence)
            gray = _prepare_gray_frame(frame)
            if gray is None:
                yield None, None
                continue
            detection = tracker.process(gray, timestamp_ns)
            if detection is None:
                output: tuple[float | None, float | None] = (None, None)
            else:
                output = (
                    float(_FRAME_CENTER_V - detection.center_v),
                    float(_FRAME_CENTER_U - detection.center_u),
                )
            if _DEBUG and not _draw_debug(
                frame,
                tracker,
                detection,
                output,
            ):
                return
            yield output
    finally:
        if capture is not None:
            capture.stop()
        cap.release()
        if capture is not None:
            capture.join()
        if _DEBUG:
            try:
                _cv2.destroyWindow(_DEBUG_WINDOW)
            except _cv2.error:
                pass
