"""Detect the dark rectangular takeoff marker in a BGR image.

This module only processes an image supplied by its caller.  It does not open
a camera, connect to the flight controller, or issue any control command.
"""

from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np


Point = Tuple[float, float]


@dataclass(frozen=True)
class TakeoffRectangle:
    """Detected takeoff marker geometry in image pixel coordinates.

    ``corners`` are ordered clockwise as top-left, top-right, bottom-right,
    bottom-left. ``center`` is the intersection of the two diagonals.
    """

    corners: Tuple[Point, Point, Point, Point]
    center: Point
    area: float


def detect_takeoff_rectangle(
    frame: np.ndarray,
    min_area_ratio: float = 0.02,
    max_area_ratio: float = 0.75,
) -> Optional[TakeoffRectangle]:
    """Return the most likely dark rectangular takeoff marker, or ``None``.

    Args:
        frame: BGR image as returned by OpenCV.
        min_area_ratio: Smallest accepted marker area divided by image area.
        max_area_ratio: Largest accepted marker area divided by image area.
    """
    if frame is None or frame.ndim != 3 or frame.shape[2] != 3:
        raise ValueError("frame must be a non-empty BGR image with three channels")
    if frame.shape[0] == 0 or frame.shape[1] == 0:
        raise ValueError("frame must not be empty")
    if not 0.0 < min_area_ratio < max_area_ratio <= 1.0:
        raise ValueError("area ratios must satisfy 0 < min < max <= 1")

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    _, mask = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_RECT, (9, 9)),
        iterations=2,
    )

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    image_area = float(frame.shape[0] * frame.shape[1])
    best = None
    best_score = -1.0
    for contour in contours:
        area = cv2.contourArea(contour)
        if not min_area_ratio * image_area <= area <= max_area_ratio * image_area:
            continue

        perimeter = cv2.arcLength(contour, True)
        if perimeter <= 0.0:
            continue
        # The marker material can create a small bright notch along an edge.
        # A 3.5% approximation retains the four physical outer corners while
        # rejecting that internal texture detail.
        polygon = cv2.approxPolyDP(contour, 0.035 * perimeter, True)
        if len(polygon) != 4 or not cv2.isContourConvex(polygon):
            continue

        corners = _order_corners(polygon.reshape(4, 2).astype(np.float32))
        rectangularity = _rectangularity(corners)
        # Perspective projection makes the marker's image angles non-right.
        # Keep a permissive limit while still rejecting strongly skewed blobs.
        if rectangularity < 0.65:
            continue

        fill_ratio = area / max(cv2.contourArea(corners), 1.0)
        if fill_ratio < 0.75:
            continue
        score = rectangularity * min(fill_ratio, 1.0) * area
        if score > best_score:
            best = corners
            best_score = score

    if best is None:
        return None

    center = _line_intersection(best[0], best[2], best[1], best[3])
    corners_tuple = tuple((float(point[0]), float(point[1])) for point in best)
    return TakeoffRectangle(corners_tuple, center, float(cv2.contourArea(best)))


def draw_takeoff_rectangle(
    frame: np.ndarray,
    detection: TakeoffRectangle,
    color: Tuple[int, int, int] = (0, 255, 0),
) -> np.ndarray:
    """Return a copy of ``frame`` with the marker outline and center drawn."""
    output = frame.copy()
    corners = np.rint(np.asarray(detection.corners)).astype(np.int32)
    center = tuple(int(round(value)) for value in detection.center)
    cv2.polylines(output, [corners], True, color, 3, cv2.LINE_AA)
    cv2.line(output, tuple(corners[0]), tuple(corners[2]), color, 1, cv2.LINE_AA)
    cv2.line(output, tuple(corners[1]), tuple(corners[3]), color, 1, cv2.LINE_AA)
    cv2.drawMarker(output, center, (0, 0, 255), cv2.MARKER_CROSS, 28, 3, cv2.LINE_AA)
    return output


def _order_corners(points: np.ndarray) -> np.ndarray:
    """Order a convex quadrilateral clockwise from its top-left image corner."""
    center = points.mean(axis=0)
    angles = np.arctan2(points[:, 1] - center[1], points[:, 0] - center[0])
    ordered = points[np.argsort(angles)]
    top_left_index = int(np.argmin(ordered.sum(axis=1)))
    return np.roll(ordered, -top_left_index, axis=0)


def _rectangularity(corners: np.ndarray) -> float:
    """Return 1 for right angles; lower values indicate a non-rectangle."""
    scores = []
    for index in range(4):
        previous = corners[(index - 1) % 4] - corners[index]
        following = corners[(index + 1) % 4] - corners[index]
        denominator = float(np.linalg.norm(previous) * np.linalg.norm(following))
        if denominator == 0.0:
            return 0.0
        cosine = abs(float(np.dot(previous, following)) / denominator)
        scores.append(1.0 - min(cosine, 1.0))
    return min(scores)


def _line_intersection(first_start: np.ndarray, first_end: np.ndarray,
                       second_start: np.ndarray, second_end: np.ndarray) -> Point:
    """Return the intersection of two non-parallel infinite lines."""
    first = first_end - first_start
    second = second_end - second_start
    denominator = float(first[0] * second[1] - first[1] * second[0])
    if abs(denominator) < 1e-6:
        midpoint = (first_start + first_end + second_start + second_end) / 4.0
        return float(midpoint[0]), float(midpoint[1])
    delta = second_start - first_start
    parameter = float(delta[0] * second[1] - delta[1] * second[0]) / denominator
    intersection = first_start + parameter * first
    return float(intersection[0]), float(intersection[1])
