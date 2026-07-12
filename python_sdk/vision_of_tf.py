"""Callable vision helpers for QR code and pure-black shape offsets.

The public functions open a USB camera only when called, read a few frames,
then release the camera before returning.

Coordinate conventions:
    - QR code: returns (z_px, y_px).
      Image up is +Z, image left is +Y.
    - Black circle/rectangle: returns (x_px, y_px).
      Image up/forward is +X, image left is +Y.

All offsets are computed from the image center to the detected target center:
    first_axis = image_center_y - target_center_y
    y_axis     = image_center_x - target_center_x

Return value is None when no matching target is found in the sampled frames.
"""

import sys
from dataclasses import dataclass
from typing import Callable, List, Optional, Tuple

import cv2
import numpy as np


Offset = Tuple[float, float]


@dataclass
class QRDetection:
    corners: np.ndarray
    text: str
    source: str

    @property
    def center(self) -> Tuple[float, float]:
        center = self.corners.mean(axis=0)
        return float(center[0]), float(center[1])


@dataclass
class ShapeDetection:
    kind: str
    center: Tuple[float, float]
    contour: np.ndarray
    box: np.ndarray
    score: float
    area: float
    radius: float = 0.0


def detect_qrcode_offset(camera_index: int) -> Optional[Offset]:
    """Return (z_px, y_px) for the QR code nearest the image center."""
    return _detect_camera_offset(camera_index, _detect_qrcodes, _center_to_first_y_offset)


def detect_black_circle_offset(camera_index: int) -> Optional[Offset]:
    """Return (x_px, y_px) for the black circle nearest the image center."""
    detector = lambda frame: _detect_black_shapes(frame, "circle")
    return _detect_camera_offset(camera_index, detector, _center_to_first_y_offset)


def detect_black_rectangle_offset(camera_index: int) -> Optional[Offset]:
    """Return (x_px, y_px) for the black rectangle nearest the image center."""
    detector = lambda frame: _detect_black_shapes(frame, "rectangle")
    return _detect_camera_offset(camera_index, detector, _center_to_first_y_offset)


def _detect_camera_offset(
    camera_index: int,
    detector: Callable[[np.ndarray], List],
    offset_func: Callable[[Tuple[float, float], Tuple[int, int]], Offset],
    width: int = 1280,
    height: int = 720,
    warmup_frames: int = 3,
    max_frames: int = 10,
) -> Optional[Offset]:
    capture = _open_usb_camera(camera_index, width, height)
    try:
        valid_frame_count = 0
        total_reads = max(1, warmup_frames + max_frames)
        for read_index in range(total_reads):
            ok, frame = capture.read()
            if not ok or frame is None:
                continue
            if read_index < warmup_frames:
                continue

            valid_frame_count += 1
            detections = detector(frame)
            if not detections:
                if valid_frame_count >= max_frames:
                    break
                continue

            image_center = (frame.shape[1] // 2, frame.shape[0] // 2)
            selected = _select_nearest_to_image_center(detections, image_center)
            return offset_func(selected.center, image_center)
        return None
    finally:
        capture.release()


def _open_usb_camera(camera_index: int, width: int, height: int) -> cv2.VideoCapture:
    if sys.platform.startswith("linux"):
        backends = (cv2.CAP_V4L2, cv2.CAP_ANY)
    elif sys.platform.startswith("win"):
        backends = (cv2.CAP_DSHOW, cv2.CAP_ANY)
    else:
        backends = (cv2.CAP_ANY,)

    for backend in backends:
        capture = cv2.VideoCapture(camera_index, backend)
        if capture.isOpened():
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            return capture
        capture.release()
    raise RuntimeError("Unable to open USB camera index {}".format(camera_index))


def _center_to_first_y_offset(
    target_center: Tuple[float, float], image_center: Tuple[int, int]
) -> Offset:
    target_x, target_y = target_center
    center_x, center_y = image_center
    first_axis_px = center_y - target_y
    y_axis_px = center_x - target_x
    return float(first_axis_px), float(y_axis_px)


def _select_nearest_to_image_center(detections: List, image_center: Tuple[int, int]):
    center_x, center_y = image_center
    return min(
        detections,
        key=lambda item: (item.center[0] - center_x) ** 2 + (item.center[1] - center_y) ** 2,
    )


def _as_qr_corners(points) -> Optional[np.ndarray]:
    if points is None:
        return None
    corners = np.asarray(points, dtype=np.float32).reshape(-1, 2)
    if len(corners) < 4:
        return None
    return corners[:4]


def _detect_qrcodes(frame: np.ndarray) -> List[QRDetection]:
    detector = cv2.QRCodeDetector()
    detections = _detect_qrcodes_with_opencv(detector, frame)
    detections.extend(_detect_qrcodes_with_pyzbar(frame))
    return _merge_qr_detections(detections)


def _detect_qrcodes_with_opencv(
    detector: cv2.QRCodeDetector, frame: np.ndarray
) -> List[QRDetection]:
    detections = []
    try:
        ok, texts, points, _ = detector.detectAndDecodeMulti(frame)
    except (cv2.error, AttributeError):
        ok, texts, points = False, (), None

    if ok and points is not None:
        for index, point_set in enumerate(points):
            corners = _as_qr_corners(point_set)
            if corners is not None:
                text = texts[index] if index < len(texts) else ""
                detections.append(QRDetection(corners, text, "opencv"))
        return detections

    try:
        found, points = detector.detectMulti(frame)
    except (cv2.error, AttributeError):
        found, points = False, None
    if found and points is not None:
        for point_set in points:
            corners = _as_qr_corners(point_set)
            if corners is not None:
                detections.append(QRDetection(corners, "", "opencv"))
        return detections

    text, points, _ = detector.detectAndDecode(frame)
    corners = _as_qr_corners(points)
    if corners is not None:
        detections.append(QRDetection(corners, text, "opencv"))
    return detections


def _detect_qrcodes_with_pyzbar(frame: np.ndarray) -> List[QRDetection]:
    try:
        from pyzbar.pyzbar import ZBarSymbol, decode
    except ImportError:
        return []

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    enhanced_gray = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(gray)
    detections = []
    for image in (gray, enhanced_gray):
        for result in decode(image, symbols=[ZBarSymbol.QRCODE]):
            corners = _as_qr_corners(result.polygon)
            if corners is None:
                x, y, width, height = result.rect
                corners = np.array(
                    [[x, y], [x + width, y], [x + width, y + height], [x, y + height]],
                    dtype=np.float32,
                )
            text = result.data.decode("utf-8", errors="replace")
            detections.append(QRDetection(corners, text, "pyzbar"))
    return detections


def _merge_qr_detections(detections: List[QRDetection]) -> List[QRDetection]:
    merged = []
    for detection in detections:
        match_index = next(
            (
                index
                for index, item in enumerate(merged)
                if _qr_center_distance(item, detection) <= 35.0
            ),
            None,
        )
        if match_index is None:
            merged.append(detection)
        elif detection.text and not merged[match_index].text:
            current = merged[match_index]
            merged[match_index] = QRDetection(current.corners, detection.text, "opencv+pyzbar")
    return merged


def _qr_center_distance(first: QRDetection, second: QRDetection) -> float:
    first_x, first_y = first.center
    second_x, second_y = second.center
    return float(np.hypot(first_x - second_x, first_y - second_y))


def _detect_black_shapes(frame: np.ndarray, target_kind: str) -> List[ShapeDetection]:
    threshold = 90
    min_area = 1200.0
    max_area_ratio = 0.45

    mask = _make_black_mask(frame, threshold)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    image_area = frame.shape[0] * frame.shape[1]
    max_area = image_area * max_area_ratio
    detections = []

    for contour in contours:
        area = cv2.contourArea(contour)
        if area < min_area or area > max_area:
            continue

        perimeter = cv2.arcLength(contour, True)
        if perimeter <= 0:
            continue

        hull_area = cv2.contourArea(cv2.convexHull(contour))
        if hull_area <= 0:
            continue
        solidity = area / hull_area
        if solidity < 0.85:
            continue

        if target_kind == "circle":
            candidate = _circle_candidate(contour, area, perimeter, min_area)
        else:
            candidate = _rectangle_candidate(contour, area, perimeter)

        if candidate is not None:
            detections.append(candidate)

    return detections


def _make_black_mask(frame: np.ndarray, threshold: int) -> np.ndarray:
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    mask = cv2.inRange(gray, 0, threshold)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    return mask


def _circle_candidate(
    contour: np.ndarray,
    area: float,
    perimeter: float,
    min_area: float,
) -> Optional[ShapeDetection]:
    (center_x, center_y), radius = cv2.minEnclosingCircle(contour)
    if radius <= 1.0:
        return None

    circle_area = np.pi * radius * radius
    circle_fill = area / circle_area if circle_area > 0 else 0.0
    circularity = 4.0 * np.pi * area / (perimeter * perimeter)
    _, _, width, height = cv2.boundingRect(contour)
    aspect_ratio = max(width, height) / float(max(1, min(width, height)))

    if radius * radius * np.pi < min_area:
        return None
    if aspect_ratio > 1.25:
        return None
    if circularity < 0.72:
        return None
    if not 0.68 <= circle_fill <= 1.12:
        return None

    box = np.array(
        [
            [center_x - radius, center_y - radius],
            [center_x + radius, center_y - radius],
            [center_x + radius, center_y + radius],
            [center_x - radius, center_y + radius],
        ],
        dtype=np.float32,
    )
    score = 0.5 * circularity + 0.5 * min(circle_fill, 1.0)
    return ShapeDetection(
        "circle",
        (float(center_x), float(center_y)),
        contour,
        box,
        float(score),
        float(area),
        float(radius),
    )


def _rectangle_candidate(
    contour: np.ndarray,
    area: float,
    perimeter: float,
) -> Optional[ShapeDetection]:
    rect = cv2.minAreaRect(contour)
    rect_width, rect_height = rect[1]
    if rect_width <= 1.0 or rect_height <= 1.0:
        return None

    rect_area = rect_width * rect_height
    fill_ratio = area / rect_area if rect_area > 0 else 0.0
    aspect_ratio = max(rect_width, rect_height) / min(rect_width, rect_height)
    polygon = cv2.approxPolyDP(contour, 0.025 * perimeter, True)

    if not 4 <= len(polygon) <= 6:
        return None
    if aspect_ratio > 8.0:
        return None
    if fill_ratio < 0.72:
        return None

    box = cv2.boxPoints(rect).astype(np.float32)
    center = _contour_center(contour)
    score = min(fill_ratio, 1.0)
    return ShapeDetection(
        "rectangle",
        (float(center[0]), float(center[1])),
        contour,
        box,
        float(score),
        float(area),
    )


def _contour_center(contour: np.ndarray) -> Tuple[float, float]:
    moments = cv2.moments(contour)
    if abs(moments["m00"]) > 1e-6:
        return moments["m10"] / moments["m00"], moments["m01"] / moments["m00"]
    rect = cv2.minAreaRect(contour)
    return float(rect[0][0]), float(rect[0][1])


__all__ = [
    "detect_qrcode_offset",
    "detect_black_circle_offset",
    "detect_black_rectangle_offset",
]
