"""YOLO-based terrain-marker detection for the simulation environment."""

from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple

import numpy as np


Point = Tuple[float, float]
Box = Tuple[float, float, float, float]
_MODEL = None
_YOLO_IMAGE_SIZE = 256
_YOLO_NMS_IOU = 0.50
_MODEL_PATH = (
    Path(__file__).resolve().parents[1]
    / "FlightController"
    / "Solutions"
    / "models"
    / "simulation.pt"
)


@dataclass(frozen=True)
class TerrainRing:
    """YOLO terrain detection expressed in original image pixel coordinates."""

    box: Box
    center: Point
    confidence: float
    class_name: str
    distance_to_image_center: float


def detect_nearest_terrain_ring(
    frame: np.ndarray, confidence_threshold: float = 0.80
) -> Optional[TerrainRing]:
    """Return the YOLO terrain box whose center is nearest the image center.

    The center is calculated from the intersection of the detection box
    diagonals: ``((x1 + x2) / 2, (y1 + y2) / 2)``.  No black-ring, contour, or
    Hough-circle detection is used.
    """
    if frame is None or frame.ndim != 3 or frame.shape[2] != 3:
        raise ValueError("frame must be a non-empty BGR image with three channels")
    if frame.shape[0] == 0 or frame.shape[1] == 0:
        raise ValueError("frame must not be empty")
    if not 0.0 <= confidence_threshold <= 1.0:
        raise ValueError("confidence_threshold must be in [0, 1]")

    result = _predict_frames([frame], confidence_threshold)[0]
    return _select_yolo_detection(result, frame.shape[:2], confidence_threshold)


def _predict_frames(frames, confidence_threshold: float):
    """Run the same YOLO model on one or more frames in a batch."""
    return _get_model()(
        frames,
        verbose=False,
        device="cpu",
        conf=confidence_threshold,
        imgsz=_YOLO_IMAGE_SIZE,
        iou=_YOLO_NMS_IOU,
    )


def _select_yolo_detection(result, frame_shape, confidence_threshold: float) -> Optional[TerrainRing]:
    """Select the nearest valid YOLO box from one inference result."""
    frame_height, frame_width = frame_shape
    image_center = np.array((frame_width / 2.0, frame_height / 2.0))
    candidates = []
    for box in result.boxes:
        confidence = float(box.conf[0])
        if confidence < confidence_threshold:
            continue
        x1, y1, x2, y2 = (float(value) for value in box.xyxy[0].tolist())
        center_x = (x1 + x2) / 2.0
        center_y = (y1 + y2) / 2.0
        distance = float(np.hypot(center_x - image_center[0], center_y - image_center[1]))
        class_id = int(box.cls[0])
        class_name = str(result.names[class_id])
        candidates.append(
            TerrainRing(
                box=(x1, y1, x2, y2),
                center=(center_x, center_y),
                confidence=confidence,
                class_name=class_name,
                distance_to_image_center=distance,
            )
        )
    return min(candidates, key=lambda item: item.distance_to_image_center) if candidates else None


def draw_terrain_ring(frame: np.ndarray, detection: TerrainRing) -> np.ndarray:
    """Return a copy of ``frame`` with the YOLO box and its diagonal center."""
    import cv2

    output = frame.copy()
    x1, y1, x2, y2 = (int(round(value)) for value in detection.box)
    center = tuple(int(round(value)) for value in detection.center)
    cv2.rectangle(output, (x1, y1), (x2, y2), (0, 255, 0), 3, cv2.LINE_AA)
    cv2.drawMarker(output, center, (0, 0, 255), cv2.MARKER_CROSS, 32, 3, cv2.LINE_AA)
    label = "{} {:.2f}".format(detection.class_name, detection.confidence)
    cv2.putText(
        output,
        label,
        (max(0, x1), max(28, y1 - 10)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 0),
        2,
        cv2.LINE_AA,
    )
    return output


def _get_model():
    """Load the simulation YOLO model lazily so importing stays hardware-free."""
    global _MODEL
    if _MODEL is None:
        if not _MODEL_PATH.is_file():
            raise FileNotFoundError("Simulation YOLO model not found: {}".format(_MODEL_PATH))
        from ultralytics import YOLO

        _MODEL = YOLO(str(_MODEL_PATH))
    return _MODEL
