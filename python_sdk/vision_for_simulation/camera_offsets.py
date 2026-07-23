"""Camera-index entry points for simulated takeoff-marker detection.

The functions in this module open a camera only for the duration of a short
sample window, then release it before returning.  They do not communicate with
the flight controller or send any control command.
"""

import sys
from typing import Callable, Optional, Tuple

import cv2
import numpy as np

from .takeoff_rectangle import detect_takeoff_rectangle
from .terrain_ring import detect_nearest_terrain_ring


PixelOffset = Tuple[float, float]
RingOffset = Tuple[float, float, str]


def detect_takeoff_point_offset(camera_index: int) -> Optional[PixelOffset]:
    """Return the takeoff rectangle offset as ``(x_px, y_px)``, or ``None``.

    ``+x`` is image up and ``+y`` is image left.  The supplied camera index is
    opened only while frames are sampled.
    """
    return _detect_camera_offset(camera_index, detect_takeoff_rectangle)


def detect_nearest_ring_offset(camera_index: int) -> Optional[RingOffset]:
    """Return the nearest YOLO terrain offset as ``(x_px, y_px, label)``.

    ``+x`` is image up and ``+y`` is image left.  Black partition lines are
    not used; ``label`` is the class name of the selected YOLO detection box.
    """
    detected = _detect_camera_detection(camera_index, detect_nearest_terrain_ring)
    if detected is None:
        return None
    detection, frame_shape = detected
    offset_x, offset_y = _center_to_offset(detection.center, frame_shape)
    return offset_x, offset_y, detection.class_name


def _detect_camera_offset(
    camera_index: int,
    detector: Callable[[np.ndarray], object],
    width: int = 1280,
    height: int = 720,
    warmup_frames: int = 3,
    max_detection_frames: int = 10,
) -> Optional[PixelOffset]:
    detected = _detect_camera_detection(
        camera_index, detector, width, height, warmup_frames, max_detection_frames
    )
    if detected is None:
        return None
    detection, frame_shape = detected
    return _center_to_offset(detection.center, frame_shape)


def _detect_camera_detection(
    camera_index: int,
    detector: Callable[[np.ndarray], object],
    width: int = 1280,
    height: int = 720,
    warmup_frames: int = 3,
    max_detection_frames: int = 10,
):
    capture = _open_usb_camera(camera_index, width, height)
    try:
        valid_detection_frames = 0
        for frame_index in range(warmup_frames + max_detection_frames):
            ok, frame = capture.read()
            if not ok or frame is None:
                continue
            if frame_index < warmup_frames:
                continue

            valid_detection_frames += 1
            detection = detector(frame)
            if detection is not None:
                return detection, frame.shape[:2]
            if valid_detection_frames >= max_detection_frames:
                break
        return None
    finally:
        capture.release()


def _open_usb_camera(camera_index: int, width: int, height: int) -> cv2.VideoCapture:
    """Open a camera using the platform's usual OpenCV backend."""
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


def _center_to_offset(center: Tuple[float, float], frame_shape: Tuple[int, int]) -> PixelOffset:
    """Convert image coordinates to the project's x-up, y-left convention."""
    target_x, target_y = center
    frame_height, frame_width = frame_shape
    return float(frame_height / 2.0 - target_y), float(frame_width / 2.0 - target_x)
