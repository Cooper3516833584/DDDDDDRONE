"""USB camera black circle/rectangle center-offset test tool.

Run examples:
    python black_shape_center_offset.py
    python black_shape_center_offset.py --camera 1 --width 1280 --height 720

Coordinate convention, with the image center as origin:
    image up is +X, image left is +Y.
Therefore dX_px > 0 means the target is above image center, and
dY_px > 0 means the target is left of image center.

Press q or Esc to quit; press m to show/hide the black mask.
"""

import argparse
import sys
from dataclasses import dataclass
from typing import List, Tuple

import cv2
import numpy as np


@dataclass
class ShapeDetection:
    """Detected pure-black geometric target."""

    kind: str
    center: Tuple[float, float]
    contour: np.ndarray
    box: np.ndarray
    score: float
    area: float
    radius: float = 0.0


def open_usb_camera(camera_index: int, width: int, height: int) -> cv2.VideoCapture:
    """Open a USB camera with a platform-appropriate backend."""
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


def make_black_mask(frame: np.ndarray, threshold: int) -> np.ndarray:
    """Extract pure-black regions from the current frame."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    mask = cv2.inRange(gray, 0, threshold)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    return mask


def contour_center(contour: np.ndarray) -> Tuple[float, float]:
    """Return contour centroid, falling back to minAreaRect center."""
    moments = cv2.moments(contour)
    if abs(moments["m00"]) > 1e-6:
        return moments["m10"] / moments["m00"], moments["m01"] / moments["m00"]
    rect = cv2.minAreaRect(contour)
    return float(rect[0][0]), float(rect[0][1])


def circle_candidate(
    contour: np.ndarray,
    area: float,
    perimeter: float,
    min_area: float,
) -> ShapeDetection:
    """Build a circle candidate, or return None when geometry is not circular."""
    (center_x, center_y), radius = cv2.minEnclosingCircle(contour)
    if radius <= 1.0:
        return None

    circle_area = np.pi * radius * radius
    circle_fill = area / circle_area if circle_area > 0 else 0.0
    circularity = 4.0 * np.pi * area / (perimeter * perimeter)
    x, y, width, height = cv2.boundingRect(contour)
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
        kind="circle",
        center=(float(center_x), float(center_y)),
        contour=contour,
        box=box,
        score=float(score),
        area=float(area),
        radius=float(radius),
    )


def rectangle_candidate(
    contour: np.ndarray,
    area: float,
    perimeter: float,
) -> ShapeDetection:
    """Build a rectangle candidate, or return None when geometry is not rectangular."""
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
    center = contour_center(contour)
    score = min(fill_ratio, 1.0)
    return ShapeDetection(
        kind="rectangle",
        center=(float(center[0]), float(center[1])),
        contour=contour,
        box=box,
        score=float(score),
        area=float(area),
    )


def detect_black_shapes(
    frame: np.ndarray,
    threshold: int,
    min_area: float,
    max_area_ratio: float,
) -> Tuple[List[ShapeDetection], np.ndarray]:
    """Detect pure-black circles and rectangles from a frame."""
    mask = make_black_mask(frame, threshold)
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

        circle = circle_candidate(contour, area, perimeter, min_area)
        if circle is not None:
            detections.append(circle)
            continue

        rectangle = rectangle_candidate(contour, area, perimeter)
        if rectangle is not None:
            detections.append(rectangle)

    return detections, mask


def select_nearest_to_image_center(
    detections: List[ShapeDetection], image_center: Tuple[int, int]
) -> ShapeDetection:
    """When multiple targets are present, select the one nearest image center."""
    center_x, center_y = image_center
    return min(
        detections,
        key=lambda item: (item.center[0] - center_x) ** 2 + (item.center[1] - center_y) ** 2,
    )


def draw_axes(frame: np.ndarray, image_center: Tuple[int, int]) -> None:
    """Draw +X up and +Y left axes."""
    center_x, center_y = image_center
    axis_length = min(frame.shape[0], frame.shape[1]) // 7
    color = (180, 180, 180)
    cv2.arrowedLine(frame, image_center, (center_x, center_y - axis_length), color, 2)
    cv2.arrowedLine(frame, image_center, (center_x - axis_length, center_y), color, 2)
    cv2.putText(frame, "+X", (center_x + 8, center_y - axis_length), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    cv2.putText(frame, "+Y", (center_x - axis_length, center_y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)


def draw_detection(
    frame: np.ndarray, detection: ShapeDetection, image_center: Tuple[int, int]
) -> None:
    """Draw target outline, target center, line to image center, and pixel offsets."""
    target_x, target_y = detection.center
    target_center = (int(round(target_x)), int(round(target_y)))
    center_x, center_y = image_center

    # Up is +X and left is +Y, so both offsets are image-center minus target-center.
    delta_x_px = center_y - target_y
    delta_y_px = center_x - target_x

    if detection.kind == "circle":
        cv2.circle(frame, target_center, int(round(detection.radius)), (0, 255, 0), 2, cv2.LINE_AA)
    else:
        box = np.rint(detection.box).astype(np.int32)
        cv2.polylines(frame, [box], True, (0, 255, 0), 2, cv2.LINE_AA)

    cv2.circle(frame, target_center, 5, (0, 255, 255), -1, cv2.LINE_AA)
    cv2.line(frame, image_center, target_center, (0, 255, 255), 2, cv2.LINE_AA)

    box_for_text = np.rint(detection.box).astype(np.int32)
    text_x = max(0, int(box_for_text[:, 0].min()))
    text_y = max(24, int(box_for_text[:, 1].min()) - 10)
    label = "{} | dX={:+.1f}px, dY={:+.1f}px".format(
        detection.kind, delta_x_px, delta_y_px
    )
    cv2.putText(frame, label, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2, cv2.LINE_AA)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="USB camera pure-black circle/rectangle center offset test")
    parser.add_argument("--camera", type=int, default=1, help="USB camera index, default 1")
    parser.add_argument("--width", type=int, default=1280, help="requested frame width, default 1280")
    parser.add_argument("--height", type=int, default=720, help="requested frame height, default 720")
    parser.add_argument("--threshold", type=int, default=90, help="black grayscale threshold [0, 255], default 90")
    parser.add_argument("--min-area", type=float, default=1200.0, help="minimum target area in pixels, default 1200")
    parser.add_argument("--max-area-ratio", type=float, default=0.45, help="maximum target area ratio, default 0.45")
    args = parser.parse_args()

    if not 0 <= args.threshold <= 255:
        parser.error("--threshold must be in [0, 255]")
    if args.min_area <= 0:
        parser.error("--min-area must be greater than 0")
    if not 0 < args.max_area_ratio <= 1:
        parser.error("--max-area-ratio must be in (0, 1]")
    return args


def main() -> None:
    args = parse_args()
    capture = open_usb_camera(args.camera, args.width, args.height)
    window_name = "Black shape center offset (+X up, +Y left)"
    mask_window_open = False
    show_mask = False

    print("Black shape test started: press q/Esc to quit, press m to show/hide mask.")
    try:
        while True:
            ok, frame = capture.read()
            if not ok or frame is None:
                print("Camera read failed, exiting.")
                break

            output = frame.copy()
            height, width = output.shape[:2]
            image_center = (width // 2, height // 2)
            cv2.drawMarker(output, image_center, (255, 255, 0), cv2.MARKER_CROSS, 18, 2, cv2.LINE_AA)
            draw_axes(output, image_center)

            detections, mask = detect_black_shapes(
                frame, args.threshold, args.min_area, args.max_area_ratio
            )
            if detections:
                detection = select_nearest_to_image_center(detections, image_center)
                draw_detection(output, detection, image_center)
            else:
                cv2.putText(output, "No pure-black circle/rectangle detected", (20, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2, cv2.LINE_AA)

            cv2.imshow(window_name, output)
            if show_mask:
                cv2.imshow("Black shape mask", mask)
                mask_window_open = True
            elif mask_window_open:
                cv2.destroyWindow("Black shape mask")
                mask_window_open = False

            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                break
            if key == ord("m"):
                show_mask = not show_mask
    finally:
        capture.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
