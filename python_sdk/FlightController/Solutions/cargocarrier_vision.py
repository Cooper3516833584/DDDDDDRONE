"""
Function:
    ooooocr(camera_index)
    ooooocr(camera_index, color, shape)

Purpose:
    Use the specified camera to detect the largest red or blue shape in the image,
    then return the line information from the image center to the target center.
    If color and shape are both given, only the matching target is searched.

Input parameters:
    camera_index:
        Camera index, such as 0, 1, or 2.
    color:
        Optional. "red" / "blue", or "红" / "蓝".
    shape:
        Optional. "triangle" / "square" / "circle",
        or "三角形" / "方形" / "正方形" / "圆形".

Return value:
    If a target is found, return:
        (angle_deg, line_length, color, shape)

    If no target is found, return:
        None

Meaning of returned tuple:
    angle_deg:
        The counterclockwise angle from the positive x-axis to the line from image
        center to target center, in [0, 360). Coordinate rule: x positive right,
        y positive up.
    line_length:
        Pixel distance from image center to target center.
    color:
        Detected target color.
    shape:
        Detected target shape.

Call example:
    result = ooooocr(0)
    if result is None:
        print("target not found")
    else:
        angle, length, color, shape = result
        print(angle, length, color, shape)

    result = ooooocr(0, "red", "triangle")
    if result is None:
        print("target not found")
    else:
        angle, length, color, shape = result
        print(angle, length, color, shape)

Notes:
    0 usually means the built-in camera, 1 or 2 may be external USB cameras.
    When TEST_MODE = True, the program shows the live camera image, mask, and the
    returned value in the top-left corner for debugging.
    When TEST_MODE = False, it behaves like a normal function and directly returns
    the result without opening debug windows.
"""

import math
from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np


TEST_MODE = False
CAMERA_INDEX = 0
red = 'red'
triangle = 'triangle'
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
WARMUP_FRAMES = 8
MEASURE_FRAMES = 6

COLOR_RANGES = {
    "red": [((0, 70, 50), (12, 255, 255)), ((165, 70, 50), (180, 255, 255))],
    "blue": [((90, 70, 50), (135, 255, 255))],
}
COLOR_ALIASES = {
    "red": "red",
    "blue": "blue",
    "红": "red",
    "蓝": "blue",
}
SHAPE_ALIASES = {
    "triangle": "triangle",
    "square": "square",
    "circle": "circle",
    "三角": "triangle",
    "三角形": "triangle",
    "方形": "square",
    "正方形": "square",
    "圆": "circle",
    "圆形": "circle",
}
ResultTuple = Tuple[float, float, str, str]
ResultType = Optional[ResultTuple]


@dataclass
class Detection:
    color: str
    shape: str
    center: Tuple[int, int]
    area: float
    contour: np.ndarray


def ensure_camera_size(frame: np.ndarray) -> np.ndarray:
    if frame.shape[1] == CAMERA_WIDTH and frame.shape[0] == CAMERA_HEIGHT:
        return frame
    return cv2.resize(frame, (CAMERA_WIDTH, CAMERA_HEIGHT), interpolation=cv2.INTER_LINEAR)


def build_color_mask(frame: np.ndarray, color_name: str) -> np.ndarray:
    blurred = cv2.GaussianBlur(frame, (5, 5), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lower, upper in COLOR_RANGES[color_name]:
        lower_np = np.array(lower, dtype=np.uint8)
        upper_np = np.array(upper, dtype=np.uint8)
        mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower_np, upper_np))

    kernel_size = max(3, int(round(min(frame.shape[:2]) * 0.004)))
    if kernel_size % 2 == 0:
        kernel_size += 1
    kernel_size = min(kernel_size, 9)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))

    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    return mask


def contour_center(cnt: np.ndarray) -> Optional[Tuple[int, int]]:
    moments = cv2.moments(cnt)
    if moments["m00"] < 1e-6:
        return None

    cx = int(moments["m10"] / moments["m00"])
    cy = int(moments["m01"] / moments["m00"])
    return cx, cy


def classify_shape(cnt: np.ndarray) -> Optional[str]:
    area = cv2.contourArea(cnt)
    if area < 100:
        return None

    perimeter = cv2.arcLength(cnt, True)
    if perimeter < 1e-6:
        return None

    approx = cv2.approxPolyDP(cnt, 0.025 * perimeter, True)
    vertices = len(approx)

    hull = cv2.convexHull(cnt)
    hull_area = cv2.contourArea(hull)
    solidity = area / (hull_area + 1e-6)

    rect = cv2.minAreaRect(cnt)
    rect_w, rect_h = rect[1]
    if rect_w < 1e-6 or rect_h < 1e-6:
        return None

    rect_ratio = max(rect_w, rect_h) / (min(rect_w, rect_h) + 1e-6)
    rect_fill = area / (rect_w * rect_h + 1e-6)
    circularity = 4.0 * math.pi * area / (perimeter * perimeter + 1e-6)

    if vertices == 3 and solidity > 0.9:
        return "triangle"

    if 4 <= vertices <= 6 and rect_ratio < 1.35 and rect_fill > 0.72 and solidity > 0.9:
        return "square"

    if vertices >= 6 and circularity > 0.72 and solidity > 0.9:
        return "circle"

    return None


def normalize_color(color: Optional[str]) -> Optional[str]:
    if color is None:
        return None

    color_text = str(color).strip()
    if not color_text:
        return None

    normalized = COLOR_ALIASES.get(color_text.lower())
    if normalized is None:
        normalized = COLOR_ALIASES.get(color_text)
    if normalized is None:
        raise ValueError("color must be red/blue or 红/蓝")
    return normalized


def normalize_shape(shape: Optional[str]) -> Optional[str]:
    if shape is None:
        return None

    shape_text = str(shape).strip()
    if not shape_text:
        return None

    normalized = SHAPE_ALIASES.get(shape_text.lower())
    if normalized is None:
        normalized = SHAPE_ALIASES.get(shape_text)
    if normalized is None:
        raise ValueError("shape must be triangle/square/circle or 三角形/方形/正方形/圆形")
    return normalized


def detect_target(
    frame: np.ndarray,
    target_color: Optional[str] = None,
    target_shape: Optional[str] = None,
) -> Tuple[Optional[Detection], np.ndarray]:
    min_area = max(150.0, frame.shape[0] * frame.shape[1] * 0.0005)
    combined_mask = np.zeros(frame.shape[:2], dtype=np.uint8)
    best_detection: Optional[Detection] = None
    best_score = -1.0

    colors_to_search = [target_color] if target_color is not None else list(COLOR_RANGES.keys())

    for color_name in colors_to_search:
        mask = build_color_mask(frame, color_name)
        combined_mask = cv2.bitwise_or(combined_mask, mask)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < min_area:
                continue

            shape = classify_shape(cnt)
            if shape is None:
                continue
            if target_shape is not None and shape != target_shape:
                continue

            center = contour_center(cnt)
            if center is None:
                continue

            if area > best_score:
                best_score = area
                best_detection = Detection(
                    color=color_name,
                    shape=shape,
                    center=center,
                    area=area,
                    contour=cnt,
                )

    return best_detection, combined_mask


def detection_to_result(frame: np.ndarray, detection: Optional[Detection]) -> ResultType:
    if detection is None:
        return None

    img_cx = frame.shape[1] / 2.0
    img_cy = frame.shape[0] / 2.0
    target_cx = float(detection.center[0])
    target_cy = float(detection.center[1])

    dx = target_cx - img_cx
    # Convert from image coordinates to the requested math coordinates:
    # x positive to the right, y positive upward.
    dy_math = img_cy - target_cy
    angle_deg = (math.degrees(math.atan2(dy_math, dx)) + 360.0) % 360.0
    line_length = math.hypot(dx, dy_math)
    return angle_deg, line_length, detection.color, detection.shape


def draw_text_lines(frame: np.ndarray, lines: Tuple[str, ...], start_xy: Tuple[int, int]) -> None:
    x, y = start_xy
    for line in lines:
        cv2.putText(
            frame,
            line,
            (x, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 0),
            4,
            cv2.LINE_AA,
        )
        cv2.putText(
            frame,
            line,
            (x, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        y += 28


def result_to_lines(result: ResultType) -> Tuple[str, ...]:
    if result is None:
        return (
            "result=None",
        )

    angle_deg, line_length, color, shape = result
    return (
        f"angle={angle_deg:.1f} deg",
        f"length={line_length:.1f} px",
        f"color={color}",
        f"shape={shape}",
    )


def draw_debug_view(frame: np.ndarray, detection: Optional[Detection], result: ResultTuple) -> np.ndarray:
    img_center = (frame.shape[1] // 2, frame.shape[0] // 2)
    cv2.circle(frame, img_center, 6, (0, 255, 0), -1)
    cv2.circle(frame, img_center, 18, (0, 255, 0), 2)

    if detection is not None:
        cv2.drawContours(frame, [detection.contour], -1, (0, 255, 255), 3)
        cv2.circle(frame, detection.center, 7, (0, 0, 255), -1)
        cv2.line(frame, img_center, detection.center, (255, 0, 0), 2)

    draw_text_lines(frame, result_to_lines(result), (20, 35))
    return frame


def open_camera(camera_index: int) -> cv2.VideoCapture:
    import sys
    # Linux优先使用V4L2，Windows使用DSHOW，其他平台使用默认后端
    if sys.platform.startswith("linux"):
        cap = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)
    elif sys.platform == "win32":
        cap = cv2.VideoCapture(camera_index, cv2.CAP_DSHOW)
    else:
        cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        try:
            cap.release()
        except Exception:
            pass
        cap = cv2.VideoCapture(camera_index)

    if not cap.isOpened():
        try:
            cap.release()
        except Exception:
            pass
        raise RuntimeError(f"Cannot open camera index {camera_index}")

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
    return cap


def read_frame(cap: cv2.VideoCapture) -> np.ndarray:
    ret, frame = cap.read()
    if not ret or frame is None:
        raise RuntimeError("Camera frame read failed.")
    return ensure_camera_size(frame)


def warm_up_camera(cap: cv2.VideoCapture) -> None:
    for _ in range(WARMUP_FRAMES):
        read_frame(cap)


def capture_single_result(
    cap: cv2.VideoCapture,
    target_color: Optional[str],
    target_shape: Optional[str],
) -> ResultType:
    best_result: ResultType = None
    best_score = -1.0

    for _ in range(MEASURE_FRAMES):
        frame = read_frame(cap)
        detection, _ = detect_target(frame, target_color, target_shape)
        if detection is None:
            continue

        result = detection_to_result(frame, detection)
        if detection.area > best_score:
            best_score = detection.area
            best_result = result

    return best_result


def run_test_session(
    cap: cv2.VideoCapture,
    target_color: Optional[str],
    target_shape: Optional[str],
) -> ResultType:
    last_valid_result: ResultType = None

    while True:
        frame = read_frame(cap)
        detection, mask = detect_target(frame, target_color, target_shape)
        result = detection_to_result(frame, detection)

        if result is not None:
            last_valid_result = result

        annotated = draw_debug_view(frame.copy(), detection, result)
        cv2.imshow("frame", annotated)
        cv2.imshow("mask", mask)

        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord("q")):
            if detection is not None:
                return result
            return last_valid_result


def ooooocr(
    camera_index: int,
    color: Optional[str] = None,
    shape: Optional[str] = None,
) -> ResultType:
    """
    Return (angle_deg, line_length, color, shape) for the detected target.

    Call forms:
        ooooocr(camera_index)
        ooooocr(camera_index, color, shape)

    If color and shape are provided, only the matching target is searched.
    If no matching target is found, return None.
    """
    target_color = normalize_color(color)
    target_shape = normalize_shape(shape)

    cap = open_camera(camera_index)
    try:
        warm_up_camera(cap)
        if TEST_MODE:
            return run_test_session(cap, target_color, target_shape)
        return capture_single_result(cap, target_color, target_shape)
    finally:
        cap.release()


if __name__ == "__main__":
    print(ooooocr(CAMERA_INDEX,red,triangle))
