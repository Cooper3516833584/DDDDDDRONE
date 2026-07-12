"""USB 摄像头二维码定位、解码和像素偏移测试工具。

运行示例：
    python qr_center_offset.py
    python qr_center_offset.py --camera 1 --width 1280 --height 720

坐标约定（以当前图像中心为原点）：
    图像左方为 +Y，图像上方为 +Z。
因此 dY_px > 0 表示二维码在画面左侧，dZ_px > 0 表示二维码在画面上方。
按 q 或 Esc 退出。
"""

import argparse
import sys
from dataclasses import dataclass
from typing import List, Optional, Tuple

import cv2
import numpy as np


@dataclass
class QRDetection:
    """二维码的像素角点、中心和已解出的文本。"""

    corners: np.ndarray
    text: str
    source: str

    @property
    def center(self) -> Tuple[float, float]:
        center = self.corners.mean(axis=0)
        return float(center[0]), float(center[1])

    @property
    def area(self) -> float:
        return abs(float(cv2.contourArea(self.corners.astype(np.float32))))


def _as_corners(points) -> Optional[np.ndarray]:
    """将 pyzbar/OpenCV 的角点统一为 shape=(4, 2) 的数组。"""
    if points is None:
        return None
    corners = np.asarray(points, dtype=np.float32).reshape(-1, 2)
    if len(corners) < 4:
        return None
    return corners[:4]


def detect_with_pyzbar(frame: np.ndarray) -> List[QRDetection]:
    """使用 ZBar 在原灰度和增强对比度图上解码 QR。"""
    try:
        from pyzbar.pyzbar import ZBarSymbol, decode
    except ImportError:
        return []

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    enhanced_gray = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(gray)
    detections = []
    for image in (gray, enhanced_gray):
        for result in decode(image, symbols=[ZBarSymbol.QRCODE]):
            corners = _as_corners(result.polygon)
            if corners is None:
                x, y, width, height = result.rect
                corners = np.array(
                    [[x, y], [x + width, y], [x + width, y + height], [x, y + height]],
                    dtype=np.float32,
                )
            text = result.data.decode("utf-8", errors="replace")
            detections.append(QRDetection(corners=corners, text=text, source="pyzbar"))
    return detections


def detect_with_opencv(detector: cv2.QRCodeDetector, frame: np.ndarray) -> List[QRDetection]:
    """优先使用 OpenCV 多码接口；解码失败时仍返回已定位的角点。"""
    detections = []
    try:
        ok, texts, points, _ = detector.detectAndDecodeMulti(frame)
    except cv2.error:
        ok, texts, points = False, (), None

    if ok and points is not None:
        for index, point_set in enumerate(points):
            corners = _as_corners(point_set)
            if corners is not None:
                text = texts[index] if index < len(texts) else ""
                detections.append(QRDetection(corners=corners, text=text, source="opencv"))
        return detections

    try:
        found, points = detector.detectMulti(frame)
    except cv2.error:
        found, points = False, None
    if found and points is not None:
        for point_set in points:
            corners = _as_corners(point_set)
            if corners is not None:
                detections.append(QRDetection(corners=corners, text="", source="opencv"))
        return detections

    # 为缺少多码 API 的旧版 OpenCV 保留单码兼容路径。
    text, points, _ = detector.detectAndDecode(frame)
    corners = _as_corners(points)
    if corners is not None:
        detections.append(QRDetection(corners=corners, text=text, source="opencv"))
    return detections


def _center_distance(first: QRDetection, second: QRDetection) -> float:
    first_x, first_y = first.center
    second_x, second_y = second.center
    return float(np.hypot(first_x - second_x, first_y - second_y))


def merge_detections(detections: List[QRDetection]) -> List[QRDetection]:
    """合并两个解码器对同一码的重复结果，并优先保留已解出的内容。"""
    merged = []
    for detection in detections:
        match_index = next(
            (index for index, item in enumerate(merged) if _center_distance(item, detection) <= 35.0),
            None,
        )
        if match_index is None:
            merged.append(detection)
        elif detection.text and not merged[match_index].text:
            current = merged[match_index]
            merged[match_index] = QRDetection(
                corners=current.corners,
                text=detection.text,
                source="opencv+pyzbar",
            )
    return merged


def detect_qrcodes(detector: cv2.QRCodeDetector, frame: np.ndarray) -> List[QRDetection]:
    """合并多码定位、ZBar 解码和对比度增强后的结果。"""
    return merge_detections(detect_with_opencv(detector, frame) + detect_with_pyzbar(frame))


def open_usb_camera(camera_index: int, width: int, height: int) -> cv2.VideoCapture:
    """按平台选择摄像头后端，默认打开 USB 摄像头索引 0。"""
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
    raise RuntimeError("无法打开 USB 摄像头索引 {}".format(camera_index))


def draw_axes(frame: np.ndarray, image_center: Tuple[int, int]) -> None:
    """绘制用户指定的 +Y（左）和 +Z（上）图像坐标轴。"""
    center_x, center_y = image_center
    axis_length = min(frame.shape[0], frame.shape[1]) // 7
    color = (180, 180, 180)
    cv2.arrowedLine(frame, (center_x, center_y), (center_x - axis_length, center_y), color, 2)
    cv2.arrowedLine(frame, (center_x, center_y), (center_x, center_y - axis_length), color, 2)
    cv2.putText(frame, "+Y", (center_x - axis_length, center_y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    cv2.putText(frame, "+Z", (center_x + 8, center_y - axis_length), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)


def draw_detection(frame: np.ndarray, detection: QRDetection, image_center: Tuple[int, int]) -> None:
    """框出二维码、绘制中心连线，并显示按 +Y/+Z 约定计算的像素偏移。"""
    corners = np.rint(detection.corners).astype(np.int32)
    qr_x, qr_y = detection.center
    qr_center = (int(round(qr_x)), int(round(qr_y)))
    center_x, center_y = image_center

    # 左为 +Y、上为 +Z，故二维码相对图像中心的偏移需反向相减。
    delta_y_px = center_x - qr_x
    delta_z_px = center_y - qr_y

    cv2.polylines(frame, [corners], True, (0, 255, 0), 2, cv2.LINE_AA)
    cv2.circle(frame, qr_center, 5, (0, 255, 255), -1, cv2.LINE_AA)
    cv2.line(frame, image_center, qr_center, (0, 255, 255), 2, cv2.LINE_AA)

    label = "{} | dY={:+.1f}px, dZ={:+.1f}px".format(
        detection.source, delta_y_px, delta_z_px
    )
    text_y = max(24, int(corners[:, 1].min()) - 10)
    text_x = max(0, int(corners[:, 0].min()))
    cv2.putText(frame, label, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2, cv2.LINE_AA)

    if detection.text:
        payload = "QR: {}".format(detection.text[:80])
        cv2.putText(frame, payload, (text_x, text_y + 24), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)


def select_nearest_to_image_center(
    detections: List[QRDetection], image_center: Tuple[int, int]
) -> QRDetection:
    """多个二维码同时出现时，选择中心离画面中心最近的一个。"""
    center_x, center_y = image_center
    return min(
        detections,
        key=lambda item: (item.center[0] - center_x) ** 2 + (item.center[1] - center_y) ** 2,
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="USB 摄像头二维码中心与像素偏移测试")
    parser.add_argument("--camera", type=int, default=1, help="USB 摄像头索引，默认 1")
    parser.add_argument("--width", type=int, default=1280, help="期望画面宽度，默认 1280")
    parser.add_argument("--height", type=int, default=720, help="期望画面高度，默认 720")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    capture = open_usb_camera(args.camera, args.width, args.height)
    detector = cv2.QRCodeDetector()
    window_name = "QR center offset (+Y left, +Z up)"

    print("二维码测试已启动：按 q 或 Esc 退出。")
    try:
        while True:
            ok, frame = capture.read()
            if not ok or frame is None:
                print("摄像头读取失败，结束。")
                break

            output = frame.copy()
            height, width = output.shape[:2]
            image_center = (width // 2, height // 2)
            cv2.drawMarker(output, image_center, (255, 255, 0), cv2.MARKER_CROSS, 18, 2, cv2.LINE_AA)
            draw_axes(output, image_center)

            detections = detect_qrcodes(detector, frame)
            if detections:
                # 多码时严格以二维码中心到画面中心的距离最小者为准。
                detection = select_nearest_to_image_center(detections, image_center)
                # 框、中心和连线必须使用同一帧的原始四角，不能跨帧平均，
                # 否则相邻二维码或不同角点顺序会把框拉偏。
                draw_detection(output, detection, image_center)
            else:
                cv2.putText(output, "No QR code detected", (20, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2, cv2.LINE_AA)

            cv2.imshow(window_name, output)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                break
    finally:
        capture.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
