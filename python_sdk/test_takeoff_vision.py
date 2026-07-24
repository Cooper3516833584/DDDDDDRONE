"""
起飞矩形视觉检测调试工具

截取自 2026_disaster_survey.py 的起飞点视觉校准逻辑。
打开摄像头，实时检测起飞矩形，在画面上标注检测框、中心点、
计算出的飞行方向与速度。不连接飞控，纯视觉验证。

按 q 退出，按 s 截图保存到当前目录。
"""
import math
import sys
import time
from typing import Optional, Tuple

import cv2
import numpy as np

from vision_for_simulation.takeoff_rectangle import (
    TakeoffRectangle,
    detect_takeoff_rectangle,
    draw_takeoff_rectangle,
)
from vision_for_simulation.camera_offsets import _center_to_offset

# ============ 可调参数 ============
CAMERA_INDEX = 0                # /dev/video0
FRAME_WIDTH = 1280
FRAME_HEIGHT = 720

# 与 2026_disaster_survey.py 相同的校准参数
CALIB_CLOSE_THRESHOLD_PX = 30   # 像素距离阈值: 小于此值认为已居中
CALIB_APPROACH_SPEED = 15       # 逼近速度 cm/s
# =================================


def _open_camera(index: int) -> cv2.VideoCapture:
    if sys.platform.startswith("linux"):
        backends = (cv2.CAP_V4L2, cv2.CAP_ANY)
    elif sys.platform.startswith("win"):
        backends = (cv2.CAP_DSHOW, cv2.CAP_ANY)
    else:
        backends = (cv2.CAP_ANY,)

    for backend in backends:
        cap = cv2.VideoCapture(index, backend)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
            return cap
        cap.release()
    raise RuntimeError(f"Unable to open camera index {index}")


def _compute_direction(
    offset: Optional[Tuple[float, float]],
) -> Tuple[Optional[float], Optional[float], Optional[float]]:
    """根据像素偏移计算飞行方向角度和距离。

    坐标约定 (与 _center_to_offset 一致):
      +x_px = 图像上方 = 机头前 (x+)
      +y_px = 图像左侧 = 机头左 (y+)

    Returns:
      (angle_deg, dist_px, speed_cm_s)
        angle_deg: 飞行方向 / deg, 0=前, +90=左, 逆时针
        dist_px:   像素距离
        speed_cm_s: 校准速度 cm/s (仅在未居中时有效)
    """
    if offset is None:
        return None, None, None
    x_px, y_px = offset
    dist_px = float(np.hypot(x_px, y_px))
    if dist_px < 1e-6:
        return 0.0, 0.0, 0.0
    angle_deg = float(np.rad2deg(np.arctan2(y_px, x_px)))
    if dist_px <= CALIB_CLOSE_THRESHOLD_PX:
        speed = 0.0   # 已居中，悬停
    else:
        speed = CALIB_APPROACH_SPEED
    return angle_deg, dist_px, speed


def _draw_hud(
    frame: np.ndarray,
    offset: Optional[Tuple[float, float]],
    angle_deg: Optional[float],
    dist_px: Optional[float],
    speed_cm_s: Optional[float],
    fps: float,
) -> np.ndarray:
    """在画面上叠加 HUD 信息: 飞行方向箭头、角度、距离、速度。"""
    h, w = frame.shape[:2]
    cx, cy = w // 2, h // 2

    # ---- 方向箭头 ----
    if angle_deg is not None and dist_px is not None and dist_px > CALIB_CLOSE_THRESHOLD_PX:
        arrow_len = int(min(w, h) * 0.25)  # 箭头长度 1/4 画面
        rad = math.radians(angle_deg)
        # 方向指向矩形（即无人机需要移动的方向）→ 箭头指向 (cx + dx, cy - dy)
        # +x = up = -y 在图像中
        dx = int(arrow_len * math.sin(rad) * 1.0)   # y分量 → 画面水平
        dy = -int(arrow_len * math.cos(rad) * 1.0)  # x分量 → 画面竖直
        end = (cx + dx, cy + dy)
        cv2.arrowedLine(frame, (cx, cy), end, (0, 255, 255), 3, cv2.LINE_AA, tipLength=0.15)

        # 角度文字
        cv2.putText(
            frame,
            f"DIR: {angle_deg:.0f} deg",
            (cx + dx + 10, cy + dy),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )

    # ---- 十字线 (画面中心/机头正下) ----
    cv2.drawMarker(frame, (cx, cy), (255, 255, 255), cv2.MARKER_CROSS, 24, 1, cv2.LINE_AA)

    # ---- 左上角状态栏 ----
    y0 = 30
    dy = 30
    status_color = (0, 255, 0)
    if dist_px is not None:
        if dist_px <= CALIB_CLOSE_THRESHOLD_PX:
            centered_text = "CENTERED"
            status_color = (0, 255, 0)
        else:
            centered_text = f"OFFSET: {dist_px:.0f} px"
            status_color = (0, 200, 255)
    else:
        centered_text = "NO DETECTION"
        status_color = (0, 0, 255)

    cv2.putText(frame, centered_text, (12, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.70, status_color, 2, cv2.LINE_AA)
    y0 += dy

    if offset is not None:
        cv2.putText(
            frame,
            f"offset: (x={offset[0]:.0f}, y={offset[1]:.0f})px",
            (12, y0),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.50,
            (200, 200, 200),
            1,
            cv2.LINE_AA,
        )
        y0 += dy

    if angle_deg is not None:
        cv2.putText(
            frame,
            f"move dir: {angle_deg:.0f} deg  (0=fwd, +90=left)",
            (12, y0),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.50,
            (200, 200, 200),
            1,
            cv2.LINE_AA,
        )
        y0 += dy

    if speed_cm_s is not None:
        cv2.putText(
            frame,
            f"move speed: {speed_cm_s:.0f} cm/s  (threshold={CALIB_CLOSE_THRESHOLD_PX}px)",
            (12, y0),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.50,
            (200, 200, 200),
            1,
            cv2.LINE_AA,
        )
        y0 += dy

    cv2.putText(
        frame,
        f"FPS: {fps:.0f}",
        (12, y0),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.50,
        (200, 200, 200),
        1,
        cv2.LINE_AA,
    )
    y0 += dy

    cv2.putText(
        frame,
        "Q:quit  S:save screenshot",
        (12, h - 12),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (150, 150, 150),
        1,
        cv2.LINE_AA,
    )

    return frame


def main():
    print(f"Opening camera /dev/video{CAMERA_INDEX} ...")
    cap = _open_camera(CAMERA_INDEX)
    print("Camera opened. Press 'q' to quit, 's' to save screenshot.")
    print(f"Calibration threshold: {CALIB_CLOSE_THRESHOLD_PX} px")
    print(f"Approach speed (simulated): {CALIB_APPROACH_SPEED} cm/s")
    print("-" * 50)

    cv2.namedWindow("Takeoff Rectangle Calibration", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Takeoff Rectangle Calibration", 960, 540)

    # 预热
    for _ in range(10):
        cap.read()

    fps_t0 = time.perf_counter()
    fps_counter = 0
    fps = 0.0

    try:
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                print("Frame read failed")
                time.sleep(0.05)
                continue

            # ---- 检测起飞矩形 ----
            detection = detect_takeoff_rectangle(frame)
            offset: Optional[Tuple[float, float]] = None

            if detection is not None:
                h, w = int(frame.shape[0]), int(frame.shape[1])
                offset = _center_to_offset(detection.center, (h, w))
                # 绘制检测框 + 对角线 + 中心十字
                frame = draw_takeoff_rectangle(frame, detection, color=(0, 255, 0))

            # ---- 计算飞行方向 & 速度 ----
            angle_deg, dist_px, speed_cm_s = _compute_direction(offset)

            # ---- HUD 叠加 ----
            frame = _draw_hud(frame, offset, angle_deg, dist_px, speed_cm_s, fps)

            # ---- FPS 统计 ----
            fps_counter += 1
            now = time.perf_counter()
            if now - fps_t0 >= 1.0:
                fps = fps_counter / (now - fps_t0)
                fps_counter = 0
                fps_t0 = now

            cv2.imshow("Takeoff Rectangle Calibration", frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            elif key == ord("s"):
                ts = time.strftime("%Y%m%d_%H%M%S")
                filename = f"takeoff_calib_{ts}.jpg"
                cv2.imwrite(filename, frame)
                print(f"[SCREENSHOT] Saved: {filename}")

    finally:
        cap.release()
        cv2.destroyAllWindows()
        print("\nDone.")


if __name__ == "__main__":
    main()
