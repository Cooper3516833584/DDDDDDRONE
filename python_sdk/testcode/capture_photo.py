"""capture_photo.py — 从 /dev/video2 拍一张照片并保存。

运行方式（上位机）:
  python3 python_sdk/testcode/capture_photo.py
  python3 python_sdk/testcode/capture_photo.py --camera 2 --output test.jpg

可选参数:
  --camera N   摄像头索引 (默认 2, 即 /dev/video2)
  --output     输出路径 (默认当前目录, 自动按时间命名)
  --width N    分辨率宽度 (默认 1280)
  --height N   分辨率高度 (默认 720)
"""

import argparse
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2


def _open_camera(index: int, width: int = 1280, height: int = 720) -> cv2.VideoCapture:
    if sys.platform.startswith("linux"):
        backends = (cv2.CAP_V4L2, cv2.CAP_ANY)
    elif sys.platform.startswith("win"):
        backends = (cv2.CAP_DSHOW, cv2.CAP_ANY)
    else:
        backends = (cv2.CAP_ANY,)

    for backend in backends:
        cap = cv2.VideoCapture(index, backend)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            return cap
        cap.release()
    raise RuntimeError(f"Unable to open camera index {index}")


def main():
    parser = argparse.ArgumentParser(description="Capture a single photo from camera")
    parser.add_argument("--camera", type=int, default=2)
    parser.add_argument("--output", type=str, default=".")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    args = parser.parse_args()

    cap = _open_camera(args.camera, args.width, args.height)
    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"[CAM] /dev/video{args.camera} opened: {actual_w}x{actual_h}")

    # 预热（丢弃前几帧，让自动曝光稳定）
    for i in range(5):
        cap.read()
        time.sleep(0.05)

    ret, frame = cap.read()
    cap.release()

    if not ret or frame is None:
        raise RuntimeError("Camera read failed")

    # 确定保存路径
    out_path = Path(args.output)
    if out_path.is_dir():
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        out_path = out_path / f"capture_{actual_w}x{actual_h}_{ts}.jpg"

    cv2.imwrite(str(out_path), frame)
    print(f"[OK] Saved: {out_path}  ({frame.shape[1]}x{frame.shape[0]})")


if __name__ == "__main__":
    main()
