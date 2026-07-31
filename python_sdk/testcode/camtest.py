import os
import cv2
from datetime import datetime

# 1) 桌面 photos 目录（兼容中文“桌面”和英文“Desktop”）
home = os.path.expanduser("~")
desktop_cn = os.path.join(home, "桌面")
desktop_en = os.path.join(home, "Desktop")
desktop = desktop_cn if os.path.isdir(desktop_cn) else desktop_en

photos_dir = os.path.join(desktop, "photos")
os.makedirs(photos_dir, exist_ok=True)

# 2) 打开摄像头（默认 /dev/video0）
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    raise RuntimeError("Camera open failed: index 0 (/dev/video0)")

# 3) 设置分辨率 800x600（如果摄像头不支持，会自动回落到它支持的尺寸）
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 800)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 600)

# 4) 锁定色温 6500K（先关闭自动白平衡，再设置色温；摄像头不支持时打印提示）
if not cap.set(cv2.CAP_PROP_AUTO_WB, 0):
    print("[warn] 关闭自动白平衡失败，色温可能无法锁定")
# 不同 OpenCV 版本该属性名不同，按可用性取第一个（标准数值为 42）
wb_prop = getattr(cv2, "CAP_PROP_WHITE_BALANCE_TEMPERATURE", None)
if wb_prop is None:
    wb_prop = getattr(cv2, "CAP_PROP_WB_TEMPERATURE", None)
if wb_prop is None:
    wb_prop = 42
if not cap.set(wb_prop, 500):
    print(f"[warn] 摄像头不支持设置色温 (prop={wb_prop})")

# 5) 取一帧并保存
ret, frame = cap.read()
cap.release()

if not ret or frame is None:
    raise RuntimeError("Camera read failed")

filename = f"photo_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jpg"
save_path = os.path.join(photos_dir, filename)

ok = cv2.imwrite(save_path, frame)
if not ok:
    raise RuntimeError(f"Failed to write image: {save_path}")

print(f"Saved: {save_path}")
