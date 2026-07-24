"""
test_camera_terrain_ring.py
===========================
从 /dev/video0 摄像头以 10 Hz 实时检测地形环，输出 detect_nearest_terrain_ring
的检测结果。仅使用视觉模块，不连接飞控。

运行方式（上位机）:
  cd ~/DDDDDrone_Cloned/python_sdk
  python3 testcode/test_camera_terrain_ring.py

或从仓库根目录:
  PYTHONPATH=. python3 python_sdk/testcode/test_camera_terrain_ring.py

可选参数:
  --camera N         摄像头索引 (默认 0, 即 /dev/video0)
  --freq N           检测频率 Hz (默认 10)
  --conf N           置信度阈值 (默认 0.80)
  --dist-warn N      像素距离告警阈值 (默认 100px, 低于此值时输出 WARN)
  --no-display       禁用实时预览窗口（无桌面环境时使用）
  --width N          摄像头分辨率宽度 (默认 1280)
  --height N         摄像头分辨率高度 (默认 720)

按键（预览窗口启用时）:
  q / ESC            退出
"""

import argparse
import os
import sys
import time
from pathlib import Path
from typing import Optional

import cv2
import numpy as np

# ---- 确保 python_sdk 包可导入 ----
_HERE = Path(__file__).resolve().parent
_SDK = _HERE.parent
if str(_SDK) not in sys.path:
    sys.path.insert(0, str(_SDK))

# ---- ultralytics 新旧版本兼容层 ----
# 上位机旧版 ultralytics 缺少 C3k2 / C3k / C2PSA / PSABlock / Attention，
# 而 simulation.pt 是用 yolo11 (ultralytics==8.4.92) 训练的。
# 以下从 yolo11-8.4.92 源码完整回植缺失的 block 类，确保 torch.load 能
# 反序列化模型且 forward() 推理结果正确。仅当旧版缺少时才注入。
try:
    import torch
    import torch.nn as _nn
    import ultralytics.nn.modules.block as _block
    from ultralytics.nn.modules.conv import Conv as _Conv
    from ultralytics.nn.modules.block import Bottleneck as _Bottleneck

    _injected = []

    # ---- Attention (yolo11-8.4.92 block.py lines 1271-1329) ----
    if not hasattr(_block, "Attention"):

        class Attention(_nn.Module):
            def __init__(self, dim: int, num_heads: int = 8, attn_ratio: float = 0.5):
                super().__init__()
                self.num_heads = num_heads
                self.head_dim = dim // num_heads
                self.key_dim = int(self.head_dim * attn_ratio)
                self.scale = self.key_dim ** -0.5
                nh_kd = self.key_dim * num_heads
                h = dim + nh_kd * 2
                self.qkv = _Conv(dim, h, 1, act=False)
                self.proj = _Conv(dim, dim, 1, act=False)
                self.pe = _Conv(dim, dim, 3, 1, g=dim, act=False)

            def forward(self, x):
                B, C, H, W = x.shape
                N = H * W
                qkv = self.qkv(x)
                q, k, v = qkv.view(
                    B, self.num_heads, self.key_dim * 2 + self.head_dim, N
                ).split([self.key_dim, self.key_dim, self.head_dim], dim=2)
                attn = (q * self.scale).transpose(-2, -1) @ k
                attn = attn.softmax(dim=-1)
                x = (v @ attn.transpose(-2, -1)).view(B, C, H, W) + self.pe(
                    v.reshape(B, C, H, W)
                )
                x = self.proj(x)
                return x

        _block.Attention = Attention
        _injected.append("Attention")

    # ---- PSABlock (yolo11-8.4.92 block.py lines 1331-1379) ----
    if not hasattr(_block, "PSABlock"):

        class PSABlock(_nn.Module):
            def __init__(
                self, c: int, attn_ratio: float = 0.5,
                num_heads: int = 4, shortcut: bool = True,
            ):
                super().__init__()
                self.attn = _block.Attention(c, attn_ratio=attn_ratio, num_heads=num_heads)
                self.ffn = _nn.Sequential(
                    _Conv(c, c * 2, 1), _Conv(c * 2, c, 1, act=False)
                )
                self.add = shortcut

            def forward(self, x):
                x = x + self.attn(x) if self.add else self.attn(x)
                x = x + self.ffn(x) if self.add else self.ffn(x)
                return x

        _block.PSABlock = PSABlock
        _injected.append("PSABlock")

    # ---- C3k (yolo11-8.4.92 block.py lines 1109-1128) ----
    if not hasattr(_block, "C3k"):

        class C3k(_block.C3):
            def __init__(
                self, c1: int, c2: int, n: int = 1,
                shortcut: bool = True, g: int = 1, e: float = 0.5, k: int = 3,
            ):
                super().__init__(c1, c2, n, shortcut, g, e)
                c_ = int(c2 * e)
                self.m = _nn.Sequential(
                    *(_Bottleneck(c_, c_, shortcut, g, k=(k, k), e=1.0)
                      for _ in range(n))
                )

        _block.C3k = C3k
        _injected.append("C3k")

    # ---- C3k2 (yolo11-8.4.92 block.py lines 1069-1107) ----
    if not hasattr(_block, "C3k2"):

        class C3k2(_block.C2f):
            def __init__(
                self, c1: int, c2: int, n: int = 1,
                c3k: bool = False, e: float = 0.5, attn: bool = False,
                g: int = 1, shortcut: bool = True,
            ):
                super().__init__(c1, c2, n, shortcut, g, e)
                self.m = _nn.ModuleList(
                    _nn.Sequential(
                        _Bottleneck(self.c, self.c, shortcut, g),
                        _block.PSABlock(
                            self.c, attn_ratio=0.5,
                            num_heads=max(self.c // 64, 1),
                        ),
                    )
                    if attn
                    else _block.C3k(self.c, self.c, 2, shortcut, g)
                    if c3k
                    else _Bottleneck(self.c, self.c, shortcut, g)
                    for _ in range(n)
                )

        _block.C3k2 = C3k2
        _injected.append("C3k2")

    # ---- C2PSA (yolo11-8.4.92 block.py lines 1436-1489) ----
    if not hasattr(_block, "C2PSA"):

        class C2PSA(_nn.Module):
            def __init__(self, c1: int, c2: int, n: int = 1, e: float = 0.5):
                super().__init__()
                assert c1 == c2
                self.c = int(c1 * e)
                self.cv1 = _Conv(c1, 2 * self.c, 1, 1)
                self.cv2 = _Conv(2 * self.c, c1, 1)
                self.m = _nn.Sequential(
                    *(_block.PSABlock(
                        self.c, attn_ratio=0.5, num_heads=self.c // 64,
                    ) for _ in range(n))
                )

            def forward(self, x):
                a, b = self.cv1(x).split((self.c, self.c), dim=1)
                b = self.m(b)
                return self.cv2(torch.cat((a, b), 1))

        _block.C2PSA = C2PSA
        _injected.append("C2PSA")

    print(
        "[COMPAT] ultralytics backport OK — injected: "
        + (", ".join(_injected) if _injected else "(all present)")
    )

except Exception as _patch_err:
    print(f"[COMPAT] ultralytics patch FAILED: {_patch_err}")
    import traceback

    traceback.print_exc()

from vision_for_simulation.terrain_ring import (
    TerrainRing,
    detect_nearest_terrain_ring,
    draw_terrain_ring,
)
from vision_for_simulation.camera_offsets import _center_to_offset


# ======================== 摄像头工具 ========================


def _open_persistent_camera(
    index: int, width: int = 1280, height: int = 720
) -> cv2.VideoCapture:
    """打开摄像头并返回 VideoCapture 对象（与 2026_disaster_survey.py 一致）。"""
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


# ======================== 10 Hz 检测循环 ========================


def run_detection_loop(
    camera_index: int = 0,
    freq: float = 10.0,
    confidence_threshold: float = 0.80,
    dist_warn: float = 100.0,
    show_display: bool = True,
    width: int = 1280,
    height: int = 720,
) -> None:
    """主循环：以 freq Hz 从摄像头读取帧并输出地形环检测结果。"""

    dt = 1.0 / max(freq, 1.0)
    cap = _open_persistent_camera(camera_index, width, height)
    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"[CAM] /dev/video{camera_index} opened: {actual_w}x{actual_h}")

    # 无桌面环境时自动关闭预览窗口（避免 Qt/X11 报错）
    if show_display and not os.environ.get("DISPLAY"):
        print("[DISP] No DISPLAY detected, automatically switching to headless mode")
        show_display = False
    print(f"[CFG] detection @ {freq} Hz, conf >= {confidence_threshold}, dist_warn < {dist_warn}px")
    print(
        f"[CTRL] Preview: {'ON' if show_display else 'OFF (headless)'}  |  "
        "Press Ctrl+C to quit\n"
    )

    if show_display:
        cv2.namedWindow("terrain_ring_detection", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("terrain_ring_detection", 960, 540)

    frame_count = 0
    detect_count = 0
    t_start = time.perf_counter()
    t_last_print = t_start

    try:
        while True:
            loop_start = time.perf_counter()

            # ---- 读取帧 ----
            ok, frame = cap.read()
            if not ok or frame is None:
                print("[WARN] Frame read failed, retrying...")
                key = cv2.waitKey(1) if show_display else -1
                if key in (27, ord("q")):
                    break
                time.sleep(dt * 0.5)
                continue

            frame_count += 1
            frame_h, frame_w = frame.shape[:2]

            # ---- 调用 detect_nearest_terrain_ring ----
            detection: Optional[TerrainRing] = detect_nearest_terrain_ring(
                frame, confidence_threshold=confidence_threshold
            )

            # ---- 结果输出 ----
            if detection is not None:
                detect_count += 1
                offset_x, offset_y = _center_to_offset(
                    detection.center, (frame_h, frame_w)
                )
                now = time.perf_counter()
                # 检测到目标时每帧都打印（不节流），避免漏报
                t_last_print = now
                elapsed = now - t_start
                print(
                    f"[{elapsed:6.1f}s] #{detect_count:4d}  "
                    f"class={detection.class_name:<16s}  "
                    f"conf={detection.confidence:.3f}  "
                    f"center=({detection.center[0]:6.1f}, {detection.center[1]:6.1f})px  "
                    f"offset=(x={offset_x:+7.1f}, y={offset_y:+7.1f})px  "
                    f"像素距离={detection.distance_to_image_center:.1f}px"
                )
                if detection.distance_to_image_center < dist_warn:
                    print(
                        f"  ⚠ WARN: 像素距离 {detection.distance_to_image_center:.1f}px "
                        f"低于阈值 {dist_warn:.0f}px —— 地形环过近！"
                    )
            else:
                now = time.perf_counter()
                if now - t_last_print >= 2.0:
                    t_last_print = now
                    elapsed = now - t_start
                    print(
                        f"[{elapsed:6.1f}s] #{detect_count:4d}  "
                        f"(no detection in this frame)"
                    )

            # ---- 预览窗口 ----
            if show_display:
                display_frame = frame.copy()
                if detection is not None:
                    display_frame = draw_terrain_ring(display_frame, detection)
                # 叠加状态栏
                status_text = "DET: {} conf={:.2f}".format(
                    detection.class_name if detection else "none",
                    detection.confidence if detection else 0.0,
                )
                # 像素距离过近时状态栏变红
                status_color = (
                    (0, 0, 255)
                    if (detection and detection.distance_to_image_center < dist_warn)
                    else (0, 255, 0) if detection
                    else (0, 0, 255)
                )
                cv2.putText(
                    display_frame,
                    status_text,
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    status_color,
                    2,
                    cv2.LINE_AA,
                )
                fps_text = f"FPS: {1.0 / max(time.perf_counter() - loop_start, 0.001):.1f}"
                cv2.putText(
                    display_frame,
                    fps_text,
                    (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 255, 255),
                    1,
                    cv2.LINE_AA,
                )
                cv2.imshow("terrain_ring_detection", display_frame)

                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord("q")):
                    print("\n[CTRL] User quit")
                    break

            # ---- 频率控制 ----
            elapsed_loop = time.perf_counter() - loop_start
            if elapsed_loop < dt:
                time.sleep(dt - elapsed_loop)

    except KeyboardInterrupt:
        print("\n[CTRL] Interrupted by user")

    finally:
        cap.release()
        if show_display:
            cv2.destroyAllWindows()

    # ---- 汇总 ----
    elapsed_total = time.perf_counter() - t_start
    print(f"\n{'='*50}")
    print(f"[SUMMARY] Runtime: {elapsed_total:.1f}s")
    print(f"[SUMMARY] Frames captured: {frame_count}")
    print(f"[SUMMARY] Detections: {detect_count}")
    print(
        f"[SUMMARY] Actual frame rate: {frame_count / max(elapsed_total, 0.001):.1f} Hz"
    )
    if detect_count > 0:
        print(
            f"[SUMMARY] Detection rate: {detect_count / max(elapsed_total, 0.001):.2f} Hz"
        )
    print(f"{'='*50}")


# ======================== 入口 ========================


def main():
    parser = argparse.ArgumentParser(
        description="10Hz terrain-ring detection from /dev/video0"
    )
    parser.add_argument(
        "--camera",
        type=int,
        default=0,
        help="Camera index (default: 0 → /dev/video0 on Linux)",
    )
    parser.add_argument(
        "--freq", type=float, default=10.0, help="Detection frequency in Hz (default: 10)"
    )
    parser.add_argument(
        "--conf",
        type=float,
        default=0.80,
        help="Confidence threshold (default: 0.80)",
    )
    parser.add_argument(
        "--dist-warn",
        type=float,
        default=100.0,
        help="Pixel-distance warning threshold in px (default: 100)",
    )
    parser.add_argument(
        "--no-display",
        action="store_true",
        help="Disable real-time preview window",
    )
    parser.add_argument(
        "--width", type=int, default=1280, help="Camera width (default: 1280)"
    )
    parser.add_argument(
        "--height", type=int, default=720, help="Camera height (default: 720)"
    )
    args = parser.parse_args()

    run_detection_loop(
        camera_index=args.camera,
        freq=args.freq,
        confidence_threshold=args.conf,
        dist_warn=args.dist_warn,
        show_display=not args.no_display,
        width=args.width,
        height=args.height,
    )


if __name__ == "__main__":
    main()
