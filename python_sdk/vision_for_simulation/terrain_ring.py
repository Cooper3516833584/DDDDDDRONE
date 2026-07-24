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


# ---- ultralytics 新旧版本兼容层 ----
# simulation.pt 使用 yolo11 (ultralytics>=8.4) 训练，包含 C3k2 / C3k /
# C2PSA / PSABlock 等新版 block 类。旧版 ultralytics (如 8.2.63) 缺少
# 这些类会导致 torch.load 反序列化失败。以下从 yolo11-8.4.92 源码回植。
def _patch_ultralytics_blocks() -> None:
    try:
        import torch.nn as _nn
        import ultralytics.nn.modules.block as _b
        from ultralytics.nn.modules.conv import Conv as _Conv
        from ultralytics.nn.modules.block import Bottleneck as _Bottleneck

        # -- Attention --
        if not hasattr(_b, "Attention"):

            class Attention(_nn.Module):
                def __init__(self, dim: int, num_heads: int = 8, attn_ratio: float = 0.5):
                    super().__init__()
                    self.num_heads = num_heads
                    self.head_dim = dim // num_heads
                    self.key_dim = int(self.head_dim * attn_ratio)
                    self.scale = self.key_dim**-0.5
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

            _b.Attention = Attention

        # -- PSABlock --
        if not hasattr(_b, "PSABlock"):

            class PSABlock(_nn.Module):
                def __init__(
                    self, c: int, attn_ratio: float = 0.5,
                    num_heads: int = 4, shortcut: bool = True,
                ):
                    super().__init__()
                    self.attn = _b.Attention(c, attn_ratio=attn_ratio, num_heads=num_heads)
                    self.ffn = _nn.Sequential(
                        _Conv(c, c * 2, 1), _Conv(c * 2, c, 1, act=False)
                    )
                    self.add = shortcut

                def forward(self, x):
                    x = x + self.attn(x) if self.add else self.attn(x)
                    x = x + self.ffn(x) if self.add else self.ffn(x)
                    return x

            _b.PSABlock = PSABlock

        # -- C3k --
        if not hasattr(_b, "C3k"):

            class C3k(_b.C3):
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

            _b.C3k = C3k

        # -- C3k2 --
        if not hasattr(_b, "C3k2"):

            class C3k2(_b.C2f):
                def __init__(
                    self, c1: int, c2: int, n: int = 1,
                    c3k: bool = False, e: float = 0.5, attn: bool = False,
                    g: int = 1, shortcut: bool = True,
                ):
                    super().__init__(c1, c2, n, shortcut, g, e)
                    self.m = _nn.ModuleList(
                        _nn.Sequential(
                            _Bottleneck(self.c, self.c, shortcut, g),
                            _b.PSABlock(
                                self.c, attn_ratio=0.5,
                                num_heads=max(self.c // 64, 1),
                            ),
                        )
                        if attn
                        else _b.C3k(self.c, self.c, 2, shortcut, g)
                        if c3k
                        else _Bottleneck(self.c, self.c, shortcut, g)
                        for _ in range(n)
                    )

            _b.C3k2 = C3k2

        # -- C2PSA --
        if not hasattr(_b, "C2PSA"):

            class C2PSA(_nn.Module):
                def __init__(self, c1: int, c2: int, n: int = 1, e: float = 0.5):
                    super().__init__()
                    assert c1 == c2
                    self.c = int(c1 * e)
                    self.cv1 = _Conv(c1, 2 * self.c, 1, 1)
                    self.cv2 = _Conv(2 * self.c, c1, 1)
                    self.m = _nn.Sequential(
                        *(_b.PSABlock(
                            self.c, attn_ratio=0.5, num_heads=self.c // 64,
                        ) for _ in range(n))
                    )

                def forward(self, x):
                    import torch as _torch
                    a, b = self.cv1(x).split((self.c, self.c), dim=1)
                    b = self.m(b)
                    return self.cv2(_torch.cat((a, b), 1))

            _b.C2PSA = C2PSA

    except Exception:
        pass  # 非关键路径：新版 ultralytics 已自带这些类时无需注入


_patch_ultralytics_blocks()


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
