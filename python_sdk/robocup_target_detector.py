# -*- coding: utf-8 -*-
"""RoboCup 2026 投放目标检测组件：识别红/蓝/绿/黄四色同心圆目标并给出像素偏移。

权重：python_sdk/FlightController/Solutions/models/robocup_target.pt（YOLO11n，4 类）
类别：0 red / 1 blue / 2 green / 3 yellow

最常用的调用方式：

    from robocup_target_detector import locate_target

    det = locate_target(frame)          # frame 是 BGR ndarray（cv2 读到的原图）
    if det is not None:
        print(det.color, det.confidence, det.offset_x_px, det.offset_y_px)

只想要像素差时：

    from robocup_target_detector import locate_target_xy

    xy = locate_target_xy(frame)        # (offset_x_px, offset_y_px) 或 None

像素偏移约定（与 rescue_drop_2026.TargetObservation 完全一致）：

    offset_x_px = 画面中心行 - 目标框中心行   -> 目标在画面中心正上方为正（机体 x 向前）
    offset_y_px = 画面中心列 - 目标框中心列   -> 目标在画面中心正左方为正（机体 y 向左）

目标中心取 YOLO 框的几何中心 ((x1+x2)/2, (y1+y2)/2)。TargetDetection 的字段名与
rescue_drop_2026.TargetObservation 一一对应，可以用 as_observation_fields() 直接填
target_id / captured_at 后构造该 dataclass。

注意：
  * 本模块 import 时不加载模型、不开相机、不访问硬件；首次调用检测时惰性加载并缓存。
  * ultralytics / torch 未安装时，只有真正调用检测才会报错，import 阶段不会崩。
  * 一个实例内部有锁，多线程调用不会崩，但推理会串行；建议视觉线程单独持有一个实例。
  * 默认不画图、不打逐帧日志，避免影响正式任务实时性（drawOutput=True 才画）。
"""

import math
import os
import threading
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

try:                                    # 与仓库其他 Vision 组件保持一致，缺失时降级
    from loguru import logger
except ImportError:                     # pragma: no cover
    import logging

    logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# 常量
# ---------------------------------------------------------------------------
MODEL_FILENAME = "robocup_target.pt"
MODEL_SUBDIR = os.path.join("FlightController", "Solutions", "models")
MODEL_PATH_ENV = "ROBOCUP_TARGET_WEIGHTS"

CLASS_NAMES = ("red", "blue", "green", "yellow")

DEFAULT_CONF_THRESHOLD = 0.35
DEFAULT_IOU_THRESHOLD = 0.7
DEFAULT_IMGSZ = 640
DEFAULT_SELECT = "conf"                 # "conf"=取置信度最高；"center"=取离画面中心最近

SELECT_MODES = ("conf", "center")

COLOR_ALIASES = {
    "red": "red", "红": "red", "红色": "red",
    "blue": "blue", "蓝": "blue", "蓝色": "blue",
    "green": "green", "绿": "green", "绿色": "green",
    "yellow": "yellow", "黄": "yellow", "黄色": "yellow",
}


def default_model_path() -> str:
    """按脚本位置拼权重路径（不依赖当前工作目录）；可用环境变量覆盖。"""
    override = os.environ.get(MODEL_PATH_ENV)
    if override:
        return override
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(here, MODEL_SUBDIR, MODEL_FILENAME)


def normalize_color(color: Optional[str]) -> Optional[str]:
    """把 red/蓝 之类的输入统一成 red/blue/green/yellow；None 表示不筛选。"""
    if color is None:
        return None
    text = str(color).strip()
    if not text:
        return None
    key = text.lower()
    if key not in COLOR_ALIASES:
        raise ValueError("color 必须是 red/blue/green/yellow 或 红/蓝/绿/黄，收到: %r" % (color,))
    return COLOR_ALIASES[key]


@dataclass(frozen=True)
class TargetDetection:
    """一次目标检测结果；偏移方向：+x 向前（画面正上），+y 向左（画面正左）。"""

    color: str
    confidence: float
    offset_x_px: float
    offset_y_px: float
    center: Tuple[float, float]                     # 目标框几何中心 (列u, 行v)
    image_center: Tuple[float, float]               # 画面中心 (列u, 行v)
    box: Tuple[float, float, float, float]          # x1, y1, x2, y2（像素）
    frame_size: Tuple[int, int]                     # (宽, 高)

    @property
    def offset_distance_px(self) -> float:
        """目标中心到画面中心的像素距离。"""
        return float(math.hypot(self.offset_x_px, self.offset_y_px))

    @property
    def box_width(self) -> float:
        return float(self.box[2] - self.box[0])

    @property
    def box_height(self) -> float:
        return float(self.box[3] - self.box[1])

    def as_observation_fields(self, target_id: str = "", captured_at: float = 0.0) -> Dict[str, object]:
        """拼出 rescue_drop_2026.TargetObservation 需要的字段，便于直接构造。"""
        return {
            "target_id": target_id,
            "color": self.color,
            "offset_x_px": self.offset_x_px,
            "offset_y_px": self.offset_y_px,
            "captured_at": captured_at,
        }


class RobocupTargetDetector:
    """YOLO11n 四色同心目标检测器；用法与 Vision_Net 中的其他网络一致。"""

    def __init__(
        self,
        model_path: Optional[str] = None,
        confThreshold: float = DEFAULT_CONF_THRESHOLD,
        iouThreshold: float = DEFAULT_IOU_THRESHOLD,
        imgsz: int = DEFAULT_IMGSZ,
        device: Optional[object] = None,
        drawOutput: bool = False,
    ):
        """
        model_path: 权重路径；None 时用 ROBOCUP_TARGET_WEIGHTS 环境变量或默认 models/robocup_target.pt
        confThreshold: 置信度阈值
        iouThreshold: NMS IoU 阈值
        imgsz: 推理尺寸（640 足够；目标在画面里通常很大）
        device: None=自动（有 CUDA 用 0，否则 cpu）；也可显式给 "cpu" / 0
        drawOutput: 是否在 detect/locate 时把框画在传入的 frame 上（默认关）
        """
        self.model_path = model_path or default_model_path()
        self.confThreshold = float(confThreshold)
        self.iouThreshold = float(iouThreshold)
        self.imgsz = int(imgsz)
        self.device = device
        self.drawOutput = bool(drawOutput)

        self._model = None
        self._names: Dict[int, str] = {}
        self._lock = threading.Lock()

    # -- 模型 ---------------------------------------------------------------
    def _ensure_model(self):
        """惰性加载模型；只在第一次调用时读权重文件。"""
        if self._model is not None:
            return self._model
        if not os.path.exists(self.model_path):
            raise FileNotFoundError(
                "找不到权重文件: %s\n请确认 FlightController/Solutions/models/robocup_target.pt 存在，"
                "或设置环境变量 %s 指向权重。" % (self.model_path, MODEL_PATH_ENV)
            )
        try:
            from ultralytics import YOLO      # 惰性导入：无该依赖时 import 本模块仍然安全
        except ImportError as exc:
            raise ImportError(
                "缺少 ultralytics，无法运行 robocup_target 检测；请先 python3 -m pip install "
                "ultralytics（python_sdk/requirements.txt 已包含）。"
            ) from exc

        self._model = YOLO(self.model_path)
        self._names = dict(getattr(self._model, "names", {}) or {})
        logger.debug("[robocup_target] 已加载权重 {} (classes={})".format(self.model_path, self._names))
        return self._model

    @property
    def names(self) -> Dict[int, str]:
        """类别表 {id: name}；未加载模型时返回空 dict。"""
        return dict(self._names)

    def _resolve_device(self):
        if self.device is not None:
            return self.device
        try:
            import torch
            return 0 if torch.cuda.is_available() else "cpu"
        except Exception:                     # 没有 torch 时交给 ultralytics 自己兜底
            return "cpu"

    # -- 检测 ---------------------------------------------------------------
    def detect(self, frame: np.ndarray, color: Optional[str] = None) -> List[TargetDetection]:
        """返回画面里所有目标（按置信度从高到低）；color 给定时只保留该颜色。"""
        if frame is None:
            raise ValueError("frame 不能为 None")
        if not isinstance(frame, np.ndarray) or frame.ndim != 3 or frame.shape[2] != 3:
            raise ValueError("frame 必须是 BGR 三通道图像 (H, W, 3)")

        want = normalize_color(color)
        model = self._ensure_model()
        height, width = frame.shape[0], frame.shape[1]
        center_u, center_v = width / 2.0, height / 2.0

        with self._lock:                      # 单实例串行推理，避免多线程同时进模型
            results = model.predict(
                source=frame,
                imgsz=self.imgsz,
                conf=self.confThreshold,
                iou=self.iouThreshold,
                device=self._resolve_device(),
                verbose=False,
            )
        result = results[0]

        detections: List[TargetDetection] = []
        for box in result.boxes:
            class_id = int(box.cls)
            name = self._names.get(class_id, str(class_id))
            if want is not None and name != want:
                continue
            x1, y1, x2, y2 = [float(v) for v in box.xyxy[0].tolist()]
            center_target_u = (x1 + x2) / 2.0
            center_target_v = (y1 + y2) / 2.0
            detections.append(
                TargetDetection(
                    color=name,
                    confidence=float(box.conf),
                    # 画面正上为 +x，画面正左为 +y
                    offset_x_px=center_v - center_target_v,
                    offset_y_px=center_u - center_target_u,
                    center=(center_target_u, center_target_v),
                    image_center=(center_u, center_v),
                    box=(x1, y1, x2, y2),
                    frame_size=(width, height),
                )
            )

        detections.sort(key=lambda d: d.confidence, reverse=True)
        if self.drawOutput:
            self.draw(frame, detections)
        return detections

    def locate(
        self,
        frame: np.ndarray,
        color: Optional[str] = None,
        select: str = DEFAULT_SELECT,
    ) -> Optional[TargetDetection]:
        """返回一个目标；找不到返回 None。

        select="conf"   取置信度最高的目标（默认）
        select="center" 取离画面中心最近的目标（下视相机里通常就是正下方那个）
        """
        if select not in SELECT_MODES:
            raise ValueError("select 必须是 %s 之一，收到: %r" % ("/".join(SELECT_MODES), select))
        detections = self.detect(frame, color=color)
        if not detections:
            return None
        if select == "center":
            return min(detections, key=lambda d: d.offset_distance_px)
        return detections[0]

    # -- 可视化（默认关闭） --------------------------------------------------
    def draw(self, frame: np.ndarray, detections: Sequence[TargetDetection]) -> np.ndarray:
        """把检测框和目标中心画到 frame 上（原地修改并返回），仅供调试。"""
        height, width = frame.shape[0], frame.shape[1]
        image_center = (int(width // 2), int(height // 2))
        cv2 = _cv2()
        cv2.circle(frame, image_center, 6, (0, 255, 0), -1)
        for det in detections:
            x1, y1, x2, y2 = [int(round(v)) for v in det.box]
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
            target_center = (int(round(det.center[0])), int(round(det.center[1])))
            cv2.circle(frame, target_center, 5, (0, 0, 255), -1)
            cv2.line(frame, image_center, target_center, (255, 0, 0), 2)
            label = "{} {:.2f} dx={:.0f} dy={:.0f}".format(
                det.color, det.confidence, det.offset_x_px, det.offset_y_px
            )
            cv2.putText(frame, label, (x1, max(18, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 255, 0), 2)
        return frame


def _cv2():
    """惰性导入 cv2：只在需要画图时才依赖 OpenCV。"""
    import cv2

    return cv2


# ---------------------------------------------------------------------------
# 模块级便捷函数：任务脚本直接调用，无需自己管理实例
# ---------------------------------------------------------------------------
_DEFAULT_DETECTOR: Optional[RobocupTargetDetector] = None
_DEFAULT_LOCK = threading.Lock()


def get_default_detector(**kwargs) -> RobocupTargetDetector:
    """取（首次调用时创建）模块级共享检测器；kwargs 只在首次创建时生效。"""
    global _DEFAULT_DETECTOR
    with _DEFAULT_LOCK:
        if _DEFAULT_DETECTOR is None:
            _DEFAULT_DETECTOR = RobocupTargetDetector(**kwargs)
        return _DEFAULT_DETECTOR


def reset_default_detector() -> None:
    """丢弃共享检测器（测试或换权重时用）。"""
    global _DEFAULT_DETECTOR
    with _DEFAULT_LOCK:
        _DEFAULT_DETECTOR = None


def locate_target(
    frame: np.ndarray,
    color: Optional[str] = None,
    select: str = DEFAULT_SELECT,
    detector: Optional[RobocupTargetDetector] = None,
) -> Optional[TargetDetection]:
    """识别画面里的投放目标，返回 TargetDetection（含像素偏移），找不到返回 None。"""
    active = detector if detector is not None else get_default_detector()
    return active.locate(frame, color=color, select=select)


def locate_target_xy(
    frame: np.ndarray,
    color: Optional[str] = None,
    select: str = DEFAULT_SELECT,
    detector: Optional[RobocupTargetDetector] = None,
) -> Optional[Tuple[float, float]]:
    """只返回 (offset_x_px, offset_y_px)：画面正上为 +x，画面正左为 +y。"""
    det = locate_target(frame, color=color, select=select, detector=detector)
    if det is None:
        return None
    return det.offset_x_px, det.offset_y_px


if __name__ == "__main__":
    # 离线自检：只读一张图片或一个图片目录，不开相机、不连飞控。
    #   python3 robocup_target_detector.py <图片路径 | 图片目录>
    import sys

    if len(sys.argv) < 2:
        raise SystemExit("用法: python3 robocup_target_detector.py <图片路径 | 图片目录>")
    image_path = sys.argv[1]
    if os.path.isdir(image_path):
        names = sorted(f for f in os.listdir(image_path)
                       if os.path.splitext(f)[1].lower() in (".jpg", ".jpeg", ".png", ".bmp"))
        if not names:
            raise SystemExit("目录里没有图片: %s" % image_path)
        image_path = os.path.join(image_path, names[0])

    cv2 = _cv2()
    image = cv2.imread(image_path)
    if image is None:
        raise SystemExit("读不到图片: %s" % image_path)

    detector = RobocupTargetDetector(drawOutput=False)
    print("权重: %s" % detector.model_path)
    print("图片: %s  %dx%d" % (image_path, image.shape[1], image.shape[0]))
    for det in detector.detect(image):
        print("  color={:<7} conf={:.3f} offset_x_px={:+.1f} offset_y_px={:+.1f} "
              "center=({:.1f},{:.1f}) box=({:.0f},{:.0f},{:.0f},{:.0f})".format(
                  det.color, det.confidence, det.offset_x_px, det.offset_y_px,
                  det.center[0], det.center[1], det.box[0], det.box[1], det.box[2], det.box[3]))
    best = detector.locate(image)
    print("locate_target -> %s" % ("None" if best is None else
                                   "{} dx={:+.1f} dy={:+.1f}".format(
                                       best.color, best.offset_x_px, best.offset_y_px)))
    print("locate_target_xy -> %s" % (locate_target_xy(image, detector=detector),))
