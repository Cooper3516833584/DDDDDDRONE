"""
2026 模拟赛 — 空地协同测绘救灾系统

任务流程:
  1. 定点起飞 → Cartographer / TF 就绪
  2. 起飞矩形视觉校准（下视 /dev/video0，检测黑色矩形标记，视觉闭环居中）
  3. 记录校准点为坐标原点
  4. 逐个航点导航（3×5 蛇形，100/170/240 × -40/-110/-180/-250/-320），
     每到达一个测绘航点记录 YOLO 地形 label 到网格
  5. 巡航期间 10 Hz 检测地形环。泥石流(debris_flow)像素距离 < 75px
     时打断（整次飞行仅触发一次），记录当前最近航点，执行泥石流动作后
     以 smooth 轨迹接续所有后续航点（不含泥石流航点），
     到达后续航点时继续记录 label
  6. 完成全部航点后降落在坐标原点

上位机已自启动 server_ros.py，本程序通过 FC_Client 连接。
ROS 建图组件启动流程参考 base_test.py 与 former_code/2024_D_24.py。
视觉闭环校准流程参考 former_code/2022_24.py 的 vision_approach 模式。
地形检测使用 vision_for_simulation 仿真视觉包（YOLO + 传统图像处理）。

视觉日志: python_sdk/vision_for_simulation/ring_detection_*.log
每次地形推理结果、航点出入事件均以时间戳记录，无需设置环境变量。
飞行录像: python_sdk/vision_for_simulation/camera_recording_*.avi
帧索引:   python_sdk/vision_for_simulation/camera_recording_*.csv
"""
import csv
import os
import queue
import sys
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from typing import Dict, List, Optional, Tuple, Union

import cv2
import numpy as np
from loguru import logger

from FlightController import FC_Client, FC_Like
from FlightController.Components import LD_Radar
from FlightController.Components.RealSense import T265
from FlightController.Solutions.Navigation import Navigation
from FlightController.Components.RosMapper import RosMapper
from FlightController.Components.RosNode import RosNodeRunner
from FlightController.Components.RosManager import RosManager
from FlightController.Components.UartScreen import UARTScreen

from vision_for_simulation.takeoff_rectangle import (
    detect_takeoff_rectangle,
)
from vision_for_simulation.terrain_ring import (
    detect_nearest_terrain_ring,
)
from vision_for_simulation.camera_offsets import _center_to_offset
from fleet_bus.air_node import attach_air_fleet_node
from fleet_bus.models import (
    AckReason,
    CommandId,
    SurveyFlags,
    SurveyState,
    TerrainCode,
)

# ============ 可调参数 ============
CRUISE_SPEED = 22            # 水平导航速度 cm/s
CRUISE_HEIGHT = 150          # 巡航高度 cm
VERTICAL_SPEED = 22          # 垂直速度 cm/s

# 起飞矩形视觉校准参数（与 2022_24.py 一致）
CALIB_CLOSE_THRESHOLD_PX = 30
CALIB_APPROACH_SPEED = 15
CALIB_FREQ = 10
CALIB_TIMEOUT = 60

# 摄像头（Sonix 0c45:636b, /dev/video2）
# 如需防枚举变化可创建 udev 规则:
#   SUBSYSTEM=="video4linux", ATTRS{idVendor}=="0c45", ATTRS{idProduct}=="636b",
#   KERNEL=="video2", SYMLINK+="video_survey"
# 然后将 CAMERA_INDEX 改为 "/dev/video_survey" 即可
CAMERA_INDEX: "Union[int, str]" = 2

# 地形环检测目标频率（实际频率受单次 YOLO 推理耗时限制）
RING_DETECT_FREQ = 10         # Hz
RING_WARMUP_ITERATIONS = 2
RING_READY_FRAMES = 2
RING_READY_TIMEOUT = 5.0      # s
VISION_FRAME_MAX_AGE = 0.5    # s
CAMERA_FIRST_FRAME_TIMEOUT = 8.0  # s
CAMERA_READ_FAILURE_LOG_INTERVAL = 2.0  # s

# Terrain YOLO only sees the centered 60% x 60% region. Keeping both ratios
# equal preserves the camera aspect ratio. Takeoff calibration and recording
# continue to use the complete camera frame.
RING_MODEL_CROP_WIDTH_RATIO = 0.60
RING_MODEL_CROP_HEIGHT_RATIO = 0.60

# 飞行画面录像：编码在独立线程中执行，队列满时保留最新帧，避免阻塞采集。
VIDEO_RECORD_ENABLED = True
VIDEO_RECORD_FPS = 10.0
VIDEO_RECORD_FOURCC = "MJPG"
VIDEO_RECORD_QUEUE_SIZE = 2
VIDEO_RECORD_MAX_SECONDS = 15 * 60
VIDEO_RECORD_JOIN_TIMEOUT = 3.0

# YOLO 仿真模型 7 类地形:
#   0:snow_mountain  1:field  2:river  3:settlements
#   4:lake           5:debris_flow(泥石流)  6:wildfire

# 泥石流打断像素距离阈值（偏移 < 75px 才触发）
DEBRIS_FLOW_PX_THRESH = 75    # px
DEBRIS_FLOW_CONFIRM_FRAMES = 2

# 航点标签确认：只统计独立推理帧，至少 2 帧且多数占比不低于 60%
SURVEY_MIN_CONFIRM_FRAMES = 2
SURVEY_MIN_CONFIRM_RATIO = 0.60
TRAJECTORY_FINISH_TIMEOUT = 45.0  # s
TRAJECTORY_ENDPOINT_THRESHOLD = 15.0  # cm

# 测绘网格坐标 → 行列映射
# 行 (3): x=100→0, x=170→1, x=240→2
# 列 (5): y=-40→0, y=-110→1, y=-180→2, y=-250→3, y=-320→4
SURVEY_X_TO_ROW: Dict[int, int] = {100: 0, 170: 1, 240: 2}
SURVEY_Y_TO_COL: Dict[int, int] = {-40: 0, -110: 1, -180: 2, -250: 3, -320: 4}
FIELD_TAKEOFF_CENTRE_CM = (75, 75)
SURVEY_CELL_POSITIONS_CM = tuple(
    (
        FIELD_TAKEOFF_CENTRE_CM[0] - local_y,
        FIELD_TAKEOFF_CENTRE_CM[1] + local_x,
    )
    for local_x in (100, 170, 240)
    for local_y in (-40, -110, -180, -250, -320)
)
TERRAIN_LABEL_TO_CODE = {
    "snow_mountain": int(TerrainCode.SNOW_MOUNTAIN),
    "field": int(TerrainCode.FIELD),
    "river": int(TerrainCode.RIVER),
    "settlements": int(TerrainCode.SETTLEMENTS),
    "lake": int(TerrainCode.LAKE),
    "debris_flow": int(TerrainCode.DEBRIS_FLOW),
    "wildfire": int(TerrainCode.WILDFIRE),
}
# =================================


@dataclass(frozen=True)
class RingObservation:
    frame_seq: int
    captured_at: float
    inferred_at: float
    label: Optional[str]
    confidence: float
    offset_x: float
    offset_y: float


def _center_crop(
    frame: np.ndarray,
    width_ratio: float,
    height_ratio: float,
) -> Tuple[np.ndarray, int, int]:
    """Return a centered crop and its ``(left, top)`` origin."""
    if frame is None or frame.ndim < 2:
        raise ValueError("frame must be a non-empty image")
    if frame.shape[0] == 0 or frame.shape[1] == 0:
        raise ValueError("frame must not be empty")
    if not 0.0 < width_ratio <= 1.0 or not 0.0 < height_ratio <= 1.0:
        raise ValueError("crop ratios must be in (0, 1]")

    frame_height, frame_width = frame.shape[:2]
    crop_width = max(1, min(frame_width, int(round(frame_width * width_ratio))))
    crop_height = max(
        1,
        min(frame_height, int(round(frame_height * height_ratio))),
    )
    left = (frame_width - crop_width) // 2
    top = (frame_height - crop_height) // 2
    return frame[top:top + crop_height, left:left + crop_width], left, top


def _open_persistent_camera(
    index: Union[int, str],
    width: int = 1280,
    height: int = 720,
) -> cv2.VideoCapture:
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


class SimVisionTask:
    """仿真视觉任务，持有一个常开 /dev/video0。

    同时承担起飞矩形检测（起飞后校准用）和地形环检测（巡航中避障/动作触发用）。
    """

    def __init__(self, camera_index: Union[int, str] = CAMERA_INDEX):
        self._cap: Optional[cv2.VideoCapture] = None
        self._camera_index = camera_index
        self._capture_stop = threading.Event()
        self._capture_condition = threading.Condition()
        self._capture_thread: Optional[threading.Thread] = None
        self._latest_frame: Optional[np.ndarray] = None
        self._latest_frame_seq = 0
        self._latest_frame_time = 0.0
        self._takeoff_frame_seq = 0
        self._capture_error: Optional[str] = None
        self._capture_read_failures = 0
        self._capture_started_at = 0.0
        self._record_stop = threading.Event()
        self._record_queue = queue.Queue(maxsize=VIDEO_RECORD_QUEUE_SIZE)
        self._record_thread: Optional[threading.Thread] = None
        self._record_video_path: Optional[str] = None
        self._record_index_path: Optional[str] = None
        self._record_dropped_frames = 0
        self._record_error: Optional[str] = None

    def open(self):
        if self._cap is None:
            self._cap = _open_persistent_camera(self._camera_index)
            self._capture_stop.clear()
            self._record_stop.clear()
            self._record_queue = queue.Queue(maxsize=VIDEO_RECORD_QUEUE_SIZE)
            self._record_dropped_frames = 0
            self._record_error = None
            with self._capture_condition:
                self._latest_frame = None
                self._latest_frame_seq = 0
                self._latest_frame_time = 0.0
                self._takeoff_frame_seq = 0
                self._capture_error = None
                self._capture_read_failures = 0
                self._capture_started_at = time.perf_counter()
            if VIDEO_RECORD_ENABLED:
                output_dir = os.path.join(
                    os.path.dirname(os.path.abspath(__file__)),
                    "vision_for_simulation",
                )
                os.makedirs(output_dir, exist_ok=True)
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
                basename = f"camera_recording_{timestamp}"
                self._record_video_path = os.path.join(
                    output_dir,
                    f"{basename}.avi",
                )
                self._record_index_path = os.path.join(
                    output_dir,
                    f"{basename}.csv",
                )
                self._record_thread = threading.Thread(
                    target=self._record_loop,
                    name="simulation-camera-recorder",
                    daemon=True,
                )
                self._record_thread.start()
            self._capture_thread = threading.Thread(
                target=self._capture_loop,
                name="simulation-camera-capture",
                daemon=True,
            )
            self._capture_thread.start()
            logger.info(f"[VISION] Camera {self._camera_index} opened")

    def close(self):
        self.stop()
        if (
            self._capture_thread is not None
            and self._capture_thread is not threading.current_thread()
        ):
            self._capture_thread.join(timeout=1.0)
        self._record_stop.set()
        if (
            self._record_thread is not None
            and self._record_thread is not threading.current_thread()
        ):
            self._record_thread.join(timeout=VIDEO_RECORD_JOIN_TIMEOUT)
            if self._record_thread.is_alive():
                logger.warning("[VISION] Camera recorder did not stop in time")
        if self._cap is not None:
            self._cap.release()
            logger.info(f"[VISION] Camera {self._camera_index} released")
        if (
            self._capture_thread is not None
            and self._capture_thread is not threading.current_thread()
            and self._capture_thread.is_alive()
        ):
            self._capture_thread.join(timeout=0.5)
            if self._capture_thread.is_alive():
                logger.warning("[VISION] Camera capture thread did not stop in time")
        self._cap = None
        self._capture_thread = None
        self._record_thread = None

    def stop(self):
        self._capture_stop.set()
        self._record_stop.set()
        with self._capture_condition:
            self._capture_condition.notify_all()

    def _enqueue_recording_frame(
        self,
        frame: np.ndarray,
        frame_seq: int,
        captured_at: float,
        captured_wall_time: float,
    ) -> None:
        record_thread = self._record_thread
        if (
            not VIDEO_RECORD_ENABLED
            or record_thread is None
            or not record_thread.is_alive()
            or self._record_stop.is_set()
        ):
            return

        item = (
            frame_seq,
            captured_at,
            captured_wall_time,
            frame,
        )
        try:
            self._record_queue.put_nowait(item)
            return
        except queue.Full:
            pass

        try:
            self._record_queue.get_nowait()
            self._record_dropped_frames += 1
        except queue.Empty:
            pass

        try:
            self._record_queue.put_nowait(item)
        except queue.Full:
            self._record_dropped_frames += 1

    def _record_loop(self) -> None:
        video_writer = None
        index_file = None
        index_writer = None
        saved_frames = 0
        first_captured_at = None

        try:
            while (
                not self._record_stop.is_set()
                or not self._record_queue.empty()
            ):
                try:
                    (
                        frame_seq,
                        captured_at,
                        captured_wall_time,
                        frame,
                    ) = self._record_queue.get(timeout=0.1)
                except queue.Empty:
                    continue

                if first_captured_at is None:
                    first_captured_at = captured_at
                elif (
                    captured_at - first_captured_at
                    > VIDEO_RECORD_MAX_SECONDS
                ):
                    logger.warning(
                        "[VISION] Camera recording reached "
                        f"{VIDEO_RECORD_MAX_SECONDS}s limit"
                    )
                    break

                if video_writer is None:
                    if (
                        self._record_video_path is None
                        or self._record_index_path is None
                    ):
                        raise RuntimeError("Camera recording paths are unset")
                    frame_height, frame_width = frame.shape[:2]
                    fourcc = cv2.VideoWriter_fourcc(
                        *VIDEO_RECORD_FOURCC
                    )
                    video_writer = cv2.VideoWriter(
                        self._record_video_path,
                        fourcc,
                        VIDEO_RECORD_FPS,
                        (frame_width, frame_height),
                    )
                    if not video_writer.isOpened():
                        raise RuntimeError(
                            "Unable to open camera video writer: "
                            f"{self._record_video_path}"
                        )
                    index_file = open(
                        self._record_index_path,
                        "w",
                        newline="",
                        encoding="utf-8",
                    )
                    index_writer = csv.writer(index_file)
                    index_writer.writerow(
                        (
                            "video_frame_index",
                            "camera_frame_seq",
                            "captured_wall_time",
                            "captured_at_monotonic",
                        )
                    )
                    logger.info(
                        "[VISION] Camera recording started: "
                        f"{self._record_video_path}"
                    )

                video_writer.write(frame)
                if index_writer is not None:
                    index_writer.writerow(
                        (
                            saved_frames,
                            frame_seq,
                            datetime.fromtimestamp(
                                captured_wall_time
                            ).isoformat(timespec="milliseconds"),
                            f"{captured_at:.9f}",
                        )
                    )
                saved_frames += 1
                if index_file is not None and saved_frames % 10 == 0:
                    index_file.flush()
        except Exception as exc:
            self._record_error = f"{type(exc).__name__}: {exc}"
            logger.exception(
                f"[VISION] Camera recording failed: {self._record_error}"
            )
        finally:
            if video_writer is not None:
                video_writer.release()
            if index_file is not None:
                index_file.flush()
                index_file.close()
            logger.info(
                "[VISION] Camera recording stopped "
                f"(saved={saved_frames}, "
                f"dropped={self._record_dropped_frames})"
            )

    def _capture_loop(self) -> None:
        """唯一读取 VideoCapture 的线程；始终只保留最新一帧。"""
        last_failure_log_at = time.perf_counter()
        while not self._capture_stop.is_set():
            cap = self._cap
            if cap is None:
                break
            try:
                ok, frame = cap.read()
            except Exception as exc:
                error = f"{type(exc).__name__}: {exc}"
                with self._capture_condition:
                    self._capture_error = error
                    self._capture_condition.notify_all()
                logger.exception(f"[VISION] Camera read raised an exception: {error}")
                return
            if not ok or frame is None:
                now = time.perf_counter()
                with self._capture_condition:
                    self._capture_read_failures += 1
                    failures = self._capture_read_failures
                if now - last_failure_log_at >= CAMERA_READ_FAILURE_LOG_INTERVAL:
                    logger.warning(
                        f"[VISION] Camera {self._camera_index} returned no frame "
                        f"({failures} failed reads)"
                    )
                    last_failure_log_at = now
                self._capture_stop.wait(0.05)
                continue
            captured_at = time.perf_counter()
            captured_wall_time = time.time()
            with self._capture_condition:
                first_frame = self._latest_frame_seq == 0
                self._latest_frame = frame
                self._latest_frame_seq += 1
                frame_seq = self._latest_frame_seq
                self._latest_frame_time = captured_at
                self._capture_condition.notify_all()
            self._enqueue_recording_frame(
                frame,
                frame_seq,
                captured_at,
                captured_wall_time,
            )
            if first_frame:
                logger.info(
                    f"[VISION] Camera {self._camera_index} first frame ready "
                    f"in {captured_at - self._capture_started_at:.2f}s "
                    f"({frame.shape[1]}x{frame.shape[0]})"
                )

    @property
    def latest_frame_seq(self) -> int:
        with self._capture_condition:
            return self._latest_frame_seq

    def _get_latest_frame(
        self,
        after_seq: int = 0,
        timeout: float = 0.5,
        max_age: float = VISION_FRAME_MAX_AGE,
    ) -> Optional[Tuple[np.ndarray, int, float]]:
        deadline = time.perf_counter() + max(0.0, timeout)
        with self._capture_condition:
            while (
                self._latest_frame_seq <= after_seq
                and not self._capture_stop.is_set()
                and self._capture_error is None
            ):
                remaining = deadline - time.perf_counter()
                if remaining <= 0:
                    return None
                self._capture_condition.wait(remaining)

            if self._latest_frame is None or self._latest_frame_seq <= after_seq:
                return None
            if time.perf_counter() - self._latest_frame_time > max_age:
                return None
            return (
                self._latest_frame.copy(),
                self._latest_frame_seq,
                self._latest_frame_time,
            )

    def warm_up_ring_detector(self, iterations: int = RING_WARMUP_ITERATIONS) -> bool:
        """起飞前用新鲜实拍帧加载并预热 YOLO；不采纳检测结果。"""
        last_seq = 0
        for index in range(max(1, iterations)):
            frame_timeout = (
                CAMERA_FIRST_FRAME_TIMEOUT if index == 0 else 2.0
            )
            snapshot = self._get_latest_frame(
                after_seq=last_seq,
                timeout=frame_timeout,
            )
            if snapshot is None:
                with self._capture_condition:
                    capture_error = self._capture_error
                    read_failures = self._capture_read_failures
                    capture_thread_alive = (
                        self._capture_thread is not None
                        and self._capture_thread.is_alive()
                    )
                logger.error(
                    "[VISION] No fresh camera frame for YOLO warm-up "
                    f"(iteration={index + 1}, "
                    f"timeout={frame_timeout:.1f}s, "
                    f"capture_thread_alive={capture_thread_alive}, "
                    f"failed_reads={read_failures}, "
                    f"capture_error={capture_error!r})"
                )
                return False
            frame, last_seq, _ = snapshot
            model_frame, _, _ = _center_crop(
                frame,
                RING_MODEL_CROP_WIDTH_RATIO,
                RING_MODEL_CROP_HEIGHT_RATIO,
            )
            t0 = time.perf_counter()
            try:
                detect_nearest_terrain_ring(model_frame)
            except Exception as exc:
                logger.exception(f"[VISION] YOLO warm-up failed: {exc}")
                return False
            logger.info(
                f"[VISION] YOLO warm-up {index + 1}/{max(1, iterations)} "
                f"done in {time.perf_counter() - t0:.2f}s "
                f"(input={model_frame.shape[1]}x{model_frame.shape[0]})"
            )
        return True

    def detect_takeoff_offset(self) -> Optional[Tuple[float, float]]:
        """检测起飞矩形，返回 ``(x_px, y_px)`` 或 None。"""
        snapshot = self._get_latest_frame(after_seq=self._takeoff_frame_seq)
        if snapshot is None:
            return None
        frame, self._takeoff_frame_seq, _ = snapshot
        detection = detect_takeoff_rectangle(frame)
        if detection is None:
            return None
        h, w = int(frame.shape[0]), int(frame.shape[1])
        return _center_to_offset(detection.center, (h, w))

    def detect_ring_observation(
        self, after_seq: int
    ) -> Optional[RingObservation]:
        """对 ``after_seq`` 之后的最新帧推理，并保留采集时序信息。"""
        snapshot = self._get_latest_frame(after_seq=after_seq)
        if snapshot is None:
            return None
        frame, frame_seq, captured_at = snapshot
        model_frame, crop_left, crop_top = _center_crop(
            frame,
            RING_MODEL_CROP_WIDTH_RATIO,
            RING_MODEL_CROP_HEIGHT_RATIO,
        )
        detection = detect_nearest_terrain_ring(model_frame)
        if detection is None:
            return RingObservation(
                frame_seq=frame_seq,
                captured_at=captured_at,
                inferred_at=time.perf_counter(),
                label=None,
                confidence=0.0,
                offset_x=0.0,
                offset_y=0.0,
            )
        h, w = int(frame.shape[0]), int(frame.shape[1])
        full_frame_center = (
            detection.center[0] + crop_left,
            detection.center[1] + crop_top,
        )
        offset_x, offset_y = _center_to_offset(full_frame_center, (h, w))
        return RingObservation(
            frame_seq=frame_seq,
            captured_at=captured_at,
            inferred_at=time.perf_counter(),
            label=detection.class_name,
            confidence=detection.confidence,
            offset_x=offset_x,
            offset_y=offset_y,
        )


class Mission(object):
    """2026 模拟赛 — 空地协同测绘救灾系统。

    第一段: 逐个航点导航 + 测绘记录，泥石流 (debris_flow) 像素距离 < 75px 时打断。
    第二段: 泥石流动作后以 smooth 轨迹遍历剩余航点，继续记录 label。
    泥石流整次飞行最多触发一次。
    """

    _VISION_LOG_DIR = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "vision_for_simulation",
    )

    def __init__(self, *args, **kwargs):
        self.fc: FC_Like = kwargs["fc"]
        self.navi: Navigation = kwargs["navi"]
        self.radar: LD_Radar = kwargs["radar"]
        self.rs: T265 = kwargs["rs"]
        self.sim_vision: Optional[SimVisionTask] = kwargs.get("sim_vision", None)
        self.cruise_height = CRUISE_HEIGHT

        # 坐标原点（视觉校准后设置）
        self._origin_x: float = 0.0
        self._origin_y: float = 0.0
        self._origin_ready = False

        # 3×5 测绘网格
        self._survey_grid: List[List[Optional[str]]] = [
            [None, None, None, None, None],
            [None, None, None, None, None],
            [None, None, None, None, None],
        ]
        self._survey_lock = threading.Lock()
        self._survey_revision = 0
        self._survey_complete = False
        self._next_disaster_event_id = 1
        self._wildfire_event = (0, 0xFF, 0xFF)
        self._debris_event = (0, 0xFF, 0xFF)
        self._reported_disasters = set()
        self._indicator_lock = threading.Lock()

        # 后台检测线程持续更新的最新 label
        self._latest_ring_label: Optional[str] = None
        self._latest_ring_observation: Optional[RingObservation] = None
        self._ring_observation_lock = threading.Lock()

        # 泥石流打断 — 单次飞行仅触发一次
        self._debris_flow_triggered_once: bool = False
        self._debris_flow_wp_index: int = -1
        self._ring_triggered = threading.Event()
        self._ring_label: str = ""
        self._debris_flow_confirm_count = 0

        # 线程控制
        self._stop_ring = threading.Event()
        self._ring_thread: Optional[threading.Thread] = None
        self._ring_ready = threading.Event()
        self._ring_failed = threading.Event()
        self._ring_actions_enabled = threading.Event()
        self._ring_start_frame_seq = 0

        # 视觉专用日志文件（独立于 loguru，无需改环境变量）
        os.makedirs(self._VISION_LOG_DIR, exist_ok=True)
        self._vision_log_path = os.path.join(
            self._VISION_LOG_DIR,
            f"ring_detection_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log",
        )
        self._vision_log_file = None  # TextIO | None, opened lazily by _tvlog
        self._vision_log_lock = threading.Lock()

    def _tvlog(self, msg: str) -> None:
        """写入视觉专用日志文件（带毫秒时间戳，线程安全）。"""
        ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        line = f"{ts} {msg}\n"
        with self._vision_log_lock:
            if self._vision_log_file is None:
                self._vision_log_file = open(self._vision_log_path, "a", encoding="utf-8")
            self._vision_log_file.write(line)
            self._vision_log_file.flush()

    def _tvlog_close(self) -> None:
        """关闭视觉日志文件。"""
        with self._vision_log_lock:
            if self._vision_log_file is not None:
                self._vision_log_file.close()
                self._vision_log_file = None

    def stop(self):
        self._stop_ring.set()
        self._ring_actions_enabled.clear()
        if (
            self._ring_thread is not None
            and self._ring_thread is not threading.current_thread()
        ):
            self._ring_thread.join(timeout=2.0)
            if self._ring_thread.is_alive():
                logger.warning("[RING] Detection thread did not stop in time")
        if self.sim_vision is not None:
            self.sim_vision.stop()
        self.navi.stop()
        self._tvlog_close()
        logger.info("[MISSION] Mission stopped")

    def navigation_pose_to_field(
        self, navigation_x_cm: float, navigation_y_cm: float
    ) -> Optional[Tuple[float, float]]:
        """将导航坐标转换为 480×400 cm 场地绝对坐标。"""
        if not self._origin_ready:
            return None
        relative_x = navigation_x_cm - self._origin_x
        relative_y = navigation_y_cm - self._origin_y
        return (
            FIELD_TAKEOFF_CENTRE_CM[0] - relative_y,
            FIELD_TAKEOFF_CENTRE_CM[1] + relative_x,
        )

    # ================================================================
    #  起飞矩形视觉校准
    # ================================================================

    def _calibrate_to_takeoff_rectangle(
        self,
        close_threshold_px: float = CALIB_CLOSE_THRESHOLD_PX,
        approach_speed: float = CALIB_APPROACH_SPEED,
        freq: float = CALIB_FREQ,
        timeout: float = CALIB_TIMEOUT,
    ) -> bool:
        if self.sim_vision is None:
            logger.error("[CALIB] sim_vision is None")
            return False

        dt = 1.0 / max(freq, 5)
        logger.info(
            f"[CALIB] thresh={close_threshold_px}px, speed={approach_speed}cm/s, "
            f"freq={freq}Hz, timeout={timeout}s"
        )
        t0 = time.perf_counter()

        while True:
            if time.perf_counter() - t0 > timeout:
                self.navi.stop_move()
                logger.warning("[CALIB] Timeout")
                return False

            offset = self.sim_vision.detect_takeoff_offset()
            if offset is None:
                self.navi.stop_move()
                time.sleep(dt)
                continue

            x_px, y_px = offset
            dist_px = float(np.hypot(x_px, y_px))

            if dist_px <= close_threshold_px:
                self.navi.stop_move()
                logger.info(f"[CALIB] Centered (dist={dist_px:.1f}px)")
                return True

            angle_deg = float(np.rad2deg(np.arctan2(y_px, x_px)))
            self.navi.move_by_direction(speed=approach_speed, direction_deg=angle_deg)
            time.sleep(dt)

    # ================================================================
    #  地形环后台检测（目标 10 Hz）
    # ================================================================

    def _start_ring_detection(self) -> bool:
        """校准后启动检测线程，并在开始轨迹前等待新帧推理就绪。"""
        if self.sim_vision is None:
            return False
        self._ring_start_frame_seq = self.sim_vision.latest_frame_seq
        with self._ring_observation_lock:
            self._latest_ring_observation = None
            self._latest_ring_label = None
        self._debris_flow_confirm_count = 0
        self._stop_ring.clear()
        self._ring_ready.clear()
        self._ring_failed.clear()
        self._ring_actions_enabled.clear()
        self._ring_thread = threading.Thread(
            target=self._ring_detection_loop,
            name="terrain-ring-detection",
            daemon=True,
        )
        self._ring_thread.start()
        logger.info(
            f"[RING] Detection thread started after frame "
            f"{self._ring_start_frame_seq}"
        )

        deadline = time.perf_counter() + RING_READY_TIMEOUT
        while time.perf_counter() < deadline:
            if self._ring_ready.wait(0.05):
                logger.info("[RING] Detection ready")
                return True
            if self._ring_failed.is_set():
                break

        self._stop_ring.set()
        if self._ring_thread is not None:
            self._ring_thread.join(timeout=1.0)
        logger.error(
            f"[RING] Detection not ready within {RING_READY_TIMEOUT:.1f}s"
        )
        return False

    def _get_latest_ring_observation(self) -> Optional[RingObservation]:
        with self._ring_observation_lock:
            return self._latest_ring_observation

    @staticmethod
    def _select_survey_label(
        observations: List[RingObservation],
    ) -> Optional[str]:
        """从独立推理帧中选择稳定标签；证据不足时返回 None。"""
        valid = [
            observation
            for observation in observations
            if observation.label in TERRAIN_LABEL_TO_CODE
        ]
        if len(valid) < SURVEY_MIN_CONFIRM_FRAMES:
            return None

        counts: Dict[str, int] = {}
        confidence_sums: Dict[str, float] = {}
        for observation in valid:
            label = str(observation.label)
            counts[label] = counts.get(label, 0) + 1
            confidence_sums[label] = (
                confidence_sums.get(label, 0.0) + observation.confidence
            )

        selected_label = max(
            counts,
            key=lambda label: (counts[label], confidence_sums[label]),
        )
        if counts[selected_label] / len(valid) < SURVEY_MIN_CONFIRM_RATIO:
            return None
        return selected_label

    def _collect_new_survey_observation(
        self,
        observations: List[RingObservation],
        after_seq: int,
    ) -> int:
        """最多追加一个尚未统计的最新独立推理结果。"""
        observation = self._get_latest_ring_observation()
        if observation is None or observation.frame_seq <= after_seq:
            return after_seq
        if observation.label in TERRAIN_LABEL_TO_CODE:
            observations.append(observation)
        return observation.frame_seq

    def _ring_detection_loop(self):
        """后台 daemon 线程：以目标频率检测地形环。

        - 每次只处理比上一结果更新的相机帧
        - 泥石流 (debris_flow) 仅在像素距离 < DEBRIS_FLOW_PX_THRESH 且
          整次飞行未触发过时打断轨迹
        """
        dt = 1.0 / max(RING_DETECT_FREQ, 5)
        logger.info(f"[RING] Loop started ({RING_DETECT_FREQ} Hz)")
        last_frame_seq = self._ring_start_frame_seq
        processed_frames = 0
        next_deadline = time.perf_counter()

        while not self._stop_ring.is_set():
            if self.sim_vision is None:
                break

            try:
                observation = self.sim_vision.detect_ring_observation(
                    after_seq=last_frame_seq
                )
            except Exception as exc:
                logger.exception(f"[RING] Detection failed: {exc}")
                self._ring_failed.set()
                break

            if observation is None:
                continue

            last_frame_seq = observation.frame_seq
            processed_frames += 1
            with self._ring_observation_lock:
                self._latest_ring_observation = observation
                self._latest_ring_label = observation.label

            frame_age_ms = (
                observation.inferred_at - observation.captured_at
            ) * 1000.0
            if observation.label is not None:
                self._tvlog(
                    f"RING frame={observation.frame_seq}: "
                    f"label={observation.label} conf={observation.confidence:.2f} "
                    f"offset=(x={observation.offset_x:.0f}, "
                    f"y={observation.offset_y:.0f})px age={frame_age_ms:.0f}ms"
                )

                # 泥石流打断：只在巡航动作启用后接受连续独立帧确认。
                dist_px = float(
                    np.hypot(observation.offset_x, observation.offset_y)
                )
                debris_candidate = (
                    self._ring_actions_enabled.is_set()
                    and observation.label == "debris_flow"
                    and dist_px < DEBRIS_FLOW_PX_THRESH
                    and not self._debris_flow_triggered_once
                )
                if debris_candidate:
                    self._debris_flow_confirm_count += 1
                    if (
                        self._debris_flow_confirm_count
                        >= DEBRIS_FLOW_CONFIRM_FRAMES
                        and not self._ring_triggered.is_set()
                    ):
                        logger.info(
                            f"[RING] debris_flow dist={dist_px:.1f}px < "
                            f"{DEBRIS_FLOW_PX_THRESH}px confirmed by "
                            f"{self._debris_flow_confirm_count} frames → interrupting"
                        )
                        self._ring_label = observation.label
                        self._ring_triggered.set()
                        self.navi.traj_running_event.clear()
                else:
                    self._debris_flow_confirm_count = 0
            else:
                self._debris_flow_confirm_count = 0
                self._tvlog(
                    f"RING frame={observation.frame_seq}: "
                    f"(none) age={frame_age_ms:.0f}ms"
                )

            if processed_frames >= RING_READY_FRAMES:
                self._ring_ready.set()

            next_deadline += dt
            delay = next_deadline - time.perf_counter()
            if delay > 0:
                self._stop_ring.wait(delay)
            else:
                next_deadline = time.perf_counter()

        logger.info("[RING] Loop stopped")

    # ================================================================
    #  测绘网格记录
    # ================================================================

    def _record_survey_label(
        self, rel_x: float, rel_y: float, label: Optional[str] = None,
        sample_count: int = 0,
        flash_wildfire: bool = True,
    ) -> None:
        """将指定 label 记录到测绘网格；识别失败时保持该格为空。"""
        x_key = int(round(rel_x))
        y_key = int(round(rel_y))
        row = SURVEY_X_TO_ROW.get(x_key)
        col = SURVEY_Y_TO_COL.get(y_key)

        if row is None or col is None:
            logger.warning(f"[SURVEY] ({x_key}, {y_key}) not in grid, skip")
            return

        new_wildfire = False
        with self._survey_lock:
            selected_label = label
            if selected_label not in TERRAIN_LABEL_TO_CODE:
                if self._survey_grid[row][col] is not None:
                    self._survey_grid[row][col] = None
                    self._survey_revision = (self._survey_revision + 1) & 0xFFFF
                logger.warning(
                    f"[SURVEY] Unknown/empty terrain label at Grid[{row}][{col}], "
                    "keep blank"
                )
                return
            if self._survey_grid[row][col] != selected_label:
                self._survey_grid[row][col] = selected_label
                self._survey_revision = (self._survey_revision + 1) & 0xFFFF
            disaster_key = (selected_label, row, col)
            if (
                selected_label in ("wildfire", "debris_flow")
                and disaster_key not in self._reported_disasters
            ):
                event_id = self._next_disaster_event_id
                self._next_disaster_event_id = event_id % 0xFFFF + 1
                self._reported_disasters.add(disaster_key)
                if selected_label == "wildfire":
                    self._wildfire_event = (event_id, row, col)
                    new_wildfire = True
                else:
                    self._debris_event = (event_id, row, col)
                self._survey_revision = (self._survey_revision + 1) & 0xFFFF
        extra = f" (from {sample_count} samples)" if sample_count > 0 else ""
        logger.info(
            f"[SURVEY] Grid[{row}][{col}] (x={x_key}, y={y_key})"
            f" = {selected_label}{extra}"
        )
        if new_wildfire and flash_wildfire:
            threading.Thread(
                target=self._flash_indicator,
                args=((255, 0, 0),),
                name="wildfire-indicator",
                daemon=True,
            ).start()
        self._log_survey_grid()

    def _flash_indicator(
        self, color: Tuple[int, int, int], flashes: int = 4, interval: float = 0.20
    ) -> None:
        with self._indicator_lock:
            for _ in range(flashes):
                self.fc.set_indicator_led(*color)
                time.sleep(interval)
                self.fc.set_indicator_led(0, 0, 0)
                time.sleep(interval)
            self.fc.set_indicator_led(0, 255, 0)

    def _mark_survey_complete(self) -> bool:
        with self._survey_lock:
            if not self._survey_complete:
                self._survey_complete = True
                self._survey_revision = (self._survey_revision + 1) & 0xFFFF
            unknown_count = sum(
                label is None for row in self._survey_grid for label in row
            )
        if unknown_count:
            logger.warning(
                f"[SURVEY] Complete with {unknown_count} unrecognized cells; "
                "they remain UNKNOWN"
            )
        return True

    def get_survey_state(self) -> SurveyState:
        with self._survey_lock:
            codes = tuple(
                (
                    TERRAIN_LABEL_TO_CODE.get(
                        label,
                        int(TerrainCode.UNKNOWN),
                    )
                    if label is not None
                    else int(TerrainCode.UNKNOWN)
                )
                for row in self._survey_grid
                for label in row
            )
            wildfire_id, wildfire_row, wildfire_col = self._wildfire_event
            debris_id, debris_row, debris_col = self._debris_event
            return SurveyState(
                survey_revision=self._survey_revision,
                survey_flags=(
                    int(SurveyFlags.ABSOLUTE_POSITIONS)
                    | (int(SurveyFlags.COMPLETE) if self._survey_complete else 0)
                ),
                wildfire_event_id=wildfire_id,
                wildfire_row=wildfire_row,
                wildfire_col=wildfire_col,
                debris_event_id=debris_id,
                debris_row=debris_row,
                debris_col=debris_col,
                terrain_codes=codes,
                cell_positions_cm=SURVEY_CELL_POSITIONS_CM,
            )

    def _log_survey_grid(self) -> None:
        rows = []
        for r, row in enumerate(self._survey_grid):
            display = [(lbl if lbl is not None else "?") for lbl in row]
            rows.append(f"  row[{r}] (x={[100,170,240][r]}): {display}")
        logger.info(f"[SURVEY] Grid:\n" + "\n".join(rows))

    # ================================================================
    #  找最近航点
    # ================================================================

    def _find_nearest_survey_waypoint(
        self, raw_waypoints: List[Tuple[float, float]]
    ) -> int:
        """返回当前相对坐标最近测绘航点（不含降落点）在 raw_waypoints 中的索引。"""
        rel_x = self.navi.current_x - self._origin_x
        rel_y = self.navi.current_y - self._origin_y
        best_idx = -1
        best_dist2 = float("inf")
        for i, (wx, wy) in enumerate(raw_waypoints):
            if wx == 0.0 and wy == 0.0:
                continue  # 跳过降落点
            d2 = (rel_x - wx) ** 2 + (rel_y - wy) ** 2
            if d2 < best_dist2:
                best_dist2 = d2
                best_idx = i
        return best_idx

    # ================================================================
    #  泥石流动作
    # ================================================================

    def _action_debris_flow(self) -> None:
        """泥石流救灾动作序列。

        黄灯 → 降至 90cm → 关泵 → 等 5s → 回升 150cm → 绿灯。
        """
        fc = self.fc
        navi = self.navi

        logger.info("[ACTION:debris_flow] Start")

        # 1. 黄灯
        fc.set_indicator_led(255, 255, 0)
        # 2. 降至 90cm
        navi.set_height(90.0)
        ok = navi.wait_for_height(time_thres=0.5, height_thres=10, timeout=10)
        if not ok:
            logger.warning(f"[ACTION:debris_flow] Low height not reached")
        # 3. 关泵
        fc.set_digital_output(0, False)
        # 4. 等 5s
        time.sleep(5.0)
        # 5. 回升
        navi.set_height(self.cruise_height)
        ok = navi.wait_for_height(time_thres=0.5, height_thres=10, timeout=10)
        if not ok:
            logger.warning(f"[ACTION:debris_flow] Cruise height not reached")
        # 6. 绿灯
        fc.set_indicator_led(0, 255, 0)
        logger.info("[ACTION:debris_flow] Complete")

    # ================================================================
    #  平滑轨迹生成
    # ================================================================

    def _build_smooth_traj(
        self, waypoints: List[Tuple[float, float]]
    ) -> List[Tuple[float, ...]]:
        start_x = float(self.navi.current_x)
        start_y = float(self.navi.current_y)
        start_height = float(self.navi.current_height)
        traj_wps = np.vstack(([[start_x, start_y]], np.asarray(waypoints, dtype=float)))
        traj_list = self.navi.create_smooth_traj_list(
            waypoints=traj_wps,
            altitude=self.cruise_height,
        )
        fp = traj_list[0]
        traj_list[0] = (fp[0], fp[1], start_height)
        logger.info(
            f"[TRAJ] {len(traj_list)} points from "
            f"({start_x:.0f}, {start_y:.0f}) through {len(waypoints)} waypoints"
        )
        return traj_list

    def _wait_for_trajectory_finish(
        self,
        name: str,
        endpoint: Tuple[float, float],
        timeout: float = TRAJECTORY_FINISH_TIMEOUT,
    ) -> bool:
        """等待异步轨迹线程清除运行事件，并核验最终水平位置。"""
        logger.info(f"[TRAJ] Waiting for {name} trajectory to finish")
        deadline = time.perf_counter() + max(1.0, timeout)
        while self.navi.traj_running_event.is_set():
            if time.perf_counter() >= deadline:
                logger.error(
                    f"[TRAJ] {name} trajectory finish timeout "
                    f"({timeout:.1f}s)"
                )
                self.navi.traj_running_event.clear()
                self.navi.stop_move()
                return False
            time.sleep(0.05)

        dx = float(self.navi.current_x) - float(endpoint[0])
        dy = float(self.navi.current_y) - float(endpoint[1])
        distance = float(np.hypot(dx, dy))
        if distance > TRAJECTORY_ENDPOINT_THRESHOLD:
            logger.error(
                f"[TRAJ] {name} trajectory ended {distance:.1f}cm "
                f"from endpoint {endpoint}"
            )
            self.navi.stop_move()
            return False

        logger.info(
            f"[TRAJ] {name} trajectory finished "
            f"({distance:.1f}cm from endpoint)"
        )
        return True

    # ================================================================
    #  主任务
    # ================================================================

    def run(self):
        fc = self.fc
        navi = self.navi

        # ---- 航点（相对原点，cm）----
        # 3×5 蛇形扫描:
        #   x=100: -40 → -110 → -180 → -250 → -320
        #   x=170: -320 → -250 → -180 → -110 → -40
        #   x=240: -40 → -110 → -180 → -250 → -320
        #   终点: (0,0) 降落
        raw_waypoints: List[Tuple[float, float]] = [
            (100, -40),    # 0
            (100, -110),   # 1
            (100, -180),   # 2
            (100, -250),   # 3
            (100, -320),   # 4
            (170, -320),   # 5
            (170, -250),   # 6
            (170, -180),   # 7
            (170, -110),   # 8
            (170, -40),    # 9
            (240, -40),    # 10
            (240, -110),   # 11
            (240, -180),   # 12
            (240, -250),   # 13
            (240, -320),   # 14
            (0, 0),        # 15 降落点
        ]

        # ---- 起飞前视觉模型预热（此时尚未解锁或移动）----
        if (
            self.sim_vision is not None
            and not self.sim_vision.warm_up_ring_detector()
        ):
            raise RuntimeError("Terrain-ring detector warm-up failed")

        # ---- 导航参数 ----
        navi.set_navigation_speed(CRUISE_SPEED)
        navi.set_vertical_speed(VERTICAL_SPEED)

        # ---- 导航 ----
        navi.start()
        navi.switch_navigation_mode("fusion-ros")
        logger.info("[MISSION] Navigation started (fusion-ros)")
        navi.set_rs_speed_report(True, 2)

        # ---- 初始化 ----
        fc.set_action_log(False)
        fc.set_indicator_led(0, 255, 0)
        fc.set_action_log(True)
        logger.info("[MISSION] Mission Started")

        # ---- Cartographer 就绪等待（起飞前）----
        # 仿照 base_test.py: 先确保 Cartographer / TF 坐标可靠，再解锁起飞。
        CART_TIMEOUT = 30.0
        logger.info(f"[MISSION] Waiting Cartographer TF ({CART_TIMEOUT}s)...")
        t0 = time.perf_counter()
        while True:
            time.sleep(1)
            logger.info(f"[MISSION] current_point: {navi.current_point}")
            if navi.current_point[0] + navi.current_point[1] != 0:
                break
            if time.perf_counter() - t0 > CART_TIMEOUT:
                raise RuntimeError(f"Cartographer TF timeout ({CART_TIMEOUT}s)")
        logger.info(f"[MISSION] Cartographer TF ok ({time.perf_counter() - t0:.1f}s)")
        fc.set_indicator_led(0, 0, 0)

        # ---- 定点起飞 ----
        logger.info(f"[MISSION] Takeoff to {self.cruise_height}cm")
        navi.pointing_takeoff((0, 0), self.cruise_height)
        navi.set_yaw(0)
        navi.wait_for_yaw()
        time.sleep(0.5)

        # ---- 起飞矩形校准 ----
        if self.sim_vision is not None:
            if not self._calibrate_to_takeoff_rectangle():
                raise RuntimeError("Takeoff calibration failed")
        else:
            logger.warning("[MISSION] sim_vision unavailable")

        # ---- 坐标原点 ----
        self._origin_x = float(navi.current_x)
        self._origin_y = float(navi.current_y)
        self._origin_ready = True
        logger.info(f"[MISSION] Origin = ({self._origin_x:.1f}, {self._origin_y:.1f})")

        # 绝对坐标
        waypoints: List[Tuple[float, float]] = [
            (x + self._origin_x, y + self._origin_y) for (x, y) in raw_waypoints
        ]
        survey_waypoints = waypoints[:-1]   # 15 个测绘航点
        landing_wp = (0.0, 0.0)             # 降落点为 navi 原点

        # 校准结束后只接受此刻之后采集的新帧。检测就绪前保持原点悬停，
        # 不启动轨迹，避免模型过渡期与航点位置错位。
        if self.sim_vision is not None:
            if not self._start_ring_detection():
                raise RuntimeError("Terrain-ring detector did not become ready")

        # ============================================================
        #  第一段: 平滑轨迹巡航 + 航点测绘记录
        #
        #  以当前位置为起点，生成一条经过所有航点（含降落点）的
        #  整体平滑轨迹，异步执行。主线程 50 Hz 轮询当前位置，
        #  对每个测绘航点在 10 cm 半径内收集 label，离开时取众数记录。
        #
        #  泥石流打断条件同前: debis_flow 距离 < 75px 且本轮未触发过。
        #  打断后: navi.traj_running_event.clear() 停止轨迹，
        #          记录最近航点 → 执行动作 → 跳转到第二段接续。
        # ============================================================
        # 构造完整轨迹（测绘航点 + navi 原点降落），起点为当前位置
        traj_waypoints = survey_waypoints + [(0.0, 0.0)]
        traj_list = self._build_smooth_traj(traj_waypoints)

        # 启动异步轨迹
        navi.navigation_follow_trajectory(traj_list, wait=False)
        logger.info(f"[TRAJ] First-leg trajectory started (async), {len(traj_list)} pts")
        self._ring_actions_enabled.set()

        # 每航点只收集独立推理帧，避免 50 Hz 主循环重复统计同一结果。
        wp_observation_bufs: List[List[RingObservation]] = [
            [] for _ in range(len(survey_waypoints))
        ]
        wp_recorded = [False] * len(survey_waypoints)
        wp_entry_radius2 = 10.0 ** 2   # 进入 10 cm 半径开始收集
        wp_exit_radius2 = 18.0 ** 2    # 离开 18 cm 半径停止收集（迟滞防抖）
        next_wp = 0
        within_wp = False
        wp_observation_seq = 0
        debris_flow_hit = False

        while next_wp < len(survey_waypoints):
            time.sleep(0.02)  # 50 Hz

            if self._ring_failed.is_set():
                raise RuntimeError("Terrain-ring detector stopped unexpectedly")

            # 泥石流打断
            if self._ring_triggered.is_set():
                logger.info("[MISSION] Interrupted by debris_flow")
                navi.stop_move()
                time.sleep(0.2)

                # 若正处在航点收集区内，先记录该航点再打断
                if within_wp and not wp_recorded[next_wp]:
                    observations = wp_observation_bufs[next_wp]
                    wp_observation_seq = self._collect_new_survey_observation(
                        observations,
                        wp_observation_seq,
                    )
                    mode_label = self._select_survey_label(observations)
                    wx, wy = raw_waypoints[next_wp]
                    self._record_survey_label(
                        wx,
                        wy,
                        label=mode_label,
                        sample_count=len(observations),
                    )
                    wp_recorded[next_wp] = True
                    next_wp += 1

                nearest_idx = self._find_nearest_survey_waypoint(raw_waypoints)
                self._debris_flow_wp_index = nearest_idx
                self._debris_flow_triggered_once = True
                logger.info(
                    f"[MISSION] Nearest survey WP index = {nearest_idx} "
                    f"({raw_waypoints[nearest_idx]})"
                )
                debris_x, debris_y = raw_waypoints[nearest_idx]
                self._record_survey_label(
                    debris_x, debris_y, label="debris_flow"
                )
                self._flash_indicator((255, 255, 0))
                self._action_debris_flow()
                self._ring_triggered.clear()
                debris_flow_hit = True
                break

            # 轨迹异常终止
            if not navi.traj_running_event.is_set() and navi.traj_progress < 0.99:
                logger.warning("[TRAJ] First-leg trajectory stopped early")
                break

            wx, wy = raw_waypoints[next_wp]
            abs_wx = wx + self._origin_x
            abs_wy = wy + self._origin_y
            dx = float(navi.current_x) - abs_wx
            dy = float(navi.current_y) - abs_wy
            dist2 = dx * dx + dy * dy

            if not within_wp:
                # 进入航点收集区
                if dist2 <= wp_entry_radius2:
                    within_wp = True
                    wp_observation_bufs[next_wp].clear()
                    if self.sim_vision is not None:
                        wp_observation_seq = self.sim_vision.latest_frame_seq
                    else:
                        wp_observation_seq = 0
                    self._tvlog(
                        f"WP {next_wp} ENTER zone ({wx:.0f}, {wy:.0f}) "
                        f"after_frame={wp_observation_seq}"
                    )
            else:
                # 在收集区内只追加尚未统计的新推理帧。
                observations = wp_observation_bufs[next_wp]
                wp_observation_seq = self._collect_new_survey_observation(
                    observations,
                    wp_observation_seq,
                )

                # 离开收集区：用独立帧的稳定多数结果记录。
                if dist2 > wp_exit_radius2:
                    mode_label = self._select_survey_label(observations)
                    self._tvlog(
                        f"WP {next_wp} EXIT zone ({wx:.0f}, {wy:.0f}) "
                        f"samples={len(observations)} consensus={mode_label}"
                    )
                    self._record_survey_label(
                        wx,
                        wy,
                        label=mode_label,
                        sample_count=len(observations),
                    )
                    wp_recorded[next_wp] = True
                    next_wp += 1
                    within_wp = False

        # 收尾: 等待轨迹线程真正清除运行事件，避免与降落轨迹并发。
        if not debris_flow_hit:
            if not self._wait_for_trajectory_finish(
                "first-leg",
                landing_wp,
            ):
                raise RuntimeError("First-leg trajectory did not reach landing point")

        # ============================================================
        #  第二段: 轨迹接续（仅泥石流打断后执行）
        #
        #  取泥石流航点之后的所有航点（不含泥石流航点），生成 smooth 轨迹。
        #  轨迹 with wait=False + 轮询到达检测 → 每到一个航点记录 label。
        # ============================================================
        if debris_flow_hit:
            resume_start = self._debris_flow_wp_index + 1
            if resume_start < len(survey_waypoints):
                resume_abs = waypoints[resume_start:len(survey_waypoints)] + [(0.0, 0.0)]
                raw_resume = raw_waypoints[
                    resume_start:len(survey_waypoints)
                ]  # 仅包含待记录的测绘航点
                logger.info(
                    f"[MISSION] Second leg: {len(resume_abs)} waypoints via smooth trajectory, "
                    f"starting from raw[{resume_start}]={raw_waypoints[resume_start]}"
                )

                traj_list = self._build_smooth_traj(resume_abs)

                # 异步启动轨迹，主线程轮询到达检测
                navi.navigation_follow_trajectory(traj_list, wait=False)
                logger.info("[TRAJ] Second-leg trajectory started (async)")

                next_rec = 0  # raw_resume 中下一个待记录的航点索引
                wp_arrival_thres2 = 15.0 ** 2  # 到达判定阈值 15cm
                wp_leave_thres2 = 22.0 ** 2
                rec_within_wp = False
                rec_observations: List[RingObservation] = []
                rec_observation_seq = 0
                while next_rec < len(raw_resume):
                    time.sleep(0.1)
                    if self._ring_failed.is_set():
                        raise RuntimeError(
                            "Terrain-ring detector stopped unexpectedly"
                        )
                    # 检查轨迹是否异常终止
                    if not navi.traj_running_event.is_set() and navi.traj_progress < 0.99:
                        logger.warning("[TRAJ] Second-leg trajectory stopped early")
                        break

                    wx, wy = raw_resume[next_rec]
                    abs_wx = wx + self._origin_x
                    abs_wy = wy + self._origin_y
                    dx = float(navi.current_x) - abs_wx
                    dy = float(navi.current_y) - abs_wy
                    dist2 = dx * dx + dy * dy

                    if (wx, wy) == (0.0, 0.0):
                        if dist2 <= wp_arrival_thres2:
                            logger.info("[SURVEY] Reached landing wp, skip grid record")
                            next_rec += 1
                        continue

                    if not rec_within_wp:
                        if dist2 <= wp_arrival_thres2:
                            rec_within_wp = True
                            rec_observations.clear()
                            if self.sim_vision is not None:
                                rec_observation_seq = (
                                    self.sim_vision.latest_frame_seq
                                )
                            else:
                                rec_observation_seq = 0
                            self._tvlog(
                                f"WP {resume_start + next_rec} ENTER zone "
                                f"({wx:.0f}, {wy:.0f}) "
                                f"after_frame={rec_observation_seq}"
                            )
                    else:
                        rec_observation_seq = (
                            self._collect_new_survey_observation(
                                rec_observations,
                                rec_observation_seq,
                            )
                        )
                        if dist2 > wp_leave_thres2:
                            selected_label = self._select_survey_label(
                                rec_observations
                            )
                            self._tvlog(
                                f"WP {resume_start + next_rec} EXIT zone "
                                f"({wx:.0f}, {wy:.0f}) "
                                f"samples={len(rec_observations)} "
                                f"consensus={selected_label}"
                            )
                            self._record_survey_label(
                                wx,
                                wy,
                                label=selected_label,
                                sample_count=len(rec_observations),
                            )
                            next_rec += 1
                            rec_within_wp = False

                if not self._wait_for_trajectory_finish(
                    "second-leg",
                    landing_wp,
                ):
                    raise RuntimeError(
                        "Second-leg trajectory did not reach landing point"
                    )
            else:
                logger.info("[MISSION] No remaining waypoints after debris_flow")

        # ---- 打印最终测绘网格 ----
        logger.info("=" * 50)
        logger.info("[MISSION] === Final Survey Grid ===")
        self._log_survey_grid()
        self._mark_survey_complete()
        logger.info("=" * 50)

        # ---- 停止地形环检测 ----
        self._ring_actions_enabled.clear()
        self._stop_ring.set()

        # ---- 降落 ----
        logger.info("[MISSION] Landing at origin")
        navi.pointing_landing(landing_wp)


# ================================================================
#  __main__
# ================================================================
if __name__ == "__main__":
    # ---- 1. 权限 ----
    rm = RosManager()
    rm.chmod("/dev/ttyUSB0")   # CP2102 雷达
    rm.chmod("/dev/ttyACM0")   # LX 飞控
    rm.chmod("/dev/video0")    # USB 摄像头

    # ---- 2. ROS 建图 ----
    rm.launch_package("ldlidar_stl_ros2", "ld19.launch.py")
    rm.launch_package("realsense2_camera", "rs_launch.py")
    rm.launch_package("cartographer_ros", "cartographer.launch.py")
    rm.run_package(
        "tf2_ros", "static_transform_publisher",
        "0 0 0 0 0 0 camera_pose_frame base_link"
    )

    # ---- 3. 飞控 ----
    fc = FC_Client()
    fc.connect()
    time.sleep(0.5)

    # ---- 4. 传感器 ----
    t265 = T265("ros")
    t265.start()
    radar = LD_Radar()
    radar.start("ros")
    screen = UARTScreen(fc)

    # ---- 5. 视觉 ----
    sim_vision = SimVisionTask(camera_index=CAMERA_INDEX)
    sim_vision.open()

    # ---- 6-7. 导航 ----
    mapper = RosMapper()
    navi = Navigation(fc=fc, rs=t265, radar=radar, mapper=mapper)
    RosNodeRunner().add_nodes().run()

    # ---- 8. Mission ----
    mission = Mission(
        fc=fc, rs=t265, radar=radar, navi=navi, sim_vision=sim_vision,
    )
    remote_stop_event = threading.Event()
    fleet_node = None
    start_command = None

    def wait_for_remote_stop():
        remote_stop_event.wait()
        logger.warning("[FLEET] Remote STOP received")
        mission.stop()

    # 地面站负责从屏幕点击时刻计满 10 秒，再发送 FleetBus START。
    # 本进程只在任务线程收到该命令后进入 mission.run()，STOP 始终可抢占。

    try:
        fleet_node = attach_air_fleet_node(
            fc,
            navi,
            remote_stop_event,
            readonly=True,
            allow_start_mission=True,
            survey_provider=mission.get_survey_state,
            position_transform=mission.navigation_pose_to_field,
            heading_offset_deg=90.0,
        )
        threading.Thread(
            target=wait_for_remote_stop,
            name="fleet-remote-stop",
            daemon=True,
        ).start()
        logger.info("[FLEET] Waiting for ground-station START")
        while not remote_stop_event.is_set():
            command = fleet_node.command_queue.receive(timeout=0.25)
            if command is None:
                continue
            if command.command_id == int(CommandId.DRONE_START_MISSION):
                start_command = command
                break
        if start_command is None:
            raise RuntimeError("Mission stopped before ground-station START")
        logger.info("[FLEET] Ground-station START accepted; running mission")
        mission.run()
        fleet_node.command_queue.complete(start_command)
        logger.info("[FLEET] Mission complete; keeping reports available for 5s")
        remote_stop_event.wait(5.0)
    except Exception as e:
        if fleet_node is not None and start_command is not None:
            fleet_node.command_queue.fail(
                start_command, int(AckReason.INTERNAL_ERROR)
            )
        logger.exception(f"[MANAGER] Mission Failed: {e}")
    finally:
        if fleet_node is not None:
            try:
                fleet_node.close()
            except Exception as e:
                logger.exception(f"[FLEET] Close failed: {e}")
        mission.stop()
        if fc.state.unlock.value:
            logger.warning("[MANAGER] Auto Landing (Emergency)")
            fc.set_flight_mode(fc.PROGRAM_MODE)
            fc.stablize()
            fc.land()
            ret = fc.wait_for_lock()
            if not ret:
                fc.lock()
        sim_vision.close()

    logger.info("[MANAGER] Mission finished")
    fc.close()
