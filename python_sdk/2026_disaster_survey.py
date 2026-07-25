"""
2026 模拟赛 — 空地协同测绘救灾系统

任务流程:
  1. 定点起飞 → Cartographer / TF 就绪
  2. 起飞矩形视觉校准（下视 /dev/video0，检测黑色矩形标记，视觉闭环居中）
  3. 记录校准点为坐标原点
  4. 逐个航点导航（3×5 蛇形，100/170/240 × -40/-110/-180/-250/-320），
     每到达一个测绘航点记录 YOLO 地形 label 到网格
  5. 巡航期间 10 Hz 检测地形环。泥石流(debris_flow)像素距离 < 50px
     时打断（整次飞行仅触发一次），记录当前最近航点，执行泥石流动作后
     以 smooth 轨迹接续所有后续航点（不含泥石流航点），
     到达后续航点时继续记录 label
  6. 完成全部航点后降落在坐标原点

上位机已自启动 server_ros.py，本程序通过 FC_Client 连接。
ROS 建图组件启动流程参考 base_test.py 与 former_code/2024_D_24.py。
视觉闭环校准流程参考 former_code/2022_24.py 的 vision_approach 模式。
地形检测使用 vision_for_simulation 仿真视觉包（YOLO + 传统图像处理）。
"""
import sys
import threading
import time
from typing import Dict, List, Optional, Tuple

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
from fleet_bus.models import SurveyFlags, SurveyState, TerrainCode

# ============ 可调参数 ============
CRUISE_SPEED = 22            # 水平导航速度 cm/s
CRUISE_HEIGHT = 150          # 巡航高度 cm
VERTICAL_SPEED = 22          # 垂直速度 cm/s

# 起飞矩形视觉校准参数（与 2022_24.py 一致）
CALIB_CLOSE_THRESHOLD_PX = 30
CALIB_APPROACH_SPEED = 15
CALIB_FREQ = 10
CALIB_TIMEOUT = 60

# 摄像头索引（校准和地形环共用 /dev/video0）
CAMERA_INDEX = 0

# 地形环检测频率
RING_DETECT_FREQ = 10         # Hz

# YOLO 仿真模型 7 类地形:
#   0:snow_mountain  1:field  2:river  3:settlements
#   4:lake           5:debris_flow(泥石流)  6:wildfire

# 泥石流打断像素距离阈值（偏移 < 50px 才触发）
DEBRIS_FLOW_PX_THRESH = 50    # px

# 测绘网格坐标 → 行列映射
# 行 (3): x=100→0, x=170→1, x=240→2
# 列 (5): y=-40→0, y=-110→1, y=-180→2, y=-250→3, y=-320→4
SURVEY_X_TO_ROW: Dict[int, int] = {100: 0, 170: 1, 240: 2}
SURVEY_Y_TO_COL: Dict[int, int] = {-40: 0, -110: 1, -180: 2, -250: 3, -320: 4}
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


def _open_persistent_camera(index: int, width: int = 1280, height: int = 720) -> cv2.VideoCapture:
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

    def __init__(self, camera_index: int = CAMERA_INDEX):
        self._cap: Optional[cv2.VideoCapture] = None
        self._camera_index = camera_index
        self._stop_flag = False

    def open(self):
        if self._cap is None:
            self._cap = _open_persistent_camera(self._camera_index)
            logger.info(f"[VISION] Camera {self._camera_index} opened")

    def close(self):
        if self._cap is not None:
            self._cap.release()
            logger.info(f"[VISION] Camera {self._camera_index} released")
        self._cap = None

    def stop(self):
        self._stop_flag = True

    def _read_frame(self) -> Optional[np.ndarray]:
        if self._cap is None:
            return None
        ok, frame = self._cap.read()
        if not ok or frame is None:
            return None
        return frame

    def detect_takeoff_offset(self) -> Optional[Tuple[float, float]]:
        """检测起飞矩形，返回 ``(x_px, y_px)`` 或 None。"""
        frame = self._read_frame()
        if frame is None:
            return None
        detection = detect_takeoff_rectangle(frame)
        if detection is None:
            return None
        h, w = int(frame.shape[0]), int(frame.shape[1])
        return _center_to_offset(detection.center, (h, w))

    def detect_ring_offset(self) -> Optional[Tuple[float, float, str]]:
        """YOLO 检测最近地形环，返回 ``(x_px, y_px, label)`` 或 None。"""
        frame = self._read_frame()
        if frame is None:
            return None
        detection = detect_nearest_terrain_ring(frame)
        if detection is None:
            return None
        h, w = int(frame.shape[0]), int(frame.shape[1])
        offset_x, offset_y = _center_to_offset(detection.center, (h, w))
        return offset_x, offset_y, detection.class_name


class Mission(object):
    """2026 模拟赛 — 空地协同测绘救灾系统。

    第一段: 逐个航点导航 + 测绘记录，泥石流 (debris_flow) 像素距离 < 50px 时打断。
    第二段: 泥石流动作后以 smooth 轨迹遍历剩余航点，继续记录 label。
    泥石流整次飞行最多触发一次。
    """

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

        # 泥石流打断 — 单次飞行仅触发一次
        self._debris_flow_triggered_once: bool = False
        self._debris_flow_wp_index: int = -1   # 打断时最近航点在 raw_waypoints 中的索引
        self._ring_triggered = threading.Event()
        self._ring_label: str = ""

        # 线程控制
        self._stop_ring = threading.Event()
        self._ring_thread: Optional[threading.Thread] = None

    def stop(self):
        self._stop_ring.set()
        if self.sim_vision is not None:
            self.sim_vision.stop()
        self.navi.stop()
        logger.info("[MISSION] Mission stopped")

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
                logger.debug("[CALIB] No rectangle detected this frame")
                self.navi.stop_move()
                time.sleep(dt)
                continue

            x_px, y_px = offset
            dist_px = float(np.hypot(x_px, y_px))
            logger.debug(
                f"[CALIB] rect offset=(x={x_px:.0f}, y={y_px:.0f})px, "
                f"dist={dist_px:.1f}px"
            )

            if dist_px <= close_threshold_px:
                self.navi.stop_move()
                logger.info(f"[CALIB] Centered (dist={dist_px:.1f}px)")
                return True

            angle_deg = float(np.rad2deg(np.arctan2(y_px, x_px)))
            self.navi.move_by_direction(speed=approach_speed, direction_deg=angle_deg)
            time.sleep(dt)

    # ================================================================
    #  地形环后台检测（10 Hz）
    # ================================================================

    def _ring_detection_loop(self):
        """后台 daemon 线程: 10 Hz 检测地形环。

        - 始终更新 _latest_ring_label（测绘记录用）
        - 泥石流 (debris_flow) 仅在像素距离 < DEBRIS_FLOW_PX_THRESH 且
          整次飞行未触发过时打断轨迹
        """
        dt = 1.0 / max(RING_DETECT_FREQ, 5)
        logger.info(f"[RING] Loop started ({RING_DETECT_FREQ} Hz)")

        while not self._stop_ring.is_set():
            if self.sim_vision is None:
                break

            offset = self.sim_vision.detect_ring_offset()
            if offset is not None:
                x_px, y_px, label = offset
                logger.debug(
                    f"[RING] frame: label={label}, "
                    f"offset=(x={x_px:.0f}, y={y_px:.0f})px"
                )
                self._latest_ring_label = label

                # 泥石流打断: 像素距离 < 50px 且本轮飞行未触发过
                if label == "debris_flow" and not self._debris_flow_triggered_once:
                    dist_px = float(np.hypot(x_px, y_px))
                    if dist_px < DEBRIS_FLOW_PX_THRESH and not self._ring_triggered.is_set():
                        logger.info(
                            f"[RING] debris_flow dist={dist_px:.1f}px < "
                            f"{DEBRIS_FLOW_PX_THRESH}px → interrupting"
                        )
                        self._ring_label = label
                        self._ring_triggered.set()
                        self.navi.traj_running_event.clear()
            else:
                logger.debug("[RING] No ring detected this frame")

            self._stop_ring.wait(dt)

        logger.info("[RING] Loop stopped")

    # ================================================================
    #  测绘网格记录
    # ================================================================

    def _record_survey_label(
        self, rel_x: float, rel_y: float, label: Optional[str] = None
    ) -> None:
        x_key = int(round(rel_x))
        y_key = int(round(rel_y))
        row = SURVEY_X_TO_ROW.get(x_key)
        col = SURVEY_Y_TO_COL.get(y_key)

        if row is None or col is None:
            logger.warning(
                f"[SURVEY] ({x_key}, {y_key}) not in grid, skip"
            )
            return

        new_wildfire = False
        with self._survey_lock:
            selected_label = self._latest_ring_label if label is None else label
            if selected_label not in TERRAIN_LABEL_TO_CODE:
                logger.warning(
                    f"[SURVEY] Unknown/empty terrain label at Grid[{row}][{col}], skip"
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
        logger.info(
            f"[SURVEY] Grid[{row}][{col}] (x={x_key}, y={y_key})"
            f" = {selected_label}"
        )
        if new_wildfire:
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
            if any(label is None for row in self._survey_grid for label in row):
                logger.error("[SURVEY] Grid is incomplete; not publishing COMPLETE")
                return False
            if not self._survey_complete:
                self._survey_complete = True
                self._survey_revision = (self._survey_revision + 1) & 0xFFFF
        return True

    def get_survey_state(self) -> SurveyState:
        with self._survey_lock:
            codes = tuple(
                TERRAIN_LABEL_TO_CODE.get(label, int(TerrainCode.UNKNOWN))
                for row in self._survey_grid
                for label in row
            )
            wildfire_id, wildfire_row, wildfire_col = self._wildfire_event
            debris_id, debris_row, debris_col = self._debris_event
            return SurveyState(
                survey_revision=self._survey_revision,
                survey_flags=(
                    int(SurveyFlags.COMPLETE) if self._survey_complete else 0
                ),
                wildfire_event_id=wildfire_id,
                wildfire_row=wildfire_row,
                wildfire_col=wildfire_col,
                debris_event_id=debris_id,
                debris_row=debris_row,
                debris_col=debris_col,
                terrain_codes=codes,
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
        logger.info(f"[MISSION] Origin = ({self._origin_x:.1f}, {self._origin_y:.1f})")

        # 绝对坐标
        waypoints: List[Tuple[float, float]] = [
            (x + self._origin_x, y + self._origin_y) for (x, y) in raw_waypoints
        ]
        survey_waypoints = waypoints[:-1]   # 15 个测绘航点
        landing_wp = waypoints[-1]           # 降落点

        # ---- 启动地形环后台检测 ----
        if self.sim_vision is not None:
            self._stop_ring.clear()
            self._ring_thread = threading.Thread(
                target=self._ring_detection_loop, daemon=True
            )
            self._ring_thread.start()

        # ============================================================
        #  第一段: 逐个航点导航 + 测绘记录
        #
        #  泥石流打断条件: debis_flow 且距离 < 50px 且本轮未触发过。
        #  打断后: 记录最近航点 → 执行动作 → 跳转到轨迹接续段。
        # ============================================================
        debris_flow_hit = False
        for i, wp in enumerate(survey_waypoints):
            rel_x = raw_waypoints[i][0]
            rel_y = raw_waypoints[i][1]
            logger.info(
                f"[MISSION] WP {i+1}/{len(survey_waypoints)}: "
                f"abs={wp} rel=({rel_x:.0f}, {rel_y:.0f})"
            )

            while True:
                success = navi.navigation_to_waypoint(wp)

                if self._ring_triggered.is_set():
                    # ---- 泥石流打断 ----
                    logger.info(f"[MISSION] Interrupted by debris_flow")
                    navi.stop_move()
                    time.sleep(0.2)

                    # 找最近测绘航点
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

                    # 执行泥石流动作
                    self._flash_indicator((255, 255, 0))
                    self._action_debris_flow()
                    self._ring_triggered.clear()
                    debris_flow_hit = True
                    break   # 跳出 while → 跳出 for 进入第二段

                elif not success:
                    raise RuntimeError(f"Nav to wp {i+1} {wp} failed")
                else:
                    self._record_survey_label(rel_x, rel_y)
                    time.sleep(0.3)
                    break

            if debris_flow_hit:
                break

        # ============================================================
        #  第二段: 轨迹接续（仅泥石流打断后执行）
        #
        #  取泥石流航点之后的所有航点（不含泥石流航点），生成 smooth 轨迹。
        #  轨迹 with wait=False + 轮询到达检测 → 每到一个航点记录 label。
        # ============================================================
        if debris_flow_hit:
            resume_start = self._debris_flow_wp_index + 1
            if resume_start < len(survey_waypoints):
                resume_abs = waypoints[resume_start:]  # 测绘航点 → … → 降落点
                raw_resume = raw_waypoints[resume_start:]  # 相对坐标用于记录
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
                while next_rec < len(raw_resume):
                    time.sleep(0.1)
                    # 检查轨迹是否异常终止
                    if not navi.traj_running_event.is_set() and navi.traj_progress < 0.99:
                        logger.warning("[TRAJ] Second-leg trajectory stopped early")
                        break

                    wx, wy = raw_resume[next_rec]
                    abs_wx = wx + self._origin_x
                    abs_wy = wy + self._origin_y
                    dx = float(navi.current_x) - abs_wx
                    dy = float(navi.current_y) - abs_wy
                    if dx * dx + dy * dy <= wp_arrival_thres2:
                        if (wx, wy) != (0.0, 0.0):
                            self._record_survey_label(wx, wy)
                        else:
                            logger.info("[SURVEY] Reached landing wp, skip grid record")
                        next_rec += 1

                # 等待轨迹线程完全结束
                logger.info("[TRAJ] Waiting for second-leg trajectory to finish")
                navi.traj_running_event.wait()
            else:
                logger.info("[MISSION] No remaining waypoints after debris_flow")

        # ---- 打印最终测绘网格 ----
        logger.info("=" * 50)
        logger.info("[MISSION] === Final Survey Grid ===")
        self._log_survey_grid()
        self._mark_survey_complete()
        logger.info("=" * 50)

        # ---- 停止地形环检测 ----
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

    def wait_for_remote_stop():
        remote_stop_event.wait()
        logger.warning("[FLEET] Remote STOP received")
        mission.stop()

    # TODO: 地面站起飞命令等待
    #   1. fc.set_digital_output(0, True)  — 打开水泵
    #   2. 参考 former_code/2024_D_24.py 的 FCWirelessTransport /
    #      start_ground_station / enable_ground_command_reception 流程，
    #      等待地面站通过飞控 UT2/HC-14 发送 START_MISSION 命令
    #   3. 收到命令后等 5s 再 mission.run()
    #   4. 完成后上报 COMPLETED / FAILED

    try:
        fleet_node = attach_air_fleet_node(
            fc,
            navi,
            remote_stop_event,
            readonly=True,
            survey_provider=mission.get_survey_state,
        )
        threading.Thread(
            target=wait_for_remote_stop,
            name="fleet-remote-stop",
            daemon=True,
        ).start()
        mission.run()
    except Exception as e:
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
