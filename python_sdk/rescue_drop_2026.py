"""2026 空地协同救援物资投放赛无人机主入口。

默认仅连接飞控并监视 LIO 位姿；真实飞行还需 ``--confirm-flight``、
视觉与避障接口实现、投放数量、继电器端口和终端 ``start_mission`` 指令。
运行前确认 server_ros.py / FC_Server 未运行，避免抢占飞控串口。

导航坐标单位为 cm，x 向前、y 向左；高度单位为 cm。
本文件中的视觉、避障 TODO 未完成时会在连接硬件前拒绝飞行。
"""

import argparse
import math
import sys
import threading
import time
from dataclasses import dataclass
from enum import Enum
from typing import Dict, List, Optional, Sequence, Tuple

from loguru import logger

from FlightController import FC_Controller
from FlightController.Components import LD_Radar
from FlightController.Components.relay_lcus import LCUSRelay
from FlightController.Solutions.Navigation import Navigation


# 用户给定的航迹参数：x_1、x_2、y_l、y_r（cm）。
TRACK_START_OFFSET_X_CM = 50.0  # x_1：起飞点前方的航迹起点
TRACK_LENGTH_X_CM = 150.0       # x_2：航迹前后方向长度
LEFT_SPAN_Y_CM = 175.0          # y_l：航迹起点左侧范围
RIGHT_SPAN_Y_CM = 175.0         # y_r：航迹起点右侧范围
CENTER_INSET_X_CM = 80.0        # 航迹 3 折返到距航迹起点前方 80 cm

FC_SERIAL_DEV = "/dev/ttyACM0"
CRUISE_SPEED = 15.0
CRUISE_HEIGHT = 150.0
VERTICAL_SPEED = 22.0
FREE_DROP_HEIGHT = 80.0
MANDATORY_DROP_HEIGHT = 100.0
TAKEOFF_POINT = (0.0, 0.0)
LANDING_HEIGHT_TIMEOUT = 8.0
LIO_POSE_READY_TIMEOUT = 15.0
MONITOR_INTERVAL = 1.0

# 视觉初值沿用 former_code/2026_disaster_survey.py；超时和丢失等待由用户指定。
VISUAL_CENTER_THRESHOLD_PX = 30.0
VISUAL_APPROACH_SPEED = 15.0
VISUAL_PERIOD = 0.1
VISUAL_MAX_AGE = 0.5
TARGET_LOSS_WAIT = 2.0
LOW_CALIBRATION_TIMEOUT = 6.0
MISSION_TIMEOUT = 20.0 * 60.0

FREE_DROP_COUNT = 4
MANDATORY_DROP_COUNT = 1
TOTAL_DROP_COUNT = FREE_DROP_COUNT + MANDATORY_DROP_COUNT
RELAY_CHANNEL_COUNT = 8
FREE_COLORS = ("red", "blue", "green")
MANDATORY_COLOR = "yellow"

Point = Tuple[float, float]


class MissionDeadline(RuntimeError):
    """任务计时到期，停止搜索和投掷。"""


class MissionState(Enum):
    ROUTE_1 = "沿线飞行（右飞、前飞）"
    FREE_DROP = "自由投掷"
    CENTER = "中心避障"
    MANDATORY_DROP = "必投投掷"
    ROUTE_2 = "沿线飞行（后飞、右飞）"
    DONE = "完成"


@dataclass(frozen=True)
class TargetObservation:
    """视觉线程提供的观测；偏移 +x 向前、+y 向左，时间为 monotonic 秒。"""

    target_id: str
    color: str
    offset_x_px: float
    offset_y_px: float
    captured_at: float


@dataclass(frozen=True)
class RouteProjection:
    point: Point
    segment_index: int
    distance_squared: float


class VisionInterface:
    """TODO：在此文件接入相机、颜色目标识别和目标身份跟踪。"""

    IMPLEMENTED = False

    def open(self) -> None:
        raise NotImplementedError("TODO: open the target-detection camera")

    def poll(self) -> Sequence[TargetObservation]:
        """快速返回最新观测；不得向飞控发送命令或长时间阻塞。"""
        raise NotImplementedError("TODO: detect free and mandatory targets")

    def close(self) -> None:
        pass


class ObstacleInterface:
    """TODO：仅在中心航迹和必投接近、返回阶段提供避障结果。"""

    IMPLEMENTED = False

    def safe_waypoint(self, current: Point, goal: Point) -> Point:
        """返回通向 goal 的实时安全航点；无有效避障数据时必须报错。"""
        raise NotImplementedError("TODO: center-route obstacle avoidance")

    def safe_velocity(self, current: Point, velocity: Point) -> Point:
        """过滤必投视觉接近时的水平速度；无有效数据时必须报错。"""
        raise NotImplementedError("TODO: mandatory-target obstacle avoidance")


def validate_allocation(red: int, blue: int, green: int) -> Dict[str, int]:
    counts = {"red": red, "blue": blue, "green": green}
    if any(isinstance(value, bool) or not isinstance(value, int) or value < 0
           for value in counts.values()):
        raise ValueError("red/blue/green counts must be non-negative integers")
    if sum(counts.values()) != FREE_DROP_COUNT:
        raise ValueError("free-target counts must total 4")
    return counts


def build_routes() -> Tuple[List[Point], List[Point], List[Point]]:
    """返回导航绝对坐标中的航迹 1、3、2，保留所有直角拐点。"""
    origin_x = TRACK_START_OFFSET_X_CM
    span = LEFT_SPAN_Y_CM + RIGHT_SPAN_Y_CM
    route_1 = [
        (origin_x, 0.0),
        (origin_x, -RIGHT_SPAN_Y_CM),
        (origin_x + TRACK_LENGTH_X_CM, -RIGHT_SPAN_Y_CM),
    ]
    route_3 = [route_1[-1]]
    # 题目原式写作 (y, x)；此处转成 Navigation 使用的 (x, y)。
    for fraction, x in (
        (0.2, TRACK_LENGTH_X_CM), (0.2, CENTER_INSET_X_CM),
        (0.4, CENTER_INSET_X_CM), (0.4, TRACK_LENGTH_X_CM),
        (0.6, TRACK_LENGTH_X_CM), (0.6, CENTER_INSET_X_CM),
        (0.8, CENTER_INSET_X_CM), (0.8, TRACK_LENGTH_X_CM),
        (1.0, TRACK_LENGTH_X_CM),
    ):
        route_3.append((origin_x + x, fraction * span - RIGHT_SPAN_Y_CM))
    route_2 = [
        route_3[-1],
        (origin_x, LEFT_SPAN_Y_CM),
        (origin_x, 0.0),
    ]
    return route_1, route_3, route_2


def nearest_point_on_route(point: Point, route: Sequence[Point]) -> RouteProjection:
    """对航迹线段逐一投影；最多九段，计算量固定且很小。"""
    if len(route) < 2:
        raise ValueError("route must contain at least two points")
    best = None
    for index, (start, end) in enumerate(zip(route, route[1:])):
        dx, dy = end[0] - start[0], end[1] - start[1]
        length_squared = dx * dx + dy * dy
        if length_squared == 0:
            fraction = 0.0
        else:
            fraction = max(0.0, min(1.0,
                ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy)
                / length_squared))
        projected = (start[0] + fraction * dx, start[1] + fraction * dy)
        distance_squared = ((point[0] - projected[0]) ** 2
                            + (point[1] - projected[1]) ** 2)
        candidate = RouteProjection(projected, index, distance_squared)
        if best is None or candidate.distance_squared < best.distance_squared:
            best = candidate
    return best


class DropLedger:
    """继电器顺位和任务牌配额；回读不确定也占用该次投掷。"""

    def __init__(self, allocation: Dict[str, int]):
        self.remaining = dict(allocation)
        self.mandatory_remaining = MANDATORY_DROP_COUNT
        self.next_channel = 1

    def has_quota(self, color: str) -> bool:
        if color == MANDATORY_COLOR:
            return self.mandatory_remaining > 0
        return self.remaining.get(color, 0) > 0

    def record_attempt(self, color: str) -> int:
        if not self.has_quota(color):
            raise RuntimeError("no remaining quota for {}".format(color))
        channel = self.next_channel
        if channel > TOTAL_DROP_COUNT:
            raise RuntimeError("planned relay channels exhausted")
        self.next_channel += 1
        if color == MANDATORY_COLOR:
            self.mandatory_remaining -= 1
        else:
            self.remaining[color] -= 1
        return channel


def wait_for_lio_basepoint(navi: Navigation, timeout: float = LIO_POSE_READY_TIMEOUT) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            navi.calibrate_basepoint(wait=False)
            if navi.lio_pose.get_pose() is not None:
                return
        except RuntimeError:
            pass
        time.sleep(0.1)
    raise RuntimeError("fresh LIO pose was not ready before timeout")


class SingleRadarNavigation(Navigation):
    """Retained for legacy bench scripts; the rescue entry does not instantiate it."""

    def _get_radar_pose(self, wait=True):
        pose = super()._get_radar_pose(wait=wait)
        if pose is not None and pose[3]:
            self._last_pose_update = time.monotonic()
        return pose


def wait_for_radar_pose(navi, radar, timeout=15.0, newer_than=0.0):
    """Retained for legacy bench scripts; the rescue entry uses LIO."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        pose_inited = getattr(radar, "_rt_pose_inited", [False, False, False])
        pose_updated_at = float(getattr(navi, "_last_pose_update", 0.0))
        if (radar.connected and all(pose_inited)
                and pose_updated_at > newer_than and navi.pose_is_fresh()):
            return
        time.sleep(0.1)
    raise RuntimeError("single-radar navigation pose was not ready before timeout")


class Mission:
    def __init__(self, fc: FC_Controller, navi: Navigation,
                 relay: Optional[LCUSRelay], vision: Optional[VisionInterface],
                 obstacle: Optional[ObstacleInterface], allocation: Dict[str, int],
                 stop_event: threading.Event):
        self.fc = fc
        self.navi = navi
        self.relay = relay
        self.vision = vision
        self.obstacle = obstacle
        self.ledger = DropLedger(allocation)
        self.stop_event = stop_event
        self.route_1, self.route_3, self.route_2 = build_routes()
        self.route_index = {MissionState.ROUTE_1: 1,
                            MissionState.CENTER: 1,
                            MissionState.ROUTE_2: 1}
        self.state = MissionState.ROUTE_1
        self.free_origin = MissionState.ROUTE_1
        self.target: Optional[TargetObservation] = None
        self.deadline: Optional[float] = None
        self.landed = False
        self._latest: Dict[str, TargetObservation] = {}
        self._vision_lock = threading.Lock()
        self._vision_stop = threading.Event()
        self._vision_error: Optional[Exception] = None
        self._vision_thread: Optional[threading.Thread] = None

    def prepare_navigation(self) -> None:
        self.navi.set_navigation_speed(CRUISE_SPEED)
        self.navi.set_vertical_speed(VERTICAL_SPEED)
        self.navi.start()
        wait_for_lio_basepoint(self.navi)
        logger.info("[RESCUE] LIO basepoint calibrated: {}", self.navi.basepoint)

    def monitor_pose(self) -> None:
        logger.warning("[RESCUE] Monitor-only mode; press Ctrl+C to exit")
        while not self.stop_event.wait(MONITOR_INTERVAL):
            if not self.navi.pose_is_fresh():
                logger.warning("[RESCUE] LIO navigation pose is stale")
                continue
            logger.info("[RESCUE] position=({:.1f},{:.1f})cm yaw={:.1f}deg",
                        self.navi.current_x, self.navi.current_y,
                        self.navi.current_yaw)

    def start_vision(self) -> None:
        if self.vision is None:
            raise RuntimeError("vision interface missing")
        self.vision.open()
        self._vision_thread = threading.Thread(
            target=self._vision_loop, name="rescue-target-observer", daemon=True)
        self._vision_thread.start()

    def _vision_loop(self) -> None:
        try:
            while not self._vision_stop.is_set():
                observations = self.vision.poll()
                now = time.monotonic()
                with self._vision_lock:
                    for observation in observations:
                        if (observation.color not in FREE_COLORS + (MANDATORY_COLOR,)
                                or not observation.target_id
                                or not math.isfinite(observation.offset_x_px)
                                or not math.isfinite(observation.offset_y_px)
                                or not 0 <= now - observation.captured_at <= VISUAL_MAX_AGE):
                            continue
                        self._latest[observation.target_id] = observation
                self._vision_stop.wait(VISUAL_PERIOD)
        except Exception as exc:
            self._vision_error = exc
            logger.exception("[RESCUE] Vision observer failed")

    def _observation(self, color: Optional[str] = None,
                     target_id: Optional[str] = None) -> Optional[TargetObservation]:
        now = time.monotonic()
        with self._vision_lock:
            candidates = [obs for obs in self._latest.values()
                          if now - obs.captured_at <= VISUAL_MAX_AGE
                          and (color is None or obs.color == color)
                          and (target_id is None or obs.target_id == target_id)
                          and (color is not None or target_id is not None
                              or (obs.color in FREE_COLORS
                              and self.ledger.has_quota(obs.color)))]
        return max(candidates, key=lambda obs: obs.captured_at) if candidates else None

    def _check(self, check_deadline: bool = True) -> None:
        if self.stop_event.is_set():
            raise RuntimeError("mission stopped")
        if self._vision_error is not None:
            raise RuntimeError("vision observer failed") from self._vision_error
        if not self.fc.state.is_fresh(0.5):
            raise RuntimeError("flight-controller telemetry is stale")
        if not self.navi.pose_is_fresh():
            raise RuntimeError("LIO navigation pose is stale")
        if check_deadline and self.deadline is not None and time.monotonic() >= self.deadline:
            raise MissionDeadline("20-minute mission deadline reached")

    def _position(self) -> Point:
        return float(self.navi.current_x), float(self.navi.current_y)

    @staticmethod
    def _same_point(first: Point, second: Point) -> bool:
        return (first[0] - second[0]) ** 2 + (first[1] - second[1]) ** 2 < 1.0

    @staticmethod
    def _at_waypoint(first: Point, second: Point) -> bool:
        return math.hypot(first[0] - second[0], first[1] - second[1]) <= 10.0

    def _start_leg_worker(self, waypoint: Point) -> Optional[threading.Thread]:
        """由任务线程发起导航；只使用 Navigation 自带的轨迹执行线程。"""
        if self._at_waypoint(self._position(), waypoint):
            return None
        previous_count = len(self.navi._thread_list)
        if not self.navi.navigation_to_waypoint(waypoint, wait=False):
            raise RuntimeError("navigation leg could not start")
        if len(self.navi._thread_list) != previous_count + 1:
            raise RuntimeError("navigation trajectory thread was not created")
        return self.navi._thread_list[-1]

    def _stop_leg_worker(self, worker: threading.Thread) -> None:
        self.navi.navigation_stop_here()
        worker.join(timeout=2.0)
        if worker.is_alive():
            raise RuntimeError("navigation leg did not stop within 2 seconds")

    def _navigate_leg(self, goal: Point, detection: Optional[str] = None,
                      protected: bool = False,
                      check_deadline: bool = True) -> Optional[TargetObservation]:
        """主线程监视导航工作线程；视觉线程只写观测，不发飞行命令。"""
        while True:
            self._check(check_deadline)
            waypoint = goal
            if protected:
                if self.obstacle is None:
                    raise RuntimeError("obstacle interface missing")
                waypoint = self.obstacle.safe_waypoint(self._position(), goal)
                if waypoint is None or not all(math.isfinite(v) for v in waypoint):
                    raise RuntimeError("no safe obstacle-avoidance waypoint")
                if self._at_waypoint(waypoint, self._position()) and not self._at_waypoint(goal, waypoint):
                    raise RuntimeError("obstacle avoidance could not make progress")
            worker = self._start_leg_worker(waypoint)
            while worker is not None and worker.is_alive():
                try:
                    self._check(check_deadline)
                    observation = self._observation(
                        color=MANDATORY_COLOR if detection == "mandatory" else None
                    ) if detection else None
                    if observation is not None:
                        self._stop_leg_worker(worker)
                        return observation
                    if protected:
                        new_waypoint = self.obstacle.safe_waypoint(self._position(), goal)
                        if new_waypoint is None or not all(math.isfinite(v) for v in new_waypoint):
                            raise RuntimeError("obstacle avoidance data invalid")
                        if not self._same_point(new_waypoint, waypoint):
                            self._stop_leg_worker(worker)
                            break
                except Exception:
                    self._stop_leg_worker(worker)
                    raise
                self.stop_event.wait(VISUAL_PERIOD)
            else:
                if worker is not None:
                    worker.join(timeout=0)
                self._check(check_deadline)
                if (not self._at_waypoint(self._position(), waypoint)
                        or (worker is not None and self.navi.traj_running_event.is_set())):
                    raise RuntimeError("navigation leg did not reach {}".format(waypoint))
                if self._same_point(waypoint, goal):
                    return self._observation(
                        color=MANDATORY_COLOR if detection == "mandatory" else None
                    ) if detection else None
            # 避障给出新的中间航点，或完成中间航点后继续向原目标前进。

    def _follow_route(self, state: MissionState) -> Optional[TargetObservation]:
        route = {MissionState.ROUTE_1: self.route_1,
                 MissionState.CENTER: self.route_3,
                 MissionState.ROUTE_2: self.route_2}[state]
        protected = state == MissionState.CENTER
        detection = "mandatory" if protected else "free"
        while self.route_index[state] < len(route):
            observation = self._navigate_leg(
                route[self.route_index[state]], detection=detection,
                protected=protected)
            if observation is not None:
                return observation
            self.route_index[state] += 1
        return None

    def _resume_route(self, state: MissionState) -> None:
        route = {MissionState.ROUTE_1: self.route_1,
                 MissionState.CENTER: self.route_3,
                 MissionState.ROUTE_2: self.route_2}[state]
        self._check()
        projection = nearest_point_on_route(self._position(), route)
        logger.info("[RESCUE] Return to {} segment {} at {}",
                    state.value, projection.segment_index, projection.point)
        self._navigate_leg(projection.point, protected=state == MissionState.CENTER)
        self.route_index[state] = projection.segment_index + 1

    def _move_toward(self, observation: TargetObservation, protected: bool) -> None:
        angle = math.atan2(observation.offset_y_px, observation.offset_x_px)
        velocity = (VISUAL_APPROACH_SPEED * math.cos(angle),
                    VISUAL_APPROACH_SPEED * math.sin(angle))
        if protected:
            if self.obstacle is None:
                raise RuntimeError("obstacle interface missing")
            velocity = self.obstacle.safe_velocity(self._position(), velocity)
            if velocity is None or not all(math.isfinite(v) for v in velocity):
                raise RuntimeError("obstacle avoidance velocity invalid")
        speed = math.hypot(*velocity)
        if speed < 1.0:
            self.navi.stop_move()
        else:
            self.navi.move_by_direction(
                speed=speed,
                direction_deg=math.degrees(math.atan2(velocity[1], velocity[0])),
            )

    def _approach_target(self, target: TargetObservation,
                         protected: bool) -> bool:
        lost_at = None
        hovering = False
        while True:
            self._check()
            observation = self._observation(target_id=target.target_id)
            if observation is None:
                if not hovering:
                    self.navi.stop_move()
                    hovering = True
                if lost_at is None:
                    lost_at = time.monotonic()
                if time.monotonic() - lost_at >= TARGET_LOSS_WAIT:
                    logger.warning("[RESCUE] Target {} lost for 2 seconds", target.target_id)
                    return False
            else:
                lost_at = None
                hovering = False
                if math.hypot(observation.offset_x_px,
                              observation.offset_y_px) <= VISUAL_CENTER_THRESHOLD_PX:
                    self.navi.stop_move()
                    return True
                self._move_toward(observation, protected)
            self.stop_event.wait(VISUAL_PERIOD)

    def _set_height(self, height: float,
                    check_deadline: bool = True) -> None:
        self._check(check_deadline)
        self.navi.stop_move()
        self.navi.set_height(height)
        if not self.navi.wait_for_height(timeout=10):
            raise RuntimeError("height {}cm was not confirmed".format(height))
        self._check(check_deadline)

    def _calibrate_low(self, target: TargetObservation,
                       protected: bool) -> bool:
        """每件物资只使用本次校准开始后采集的新观测。"""
        started_at = time.monotonic()
        deadline = time.monotonic() + LOW_CALIBRATION_TIMEOUT
        hovering = False
        while time.monotonic() < deadline:
            self._check()
            observation = self._observation(target_id=target.target_id)
            if observation is not None and observation.captured_at <= started_at:
                observation = None
            if observation is None:
                if not hovering:
                    self.navi.stop_move()
                    hovering = True
            elif math.hypot(observation.offset_x_px,
                            observation.offset_y_px) <= VISUAL_CENTER_THRESHOLD_PX:
                self.navi.stop_move()
                return True
            else:
                hovering = False
                self._move_toward(observation, protected)
            self.stop_event.wait(VISUAL_PERIOD)
        self.navi.stop_move()
        logger.warning("[RESCUE] Low-altitude calibration timed out for {}", target.target_id)
        return False

    def _drop(self, color: str, target_id: str, calibrated: bool) -> None:
        self._check()
        if self.relay is None:
            raise RuntimeError("relay missing")
        # 先记顺位。即使回读失败，下一件也使用下一路，避免可能已释放时重投。
        channel = self.ledger.record_attempt(color)
        logger.info("[DROP] Attempt {} target={} color={} calibrated={}",
                    channel, target_id, color, calibrated)
        confirmed = self.relay.turn_off(channel, verify=True, retries=1)
        if confirmed:
            logger.info("[DROP] Channel {} OFF confirmed; cargo release not sensor-verified",
                        channel)
        else:
            logger.error("[DROP] Channel {} OFF uncertain after one retry; continuing",
                         channel)

    def _free_drop(self) -> None:
        target = self.target
        if target is None or target.color not in FREE_COLORS:
            raise RuntimeError("free target missing")
        if not self._approach_target(target, protected=False):
            self._resume_route(self.free_origin)
            return
        self._set_height(FREE_DROP_HEIGHT)
        while self.ledger.has_quota(target.color):
            calibrated = self._calibrate_low(target, protected=False)
            self._drop(target.color, target.target_id, calibrated)
        self._set_height(CRUISE_HEIGHT)
        self._resume_route(self.free_origin)

    def _mandatory_drop(self) -> bool:
        target = self.target
        if target is None or target.color != MANDATORY_COLOR:
            raise RuntimeError("mandatory target missing")
        if not self._approach_target(target, protected=True):
            self._resume_route(MissionState.CENTER)
            return False
        self._set_height(MANDATORY_DROP_HEIGHT)
        calibrated = self._calibrate_low(target, protected=True)
        self._drop(MANDATORY_COLOR, target.target_id, calibrated)
        self._set_height(CRUISE_HEIGHT)
        self._navigate_leg(self.route_3[-1], protected=True)
        return True

    def _return_and_land(self, check_deadline: bool = True,
                         protected: bool = False) -> None:
        if self.navi.current_height < CRUISE_HEIGHT - 8.0:
            self._set_height(CRUISE_HEIGHT, check_deadline=check_deadline)
        self._navigate_leg(TAKEOFF_POINT, protected=protected,
                           check_deadline=check_deadline)
        self._check(check_deadline)
        if not self.navi.pointing_landing(
                TAKEOFF_POINT, height_timeout=LANDING_HEIGHT_TIMEOUT):
            raise RuntimeError("pointing landing at takeoff point was not confirmed")
        self.landed = True
        logger.info("[RESCUE] Landed and locked at takeoff point")

    def run(self) -> None:
        self._check()
        self.start_vision()
        self._check()
        logger.warning("[RESCUE] Confirmed real-flight mission started")
        try:
            self.navi.pointing_takeoff(TAKEOFF_POINT, target_height=CRUISE_HEIGHT)
            if not self.navi.wait_for_height(timeout=10):
                raise RuntimeError("150cm takeoff height was not confirmed")
            self.navi.set_yaw(0)
            if not self.navi.wait_for_yaw():
                raise RuntimeError("yaw stabilization was not confirmed")
            self._navigate_leg(self.route_1[0])

            while self.state is not MissionState.DONE:
                self._check()
                logger.info("[RESCUE] State: {}", self.state.value)
                if self.state is MissionState.ROUTE_1:
                    self.target = self._follow_route(self.state)
                    if self.target is None:
                        self.state = MissionState.CENTER
                    else:
                        self.free_origin = self.state
                        self.state = MissionState.FREE_DROP
                elif self.state is MissionState.FREE_DROP:
                    self._free_drop()
                    self.state = self.free_origin
                elif self.state is MissionState.CENTER:
                    self.target = self._follow_route(self.state)
                    if self.target is None:
                        logger.warning("[RESCUE] Mandatory yellow target not found")
                        self.state = MissionState.ROUTE_2
                    else:
                        self.state = MissionState.MANDATORY_DROP
                elif self.state is MissionState.MANDATORY_DROP:
                    self.state = (MissionState.ROUTE_2 if self._mandatory_drop()
                                  else MissionState.CENTER)
                elif self.state is MissionState.ROUTE_2:
                    self.target = self._follow_route(self.state)
                    if self.target is None:
                        self.state = MissionState.DONE
                    else:
                        self.free_origin = self.state
                        self.state = MissionState.FREE_DROP

            self._return_and_land()
        except MissionDeadline:
            logger.warning("[RESCUE] Deadline reached; stop search and return")
            self.navi.navigation_stop_here()
            if not self.navi.pose_is_fresh():
                raise
            protected = self.state in (MissionState.CENTER,
                                       MissionState.MANDATORY_DROP)
            self._return_and_land(check_deadline=False, protected=protected)

    def stop(self) -> None:
        self.stop_event.set()
        self._vision_stop.set()
        if self._vision_thread is not None:
            self._vision_thread.join(timeout=2.0)
            if self._vision_thread.is_alive():
                logger.error("[RESCUE] Vision observer did not stop within 2 seconds")
        try:
            if self.vision is not None:
                self.vision.close()
        finally:
            self.navi.stop()


def emergency_land(fc: FC_Controller) -> bool:
    """请求降落；未确认落地前不强制锁桨，也不开断剩余物资。"""
    logger.warning("[RESCUE] Flight interrupted; requesting emergency landing")
    fc.set_flight_mode(fc.PROGRAM_MODE)
    time.sleep(0.1)
    fc.stablize()
    fc.land()
    if fc.wait_for_lock(timeout_s=20):
        return True
    logger.error("[RESCUE] Landing lock not confirmed; keep landing command active")
    fc.land()
    return False


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="2026 救援物资投放无人机主入口")
    parser.add_argument("--confirm-flight", action="store_true",
                        help="启用真实飞行；默认仅监视 LIO 位姿")
    parser.add_argument("--fc-port", default=FC_SERIAL_DEV)
    parser.add_argument("--relay-port", default=None,
                        help="LCUS 继电器串口；也可用 D_TASK_RELAY_PORT 环境变量")
    parser.add_argument("--red-count", type=int)
    parser.add_argument("--blue-count", type=int)
    parser.add_argument("--green-count", type=int)
    return parser.parse_args()


def require_flight_interfaces(vision: VisionInterface,
                              obstacle: ObstacleInterface) -> None:
    """占位接口未实现时，在连接任何硬件前拒绝飞行。"""
    missing = []
    if not vision.IMPLEMENTED:
        missing.append("visual target detection")
    if not obstacle.IMPLEMENTED:
        missing.append("center/mandatory obstacle avoidance")
    if missing:
        raise RuntimeError("flight disabled until TODO interfaces are implemented: "
                           + ", ".join(missing))


def wait_for_start_command() -> None:
    logger.warning("[RESCUE] Payload channels 1-5 ON; enter start_mission to take off")
    while True:
        command = input("[RESCUE] start_mission> ")
        if command == "start_mission":
            return
        logger.warning("[RESCUE] Ignored command; enter exactly start_mission")


def main() -> int:
    args = parse_args()
    stop_event = threading.Event()
    fc: Optional[FC_Controller] = None
    navi: Optional[Navigation] = None
    relay: Optional[LCUSRelay] = None
    mission: Optional[Mission] = None
    takeoff_attempted = False
    relay_modified = False
    allocation = {color: 0 for color in FREE_COLORS}
    vision = None
    obstacle = None

    try:
        if args.confirm_flight:
            allocation = validate_allocation(
                args.red_count, args.blue_count, args.green_count)
            vision = VisionInterface()
            obstacle = ObstacleInterface()
            require_flight_interfaces(vision, obstacle)

        fc = FC_Controller()
        fc.start_listen_serial(serial_dev=args.fc_port, print_state=False)
        if not fc.wait_for_connection(timeout_s=10):
            raise RuntimeError("flight-controller connection timeout")
        if not fc.state.is_fresh(0.5):
            raise RuntimeError("flight-controller telemetry is stale")
        if fc.state.unlock.value:
            raise RuntimeError("flight controller already unlocked; refuse takeover")

        navi = Navigation(fc=fc, stop_event=stop_event)
        mission = Mission(fc, navi, None, vision, obstacle, allocation, stop_event)
        mission.prepare_navigation()

        if not args.confirm_flight:
            mission.monitor_pose()
            return 0

        relay = LCUSRelay(port=args.relay_port, channel_count=RELAY_CHANNEL_COUNT)
        relay.open()
        if relay.detect_channel_count() != RELAY_CHANNEL_COUNT:
            raise RuntimeError("8-channel LCUS board was not confirmed")
        states = relay.query_status()
        if states is None or any(states.get(channel) is not False
                                 for channel in range(1, RELAY_CHANNEL_COUNT + 1)):
            raise RuntimeError("all eight relay channels must initially report OFF")
        for channel in range(1, TOTAL_DROP_COUNT + 1):
            relay_modified = True
            if not relay.turn_on(channel, verify=True, retries=1):
                raise RuntimeError("failed to confirm payload channel {} ON".format(channel))
        mission.relay = relay
        wait_for_start_command()
        mission.deadline = time.monotonic() + MISSION_TIMEOUT
        mission._check()
        takeoff_attempted = True
        mission.run()
        return 0
    except KeyboardInterrupt:
        logger.warning("[RESCUE] Interrupted by user")
        return 130
    except Exception:
        logger.exception("[RESCUE] Mission failed")
        return 1
    finally:
        if mission is not None:
            try:
                mission.stop()
            except Exception:
                logger.exception("[RESCUE] Failed to stop mission")
        elif navi is not None:
            try:
                navi.stop()
            except Exception:
                logger.exception("[RESCUE] Failed to stop navigation")

        if (takeoff_attempted and fc is not None and fc.connected
                and fc.state.unlock.value):
            try:
                emergency_land(fc)
            except Exception:
                logger.exception("[RESCUE] Emergency landing request failed")

        if relay is not None:
            try:
                locked = (fc is not None and fc.connected
                          and fc.state.is_fresh(0.5)
                          and not fc.state.unlock.value)
                if relay_modified and locked:
                    if not relay.all_off(verify=True):
                        logger.error("[RESCUE] Relay all_off was not confirmed")
                elif relay_modified:
                    logger.error("[RESCUE] Lock not confirmed; keep remaining payload channels ON")
            except Exception:
                logger.exception("[RESCUE] Failed to reset relay after lock check")
            finally:
                relay.close()

        if fc is not None:
            try:
                fc.close()
            except Exception:
                logger.exception("[RESCUE] Failed to close flight controller")


if __name__ == "__main__":
    sys.exit(main())
