"""
单雷达定位和下视视觉伴飞的任务一程序。

坐标与单位：
- 水平坐标和高度均为 cm；
- x 向前为正，y 向左为正；
- track_landing_marker() 的 x_px 向图像上方为正，对应前方；
- track_landing_marker() 的 y_px 向图像左侧为正，对应左方；
- PURSUIT_SPEED 是 set_navigation_speed() 的参数，不保证实际飞行速度；
- 伴飞速度是发送给飞控的水平速度指令，范围为 6.4～9.6 cm/s。

起飞信号尚未接入，wait_for_takeoff_signal() 当前使用占位实现并立即返回。
"""

import csv
import math
import threading
import time
from collections import deque
from dataclasses import replace
from datetime import datetime
from pathlib import Path
from typing import Deque, Dict, Iterator, List, Optional, Tuple

import numpy as np
from loguru import logger

from FlightController import FC_Controller
from FlightController.Components import LD_Radar
from FlightController.Solutions.Navigation import Navigation
from fleet_bus.air_node import attach_air_fleet_node
from fleet_bus.models import AirFleetState, NodeFlags
from fleet_bus.pose_provider import NavigationAirStateProvider
from landing_marker_offset import track_landing_marker


FC_SERIAL_DEV = "/dev/ttyACM0"
CAMERA_INDEX = 0

TAKEOFF_POINT = np.array([0.0, 0.0])
ENTRY_POINT = np.array([87.5, -37.5])
CRUISE_HEIGHT = 150
VERTICAL_SPEED = 20

PURSUIT_SPEED = 30
TARGET_DETECTION_PIXEL_THRESHOLD = 30.0

ESCORT_SPEED_MIDPOINT = 7.2
ESCORT_SPEED_ADJUST_RATIO = 0.20
ESCORT_SPEED_MIN = ESCORT_SPEED_MIDPOINT * (1.0 - ESCORT_SPEED_ADJUST_RATIO)
ESCORT_SPEED_MAX = ESCORT_SPEED_MIDPOINT * (1.0 + ESCORT_SPEED_ADJUST_RATIO)
ESCORT_OUTPUT_ON_SECONDS = 5.0
ESCORT_OUTPUT_OFF_SECONDS = 2.0
ESCORT_TOTAL_SECONDS = ESCORT_OUTPUT_ON_SECONDS + ESCORT_OUTPUT_OFF_SECONDS

# 30 px 的持续偏移最多对应中值速度 20% 的比例修正；积分项继续试出
# 移动目标的速度和方向，低通滤波用于抑制小范围像素抖动。
ESCORT_PROPORTIONAL_GAIN = (
    ESCORT_SPEED_MIDPOINT
    * ESCORT_SPEED_ADJUST_RATIO
    / TARGET_DETECTION_PIXEL_THRESHOLD
)
ESCORT_INTEGRAL_GAIN = ESCORT_PROPORTIONAL_GAIN
ESCORT_FILTER_ALPHA = 0.25
ESCORT_CONTROL_PERIOD = 0.05
VISION_SAMPLE_STALE_SECONDS = 0.35
VISION_STARTUP_TIMEOUT_SECONDS = 8.0

VISION_SAMPLE_BUFFER_SIZE = 8
MAX_ESCORT_RECORDS = 2000
RADAR_POSE_READY_TIMEOUT_SECONDS = 15.0

# 仅用作持续沿 +x 飞行的 PID 引导目标，不代表任务要求到达该点。
# 若到达该边界仍未发现目标，任务进入异常降落兜底，避免无界前飞。
FORWARD_GUIDANCE_DISTANCE = 150.0
FORWARD_BOUNDARY_MARGIN = 10.0


def wait_for_radar_initialization(
    radar: LD_Radar,
    timeout: float = RADAR_POSE_READY_TIMEOUT_SECONDS,
) -> None:
    """等待雷达连接且 x、y、yaw 三轴位姿均完成初始化。"""
    deadline = time.monotonic() + float(timeout)
    while time.monotonic() < deadline:
        pose_inited = getattr(radar, "_rt_pose_inited", [False, False, False])
        if radar.connected and all(pose_inited):
            x, y, yaw = radar.rt_pose
            logger.info(
                "[MISSION] Radar initialized: ({:.1f}, {:.1f})cm, "
                "yaw={:.1f}deg",
                x,
                y,
                yaw,
            )
            return
        time.sleep(0.1)
    raise RuntimeError("Single-radar initialization timed out")


class MissionOperationState:
    """FleetBus D-task operation-state values rendered by the ground station."""

    IDLE = 0
    READY = 1
    TAKEOFF = 3
    HOVERING = 4
    ESCORTING = 5
    RETURNING_HOME = 9
    LANDING_HOME = 10
    COMPLETED = 11
    STOPPED = 12
    FAULT = 13


class MissionFleetStateProvider:
    """Publish the mission phase and launch-point-relative navigation pose."""

    def __init__(self, fc: FC_Controller, navi: Navigation, mission: "Mission"):
        self._mission = mission
        self._navigation_state = NavigationAirStateProvider(
            fc,
            navi,
            position_transform=lambda x_cm, y_cm: (
                x_cm - float(TAKEOFF_POINT[0]),
                y_cm - float(TAKEOFF_POINT[1]),
            ),
        )

    def __call__(self) -> AirFleetState:
        state = self._navigation_state()
        operation_state, error_code = self._mission.fleet_status()
        node_flags = state.node_flags
        if operation_state in (
            MissionOperationState.TAKEOFF,
            MissionOperationState.HOVERING,
            MissionOperationState.ESCORTING,
            MissionOperationState.RETURNING_HOME,
            MissionOperationState.LANDING_HOME,
        ):
            node_flags |= int(NodeFlags.BUSY)
        return replace(
            state,
            node_flags=node_flags,
            operation_state=operation_state,
            error_code=error_code,
        )


class Mission:
    def __init__(
        self,
        fc: FC_Controller,
        radar: LD_Radar,
        navi: Navigation,
        stop_event: threading.Event,
    ):
        self.fc = fc
        self.radar = radar
        self.navi = navi
        self.stop_event = stop_event
        self.takeoff_signal = threading.Event()
        self._fleet_status_lock = threading.Lock()
        self._fleet_operation_state = MissionOperationState.IDLE
        self._fleet_error_code = 0

        self._vision_stop_event = threading.Event()
        self._vision_ready_event = threading.Event()
        self._vision_lock = threading.Lock()
        self._vision_thread: Optional[threading.Thread] = None
        self._vision_error: Optional[BaseException] = None
        self._vision_sequence = 0
        self._vision_samples: Deque[
            Tuple[int, float, Optional[float], Optional[float]]
        ] = deque(maxlen=VISION_SAMPLE_BUFFER_SIZE)

        self._escort_active = False
        self._escort_started_at = 0.0
        self._escort_output_enabled = True
        self._escort_command = (0, 0)
        self._escort_velocity_estimate = np.array(
            [ESCORT_SPEED_MIDPOINT, 0.0],
            dtype=float,
        )
        self._escort_filtered_velocity = self._escort_velocity_estimate.copy()
        self._escort_records: Deque[Dict[str, object]] = deque(
            maxlen=MAX_ESCORT_RECORDS
        )
        self._escort_records_dropped = 0

    def stop(self):
        self.stop_event.set()
        with self._fleet_status_lock:
            if self._fleet_operation_state not in (
                MissionOperationState.COMPLETED,
                MissionOperationState.FAULT,
            ):
                self._fleet_operation_state = MissionOperationState.STOPPED
        self._stop_vision_tracker()
        self.navi.stop()
        logger.info("[MISSION] Mission stopped")

    def set_fleet_status(self, operation_state: int, error_code: int = 0):
        with self._fleet_status_lock:
            self._fleet_operation_state = operation_state
            self._fleet_error_code = error_code

    def fleet_status(self) -> Tuple[int, int]:
        with self._fleet_status_lock:
            return self._fleet_operation_state, self._fleet_error_code

    def notify_takeoff_signal(self):
        """供后续无线、按键或其他信号回调通知起飞。"""
        self.takeoff_signal.set()

    def wait_for_takeoff_signal(self):
        """
        起飞信号占位函数。

        TODO: 后续合作者接入真实信号源时，让信号回调调用
        notify_takeoff_signal()，再取消下面三行等待代码的注释。
        当前不等待任何外部信号，会立即继续任务。
        """
        logger.warning(
            "[MISSION] Takeoff signal is not implemented; placeholder continues immediately"
        )
        # self.takeoff_signal.clear()
        # self.takeoff_signal.wait()
        # self.takeoff_signal.clear()

    def _vision_worker(self):
        offsets: Optional[
            Iterator[Tuple[Optional[float], Optional[float]]]
        ] = None
        try:
            offsets = track_landing_marker(CAMERA_INDEX)
            for x_px, y_px in offsets:
                now = time.monotonic()
                with self._vision_lock:
                    self._vision_sequence += 1
                    sample = (self._vision_sequence, now, x_px, y_px)
                    self._vision_samples.append(sample)
                    self._vision_ready_event.set()
                    if self._escort_active:
                        if len(self._escort_records) == self._escort_records.maxlen:
                            self._escort_records_dropped += 1
                        vel_x, vel_y = self._escort_command
                        speed = math.hypot(vel_x, vel_y)
                        pixel_distance = (
                            math.hypot(x_px, y_px)
                            if x_px is not None and y_px is not None
                            else None
                        )
                        self._escort_records.append(
                            {
                                "elapsed_s": now - self._escort_started_at,
                                "x_px": x_px,
                                "y_px": y_px,
                                "pixel_distance_px": pixel_distance,
                                "velocity_x_cm_s": vel_x,
                                "velocity_y_cm_s": vel_y,
                                "speed_cm_s": speed,
                                "direction_deg": (
                                    math.degrees(math.atan2(vel_y, vel_x))
                                    if speed > 0
                                    else None
                                ),
                                "digital_output_0_enabled": (
                                    self._escort_output_enabled
                                ),
                            }
                        )
                if self._vision_stop_event.is_set():
                    break
        except Exception as exc:
            with self._vision_lock:
                self._vision_error = exc
            self._vision_ready_event.set()
        finally:
            if offsets is not None:
                try:
                    offsets.close()  # type: ignore[attr-defined]
                except Exception:
                    logger.exception("[VISION] Failed to close marker tracker")

    def _start_vision_tracker(self):
        self._vision_stop_event.clear()
        self._vision_ready_event.clear()
        self._vision_thread = threading.Thread(
            target=self._vision_worker,
            name="mission1-marker-tracker",
            daemon=True,
        )
        self._vision_thread.start()

        deadline = time.monotonic() + VISION_STARTUP_TIMEOUT_SECONDS
        while not self._vision_ready_event.wait(0.05):
            if self.stop_event.is_set():
                raise RuntimeError("Mission stopped while starting vision")
            if time.monotonic() >= deadline:
                raise RuntimeError("Landing-marker vision startup timeout")
        self._raise_if_vision_failed()
        logger.info("[VISION] Landing-marker tracker ready on camera {}", CAMERA_INDEX)

    def _stop_vision_tracker(self):
        self._vision_stop_event.set()
        thread = self._vision_thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=1.0)
            if thread.is_alive():
                logger.warning(
                    "[VISION] Marker tracker did not stop within 1s; "
                    "camera will be released when its daemon thread exits"
                )

    def _raise_if_vision_failed(self):
        with self._vision_lock:
            error = self._vision_error
        if error is not None:
            raise RuntimeError("Landing-marker vision failed") from error

    def _latest_vision_sample(
        self,
    ) -> Optional[Tuple[int, float, Optional[float], Optional[float]]]:
        with self._vision_lock:
            if not self._vision_samples:
                return None
            return self._vision_samples[-1]

    def _clear_vision_samples(self):
        with self._vision_lock:
            self._vision_samples.clear()

    @staticmethod
    def _limit_velocity_max(vector: np.ndarray) -> np.ndarray:
        speed = float(np.linalg.norm(vector))
        if speed > ESCORT_SPEED_MAX:
            return vector * (ESCORT_SPEED_MAX / speed)
        return vector

    @staticmethod
    def _clamp_command_velocity(
        vector: np.ndarray,
        fallback_direction: np.ndarray,
    ) -> np.ndarray:
        speed = float(np.linalg.norm(vector))
        if speed <= 1e-9:
            fallback_speed = float(np.linalg.norm(fallback_direction))
            if fallback_speed <= 1e-9:
                return np.array([ESCORT_SPEED_MIDPOINT, 0.0], dtype=float)
            return fallback_direction * (ESCORT_SPEED_MIN / fallback_speed)
        if speed < ESCORT_SPEED_MIN:
            return vector * (ESCORT_SPEED_MIN / speed)
        if speed > ESCORT_SPEED_MAX:
            return vector * (ESCORT_SPEED_MAX / speed)
        return vector

    @staticmethod
    def _quantize_escort_velocity(vector: np.ndarray) -> Tuple[int, int]:
        """
        飞控速度字段为整数，选择最接近目标矢量且合速度仍在允许范围内的整数值。
        """
        candidates = (
            (vel_x, vel_y)
            for vel_x in range(-10, 11)
            for vel_y in range(-10, 11)
            if ESCORT_SPEED_MIN
            <= math.hypot(vel_x, vel_y)
            <= ESCORT_SPEED_MAX
        )
        return min(
            candidates,
            key=lambda item: (
                (item[0] - vector[0]) ** 2 + (item[1] - vector[1]) ** 2
            ),
        )

    def _update_escort_velocity(
        self,
        x_px: float,
        y_px: float,
        dt: float,
    ) -> Tuple[int, int]:
        pixel_error = np.array([x_px, y_px], dtype=float)
        dt = min(max(float(dt), 0.0), 0.2)

        self._escort_velocity_estimate += (
            pixel_error * ESCORT_INTEGRAL_GAIN * dt
        )
        # 估计矢量只限制上限，允许穿过零点，从而能够试出目标的反向运动。
        self._escort_velocity_estimate = self._limit_velocity_max(
            self._escort_velocity_estimate
        )

        desired_velocity = (
            self._escort_velocity_estimate
            + pixel_error * ESCORT_PROPORTIONAL_GAIN
        )
        desired_velocity = self._clamp_command_velocity(
            desired_velocity,
            pixel_error,
        )
        self._escort_filtered_velocity += ESCORT_FILTER_ALPHA * (
            desired_velocity - self._escort_filtered_velocity
        )
        # 滤波内部状态允许穿过零点；仅对本周期下发值补足最小速度，
        # 否则每周期把内部状态拉回正方向会导致无法识别反向运动。
        self._escort_filtered_velocity = self._limit_velocity_max(
            self._escort_filtered_velocity
        )
        command_velocity = self._clamp_command_velocity(
            self._escort_filtered_velocity,
            desired_velocity,
        )
        return self._quantize_escort_velocity(command_velocity)

    def _set_escort_command(self, vel_x: int, vel_y: int):
        # 视觉偏移和实时控制均使用机体前/左坐标；偏航置零以保持原方向。
        self.navi.update_realtime_control(vel_x=vel_x, vel_y=vel_y, yaw=0)
        with self._vision_lock:
            self._escort_command = (vel_x, vel_y)

    def _wait_until_target_detected(
        self,
        forward_boundary_x: float,
    ) -> Tuple[float, float]:
        last_sequence = -1
        while not self.stop_event.is_set():
            self._raise_if_vision_failed()
            sample = self._latest_vision_sample()
            if sample is not None and sample[0] != last_sequence:
                sequence, captured_at, x_px, y_px = sample
                last_sequence = sequence
                if (
                    time.monotonic() - captured_at <= VISION_SAMPLE_STALE_SECONDS
                    and x_px is not None
                    and y_px is not None
                ):
                    pixel_distance = math.hypot(x_px, y_px)
                    if pixel_distance < TARGET_DETECTION_PIXEL_THRESHOLD:
                        logger.info(
                            "[VISION] Target detected: x_px={:.2f}, "
                            "y_px={:.2f}, distance={:.2f}px",
                            x_px,
                            y_px,
                            pixel_distance,
                        )
                        return x_px, y_px

            if self.navi.current_x >= forward_boundary_x - FORWARD_BOUNDARY_MARGIN:
                raise RuntimeError(
                    "Target not detected before forward guidance boundary"
                )
            self.stop_event.wait(ESCORT_CONTROL_PERIOD)

        raise RuntimeError("Target detection stopped")

    def _escort_target(self, initial_offset: Tuple[float, float]):
        self.navi.navigation_stop_here()
        self.navi.navigation_flag = False

        self._escort_velocity_estimate = np.array(
            [ESCORT_SPEED_MIDPOINT, 0.0],
            dtype=float,
        )
        self._escort_filtered_velocity = self._escort_velocity_estimate.copy()
        started_at = time.monotonic()
        last_control_at = started_at
        last_sequence = -1
        output_disabled = False

        with self._vision_lock:
            self._escort_started_at = started_at
            self._escort_output_enabled = True
            self._escort_active = True

        initial_velocity = self._update_escort_velocity(
            initial_offset[0],
            initial_offset[1],
            ESCORT_CONTROL_PERIOD,
        )
        self._set_escort_command(*initial_velocity)
        logger.info(
            "[MISSION] Escort started: midpoint={}cm/s, range={:.1f}-{:.1f}cm/s",
            ESCORT_SPEED_MIDPOINT,
            ESCORT_SPEED_MIN,
            ESCORT_SPEED_MAX,
        )

        try:
            while not self.stop_event.is_set():
                now = time.monotonic()
                elapsed = now - started_at
                if (
                    not output_disabled
                    and elapsed >= ESCORT_OUTPUT_ON_SECONDS
                ):
                    self.fc.set_digital_output(0, False)
                    output_disabled = True
                    with self._vision_lock:
                        self._escort_output_enabled = False
                    logger.info("[MISSION] Digital output 0 disabled")

                if elapsed >= ESCORT_TOTAL_SECONDS:
                    break

                self._raise_if_vision_failed()
                sample = self._latest_vision_sample()
                if sample is None:
                    velocity = (0, 0)
                else:
                    sequence, captured_at, x_px, y_px = sample
                    sample_fresh = (
                        now - captured_at <= VISION_SAMPLE_STALE_SECONDS
                    )
                    if (
                        sequence != last_sequence
                        and sample_fresh
                        and x_px is not None
                        and y_px is not None
                    ):
                        last_sequence = sequence
                        velocity = self._update_escort_velocity(
                            x_px,
                            y_px,
                            now - last_control_at,
                        )
                        last_control_at = now
                    elif not sample_fresh or x_px is None or y_px is None:
                        # 视觉丢失时不继续发送旧水平速度，等待重新捕获目标。
                        velocity = (0, 0)
                    else:
                        with self._vision_lock:
                            velocity = self._escort_command

                self._set_escort_command(*velocity)
                self.stop_event.wait(ESCORT_CONTROL_PERIOD)

            if self.stop_event.is_set():
                raise RuntimeError("Mission stopped during escort")
        finally:
            with self._vision_lock:
                self._escort_active = False
            self._set_escort_command(0, 0)
            self.navi.stop_move()

    def write_escort_log(self) -> Optional[Path]:
        """飞行控制停止后，将伴飞期视觉输出和实际发送速度一次性写入 CSV。"""
        with self._vision_lock:
            records: List[Dict[str, object]] = list(self._escort_records)
            dropped = self._escort_records_dropped
        if not records:
            logger.warning("[MISSION] No escort records to write")
            return None

        log_dir = Path(__file__).resolve().parent / "fc_log"
        log_dir.mkdir(parents=True, exist_ok=True)
        log_path = log_dir / (
            "mission1_26_escort_"
            + datetime.now().strftime("%Y%m%d_%H%M%S")
            + ".csv"
        )
        fieldnames = [
            "elapsed_s",
            "x_px",
            "y_px",
            "pixel_distance_px",
            "velocity_x_cm_s",
            "velocity_y_cm_s",
            "speed_cm_s",
            "direction_deg",
            "digital_output_0_enabled",
        ]
        with log_path.open("w", newline="", encoding="utf-8") as file:
            writer = csv.DictWriter(file, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(records)
        if dropped:
            logger.warning(
                "[MISSION] Escort log buffer discarded {} oldest records",
                dropped,
            )
        logger.info("[MISSION] Escort log written to {}", log_path)
        return log_path

    def run(self):
        fc = self.fc
        navi = self.navi

        navi.set_navigation_speed(PURSUIT_SPEED)
        navi.set_vertical_speed(VERTICAL_SPEED)
        navi.start(mode="radar")
        logger.info("[MISSION] Single-radar navigation started")

        # 三轴全部初始化后，才允许建立任务坐标原点。
        wait_for_radar_initialization(self.radar)
        navi.calibrate_basepoint()
        logger.info("[MISSION] Radar basepoint calibrated: {}", navi.basepoint)
        time.sleep(1)

        # 起飞前确认相机和视觉生成器能够持续给出结果，失败时拒绝起飞。
        self._start_vision_tracker()
        self.set_fleet_status(MissionOperationState.READY)
        self.wait_for_takeoff_signal()
        if self.stop_event.is_set():
            return

        logger.info(
            "[MISSION] Pointing takeoff to {}cm at {}",
            CRUISE_HEIGHT,
            TAKEOFF_POINT,
        )
        self.set_fleet_status(MissionOperationState.TAKEOFF)
        navi.pointing_takeoff(
            TAKEOFF_POINT,
            CRUISE_HEIGHT,
        )

        self.set_fleet_status(MissionOperationState.HOVERING)
        logger.info("[MISSION] Navigate to entry point {}", ENTRY_POINT)
        if not navi.navigation_to_waypoint(ENTRY_POINT, wait=True):
            raise RuntimeError("Failed to reach entry point")

        forward_target = np.array(
            [ENTRY_POINT[0] + FORWARD_GUIDANCE_DISTANCE, ENTRY_POINT[1]]
        )
        self._clear_vision_samples()
        navi.set_navigation_speed(PURSUIT_SPEED)
        navi.switch_pid("navi")
        navi.direct_set_waypoint(forward_target)
        logger.info(
            "[MISSION] Pursuing along +x with navigation-speed parameter {}",
            PURSUIT_SPEED,
        )

        initial_offset = self._wait_until_target_detected(forward_target[0])
        self.set_fleet_status(MissionOperationState.ESCORTING)
        self._escort_target(initial_offset)

        # 视觉任务到此结束，先释放相机，再返航并定点降落。
        self._stop_vision_tracker()
        navi.set_navigation_speed(ESCORT_SPEED_MIDPOINT)
        self.set_fleet_status(MissionOperationState.RETURNING_HOME)
        logger.info("[MISSION] Returning to takeoff point {}", TAKEOFF_POINT)
        if not navi.navigation_to_waypoint(TAKEOFF_POINT, wait=True):
            raise RuntimeError("Failed to return to takeoff point")
        logger.info("[MISSION] Returned to takeoff point; starting landing")
        self.set_fleet_status(MissionOperationState.LANDING_HOME)
        if not navi.pointing_landing(TAKEOFF_POINT):
            raise RuntimeError("Failed to land at takeoff point")
        self.set_fleet_status(MissionOperationState.COMPLETED)
        logger.info("[MISSION] Landed at takeoff point")


def main():
    fc = FC_Controller()
    radar = LD_Radar()
    stop_event = threading.Event()
    navi = None
    mission = None
    fleet_node = None
    digital_output_enabled = False

    try:
        # server_ros.py 已关闭：本程序按单雷达方案直连飞控串口。
        fc.start_listen_serial(serial_dev=FC_SERIAL_DEV, print_state=False)
        if not fc.wait_for_connection(timeout_s=10):
            raise RuntimeError("Flight controller connection timeout")
        logger.info("[MANAGER] Flight controller connected")

        # 按任务要求，在确认飞控连接后立即打开数字输出 0。
        fc.set_digital_output(0, True)
        digital_output_enabled = True
        logger.info("[MANAGER] Digital output 0 enabled")

        radar.debug = False
        radar.start()
        logger.info("[MANAGER] Single radar started")

        navi = Navigation(fc=fc, radar=radar, stop_event=stop_event)
        mission = Mission(
            fc=fc,
            radar=radar,
            navi=navi,
            stop_event=stop_event,
        )
        fleet_node = attach_air_fleet_node(
            fc,
            navi,
            stop_event,
            readonly=True,
            state_provider=MissionFleetStateProvider(fc, navi, mission),
        )
        mission.run()
    except KeyboardInterrupt:
        logger.warning("[MANAGER] Mission interrupted by user")
    except Exception:
        if mission is not None:
            mission.set_fleet_status(MissionOperationState.FAULT, error_code=1)
        logger.exception("[MANAGER] Mission failed")
    finally:
        if mission is not None:
            mission.stop()
            try:
                mission.write_escort_log()
            except Exception:
                logger.exception("[MANAGER] Failed to write escort log")
        elif navi is not None:
            navi.stop()

        if digital_output_enabled:
            try:
                fc.set_digital_output(0, False)
            except Exception:
                logger.exception("[MANAGER] Failed to disable digital output 0")

        # 异常退出时执行已有安全降落兜底；正常定点降落后不会重复触发。
        try:
            if fc.state.unlock.value:
                logger.warning("[MANAGER] Auto landing")
                fc.set_flight_mode(fc.PROGRAM_MODE)
                fc.stablize()
                fc.land()
                if not fc.wait_for_lock(timeout_s=20):
                    logger.error(
                        "[MANAGER] Landing lock not confirmed; keep landing "
                        "command active and refuse airborne force-lock"
                    )
                    fc.land()
        except Exception:
            logger.exception("[MANAGER] Auto landing failed")

        try:
            if radar.running:
                radar.stop()
        except Exception:
            logger.exception("[MANAGER] Failed to stop radar")

        if fleet_node is not None:
            fleet_node.close()
        fc.close()
        logger.info("[MANAGER] Mission finished")


if __name__ == "__main__":
    main()
