"""
静止目标视觉下降与低空悬停测试。

真实飞行流程：
1. 单雷达定位，定点起飞至 150 cm；
2. 飞到任务入口点并沿 +x 方向追及；
3. 目标像素距离小于 30 px 后，停止追及；
4. 以高度 PID 下降至 40 cm，同时按视觉像素偏移修正水平位置；
5. 在 40 cm 高度继续视觉校准并悬停 2 s；
6. 回升至 150 cm，返回起飞点并定点降落。

坐标与单位沿用 mission1_26.py：水平位置、高度和速度使用 cm，
x 向前为正、y 向左为正；视觉 x_px 向前为正、y_px 向左为正。

本文件会连接真实飞控、雷达和相机并执行飞行，不能用于无保护条件的
桌面测试。运行前必须确认 server_ros.py 及其他 FC_Server 程序已关闭。

飞控连接后会立即打开数字输出 0，并暂停等待终端输入 ``s``；进入
40 cm 低空悬停阶段时关闭数字输出 0。
"""

import csv
import math
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path
from typing import Deque, Dict, List, Optional, Tuple

import numpy as np
from loguru import logger

from FlightController import FC_Controller
from FlightController.Components import LD_Radar
from FlightController.Solutions.Navigation import Navigation
import mission1_26 as mission1


TAKEOFF_ALT_THRESHOLD = 13.0
START_COMMAND = "s"
DESCENT_TARGET_HEIGHT = 40.0
LOW_HOVER_SECONDS = 2.0
DESCENT_TIMEOUT_SECONDS = 15.0
ASCENT_TIMEOUT_SECONDS = 12.0
LANDING_HEIGHT_TIMEOUT_SECONDS = 8.0
RADAR_POSE_READY_TIMEOUT_SECONDS = 15.0

# 以下视觉控制量是首次试飞估计值，应根据 CSV 和实际响应再调整。
# 30 px 偏移对应约 4.5 cm/s 修正，合速度最大限制为 8 cm/s。
VISUAL_CORRECTION_GAIN = 0.15
VISUAL_CORRECTION_DEADBAND_PX = 3.0
VISUAL_HORIZONTAL_SPEED_LIMIT = 8.0
VISUAL_FILTER_ALPHA = 0.30
VISUAL_CONTROL_PERIOD_SECONDS = 0.05
VISION_LOSS_TIMEOUT_SECONDS = 1.0
HEIGHT_TOLERANCE = 5.0
HEIGHT_CONFIRM_SECONDS = 0.4
MAX_VISUAL_DESCENT_RECORDS = 3000


def wait_for_terminal_start_command() -> None:
    """阻塞等待操作者在终端输入启动字符。"""
    logger.warning(
        "[TEST] Digital output 0 is enabled; enter '{}' to continue",
        START_COMMAND,
    )
    while True:
        try:
            command = input(
                "[TEST] Enter '{}' to start the flight: ".format(
                    START_COMMAND
                )
            ).strip().lower()
        except EOFError as exc:
            raise RuntimeError(
                "Terminal input closed before the start command"
            ) from exc
        if command == START_COMMAND:
            logger.info("[TEST] Terminal start command accepted")
            return
        logger.warning(
            "[TEST] Ignored terminal input; enter '{}' to continue",
            START_COMMAND,
        )


class SingleRadarNavigation(Navigation):
    """在底层确认雷达三轴有效后记录位姿新鲜度。"""

    def _get_radar_pose(self, wait=True):
        pose = super()._get_radar_pose(wait=wait)
        if pose is not None and pose[3]:
            self._last_pose_update = time.monotonic()
        return pose


def wait_for_radar_pose(
    navi: Navigation,
    radar: LD_Radar,
    timeout: float = RADAR_POSE_READY_TIMEOUT_SECONDS,
    newer_than: float = 0.0,
) -> None:
    """等待雷达连接、三轴位姿初始化完成且位姿保持新鲜。"""
    deadline = time.monotonic() + float(timeout)
    while time.monotonic() < deadline:
        pose_inited = getattr(radar, "_rt_pose_inited", [False, False, False])
        pose_updated_at = float(getattr(navi, "_last_pose_update", 0.0))
        if (
            radar.connected
            and all(pose_inited)
            and pose_updated_at > newer_than
            and navi.pose_is_fresh()
        ):
            logger.info(
                "[TEST] Radar pose ready: ({:.1f}, {:.1f})cm, yaw={:.1f}deg",
                navi.current_x,
                navi.current_y,
                navi.current_yaw,
            )
            return
        time.sleep(0.1)
    raise RuntimeError("Single-radar pose was not ready before timeout")


class StaticTargetVisualDescentMission(mission1.Mission):
    """
    复用任务一的视觉采集和目标发现流程，增加视觉闭环下降测试。

    visual_descend_and_hover() 接受 base_velocity。静止目标测试传 (0, 0)；
    后续移动目标伴飞下降时可传入伴飞速度估计，再叠加像素误差修正。
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._visual_descent_records: Deque[Dict[str, object]] = deque(
            maxlen=MAX_VISUAL_DESCENT_RECORDS
        )
        self._visual_descent_records_dropped = 0
        self._digital_output_enabled = True

    @staticmethod
    def _limit_horizontal_velocity(vector: np.ndarray) -> np.ndarray:
        speed = float(np.linalg.norm(vector))
        if speed > VISUAL_HORIZONTAL_SPEED_LIMIT:
            return vector * (VISUAL_HORIZONTAL_SPEED_LIMIT / speed)
        return vector

    @staticmethod
    def _pixel_correction(x_px: float, y_px: float) -> np.ndarray:
        error = np.array([x_px, y_px], dtype=float)
        error[np.abs(error) <= VISUAL_CORRECTION_DEADBAND_PX] = 0.0
        correction = error * VISUAL_CORRECTION_GAIN
        return StaticTargetVisualDescentMission._limit_horizontal_velocity(
            correction
        )

    def _record_visual_descent(
        self,
        started_at: float,
        phase: str,
        x_px: Optional[float],
        y_px: Optional[float],
        vel_x: int,
        vel_y: int,
    ) -> None:
        if (
            len(self._visual_descent_records)
            == self._visual_descent_records.maxlen
        ):
            self._visual_descent_records_dropped += 1
        self._visual_descent_records.append(
            {
                "elapsed_s": time.monotonic() - started_at,
                "phase": phase,
                "height_cm": float(self.navi.current_height),
                "x_px": x_px,
                "y_px": y_px,
                "pixel_distance_px": (
                    math.hypot(x_px, y_px)
                    if x_px is not None and y_px is not None
                    else None
                ),
                "velocity_x_cm_s": vel_x,
                "velocity_y_cm_s": vel_y,
                "speed_cm_s": math.hypot(vel_x, vel_y),
                "digital_output_0_enabled": self._digital_output_enabled,
            }
        )

    def _set_visual_horizontal_command(self, vel_x: int, vel_y: int) -> None:
        # track_landing_marker 与飞控实时控制均采用前/左为正的机体系。
        self.navi.update_realtime_control(
            vel_x=int(vel_x),
            vel_y=int(vel_y),
            yaw=0,
        )

    def visual_descend_and_hover(
        self,
        target_height: float,
        hover_seconds: float,
        base_velocity: Tuple[float, float] = (0.0, 0.0),
    ) -> None:
        """
        下降过程中用视觉修正水平位置，并在目标高度继续视觉悬停。

        base_velocity 是目标移动速度的估计值，单位 cm/s、前/左为正。
        静止目标使用 (0, 0)。视觉丢失时不继续发送旧水平速度，并暂停
        下降；在限定时间内未重新发现目标则中止测试并保持当前高度。
        """
        target_height = float(target_height)
        hover_seconds = float(hover_seconds)
        base = np.asarray(base_velocity, dtype=float)
        if (
            target_height < 0
            or hover_seconds < 0
            or base.shape != (2,)
            or not np.all(np.isfinite(base))
        ):
            raise ValueError("Invalid visual descent parameters")
        base = self._limit_horizontal_velocity(base)

        navi = self.navi
        navi.navigation_stop_here()
        navi.navigation_flag = False
        navi.keep_height_flag = True
        navi.set_height(target_height)

        filtered_velocity = base.copy()
        started_at = time.monotonic()
        lost_started_at: Optional[float] = None
        height_reached_since: Optional[float] = None
        low_hover_elapsed = 0.0
        last_loop_at = started_at
        descent_reached = False
        logger.info(
            "[TEST] Visual descent started: target={}cm, base=({:.1f}, {:.1f})cm/s",
            target_height,
            base[0],
            base[1],
        )

        try:
            while not self.stop_event.is_set():
                now = time.monotonic()
                dt = min(max(now - last_loop_at, 0.0), 0.2)
                last_loop_at = now

                state_fresh = bool(
                    getattr(self.fc.state, "is_fresh", lambda _age: False)(0.5)
                )
                if not state_fresh:
                    raise RuntimeError("Flight-controller telemetry became stale")
                if not self.fc.state.unlock.value:
                    raise RuntimeError("Aircraft locked during visual descent")
                if not navi.pose_is_fresh():
                    raise RuntimeError("Radar navigation pose became stale")
                if (
                    not descent_reached
                    and now - started_at > DESCENT_TIMEOUT_SECONDS
                ):
                    raise RuntimeError("Visual descent height timeout")

                self._raise_if_vision_failed()
                sample = self._latest_vision_sample()
                sample_fresh = bool(
                    sample is not None
                    and now - sample[1] <= mission1.VISION_SAMPLE_STALE_SECONDS
                    and sample[2] is not None
                    and sample[3] is not None
                )

                if not sample_fresh:
                    if lost_started_at is None:
                        lost_started_at = now
                        navi.set_height(float(navi.current_height))
                        logger.warning(
                            "[TEST] Marker lost; pause descent and clear horizontal speed"
                        )
                    self._set_visual_horizontal_command(0, 0)
                    self._record_visual_descent(
                        started_at,
                        "vision_lost",
                        None,
                        None,
                        0,
                        0,
                    )
                    if now - lost_started_at > VISION_LOSS_TIMEOUT_SECONDS:
                        raise RuntimeError(
                            "Marker was not reacquired during visual descent"
                        )
                    self.stop_event.wait(VISUAL_CONTROL_PERIOD_SECONDS)
                    continue

                _, _, x_px_value, y_px_value = sample
                x_px = float(x_px_value)
                y_px = float(y_px_value)
                if lost_started_at is not None:
                    lost_started_at = None
                    navi.set_height(target_height)
                    logger.info("[TEST] Marker reacquired; resume descent")

                desired_velocity = base + self._pixel_correction(x_px, y_px)
                desired_velocity = self._limit_horizontal_velocity(
                    desired_velocity
                )
                filtered_velocity += VISUAL_FILTER_ALPHA * (
                    desired_velocity - filtered_velocity
                )
                filtered_velocity = self._limit_horizontal_velocity(
                    filtered_velocity
                )
                vel_x = int(round(float(filtered_velocity[0])))
                vel_y = int(round(float(filtered_velocity[1])))
                self._set_visual_horizontal_command(vel_x, vel_y)

                height_in_range = bool(
                    abs(float(navi.current_height) - target_height)
                    <= HEIGHT_TOLERANCE
                )
                if not descent_reached:
                    if height_in_range:
                        if height_reached_since is None:
                            height_reached_since = now
                        elif now - height_reached_since >= HEIGHT_CONFIRM_SECONDS:
                            descent_reached = True
                            low_hover_elapsed = 0.0
                            self.fc.set_digital_output(0, False)
                            self._digital_output_enabled = False
                            logger.info(
                                "[TEST] Reached {}cm; digital output 0 disabled; "
                                "start {:.1f}s visual hover",
                                target_height,
                                hover_seconds,
                            )
                    else:
                        height_reached_since = None
                    phase = "height_confirm" if height_in_range else "descent"
                else:
                    phase = "low_hover"
                    if height_in_range:
                        low_hover_elapsed += dt
                    else:
                        low_hover_elapsed = 0.0
                    if low_hover_elapsed >= hover_seconds:
                        break

                self._record_visual_descent(
                    started_at,
                    phase,
                    x_px,
                    y_px,
                    vel_x,
                    vel_y,
                )
                self.stop_event.wait(VISUAL_CONTROL_PERIOD_SECONDS)

            if self.stop_event.is_set():
                raise RuntimeError("Mission stopped during visual descent")
        finally:
            # 退出视觉接管后，以当前点恢复水平 PID；高度 PID 保持当前设定值。
            self._set_visual_horizontal_command(0, 0)
            navi.stop_move()

        logger.info(
            "[TEST] Visual descent and {:.1f}s low hover completed",
            hover_seconds,
        )

    def write_visual_descent_log(self) -> Optional[Path]:
        records: List[Dict[str, object]] = list(self._visual_descent_records)
        if not records:
            logger.warning("[TEST] No visual-descent records to write")
            return None

        log_dir = Path(__file__).resolve().parent / "fc_log"
        log_dir.mkdir(parents=True, exist_ok=True)
        log_path = log_dir / (
            "mission1_26_visual_descent_"
            + datetime.now().strftime("%Y%m%d_%H%M%S")
            + ".csv"
        )
        fieldnames = [
            "elapsed_s",
            "phase",
            "height_cm",
            "x_px",
            "y_px",
            "pixel_distance_px",
            "velocity_x_cm_s",
            "velocity_y_cm_s",
            "speed_cm_s",
            "digital_output_0_enabled",
        ]
        with log_path.open("w", newline="", encoding="utf-8") as file:
            writer = csv.DictWriter(file, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(records)
        if self._visual_descent_records_dropped:
            logger.warning(
                "[TEST] Visual-descent log discarded {} oldest records",
                self._visual_descent_records_dropped,
            )
        logger.info("[TEST] Visual-descent log written to {}", log_path)
        return log_path

    def run(self) -> None:
        navi = self.navi
        navi.set_navigation_speed(mission1.PURSUIT_SPEED)
        navi.set_vertical_speed(mission1.VERTICAL_SPEED)
        navi.start(mode="radar")
        logger.info("[TEST] Single-radar navigation started")

        wait_for_radar_pose(navi, self.radar)
        navi.calibrate_basepoint()
        calibrated_at = time.monotonic()
        wait_for_radar_pose(navi, self.radar, newer_than=calibrated_at)
        logger.info("[TEST] Radar basepoint calibrated: {}", navi.basepoint)

        self._start_vision_tracker()
        self.wait_for_takeoff_signal()
        if self.stop_event.is_set():
            return

        navi.pointing_takeoff(
            mission1.TAKEOFF_POINT,
            target_height=mission1.CRUISE_HEIGHT,
            takeoff_alt_thres=TAKEOFF_ALT_THRESHOLD,
        )
        navi.set_yaw(0)
        if not navi.wait_for_yaw():
            raise RuntimeError("Yaw stabilization was not confirmed")

        logger.info("[TEST] Navigate to entry point {}", mission1.ENTRY_POINT)
        if not navi.navigation_to_waypoint(mission1.ENTRY_POINT, wait=True):
            raise RuntimeError("Failed to reach entry point")

        forward_target = np.array(
            [
                mission1.ENTRY_POINT[0] + mission1.FORWARD_GUIDANCE_DISTANCE,
                mission1.ENTRY_POINT[1],
            ]
        )
        self._clear_vision_samples()
        navi.set_navigation_speed(mission1.PURSUIT_SPEED)
        navi.switch_pid("navi")
        navi.direct_set_waypoint(forward_target)
        logger.info("[TEST] Pursuing stationary target along +x")
        self._wait_until_target_detected(forward_target[0])

        self.visual_descend_and_hover(
            target_height=DESCENT_TARGET_HEIGHT,
            hover_seconds=LOW_HOVER_SECONDS,
            base_velocity=(0.0, 0.0),
        )
        self._stop_vision_tracker()

        navi.set_height(float(mission1.CRUISE_HEIGHT))
        navi.keep_height_flag = True
        if not navi.wait_for_height(
            height_thres=HEIGHT_TOLERANCE,
            timeout=ASCENT_TIMEOUT_SECONDS,
        ):
            raise RuntimeError("Failed to return to cruise height")
        logger.info("[TEST] Returned to {}cm cruise height", mission1.CRUISE_HEIGHT)

        navi.set_navigation_speed(mission1.PURSUIT_SPEED)
        if not navi.navigation_to_waypoint(mission1.TAKEOFF_POINT, wait=True):
            raise RuntimeError("Failed to return to takeoff point")
        if not navi.pointing_landing(
            mission1.TAKEOFF_POINT,
            height_timeout=LANDING_HEIGHT_TIMEOUT_SECONDS,
        ):
            raise RuntimeError("Failed to land at takeoff point")
        logger.info("[TEST] Visual descent flight completed")


def emergency_land(fc: FC_Controller) -> None:
    """异常退出时请求降落；未确认落地前不强制锁桨。"""
    logger.warning("[TEST] Flight interrupted; requesting emergency landing")
    fc.set_flight_mode(fc.PROGRAM_MODE)
    time.sleep(0.1)
    fc.stablize()
    fc.land()
    if not fc.wait_for_lock(timeout_s=20):
        logger.error(
            "[TEST] Landing lock was not confirmed; keep landing command active"
        )
        fc.land()


def main() -> None:
    fc = FC_Controller()
    radar = LD_Radar()
    stop_event = threading.Event()
    navi: Optional[Navigation] = None
    mission: Optional[StaticTargetVisualDescentMission] = None
    digital_output_enabled = False

    try:
        fc.start_listen_serial(
            serial_dev=mission1.FC_SERIAL_DEV,
            print_state=False,
        )
        if not fc.wait_for_connection(timeout_s=10):
            raise RuntimeError("Flight-controller connection timeout")
        if not fc.state.is_fresh(0.5):
            raise RuntimeError("Flight-controller telemetry is stale")
        if fc.state.unlock.value:
            raise RuntimeError(
                "Flight controller is already unlocked; test will not take control"
            )
        logger.info("[TEST] Flight controller connected through direct serial")

        fc.set_digital_output(0, True)
        digital_output_enabled = True
        logger.info("[TEST] Digital output 0 enabled")
        wait_for_terminal_start_command()

        radar.debug = False
        radar.start()
        logger.info("[TEST] Single radar started")

        navi = SingleRadarNavigation(
            fc=fc,
            radar=radar,
            stop_event=stop_event,
        )
        mission = StaticTargetVisualDescentMission(
            fc=fc,
            radar=radar,
            navi=navi,
            stop_event=stop_event,
        )
        mission.run()
    except KeyboardInterrupt:
        logger.warning("[TEST] Interrupted by user")
    except Exception:
        logger.exception("[TEST] Static-target visual descent test failed")
    finally:
        if mission is not None:
            try:
                mission.stop()
            except Exception:
                logger.exception("[TEST] Failed to stop mission")
            try:
                mission.write_visual_descent_log()
            except Exception:
                logger.exception("[TEST] Failed to write visual-descent log")
        elif navi is not None:
            try:
                navi.stop()
            except Exception:
                logger.exception("[TEST] Failed to stop navigation")

        if digital_output_enabled:
            try:
                fc.set_digital_output(0, False)
                logger.info("[TEST] Digital output 0 disabled during cleanup")
            except Exception:
                logger.exception("[TEST] Failed to disable digital output 0")

        try:
            if fc.connected and fc.state.unlock.value:
                emergency_land(fc)
        except Exception:
            logger.exception("[TEST] Emergency landing request failed")

        try:
            if radar.running:
                radar.stop()
        except Exception:
            logger.exception("[TEST] Failed to stop radar")

        try:
            fc.close()
        except Exception:
            logger.exception("[TEST] Failed to close flight controller")
        logger.info("[TEST] Static-target visual descent test finished")


if __name__ == "__main__":
    main()
