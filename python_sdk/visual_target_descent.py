"""
视觉目标下降与不锁桨接地控制。

该模块只负责任务层控制编排，不初始化飞控、雷达或相机。调用方提供
最新视觉样本读取函数，两个公开功能共享相同的像素修正、视觉丢失、
实时控制租约和安全检查：

- descend_to_height(): 视觉修正下下降到指定激光高度并悬停；
- land_without_lock(): 先快速下降到末段高度，再视觉修正下降到目标
  平面，组合确认接地后保持解锁和零速度。

视觉坐标和速度均采用机体系：x 向前为正，y 向左为正，单位分别为
像素和 cm/s。base_velocity 可用于后续叠加移动目标的伴飞速度估计。
"""

import time
from typing import Callable, Optional, Tuple

import numpy as np
from loguru import logger


VisionSample = Tuple[int, float, Optional[float], Optional[float]]
LatestVisionSample = Callable[[], Optional[VisionSample]]
RecordCallback = Callable[
    [float, str, Optional[float], Optional[float], int, int],
    None,
]


class VisualTargetDescentController:
    """复用视觉水平修正的指定高度下降和不锁桨接地控制器。"""

    def __init__(
        self,
        fc,
        navi,
        stop_event,
        latest_vision_sample: LatestVisionSample,
        raise_if_vision_failed: Callable[[], None],
        record_callback: Optional[RecordCallback] = None,
        correction_gain: float = 0.15,
        correction_deadband_px: float = 3.0,
        horizontal_speed_limit: float = 8.0,
        filter_alpha: float = 0.30,
        control_period: float = 0.05,
        vision_sample_stale_seconds: float = 0.35,
        vision_loss_timeout: float = 1.0,
    ):
        values = np.asarray(
            [
                correction_gain,
                correction_deadband_px,
                horizontal_speed_limit,
                filter_alpha,
                control_period,
                vision_sample_stale_seconds,
                vision_loss_timeout,
            ],
            dtype=float,
        )
        if not np.all(np.isfinite(values)):
            raise ValueError("Visual descent controller parameters must be finite")
        if correction_gain < 0 or correction_deadband_px < 0:
            raise ValueError("Visual correction gain and deadband must be non-negative")
        if horizontal_speed_limit <= 0:
            raise ValueError("Horizontal speed limit must be greater than 0")
        if not 0 < filter_alpha <= 1:
            raise ValueError("Filter alpha must be in (0, 1]")
        if min(
            control_period,
            vision_sample_stale_seconds,
            vision_loss_timeout,
        ) <= 0:
            raise ValueError("Visual timing parameters must be greater than 0")

        self.fc = fc
        self.navi = navi
        self.stop_event = stop_event
        self._latest_vision_sample = latest_vision_sample
        self._raise_if_vision_failed = raise_if_vision_failed
        self._record_callback = record_callback
        self.correction_gain = float(correction_gain)
        self.correction_deadband_px = float(correction_deadband_px)
        self.horizontal_speed_limit = float(horizontal_speed_limit)
        self.filter_alpha = float(filter_alpha)
        self.control_period = float(control_period)
        self.vision_sample_stale_seconds = float(
            vision_sample_stale_seconds
        )
        self.vision_loss_timeout = float(vision_loss_timeout)

    def _limit_horizontal_velocity(self, vector: np.ndarray) -> np.ndarray:
        speed = float(np.linalg.norm(vector))
        if speed > self.horizontal_speed_limit:
            return vector * (self.horizontal_speed_limit / speed)
        return vector

    def _validate_base_velocity(
        self,
        base_velocity: Tuple[float, float],
    ) -> np.ndarray:
        base = np.asarray(base_velocity, dtype=float)
        if base.shape != (2,) or not np.all(np.isfinite(base)):
            raise ValueError("base_velocity must contain two finite values")
        return self._limit_horizontal_velocity(base)

    def _fresh_offset(
        self,
        now: float,
    ) -> Optional[Tuple[float, float]]:
        self._raise_if_vision_failed()
        sample = self._latest_vision_sample()
        if (
            sample is None
            or now - sample[1] > self.vision_sample_stale_seconds
            or sample[2] is None
            or sample[3] is None
        ):
            return None
        x_px = float(sample[2])
        y_px = float(sample[3])
        if not np.all(np.isfinite(np.asarray([x_px, y_px], dtype=float))):
            return None
        return x_px, y_px

    def _next_horizontal_velocity(
        self,
        base: np.ndarray,
        filtered: np.ndarray,
        x_px: float,
        y_px: float,
    ) -> Tuple[np.ndarray, int, int]:
        error = np.asarray([x_px, y_px], dtype=float)
        error[
            np.abs(error) <= self.correction_deadband_px
        ] = 0.0
        desired = base + error * self.correction_gain
        desired = self._limit_horizontal_velocity(desired)
        filtered += self.filter_alpha * (desired - filtered)
        filtered = self._limit_horizontal_velocity(filtered)
        return (
            filtered,
            int(round(float(filtered[0]))),
            int(round(float(filtered[1]))),
        )

    def _record(
        self,
        started_at: float,
        phase: str,
        offset: Optional[Tuple[float, float]],
        vel_x: int,
        vel_y: int,
    ) -> None:
        if self._record_callback is None:
            return
        x_px = offset[0] if offset is not None else None
        y_px = offset[1] if offset is not None else None
        self._record_callback(
            started_at,
            phase,
            x_px,
            y_px,
            vel_x,
            vel_y,
        )

    def _validate_flight_state(self, require_pose: bool = True) -> None:
        if self.stop_event is not None and self.stop_event.is_set():
            raise RuntimeError("External stop requested during visual descent")
        state_fresh = bool(
            getattr(self.fc.state, "is_fresh", lambda _age: False)(0.5)
        )
        if not state_fresh:
            raise RuntimeError("Flight-controller telemetry became stale")
        if not self.fc.state.unlock.value:
            raise RuntimeError("Aircraft locked during visual descent")
        if require_pose and not self.navi.pose_is_fresh():
            raise RuntimeError("Navigation pose became stale")

    def _start_override(self, keep_height: bool) -> None:
        if not self.navi._start_velocity_override(
            keep_height=keep_height,
            require_pose=True,
        ):
            raise RuntimeError("Failed to start visual velocity override")

    def _update_override(
        self,
        vel_x: int,
        vel_y: int,
        vel_z: Optional[float] = None,
    ) -> None:
        self.navi._update_velocity_override(
            vel_x=vel_x,
            vel_y=vel_y,
            vel_z=vel_z,
            yaw=0,
            frame="body",
        )

    def descend_to_height(
        self,
        target_height: float,
        hover_seconds: float = 0.0,
        base_velocity: Tuple[float, float] = (0.0, 0.0),
        height_tolerance: float = 5.0,
        height_confirm_time: float = 0.4,
        timeout: float = 15.0,
        on_height_reached: Optional[Callable[[], None]] = None,
    ) -> None:
        """伴随视觉水平修正下降到指定激光高度，并继续悬停指定时间。"""
        values = np.asarray(
            [
                target_height,
                hover_seconds,
                height_tolerance,
                height_confirm_time,
                timeout,
            ],
            dtype=float,
        )
        if not np.all(np.isfinite(values)):
            raise ValueError("Height-descent parameters must be finite")
        target_height = float(target_height)
        hover_seconds = float(hover_seconds)
        height_tolerance = float(height_tolerance)
        height_confirm_time = float(height_confirm_time)
        timeout = float(timeout)
        if target_height < 0 or hover_seconds < 0:
            raise ValueError("Target height and hover time must be non-negative")
        if min(height_tolerance, height_confirm_time, timeout) <= 0:
            raise ValueError(
                "Height tolerance, confirmation time and timeout must be positive"
            )

        base = self._validate_base_velocity(base_velocity)
        filtered = base.copy()
        started_at = time.monotonic()
        last_loop_at = started_at
        lost_started_at: Optional[float] = None
        height_reached_since: Optional[float] = None
        reached = False
        hover_elapsed = 0.0

        self._start_override(keep_height=True)
        self.navi.set_height(target_height)
        logger.info(
            "[VISUAL-DESCENT] Descend to {}cm, base=({:.1f}, {:.1f})cm/s",
            target_height,
            base[0],
            base[1],
        )
        try:
            while True:
                now = time.monotonic()
                dt = min(max(now - last_loop_at, 0.0), 0.2)
                last_loop_at = now
                self._validate_flight_state()
                if not reached and now - started_at > timeout:
                    raise RuntimeError("Visual height descent timeout")

                offset = self._fresh_offset(now)
                if offset is None:
                    if lost_started_at is None:
                        lost_started_at = now
                        self.navi.set_height(float(self.navi.current_height))
                        logger.warning(
                            "[VISUAL-DESCENT] Marker lost; pause descent"
                        )
                    self._update_override(0, 0)
                    self._record(started_at, "vision_lost", None, 0, 0)
                    if now - lost_started_at > self.vision_loss_timeout:
                        raise RuntimeError(
                            "Marker was not reacquired during height descent"
                        )
                    self.stop_event.wait(self.control_period)
                    continue

                if lost_started_at is not None:
                    lost_started_at = None
                    self.navi.set_height(target_height)
                    logger.info(
                        "[VISUAL-DESCENT] Marker reacquired; resume descent"
                    )
                filtered, vel_x, vel_y = self._next_horizontal_velocity(
                    base,
                    filtered,
                    offset[0],
                    offset[1],
                )
                self._update_override(vel_x, vel_y)

                height_in_range = bool(
                    abs(float(self.navi.current_height) - target_height)
                    <= height_tolerance
                )
                if not reached:
                    if height_in_range:
                        if height_reached_since is None:
                            height_reached_since = now
                        elif (
                            now - height_reached_since
                            >= height_confirm_time
                        ):
                            reached = True
                            hover_elapsed = 0.0
                            if on_height_reached is not None:
                                on_height_reached()
                            logger.info(
                                "[VISUAL-DESCENT] Reached {}cm; hover {:.1f}s",
                                target_height,
                                hover_seconds,
                            )
                    else:
                        height_reached_since = None
                    phase = (
                        "height_confirm" if height_in_range else "descent"
                    )
                else:
                    phase = "low_hover"
                    if height_in_range:
                        hover_elapsed += dt
                    else:
                        hover_elapsed = 0.0
                    if hover_elapsed >= hover_seconds:
                        self._record(
                            started_at,
                            phase,
                            offset,
                            vel_x,
                            vel_y,
                        )
                        break

                self._record(
                    started_at,
                    phase,
                    offset,
                    vel_x,
                    vel_y,
                )
                self.stop_event.wait(self.control_period)
        except BaseException:
            hover_height = float(self.navi.current_height)
            if not self.navi._stop_velocity_override(
                restore_hover=True,
                hover_height=hover_height,
            ):
                logger.error(
                    "[VISUAL-DESCENT] Failed to restore hover after abort"
                )
            raise

        if not self.navi._stop_velocity_override(
            restore_hover=True,
            hover_height=target_height,
        ):
            raise RuntimeError(
                "Visual descent reached target but hover was not restored"
            )

    def land_without_lock(
        self,
        base_velocity: Tuple[float, float] = (0.0, 0.0),
        approach_height: float = 20.0,
        approach_timeout: float = 15.0,
        approach_height_confirm_time: float = 0.2,
        final_descent_speed: float = 6.0,
        touchdown_alt_thres: float = 12.0,
        touchdown_vertical_speed_thres: float = 2.5,
        touchdown_confirm_time: float = 0.4,
        touchdown_height_range: float = 1.5,
        final_descent_timeout: float = 8.0,
        dwell_seconds: float = 5.0,
    ) -> None:
        """
        视觉修正下降并组合确认接地；接地后保持解锁、零速度并短暂停留。

        不调用 fc.land() 和 fc.lock()。平台造成的低高度只满足一个条件，
        还必须同时满足低垂直速度和持续高度稳定，才认为接触目标平面。
        """
        values = np.asarray(
            [
                approach_height,
                approach_timeout,
                approach_height_confirm_time,
                final_descent_speed,
                touchdown_alt_thres,
                touchdown_vertical_speed_thres,
                touchdown_confirm_time,
                touchdown_height_range,
                final_descent_timeout,
                dwell_seconds,
            ],
            dtype=float,
        )
        if not np.all(np.isfinite(values)):
            raise ValueError("Visual landing parameters must be finite")
        if min(
            approach_height,
            approach_timeout,
            approach_height_confirm_time,
            final_descent_speed,
            touchdown_alt_thres,
            touchdown_vertical_speed_thres,
            touchdown_confirm_time,
            touchdown_height_range,
            final_descent_timeout,
        ) <= 0:
            raise ValueError("Visual landing parameters must be greater than 0")
        if dwell_seconds < 0:
            raise ValueError("dwell_seconds must be non-negative")
        if approach_height <= touchdown_alt_thres:
            raise ValueError(
                "approach_height must be above touchdown_alt_thres"
            )
        if final_descent_speed > 30:
            raise ValueError("final_descent_speed must not exceed 30cm/s")
        if touchdown_vertical_speed_thres > 10:
            raise ValueError(
                "touchdown_vertical_speed_thres must not exceed 10cm/s"
            )

        base = self._validate_base_velocity(base_velocity)
        self.descend_to_height(
            target_height=float(approach_height),
            hover_seconds=0.0,
            base_velocity=base_velocity,
            height_confirm_time=float(approach_height_confirm_time),
            timeout=float(approach_timeout),
        )

        filtered = base.copy()
        started_at = time.monotonic()
        lost_started_at: Optional[float] = None
        touchdown_candidate_since: Optional[float] = None
        candidate_min_height = 0.0
        candidate_max_height = 0.0
        touchdown_confirmed = False

        self._start_override(keep_height=False)
        logger.info(
            "[VISUAL-LANDING] Final descent: speed={}cm/s, alt<={}cm, "
            "|vel_z|<={}cm/s",
            final_descent_speed,
            touchdown_alt_thres,
            touchdown_vertical_speed_thres,
        )
        try:
            while True:
                now = time.monotonic()
                self._validate_flight_state()
                if now - started_at > final_descent_timeout:
                    raise RuntimeError("Visual final descent timeout")

                offset = self._fresh_offset(now)
                if offset is None:
                    if lost_started_at is None:
                        lost_started_at = now
                        logger.warning(
                            "[VISUAL-LANDING] Marker lost; pause final descent"
                        )
                    touchdown_candidate_since = None
                    self._update_override(0, 0, vel_z=0)
                    self._record(
                        started_at,
                        "landing_vision_lost",
                        None,
                        0,
                        0,
                    )
                    if now - lost_started_at > self.vision_loss_timeout:
                        raise RuntimeError(
                            "Marker was not reacquired during final descent"
                        )
                    self.stop_event.wait(self.control_period)
                    continue

                if lost_started_at is not None:
                    lost_started_at = None
                    logger.info(
                        "[VISUAL-LANDING] Marker reacquired; resume final descent"
                    )
                filtered, vel_x, vel_y = self._next_horizontal_velocity(
                    base,
                    filtered,
                    offset[0],
                    offset[1],
                )

                alt_now = float(self.fc.state.alt_add.value)
                vel_z_now = abs(float(self.fc.state.vel_z.value))
                touchdown_candidate = bool(
                    alt_now <= touchdown_alt_thres
                    and vel_z_now <= touchdown_vertical_speed_thres
                )
                if touchdown_candidate:
                    vel_x = 0
                    vel_y = 0
                    if touchdown_candidate_since is None:
                        touchdown_candidate_since = now
                        candidate_min_height = alt_now
                        candidate_max_height = alt_now
                    else:
                        candidate_min_height = min(
                            candidate_min_height,
                            alt_now,
                        )
                        candidate_max_height = max(
                            candidate_max_height,
                            alt_now,
                        )
                        if (
                            now - touchdown_candidate_since
                            >= touchdown_confirm_time
                        ):
                            if (
                                candidate_max_height - candidate_min_height
                                <= touchdown_height_range
                            ):
                                touchdown_confirmed = True
                                self._record(
                                    started_at,
                                    "touchdown_confirmed",
                                    offset,
                                    0,
                                    0,
                                )
                                break
                            touchdown_candidate_since = now
                            candidate_min_height = alt_now
                            candidate_max_height = alt_now
                else:
                    touchdown_candidate_since = None

                self._update_override(
                    vel_x,
                    vel_y,
                    vel_z=-float(final_descent_speed),
                )
                self._record(
                    started_at,
                    (
                        "touchdown_confirm"
                        if touchdown_candidate
                        else "final_descent"
                    ),
                    offset,
                    vel_x,
                    vel_y,
                )
                self.stop_event.wait(self.control_period)
        except BaseException:
            hover_height = float(self.navi.current_height)
            if not self.navi._stop_velocity_override(
                restore_hover=True,
                hover_height=hover_height,
            ):
                logger.error(
                    "[VISUAL-LANDING] Failed to restore hover after abort"
                )
            raise

        if not touchdown_confirmed:
            raise RuntimeError("Visual landing ended without touchdown")
        if not self.navi._stop_velocity_override(restore_hover=False):
            raise RuntimeError("Touchdown confirmed but zero control failed")
        if not self.fc.state.unlock.value:
            raise RuntimeError("Aircraft unexpectedly locked after touchdown")

        dwell_started_at = time.monotonic()
        logger.warning(
            "[VISUAL-LANDING] Touchdown confirmed; remain unlocked for {:.1f}s",
            dwell_seconds,
        )
        while time.monotonic() - dwell_started_at < dwell_seconds:
            self._validate_flight_state(require_pose=False)
            self.navi.update_realtime_control(
                vel_x=0,
                vel_y=0,
                vel_z=0,
                yaw=0,
            )
            alt_now = float(self.fc.state.alt_add.value)
            if alt_now > touchdown_alt_thres + 3.0:
                raise RuntimeError(
                    "Aircraft left the touchdown-height zone during dwell"
                )
            self._record(
                dwell_started_at,
                "ground_dwell",
                None,
                0,
                0,
            )
            self.stop_event.wait(min(0.1, self.control_period))
        logger.info("[VISUAL-LANDING] Unlocked ground dwell completed")
