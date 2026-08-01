"""任务二：追及、移动目标伴飞降落、平台复飞和返航。

本入口会直连真实飞控、单雷达和相机并执行两次起飞与两次降落。运行前
必须确认 ``server_ros.py`` 及其他 ``FC_Server`` 已关闭，并清空追及
路线、移动平台和返航区域。任务二不控制任何数字输出通道。

起飞采用非定点垂直起飞（90 cm 一键离地后垂直爬升至 150 cm），
该阶段垂直速度设为 30 cm/s。返航开始时切换到 H 降落点检测，
下降至 60 cm 后以 30 像素阈值完成视觉校准，再在该点定点降落，
降落阶段垂直速度设为 15 cm/s。相机全程保持开启，不重复开关。
"""

import math
import threading
import time
from typing import Optional, Tuple

from loguru import logger

from FlightController import FC_Controller
from FlightController.Components import LD_Radar
from FlightController.Solutions.Navigation import Navigation
from fleet_bus.trace_buffer import TraceSamplingOptions
import mission1_26 as mission1
import mission1_26_base as mission_base
import mission1_26_visual_descent_test as descent_test
from moving_target_descent import (
    MovingTargetDescentConfig,
    MovingTargetDescentController,
)
from mission2_26_logic import (
    ARC_END,
    ARC_START,
    ClockwiseArcVelocityPredictor,
    LowAltitudeTargetOffset,
    PURSUIT_SLOWDOWN_POINT,
    PursuitSpeedSchedule,
    RoutePassGate,
    build_pursuit_trajectory_to_b,
    land_on_target_and_confirm_lock,
    locked_red_led_dwell,
    retakeoff_from_moving_platform,
)
from mission2_26_safety import EscortXBoundaryVelocityGuard
from visual_target_descent import PreDescentTimeoutError


CRUISE_HEIGHT = 150.0
VERTICAL_SPEED = 20.0
PURSUIT_SPEED = 20.0
PURSUIT_APPROACH_SPEED = 25.0
PURSUIT_AFTER_SLOWDOWN_SPEED = 15.0
RETURN_SPEED = 20.0
PURSUIT_POSITION_THRESHOLD = 7.5
TARGET_DETECTION_PIXEL_THRESHOLD = 30.0
C_POINT_SEARCH_TIMEOUT_SECONDS = 20.0
ARC_VELOCITY_POSITION_TOLERANCE = 20.0
ESCORT_MAX_X = 357.5

ESCORT_SPEED_MIDPOINT = 12.0
ESCORT_STABLE_SECONDS = 4.0
ESCORT_GATE_TIMEOUT_SECONDS = 90.0
# 任务二专用：保留 12cm/s 初始估计，估计器合速度最多 15cm/s，
# 视觉控制实际输出最多 20cm/s。任务一仍使用共享默认配置。
TASK2_ESTIMATOR_SPEED_LIMIT = 15.0
TASK2_OUTPUT_SPEED_LIMIT = 20.0
# 目标仍可稳定识别时允许在圆弧终点附近提前进入最终下降。
TARGET_DESCENT_GATE_RADIUS = 40.0
# 稳定伴飞 4 秒后先下降到该中间高度；经过门控点后继续下降到最终降落高度。
TARGET_DESCENT_INTERMEDIATE_HEIGHT = 100.0
TARGET_LANDING_HEIGHT = 25.0
TARGET_OFFSET_START_HEIGHT = 50.0
TARGET_OFFSET_FINAL_X_PX = -30.0
TARGET_DESCENT_TIMEOUT_SECONDS = 15.0
TARGET_LANDING_LOCK_TIMEOUT_SECONDS = 20.0
LOCKED_DWELL_SECONDS = 5.0
PLATFORM_RETAKEOFF_HEIGHT = 30
PLATFORM_RETAKEOFF_HEIGHT_TIMEOUT_SECONDS = 15.0


class TargetNotFoundError(RuntimeError):
    """Raised when the pursuit trajectory ends without detecting the target."""


class Mission2Signals(mission1.MissionGroundStationSignals):
    """任务二新增阶段的通信占位和 FleetBus 状态映射。"""

    def send_takeoff_succeeded(self) -> None:
        self._send(
            "task2_cruise_height_reached",
            mission_base.MissionOperationState.CRUISING,
        )

    def send_pursuit_started(self) -> None:
        self._send(
            "task2_pursuit_started",
            mission_base.MissionOperationState.CRUISING,
        )

    def send_target_descent_started(self) -> None:
        self._send(
            "task2_target_descent_started",
            mission_base.MissionOperationState.ESCORTING,
        )

    def send_target_locked(self) -> None:
        self._send(
            "task2_target_locked",
            mission_base.MissionOperationState.ON_CAR,
        )

    def send_target_landing_started(self) -> None:
        self._send(
            "task2_target_landing_started",
            mission_base.MissionOperationState.LANDING_ON_CAR,
        )

    def send_retakeoff_started(self) -> None:
        self._send(
            "task2_retakeoff_started",
            mission_base.MissionOperationState.TAKEOFF,
        )

    def send_retakeoff_succeeded(self) -> None:
        self._send(
            "task2_retakeoff_succeeded",
            mission_base.MissionOperationState.CRUISING,
        )


class Task2Mission(mission1.MovingTargetVisualDescentMission):
    """执行任务二完整飞行状态机。"""

    LOG_PREFIX = "mission2_26_visual_landing_"

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.signals = Mission2Signals(self)
        self._digital_output_enabled = False
        self.moving_target_descent = MovingTargetDescentController(
            fc=self.fc,
            navi=self.navi,
            stop_event=self.stop_event,
            latest_vision_sample=self._latest_vision_sample,
            raise_if_vision_failed=self._raise_if_vision_failed,
            record_callback=self._record_moving_descent,
            config=MovingTargetDescentConfig(
                estimator_speed_limit=TASK2_ESTIMATOR_SPEED_LIMIT,
                horizontal_command_limit=TASK2_OUTPUT_SPEED_LIMIT,
            ),
        )
        self._route_gate = RoutePassGate(radius=TARGET_DESCENT_GATE_RADIUS)
        self._arc_velocity_predictor = ClockwiseArcVelocityPredictor(
            position_tolerance=ARC_VELOCITY_POSITION_TOLERANCE,
        )
        self._low_altitude_target_offset = LowAltitudeTargetOffset(
            start_height=TARGET_OFFSET_START_HEIGHT,
            final_height=TARGET_LANDING_HEIGHT,
            final_x_px=TARGET_OFFSET_FINAL_X_PX,
            final_y_px=0.0,
        )
        self._escort_x_boundary_guard = EscortXBoundaryVelocityGuard(
            max_x=ESCORT_MAX_X,
        )
        self._escort_x_boundary_active = False
        self._pursuit_speed_schedule = PursuitSpeedSchedule(
            initial_speed=PURSUIT_SPEED,
            approach_speed=PURSUIT_APPROACH_SPEED,
            after_slowdown_speed=PURSUIT_AFTER_SLOWDOWN_SPEED,
        )
        self._pursuit_trajectory = build_pursuit_trajectory_to_b(
            altitude=CRUISE_HEIGHT,
            arc_step_degrees=10,
        )
        self._platform_retakeoff_hold_point: Optional[
            Tuple[float, float]
        ] = None

    def _route_gate_is_open(self) -> bool:
        was_open = self._route_gate.passed
        is_open = self._route_gate.update(
            self.navi.current_x,
            self.navi.current_y,
        )
        if is_open and not was_open:
            logger.info(
                "[MISSION2] Route gate passed near {} at "
                "({:.1f}, {:.1f})cm",
                ARC_END,
                self.navi.current_x,
                self.navi.current_y,
            )
        return is_open

    def _stop_pursuit_trajectory(self) -> None:
        self.navi.navigation_stop_here()
        deadline = time.monotonic() + 0.5
        while (
            self.navi.traj_running_event.is_set()
            and time.monotonic() < deadline
        ):
            self.stop_event.wait(0.02)
        if self.navi.traj_running_event.is_set():
            raise RuntimeError("Pursuit trajectory did not stop in time")

    def _update_pursuit_speed(self) -> None:
        target_x, target_y = self.navi.navigation_target
        new_speed = self._pursuit_speed_schedule.update(
            target_x,
            target_y,
            self.navi.current_x,
            self.navi.current_y,
        )
        if new_speed is None:
            return
        self.navi.set_navigation_speed(new_speed)
        logger.info(
            "[MISSION2] Pursuit speed changed to {:.1f}cm/s at "
            "position ({:.1f}, {:.1f}); trajectory target "
            "({:.1f}, {:.1f})",
            new_speed,
            self.navi.current_x,
            self.navi.current_y,
            target_x,
            target_y,
        )

    def _wait_until_target_detected_on_trajectory(
        self,
    ) -> Tuple[float, float]:
        last_sequence = -1
        while not self.stop_event.is_set():
            self._update_pursuit_speed()
            self._route_gate_is_open()
            self._raise_if_vision_failed()
            sample = self._latest_vision_sample()
            if sample is not None and sample[0] != last_sequence:
                sequence, captured_at, x_px, y_px = sample
                last_sequence = sequence
                if (
                    time.monotonic() - captured_at
                    <= mission_base.VISION_SAMPLE_STALE_SECONDS
                    and x_px is not None
                    and y_px is not None
                    and math.isfinite(float(x_px))
                    and math.isfinite(float(y_px))
                    and math.hypot(float(x_px), float(y_px))
                    < TARGET_DETECTION_PIXEL_THRESHOLD
                ):
                    self._stop_pursuit_trajectory()
                    logger.info(
                        "[MISSION2] Target detected during pursuit: "
                        "x_px={:.2f}, y_px={:.2f}",
                        x_px,
                        y_px,
                    )
                    return float(x_px), float(y_px)

            if not self.navi.traj_running_event.is_set():
                return self._wait_for_target_at_c()
            self.stop_event.wait(mission_base.ESCORT_CONTROL_PERIOD)
        raise RuntimeError("Task 2 stopped during target pursuit")

    def _wait_for_target_at_c(self) -> Tuple[float, float]:
        logger.warning(
            "[MISSION2] Target not detected by B {}; navigate directly "
            "to C {} and scan for up to {:.0f}s",
            ARC_START,
            ARC_END,
            C_POINT_SEARCH_TIMEOUT_SECONDS,
        )
        if not self.navi.navigation_to_waypoint(ARC_END, wait=True):
            raise RuntimeError("Failed to reach C for target search")

        self._clear_vision_samples()
        last_sequence = -1
        deadline = time.monotonic() + C_POINT_SEARCH_TIMEOUT_SECONDS
        while time.monotonic() < deadline:
            if self.stop_event.is_set():
                raise RuntimeError("Task 2 stopped during target search at C")
            self._raise_if_vision_failed()
            sample = self._latest_vision_sample()
            if sample is not None and sample[0] != last_sequence:
                sequence, captured_at, x_px, y_px = sample
                last_sequence = sequence
                if (
                    time.monotonic() - captured_at
                    <= mission_base.VISION_SAMPLE_STALE_SECONDS
                    and x_px is not None
                    and y_px is not None
                    and math.isfinite(float(x_px))
                    and math.isfinite(float(y_px))
                    and math.hypot(float(x_px), float(y_px))
                    < TARGET_DETECTION_PIXEL_THRESHOLD
                ):
                    logger.info(
                        "[MISSION2] Target detected while waiting at C: "
                        "x_px={:.2f}, y_px={:.2f}",
                        x_px,
                        y_px,
                    )
                    return float(x_px), float(y_px)
            self.stop_event.wait(mission_base.ESCORT_CONTROL_PERIOD)

        raise TargetNotFoundError(
            "Target was not detected within {:.0f}s at C".format(
                C_POINT_SEARCH_TIMEOUT_SECONDS
            )
        )

    def _follow_descend_and_land_on_target(self) -> None:
        def predict_target_velocity(
            velocity: Tuple[float, float],
            sample_dt: float,
        ) -> Tuple[float, float]:
            return self._arc_velocity_predictor.predict(
                velocity,
                sample_dt,
                self.navi.current_x,
                self.navi.current_y,
            )

        def apply_escort_boundary(
            velocity_x: int,
            velocity_y: int,
        ) -> Tuple[int, int]:
            active = self._escort_x_boundary_guard.is_active(
                self.navi.current_x
            )
            if active and not self._escort_x_boundary_active:
                logger.warning(
                    "[MISSION2-SAFETY] x={:.1f}cm exceeds {:.1f}cm; "
                    "force escort vx to zero",
                    self.navi.current_x,
                    ESCORT_MAX_X,
                )
            elif not active and self._escort_x_boundary_active:
                logger.info(
                    "[MISSION2-SAFETY] x returned within boundary; "
                    "release escort vx guard"
                )
            self._escort_x_boundary_active = active
            return self._escort_x_boundary_guard.apply(
                self.navi.current_x,
                velocity_x,
                velocity_y,
            )

        # 阶段1：稳定伴飞 4 秒后先下降到中间高度，不要求先经过门控点。
        self.moving_target_descent.follow_and_descend(
            target_height=TARGET_DESCENT_INTERMEDIATE_HEIGHT,
            stabilize_seconds=ESCORT_STABLE_SECONDS,
            stabilize_timeout=ESCORT_GATE_TIMEOUT_SECONDS,
            hover_seconds=0.0,
            initial_target_velocity=(ESCORT_SPEED_MIDPOINT, 0.0),
            height_tolerance=descent_test.HEIGHT_TOLERANCE,
            height_confirm_time=descent_test.HEIGHT_CONFIRM_SECONDS,
            descent_timeout=TARGET_DESCENT_TIMEOUT_SECONDS,
            on_descent_start=self.signals.send_target_descent_started,
            pre_descent_max_error_px=TARGET_DETECTION_PIXEL_THRESHOLD,
            velocity_predictor=predict_target_velocity,
            horizontal_command_guard=apply_escort_boundary,
        )
        logger.info(
            "[MISSION2] Reached intermediate descent height {:.0f}cm; "
            "waiting for route gate before continuing descent",
            TARGET_DESCENT_INTERMEDIATE_HEIGHT,
        )

        # 阶段2：经过门控点 (207.5,-187.5) 后继续下降到最终降落高度。
        # 沿用阶段1伴飞器的目标速度估计（不重置），目标速度不会突变。
        final_velocity = self.moving_target_descent.follow_and_descend(
            target_height=TARGET_LANDING_HEIGHT,
            stabilize_seconds=0.0,
            stabilize_timeout=ESCORT_GATE_TIMEOUT_SECONDS,
            hover_seconds=0.0,
            initial_target_velocity=(
                self.moving_target_descent.estimated_target_velocity
            ),
            height_tolerance=descent_test.HEIGHT_TOLERANCE,
            height_confirm_time=descent_test.HEIGHT_CONFIRM_SECONDS,
            descent_timeout=TARGET_DESCENT_TIMEOUT_SECONDS,
            on_descent_start=None,
            pre_descent_gate=self._route_gate_is_open,
            pre_descent_max_error_px=TARGET_DETECTION_PIXEL_THRESHOLD,
            velocity_predictor=predict_target_velocity,
            target_offset_provider=self._low_altitude_target_offset.offset,
            horizontal_command_guard=apply_escort_boundary,
            reset_estimator=False,
        )
        logger.info(
            "[MISSION2] Reached target landing height; estimated target "
            "velocity=({:.2f}, {:.2f})cm/s",
            final_velocity[0],
            final_velocity[1],
        )

        self.signals.send_target_landing_started()
        land_on_target_and_confirm_lock(
            self.fc,
            self.navi,
            lock_timeout=TARGET_LANDING_LOCK_TIMEOUT_SECONDS,
        )
        self.signals.send_target_locked()
        logger.info("[MISSION2] Target landing confirmed motor lock")

        locked_red_led_dwell(
            self.fc,
            self.stop_event,
            dwell_seconds=LOCKED_DWELL_SECONDS,
        )

        self.signals.send_retakeoff_started()
        hold_point = retakeoff_from_moving_platform(
            self.fc,
            self.navi,
            self.stop_event,
            target_height=CRUISE_HEIGHT,
            first_lift_height=PLATFORM_RETAKEOFF_HEIGHT,
            height_timeout=PLATFORM_RETAKEOFF_HEIGHT_TIMEOUT_SECONDS,
            height_tolerance=descent_test.HEIGHT_TOLERANCE,
        )
        self._platform_retakeoff_hold_point = hold_point
        self.signals.send_retakeoff_succeeded()
        logger.info(
            "[MISSION2] Platform retakeoff reached {}cm; hold point={}",
            CRUISE_HEIGHT,
            hold_point,
        )

        self.navi.set_yaw(0)
        if not self.navi.wait_for_yaw():
            raise RuntimeError(
                "Yaw stabilization was not confirmed after platform retakeoff"
            )

    def _return_home_and_land(self) -> None:
        self.signals.send_return_started()
        # 返航开始时切换到 H 降落点检测；相机保持全程开启。
        self.enable_h_landing_vision()
        self.navi.set_navigation_speed(RETURN_SPEED)
        if not self.navi.navigation_to_waypoint(
            mission_base.TAKEOFF_POINT,
            wait=True,
        ):
            raise RuntimeError("Failed to return to initial takeoff point")
        self.signals.send_landing_started()
        self._visual_h_landing_at_takeoff()

    def _recover_home_after_expected_failure(self, reason: str) -> None:
        logger.error("[MISSION2] Safe return requested: {}", reason)
        self.navi.navigation_stop_here()
        if not self.fc.state.unlock.value:
            self.set_fleet_status(
                mission_base.MissionOperationState.FAULT,
                error_code=2,
            )
            return
        if not self.fc.state.is_fresh(0.5) or not self.navi.pose_is_fresh():
            raise RuntimeError(
                "Cannot perform safe return with stale flight state"
            )
        if self.fc.state.mode.value != self.fc.HOLD_POS_MODE:
            self.fc.set_flight_mode(self.fc.HOLD_POS_MODE)
            deadline = time.monotonic() + 1.5
            while time.monotonic() < deadline:
                if (
                    self.fc.state.is_fresh(0.5)
                    and self.fc.state.mode.value == self.fc.HOLD_POS_MODE
                ):
                    break
                self.stop_event.wait(0.05)
            else:
                raise RuntimeError(
                    "HOLD_POS was not confirmed for safe return"
                )

        self.navi.direct_set_waypoint(self.navi.current_point)
        self.navi.set_height(float(self.navi.current_height))
        self.navi.switch_pid("hover")
        self.navi.navigation_flag = True
        self.navi.keep_height_flag = True
        self.navi.set_height(CRUISE_HEIGHT)
        if not self.navi.wait_for_height(
            height_thres=descent_test.HEIGHT_TOLERANCE,
            timeout=descent_test.ASCENT_TIMEOUT_SECONDS,
        ):
            raise RuntimeError(
                "Failed to reach cruise height during safe return"
            )
        self._return_home_and_land()
        self.set_fleet_status(
            mission_base.MissionOperationState.FAULT,
            error_code=2,
        )

    def run(self) -> None:
        navi = self.navi
        navi.set_navigation_speed(mission_base.PURSUIT_SPEED)
        navi.set_vertical_speed(VERTICAL_SPEED)
        navi.start(mode="radar")
        logger.info("[MISSION2] Single-radar navigation started")

        descent_test.wait_for_radar_pose(navi, self.radar)
        navi.calibrate_basepoint()
        calibrated_at = time.monotonic()
        descent_test.wait_for_radar_pose(
            navi,
            self.radar,
            newer_than=calibrated_at,
        )
        logger.info(
            "[MISSION2] Radar basepoint calibrated: {}", navi.basepoint
        )

        self._start_vision_tracker()
        self.fc.set_indicator_led(255, 0, 0)
        prepare_command = self._wait_for_ground_command(
            mission1.CommandId.DRONE_PREPARE_MISSION
        )
        try:
            self.signals.send_initialization_success()
            self._ground_commands.complete(prepare_command)
        except Exception:
            self._ground_commands.fail(prepare_command, error_code=1)
            raise
        takeoff_command = self._wait_for_ground_command(
            mission1.CommandId.DRONE_START_MISSION
        )
        self.signals.send_takeoff_signal_received()
        if self.stop_event.is_set():
            self._ground_commands.fail(takeoff_command, error_code=1)
            return
        self.fc.set_indicator_led(0, 255, 0)
        self.signals.send_takeoff_started()

        try:
            navi.set_vertical_speed(mission_base.FAST_TAKEOFF_VERTICAL_SPEED)
            navi.fast_non_pointing_takeoff(
                target_height=CRUISE_HEIGHT,
            )
            navi.set_vertical_speed(VERTICAL_SPEED)
            self._ground_commands.complete(takeoff_command)
        except Exception:
            self._ground_commands.fail(takeoff_command, error_code=1)
            raise
        self.signals.send_takeoff_succeeded()
        self.fc.set_indicator_led(0, 0, 0)
        navi.set_yaw(0)
        if not navi.wait_for_yaw():
            raise RuntimeError("Yaw stabilization was not confirmed")

        self._clear_vision_samples()
        self._pursuit_speed_schedule = PursuitSpeedSchedule(
            initial_speed=PURSUIT_SPEED,
            approach_speed=PURSUIT_APPROACH_SPEED,
            after_slowdown_speed=PURSUIT_AFTER_SLOWDOWN_SPEED,
        )
        navi.set_navigation_speed(PURSUIT_SPEED)
        navi.switch_pid("navi")
        if not navi.navigation_follow_trajectory(
            self._pursuit_trajectory,
            wait=False,
            pos_thres=PURSUIT_POSITION_THRESHOLD,
        ):
            raise RuntimeError("Failed to start task 2 pursuit trajectory")
        self.signals.send_pursuit_started()
        logger.info(
            "[MISSION2] Pursuit trajectory started with {} points; "
            "speed {}cm/s, then {}cm/s toward {}, then {}cm/s",
            len(self._pursuit_trajectory),
            PURSUIT_SPEED,
            PURSUIT_APPROACH_SPEED,
            PURSUIT_SLOWDOWN_POINT,
            PURSUIT_AFTER_SLOWDOWN_SPEED,
        )

        try:
            self._wait_until_target_detected_on_trajectory()
            self.signals.send_escort_started()
            self._follow_descend_and_land_on_target()
        except (TargetNotFoundError, PreDescentTimeoutError) as exc:
            self._recover_home_after_expected_failure(str(exc))
            raise RuntimeError(
                "Task 2 ended after a safe failure return"
            ) from exc

        self._return_home_and_land()
        self.signals.send_mission_completed()
        logger.info("[MISSION2] Mission completed")


def main() -> None:
    fc = FC_Controller()
    radar = LD_Radar()
    stop_event = threading.Event()
    navi: Optional[Navigation] = None
    mission: Optional[Task2Mission] = None
    fleet_node = None

    try:
        fc.start_listen_serial(
            serial_dev=mission_base.FC_SERIAL_DEV,
            print_state=False,
        )
        if not fc.wait_for_connection(timeout_s=10):
            raise RuntimeError("Flight-controller connection timeout")
        if not fc.state.is_fresh(0.5):
            raise RuntimeError("Flight-controller telemetry is stale")
        if fc.state.unlock.value:
            raise RuntimeError(
                "Flight controller is already unlocked; mission will not take control"
            )
        logger.info(
            "[MISSION2] Flight controller connected through direct serial"
        )

        radar.debug = False
        radar.start()
        logger.info("[MISSION2] Single radar started")
        navi = descent_test.SingleRadarNavigation(
            fc=fc,
            radar=radar,
            stop_event=stop_event,
        )
        mission = Task2Mission(
            fc=fc,
            radar=radar,
            navi=navi,
            stop_event=stop_event,
        )
        fleet_node = mission_base.attach_air_fleet_node(
            fc,
            navi,
            stop_event,
            readonly=True,
            allow_start_mission=True,
            state_provider=mission_base.MissionFleetStateProvider(
                fc,
                navi,
                mission,
            ),
            trace_options=TraceSamplingOptions(
                enabled=True,
                sample_interval_s=mission_base.FLEET_TRACE_SAMPLE_INTERVAL_SECONDS,
                buffer_capacity=mission_base.FLEET_TRACE_BUFFER_CAPACITY,
                min_distance_cm=mission_base.FLEET_TRACE_MIN_DISTANCE_CM,
                stationary_keepalive_s=(
                    mission_base.FLEET_TRACE_STATIONARY_KEEPALIVE_SECONDS
                ),
            ),
        )
        mission.bind_ground_commands(fleet_node.command_queue)
        mission.run()
    except KeyboardInterrupt:
        logger.warning("[MISSION2] Interrupted by user")
    except Exception:
        if mission is not None:
            mission.set_fleet_status(
                mission_base.MissionOperationState.FAULT,
                error_code=1,
            )
        logger.exception("[MISSION2] Mission failed")
    finally:
        if mission is not None:
            try:
                mission.stop()
            except Exception:
                logger.exception("[MISSION2] Failed to stop mission")
            try:
                mission.write_visual_descent_log()
            except Exception:
                logger.exception("[MISSION2] Failed to write visual log")
        elif navi is not None:
            try:
                navi.stop()
            except Exception:
                logger.exception("[MISSION2] Failed to stop navigation")

        try:
            if fc.connected:
                fc.set_indicator_led(0, 0, 0)
        except Exception:
            logger.exception("[MISSION2] Failed to turn off indicator LED")

        try:
            if fc.connected and fc.state.unlock.value:
                descent_test.emergency_land(fc)
        except Exception:
            logger.exception("[MISSION2] Emergency landing request failed")

        try:
            if radar.running:
                radar.stop()
        except Exception:
            logger.exception("[MISSION2] Failed to stop radar")

        if fleet_node is not None:
            mission_base.drain_terminal_fleet_trace(fleet_node)
            fleet_node.close()
        try:
            fc.close()
        except Exception:
            logger.exception("[MISSION2] Failed to close flight controller")
        logger.info("[MISSION2] Mission finished")


if __name__ == "__main__":
    main()
