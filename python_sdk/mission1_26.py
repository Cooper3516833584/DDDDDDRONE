"""
任务一：移动目标视觉伴飞、同步下降、抛投、返航与定点降落。

追及路径与任务二保持一致：直线 + 右侧顺时针半圆弧 + 末段直线的
曲线追及轨迹，非阻塞调用，速度按 初始->接近->减速后 三段调度。
发现移动目标后，连续有效伴飞 10 秒，
在同一视觉速度接管内从 150 cm 下降到 40 cm，并继续伴飞 2 秒。

起飞采用非定点垂直起飞（90 cm 一键离地后垂直爬升至 150 cm），
该阶段垂直速度设为 30 cm/s；起飞完成后先稳定偏航，再悬停 2.5s，
随后关闭指示灯继续追及。返航开始时切换到 H 降落点检测，下降至
60 cm 后以 30 像素阈值完成视觉校准，再在该点定点降落，降落阶段
垂直速度设为 15 cm/s。相机全程保持开启，不重复开关。

本文件会连接真实飞控、雷达和相机并执行飞行。运行前必须确认
server_ros.py 及其他 FC_Server 已关闭、现场和投放区域安全。
地面站通过 FleetBus 依次发送准备和起飞命令；准备命令开启电磁铁，
起飞命令仅在地面站完成三端联调时序后放行非定点起飞。
"""

import csv
import math
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple

from loguru import logger

from FlightController import FC_Controller
from FlightController.Components import LD_Radar
from FlightController.Solutions.Navigation import Navigation
from fleet_bus.models import CommandId
from fleet_bus.trace_buffer import TraceSamplingOptions
import mission1_26_base as mission1
import mission1_26_visual_descent_test as descent_test
from mission2_26_logic import (
    ARC_END,
    PURSUIT_SLOWDOWN_POINT,
    PursuitSpeedSchedule,
    ROUTE_GATE_RADIUS,
    RoutePassGate,
    build_pursuit_trajectory,
)
from moving_target_descent import MovingTargetDescentController


DESCENT_TARGET_HEIGHT = 40.0
STABILIZE_SECONDS = 10.0
STABILIZE_TIMEOUT_SECONDS = 20.0
LOW_HOVER_SECONDS = 2.0
DESCENT_TIMEOUT_SECONDS = 15.0
INITIAL_TARGET_VELOCITY = (3.6, 0.0)

# 起飞后、开始追及前的悬停时间（秒）。
HOVER_BEFORE_PURSUIT_SECONDS = 2.5

# 追及轨迹与速度规划，与任务二保持一致：直线 + 右侧顺时针半圆弧 +
# 末段直线，非阻塞调用，速度按 初始->接近->减速后 三段调度。
PURSUIT_SPEED = 20.0
PURSUIT_APPROACH_SPEED = 25.0
PURSUIT_AFTER_SLOWDOWN_SPEED = 15.0
PURSUIT_POSITION_THRESHOLD = 7.5

# 完成抛投并恢复巡航高度后的水平返航速度（cm/s）。
RETURN_SPEED = 40.0


class MissionGroundStationSignals:
    """Publish moving-target mission phases through the existing FleetBus link."""

    TAKEOFF_SIGNAL_RECEIVED = 2
    DROP_STARTED = 6
    DROP_COMPLETED = 7

    def __init__(self, mission: "MovingTargetVisualDescentMission") -> None:
        self._mission = mission

    def _send(self, name: str, operation_state: int) -> None:
        self._mission.set_fleet_status(operation_state)
        logger.info("[GROUND] Mission signal sent: {}", name)

    def send_initialization_success(self) -> None:
        self._send("initialization_success", mission1.MissionOperationState.READY)

    def send_takeoff_signal_received(self) -> None:
        self._send("takeoff_signal_received", self.TAKEOFF_SIGNAL_RECEIVED)

    def send_takeoff_started(self) -> None:
        self._send("takeoff_started", mission1.MissionOperationState.TAKEOFF)

    def send_takeoff_succeeded(self) -> None:
        self._send(
            "takeoff_succeeded", mission1.MissionOperationState.HOVERING
        )

    def send_escort_started(self) -> None:
        self._send("escort_started", mission1.MissionOperationState.ESCORTING)

    def send_drop_started(self) -> None:
        self._send("drop_started", self.DROP_STARTED)

    def send_drop_completed(self) -> None:
        self._send("drop_completed", self.DROP_COMPLETED)

    def send_return_started(self) -> None:
        self._send(
            "return_started", mission1.MissionOperationState.RETURNING_HOME
        )

    def send_landing_started(self) -> None:
        self._send(
            "landing_started", mission1.MissionOperationState.LANDING_HOME
        )

    def send_mission_completed(self) -> None:
        self._send(
            "mission_completed", mission1.MissionOperationState.COMPLETED
        )


class MovingTargetVisualDescentMission(
    descent_test.StaticTargetVisualDescentMission
):
    """移动目标伴飞、同步下降、抛投、返航和定点降落任务。"""

    LOG_PREFIX = "mission1_26_moving_target_descent_"

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.signals = MissionGroundStationSignals(self)
        self._ground_commands = None
        self._digital_output_enabled = False
        self._drop_indicator_enabled = False
        self.moving_target_descent = MovingTargetDescentController(
            fc=self.fc,
            navi=self.navi,
            stop_event=self.stop_event,
            latest_vision_sample=self._latest_vision_sample,
            raise_if_vision_failed=self._raise_if_vision_failed,
            record_callback=self._record_moving_descent,
        )
        # 追及轨迹与速度规划与任务二保持一致。
        self._route_gate = RoutePassGate(radius=ROUTE_GATE_RADIUS)
        self._pursuit_speed_schedule = PursuitSpeedSchedule(
            initial_speed=PURSUIT_SPEED,
            approach_speed=PURSUIT_APPROACH_SPEED,
            after_slowdown_speed=PURSUIT_AFTER_SLOWDOWN_SPEED,
        )
        self._pursuit_trajectory = build_pursuit_trajectory(
            altitude=float(mission1.CRUISE_HEIGHT),
            arc_step_degrees=10,
        )

    def bind_ground_commands(self, command_queue) -> None:
        self._ground_commands = command_queue

    def _route_gate_is_open(self) -> bool:
        was_open = self._route_gate.passed
        is_open = self._route_gate.update(
            self.navi.current_x,
            self.navi.current_y,
        )
        if is_open and not was_open:
            logger.info(
                "[MISSION1] Route gate passed near {} at "
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
            "[MISSION1] Pursuit speed changed to {:.1f}cm/s at "
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
                    <= mission1.VISION_SAMPLE_STALE_SECONDS
                    and x_px is not None
                    and y_px is not None
                    and math.isfinite(float(x_px))
                    and math.isfinite(float(y_px))
                    and math.hypot(float(x_px), float(y_px))
                    < mission1.TARGET_DETECTION_PIXEL_THRESHOLD
                ):
                    self._stop_pursuit_trajectory()
                    logger.info(
                        "[MISSION1] Target detected during pursuit: "
                        "x_px={:.2f}, y_px={:.2f}",
                        x_px,
                        y_px,
                    )
                    return float(x_px), float(y_px)

            if not self.navi.traj_running_event.is_set():
                raise RuntimeError(
                    "Pursuit trajectory finished without target detection"
                )
            self.stop_event.wait(mission1.ESCORT_CONTROL_PERIOD)
        raise RuntimeError("Task 1 stopped during target pursuit")

    def _wait_for_ground_command(self, expected: CommandId):
        if self._ground_commands is None:
            raise RuntimeError("FleetBus command queue is not attached")
        logger.info("[GROUND] Waiting for {} command", expected.name)
        while not self.stop_event.is_set():
            command = self._ground_commands.receive(timeout=0.2)
            if command is None:
                continue
            if command.command_id == int(CommandId.TARGETED_STOP):
                self._ground_commands.complete(command)
                raise RuntimeError("Mission stopped by ground station")
            if command.command_id != int(expected):
                self._ground_commands.fail(command, error_code=1)
                logger.warning(
                    "[GROUND] Rejected command {} while waiting for {}",
                    command.command_id,
                    expected.name,
                )
                continue
            return command
        raise RuntimeError("Mission stopped while waiting for ground command")

    def _record_moving_descent(
        self,
        started_at: float,
        phase: str,
        x_px: Optional[float],
        y_px: Optional[float],
        estimated_target_vx: float,
        estimated_target_vy: float,
        command_vx: int,
        command_vy: int,
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
                "estimated_target_vx_cm_s": estimated_target_vx,
                "estimated_target_vy_cm_s": estimated_target_vy,
                "estimated_target_speed_cm_s": math.hypot(
                    estimated_target_vx,
                    estimated_target_vy,
                ),
                "command_vx_cm_s": command_vx,
                "command_vy_cm_s": command_vy,
                "command_speed_cm_s": math.hypot(command_vx, command_vy),
                "digital_output_0_enabled": self._digital_output_enabled,
            }
        )

    def write_visual_descent_log(self) -> Optional[Path]:
        records: List[Dict[str, object]] = list(
            self._visual_descent_records
        )
        if not records:
            logger.warning("[MISSION1] No moving-target descent records to write")
            return None

        log_dir = Path(__file__).resolve().parent / "fc_log"
        log_dir.mkdir(parents=True, exist_ok=True)
        log_path = log_dir / (
            self.LOG_PREFIX
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
            "estimated_target_vx_cm_s",
            "estimated_target_vy_cm_s",
            "estimated_target_speed_cm_s",
            "command_vx_cm_s",
            "command_vy_cm_s",
            "command_speed_cm_s",
            "digital_output_0_enabled",
        ]
        with log_path.open("w", newline="", encoding="utf-8") as file:
            writer = csv.DictWriter(file, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(records)
        if self._visual_descent_records_dropped:
            logger.warning(
                "[MISSION1] Moving-target log discarded {} oldest records",
                self._visual_descent_records_dropped,
            )
        logger.info(
            "[MISSION1] Moving-target descent log written to {}", log_path
        )
        return log_path

    def _disable_output_and_report(self) -> None:
        self.fc.set_digital_output(0, False)
        self._digital_output_enabled = False
        logger.info(
            "[MISSION1] Digital output 0 disabled at {}cm",
            DESCENT_TARGET_HEIGHT,
        )
        self.signals.send_drop_completed()

    def _start_drop_and_indicator(self) -> None:
        self.fc.set_indicator_led(255, 255, 0)
        self._drop_indicator_enabled = True
        logger.info("[MISSION1] Drop indicator LED set to yellow")
        self.signals.send_drop_started()

    def _stop_drop_indicator(self) -> None:
        if not self._drop_indicator_enabled:
            return
        self.fc.set_indicator_led(0, 0, 0)
        self._drop_indicator_enabled = False
        logger.info("[MISSION1] Drop indicator LED turned off")

    def _perform_target_action(self) -> None:
        try:
            final_velocity = self.moving_target_descent.follow_and_descend(
                target_height=DESCENT_TARGET_HEIGHT,
                stabilize_seconds=STABILIZE_SECONDS,
                stabilize_timeout=STABILIZE_TIMEOUT_SECONDS,
                hover_seconds=LOW_HOVER_SECONDS,
                initial_target_velocity=INITIAL_TARGET_VELOCITY,
                height_tolerance=descent_test.HEIGHT_TOLERANCE,
                height_confirm_time=descent_test.HEIGHT_CONFIRM_SECONDS,
                descent_timeout=DESCENT_TIMEOUT_SECONDS,
                on_descent_start=self._start_drop_and_indicator,
                on_height_reached=self._disable_output_and_report,
            )
        finally:
            self._stop_drop_indicator()
        logger.info(
            "[MISSION1] Moving-target descent finished; estimated target "
            "velocity=({:.2f}, {:.2f})cm/s",
            final_velocity[0],
            final_velocity[1],
        )

        self.signals.send_return_started()
        self.navi.set_height(float(mission1.CRUISE_HEIGHT))
        self.navi.keep_height_flag = True
        if not self.navi.wait_for_height(
            height_thres=descent_test.HEIGHT_TOLERANCE,
            timeout=descent_test.ASCENT_TIMEOUT_SECONDS,
        ):
            raise RuntimeError("Failed to return to cruise height")
        logger.info(
            "[MISSION1] Returned to {}cm cruise height",
            mission1.CRUISE_HEIGHT,
        )

    def run(self) -> None:
        navi = self.navi
        navi.set_navigation_speed(mission1.PURSUIT_SPEED)
        navi.set_vertical_speed(mission1.VERTICAL_SPEED)
        navi.start(mode="radar")
        logger.info("[MISSION1] Single-radar navigation started")

        descent_test.wait_for_radar_pose(navi, self.radar)
        navi.calibrate_basepoint()
        calibrated_at = time.monotonic()
        descent_test.wait_for_radar_pose(
            navi,
            self.radar,
            newer_than=calibrated_at,
        )
        logger.info(
            "[MISSION1] Radar basepoint calibrated: {}", navi.basepoint
        )

        self._start_vision_tracker()
        self.fc.set_indicator_led(255, 0, 0)

        prepare_command = self._wait_for_ground_command(
            CommandId.DRONE_PREPARE_MISSION
        )
        try:
            self.fc.set_digital_output(0, True)
            self._digital_output_enabled = True
            self.signals.send_initialization_success()
            self._ground_commands.complete(prepare_command)
        except Exception:
            self._ground_commands.fail(prepare_command, error_code=1)
            raise
        logger.warning(
            "[MISSION1] Digital output 0 enabled; "
            "confirm payload and drop-area safety"
        )

        takeoff_command = self._wait_for_ground_command(
            CommandId.DRONE_START_MISSION
        )
        self.signals.send_takeoff_signal_received()
        if self.stop_event.is_set():
            self._ground_commands.fail(takeoff_command, error_code=1)
            return
        self.fc.set_indicator_led(0, 255, 0)
        self.signals.send_takeoff_started()

        try:
            navi.set_vertical_speed(mission1.FAST_TAKEOFF_VERTICAL_SPEED)
            navi.fast_non_pointing_takeoff(
                target_height=mission1.CRUISE_HEIGHT,
            )
            navi.set_vertical_speed(mission1.VERTICAL_SPEED)
            self._ground_commands.complete(takeoff_command)
        except Exception:
            self._ground_commands.fail(takeoff_command, error_code=1)
            raise
        self.signals.send_takeoff_succeeded()

        # 起飞完成后先稳定偏航，再悬停 2.5s，随后关闭指示灯继续追及。
        navi.set_yaw(0)
        if not navi.wait_for_yaw():
            raise RuntimeError("Yaw stabilization was not confirmed")
        logger.info(
            "[MISSION1] Hovering {:.1f}s after takeoff before pursuit",
            HOVER_BEFORE_PURSUIT_SECONDS,
        )
        time.sleep(HOVER_BEFORE_PURSUIT_SECONDS)
        self.fc.set_indicator_led(0, 0, 0)

        logger.info(
            "[MISSION1] Pursuit trajectory started with {} points; "
            "speed {}cm/s, then {}cm/s toward {}, then {}cm/s",
            len(self._pursuit_trajectory),
            PURSUIT_SPEED,
            PURSUIT_APPROACH_SPEED,
            PURSUIT_SLOWDOWN_POINT,
            PURSUIT_AFTER_SLOWDOWN_SPEED,
        )
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
            raise RuntimeError("Failed to start task 1 pursuit trajectory")

        self._wait_until_target_detected_on_trajectory()
        self.signals.send_escort_started()
        self._perform_target_action()

        # 返航开始时切换到 H 降落点检测；相机保持全程开启。
        self.enable_h_landing_vision()
        navi.set_navigation_speed(RETURN_SPEED)
        if not navi.navigation_to_waypoint(
            mission1.TAKEOFF_POINT,
            wait=True,
        ):
            raise RuntimeError("Failed to return to takeoff point")

        self.signals.send_landing_started()
        self._visual_h_landing_at_takeoff()
        self.signals.send_mission_completed()
        logger.info(
            "[MISSION1] Moving-target visual descent flight completed"
        )


def main() -> None:
    fc = FC_Controller()
    radar = LD_Radar()
    stop_event = threading.Event()
    navi: Optional[Navigation] = None
    mission: Optional[MovingTargetVisualDescentMission] = None
    fleet_node = None

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
                "Flight controller is already unlocked; mission will not take control"
            )
        logger.info(
            "[MISSION1] Flight controller connected through direct serial"
        )

        radar.debug = False
        radar.start()
        logger.info("[MISSION1] Single radar started")

        navi = descent_test.SingleRadarNavigation(
            fc=fc,
            radar=radar,
            stop_event=stop_event,
        )
        mission = MovingTargetVisualDescentMission(
            fc=fc,
            radar=radar,
            navi=navi,
            stop_event=stop_event,
        )
        fleet_node = mission1.attach_air_fleet_node(
            fc,
            navi,
            stop_event,
            readonly=True,
            allow_start_mission=True,
            state_provider=mission1.MissionFleetStateProvider(fc, navi, mission),
            trace_options=TraceSamplingOptions(
                enabled=True,
                sample_interval_s=0.50,
                buffer_capacity=600,
                min_distance_cm=5.0,
                stationary_keepalive_s=2.0,
            ),
        )
        mission.bind_ground_commands(fleet_node.command_queue)
        mission.run()
    except KeyboardInterrupt:
        logger.warning("[MISSION1] Interrupted by user")
    except Exception:
        if mission is not None:
            mission.set_fleet_status(
                mission1.MissionOperationState.FAULT,
                error_code=1,
            )
        logger.exception("[MISSION1] Moving-target visual descent mission failed")
    finally:
        if mission is not None:
            try:
                mission.stop()
            except Exception:
                logger.exception("[MISSION1] Failed to stop mission")
            try:
                mission.write_visual_descent_log()
            except Exception:
                logger.exception(
                    "[MISSION1] Failed to write moving-target log"
                )
        elif navi is not None:
            try:
                navi.stop()
            except Exception:
                logger.exception("[MISSION1] Failed to stop navigation")

        try:
            if fc.connected:
                fc.set_indicator_led(0, 0, 0)
        except Exception:
            logger.exception("[MISSION1] Failed to turn off indicator LED")

        try:
            if fc.connected:
                fc.set_digital_output(0, False)
        except Exception:
            logger.exception(
                "[MISSION1] Failed to disable digital output 0"
            )

        try:
            if fc.connected and fc.state.unlock.value:
                descent_test.emergency_land(fc)
        except Exception:
            logger.exception("[MISSION1] Emergency landing request failed")

        try:
            if radar.running:
                radar.stop()
        except Exception:
            logger.exception("[MISSION1] Failed to stop radar")

        if fleet_node is not None:
            fleet_node.close()
        try:
            fc.close()
        except Exception:
            logger.exception("[MISSION1] Failed to close flight controller")
        logger.info("[MISSION1] Moving-target visual descent mission finished")


if __name__ == "__main__":
    main()
