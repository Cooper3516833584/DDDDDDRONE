"""
移动目标视觉伴飞与同步下降测试。

流程复用静止目标测试的单雷达定位、视觉线程、沿 +x 搜索、返航和
定点降落框架。发现沿 +x 直线运动的目标后，连续有效伴飞 10 秒，
在同一视觉速度接管内从 150 cm 下降到 40 cm，并继续伴飞 2 秒。

本文件会连接真实飞控、雷达和相机并执行飞行。运行前必须确认
server_ros.py 及其他 FC_Server 已关闭、现场和投放区域安全。
地面站通过 FleetBus 依次发送准备和起飞命令；准备命令开启电磁铁，
起飞命令仅在地面站完成三端联调时序后放行定点起飞。
"""

import csv
import math
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np
from loguru import logger

from FlightController import FC_Controller
from FlightController.Components import LD_Radar
from FlightController.Solutions.Navigation import Navigation
import mission1_26 as mission1
import mission1_26_visual_descent_test as descent_test
from moving_target_descent import MovingTargetDescentController
from fleet_bus.models import CommandId


DESCENT_TARGET_HEIGHT = 40.0
STABILIZE_SECONDS = 10.0
STABILIZE_TIMEOUT_SECONDS = 20.0
LOW_HOVER_SECONDS = 2.0
DESCENT_TIMEOUT_SECONDS = 15.0
INITIAL_TARGET_VELOCITY = (3.6, 0.0)


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
    """移动目标伴飞、同步下降、返航和定点降落测试任务。"""

    LOG_PREFIX = "mission1_26_moving_target_descent_"

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.signals = MissionGroundStationSignals(self)
        self._ground_commands = None
        self._digital_output_enabled = False
        self.moving_target_descent = MovingTargetDescentController(
            fc=self.fc,
            navi=self.navi,
            stop_event=self.stop_event,
            latest_vision_sample=self._latest_vision_sample,
            raise_if_vision_failed=self._raise_if_vision_failed,
            record_callback=self._record_moving_descent,
        )

    def bind_ground_commands(self, command_queue) -> None:
        self._ground_commands = command_queue

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
            logger.warning("[TEST] No moving-target descent records to write")
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
                "[TEST] Moving-target log discarded {} oldest records",
                self._visual_descent_records_dropped,
            )
        logger.info("[TEST] Moving-target descent log written to {}", log_path)
        return log_path

    def _disable_output_and_report(self) -> None:
        self.fc.set_digital_output(0, False)
        self._digital_output_enabled = False
        logger.info(
            "[TEST] Digital output 0 disabled at {}cm",
            DESCENT_TARGET_HEIGHT,
        )
        self.signals.send_drop_completed()

    def _perform_target_action(self) -> None:
        final_velocity = self.moving_target_descent.follow_and_descend(
            target_height=DESCENT_TARGET_HEIGHT,
            stabilize_seconds=STABILIZE_SECONDS,
            stabilize_timeout=STABILIZE_TIMEOUT_SECONDS,
            hover_seconds=LOW_HOVER_SECONDS,
            initial_target_velocity=INITIAL_TARGET_VELOCITY,
            height_tolerance=descent_test.HEIGHT_TOLERANCE,
            height_confirm_time=descent_test.HEIGHT_CONFIRM_SECONDS,
            descent_timeout=DESCENT_TIMEOUT_SECONDS,
            on_descent_start=self.signals.send_drop_started,
            on_height_reached=self._disable_output_and_report,
        )
        logger.info(
            "[TEST] Moving-target descent finished; estimated target "
            "velocity=({:.2f}, {:.2f})cm/s",
            final_velocity[0],
            final_velocity[1],
        )

        self.signals.send_return_started()
        self._stop_vision_tracker()
        self.navi.set_height(float(mission1.CRUISE_HEIGHT))
        self.navi.keep_height_flag = True
        if not self.navi.wait_for_height(
            height_thres=descent_test.HEIGHT_TOLERANCE,
            timeout=descent_test.ASCENT_TIMEOUT_SECONDS,
        ):
            raise RuntimeError("Failed to return to cruise height")
        logger.info(
            "[TEST] Returned to {}cm cruise height",
            mission1.CRUISE_HEIGHT,
        )

    def run(self) -> None:
        navi = self.navi
        navi.set_navigation_speed(mission1.PURSUIT_SPEED)
        navi.set_vertical_speed(mission1.VERTICAL_SPEED)
        navi.start(mode="radar")
        logger.info("[TEST] Single-radar navigation started")

        descent_test.wait_for_radar_pose(navi, self.radar)
        navi.calibrate_basepoint()
        calibrated_at = time.monotonic()
        descent_test.wait_for_radar_pose(
            navi,
            self.radar,
            newer_than=calibrated_at,
        )
        logger.info("[TEST] Radar basepoint calibrated: {}", navi.basepoint)

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
            "[TEST] Digital output 0 enabled; confirm payload and drop-area safety"
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
            navi.pointing_takeoff(
                mission1.TAKEOFF_POINT,
                target_height=mission1.CRUISE_HEIGHT,
            )
            self._ground_commands.complete(takeoff_command)
        except Exception:
            self._ground_commands.fail(takeoff_command, error_code=1)
            raise
        self.signals.send_takeoff_succeeded()
        self.fc.set_indicator_led(0, 0, 0)

        navi.set_yaw(0)
        if not navi.wait_for_yaw():
            raise RuntimeError("Yaw stabilization was not confirmed")

        logger.info("[TEST] Navigate to entry point {}", mission1.ENTRY_POINT)
        if not navi.navigation_to_waypoint(mission1.ENTRY_POINT, wait=True):
            raise RuntimeError("Failed to reach entry point")

        forward_target = np.array(
            [
                mission1.ENTRY_POINT[0]
                + mission1.FORWARD_GUIDANCE_DISTANCE,
                mission1.ENTRY_POINT[1],
            ]
        )
        self._clear_vision_samples()
        navi.set_navigation_speed(mission1.PURSUIT_SPEED)
        navi.switch_pid("navi")
        navi.direct_set_waypoint(forward_target)
        logger.info("[TEST] Pursuing moving target along +x")
        self._wait_until_target_detected(forward_target[0])

        self.signals.send_escort_started()
        self._perform_target_action()

        navi.set_navigation_speed(mission1.PURSUIT_SPEED)
        if not navi.navigation_to_waypoint(
            mission1.TAKEOFF_POINT,
            wait=True,
        ):
            raise RuntimeError("Failed to return to takeoff point")

        self.signals.send_landing_started()
        self._finish_at_takeoff_point()
        self.signals.send_mission_completed()
        logger.info("[TEST] Moving-target visual descent flight completed")


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
                "Flight controller is already unlocked; test will not take control"
            )
        logger.info("[TEST] Flight controller connected through direct serial")

        radar.debug = False
        radar.start()
        logger.info("[TEST] Single radar started")

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
        )
        mission.bind_ground_commands(fleet_node.command_queue)
        mission.run()
    except KeyboardInterrupt:
        logger.warning("[TEST] Interrupted by user")
    except Exception:
        if mission is not None:
            mission.set_fleet_status(
                mission1.MissionOperationState.FAULT,
                error_code=1,
            )
        logger.exception("[TEST] Moving-target visual descent test failed")
    finally:
        if mission is not None:
            try:
                mission.stop()
            except Exception:
                logger.exception("[TEST] Failed to stop mission")
            try:
                mission.write_visual_descent_log()
            except Exception:
                logger.exception("[TEST] Failed to write moving-target log")
        elif navi is not None:
            try:
                navi.stop()
            except Exception:
                logger.exception("[TEST] Failed to stop navigation")

        try:
            if fc.connected:
                fc.set_indicator_led(0, 0, 0)
        except Exception:
            logger.exception("[TEST] Failed to turn off indicator LED")

        try:
            if fc.connected:
                fc.set_digital_output(0, False)
        except Exception:
            logger.exception("[TEST] Failed to disable digital output 0")

        try:
            if fc.connected and fc.state.unlock.value:
                descent_test.emergency_land(fc)
        except Exception:
            logger.exception("[TEST] Emergency landing request failed")

        try:
            if radar.running:
                radar.stop()
        except Exception:
            logger.exception("[TEST] Failed to stop radar")

        if fleet_node is not None:
            fleet_node.close()
        try:
            fc.close()
        except Exception:
            logger.exception("[TEST] Failed to close flight controller")
        logger.info("[TEST] Moving-target visual descent test finished")


if __name__ == "__main__":
    main()
