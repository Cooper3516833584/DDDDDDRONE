"""
移动目标视觉伴飞与同步下降测试。

流程复用静止目标测试的单雷达定位、视觉线程、沿 +x 搜索、返航和
定点降落框架。发现沿 +x 直线运动的目标后，连续有效伴飞 10 秒，
在同一视觉速度接管内从 150 cm 下降到 40 cm，并继续伴飞 2 秒。

本文件会连接真实飞控、雷达和相机并执行飞行。运行前必须确认
server_ros.py 及其他 FC_Server 已关闭、现场和投放区域安全。
通信尚未接入；除起飞信号使用终端输入 ``s`` 外，其余信号仅记录日志。
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


DESCENT_TARGET_HEIGHT = 40.0
STABILIZE_SECONDS = 10.0
STABILIZE_TIMEOUT_SECONDS = 20.0
LOW_HOVER_SECONDS = 2.0
DESCENT_TIMEOUT_SECONDS = 15.0
INITIAL_TARGET_VELOCITY = (3.6, 0.0)


class MissionSignalPlaceholders:
    """任务通信占位；发送函数只写日志，不进行网络或串口通信。"""

    @staticmethod
    def _send(name: str, purpose: str) -> None:
        logger.info("[SIGNAL-PLACEHOLDER] send {}: {}", name, purpose)

    def send_initialization_success(self) -> None:
        self._send("initialization_success", "aircraft is ready for takeoff")

    @staticmethod
    def wait_for_takeoff_signal() -> None:
        logger.info(
            "[SIGNAL-PLACEHOLDER] wait takeoff_signal: terminal input is active"
        )
        descent_test.wait_for_terminal_start_command()

    def send_takeoff_started(self) -> None:
        self._send("takeoff_started", "takeoff stage started")

    def send_takeoff_succeeded(self) -> None:
        self._send(
            "takeoff_succeeded",
            "cruise height reached; moving target may start",
        )

    def send_escort_started(self) -> None:
        self._send("escort_started", "visual escort started")

    def send_drop_started(self) -> None:
        self._send("drop_started", "synchronized descent started")

    def send_drop_completed(self) -> None:
        self._send("drop_completed", "digital output 0 was disabled")

    def send_return_started(self) -> None:
        self._send("return_started", "return-to-home stage started")

    def send_landing_started(self) -> None:
        self._send("landing_started", "pointing landing started")

    def send_mission_completed(self) -> None:
        self._send("mission_completed", "landing and motor lock confirmed")


class MovingTargetVisualDescentMission(
    descent_test.StaticTargetVisualDescentMission
):
    """移动目标伴飞、同步下降、返航和定点降落测试任务。"""

    LOG_PREFIX = "mission1_26_moving_target_descent_"

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.signals = MissionSignalPlaceholders()
        self._digital_output_enabled = False
        self.moving_target_descent = MovingTargetDescentController(
            fc=self.fc,
            navi=self.navi,
            stop_event=self.stop_event,
            latest_vision_sample=self._latest_vision_sample,
            raise_if_vision_failed=self._raise_if_vision_failed,
            record_callback=self._record_moving_descent,
        )

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
        self.fc.set_digital_output(0, True)
        self._digital_output_enabled = True
        logger.warning(
            "[TEST] Digital output 0 enabled; confirm payload and drop-area safety"
        )
        self.signals.send_initialization_success()

        self.fc.set_indicator_led(255, 0, 0)
        self.signals.wait_for_takeoff_signal()
        if self.stop_event.is_set():
            return
        self.fc.set_indicator_led(0, 255, 0)
        self.signals.send_takeoff_started()

        navi.pointing_takeoff(
            mission1.TAKEOFF_POINT,
            target_height=mission1.CRUISE_HEIGHT,
        )
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
        mission.run()
    except KeyboardInterrupt:
        logger.warning("[TEST] Interrupted by user")
    except Exception:
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

        try:
            fc.close()
        except Exception:
            logger.exception("[TEST] Failed to close flight controller")
        logger.info("[TEST] Moving-target visual descent test finished")


if __name__ == "__main__":
    main()
