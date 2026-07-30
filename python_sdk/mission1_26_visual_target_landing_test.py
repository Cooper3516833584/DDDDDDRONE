"""
静止目标视觉下降、一键降落锁桨和再次起飞测试。

前半段复用 mission1_26_visual_descent_test.py 的单雷达起飞、入口点、
沿 +x 追及和目标发现流程。发现目标后：

1. 视觉修正下降并稳定在目标平面上方 25 cm；
2. 停止视觉控制，调用飞控一键降落并确认锁桨；
3. 在目标平面锁桨停留 5 s；
4. 由定点起飞流程重新解锁并起飞至 150 cm；
5. 返回起飞点并执行定点降落。

该程序会执行真实飞行。运行前必须确认 server_ros.py 及其他 FC_Server
程序已关闭，并确保人员、桨叶和目标平台周围已经清空。
"""

import threading
import time
from typing import Optional

from loguru import logger

from FlightController import FC_Controller
from FlightController.Components import LD_Radar
from FlightController.Solutions.Navigation import Navigation
import mission1_26 as mission1
import mission1_26_visual_descent_test as descent_test


TARGET_LANDING_HEIGHT = 25.0
TARGET_LANDING_TIMEOUT_SECONDS = 15.0
TARGET_LANDING_LOCK_TIMEOUT_SECONDS = 20.0
LOCKED_DWELL_SECONDS = 5.0


class StaticTargetVisualLandingMission(
    descent_test.StaticTargetVisualDescentMission
):
    """在静止目标上方完成视觉下降、一键降落锁桨和复飞。"""

    LOG_PREFIX = "mission1_26_visual_target_landing_"

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        # 此测试不控制数字输出；该字段仅用于共用 CSV 结构。
        self._digital_output_enabled = False

    def _perform_target_action(self) -> None:
        self.visual_descent.descend_to_height(
            target_height=TARGET_LANDING_HEIGHT,
            hover_seconds=0.0,
            base_velocity=(0.0, 0.0),
            height_tolerance=descent_test.HEIGHT_TOLERANCE,
            height_confirm_time=descent_test.HEIGHT_CONFIRM_SECONDS,
            timeout=TARGET_LANDING_TIMEOUT_SECONDS,
        )

        target_point = self.navi.current_point.copy()
        self._stop_vision_tracker()
        self.navi.navigation_stop_here()
        self.navi.navigation_flag = False
        self.navi.keep_height_flag = False
        self.fc.set_flight_mode(self.fc.PROGRAM_MODE)
        time.sleep(0.1)
        self.fc.stablize()
        self.fc.land()
        if not self.fc.wait_for_lock(
            timeout_s=TARGET_LANDING_LOCK_TIMEOUT_SECONDS
        ):
            self.fc.land()
            raise RuntimeError(
                "One-key landing on target did not confirm motor lock"
            )
        if not self.fc.state.is_fresh(0.5):
            raise RuntimeError(
                "Flight-controller telemetry became stale after target landing"
            )
        logger.info("[TEST] One-key landing on target confirmed motor lock")

        logger.warning(
            "[TEST] Locked on target; take off again in {:.1f}s at {}",
            LOCKED_DWELL_SECONDS,
            target_point,
        )
        dwell_deadline = time.monotonic() + LOCKED_DWELL_SECONDS
        while time.monotonic() < dwell_deadline:
            if self.stop_event.is_set():
                raise RuntimeError("External stop requested during locked dwell")
            if not self.fc.state.is_fresh(0.5):
                raise RuntimeError("Flight-controller telemetry became stale")
            if self.fc.state.unlock.value:
                raise RuntimeError(
                    "Aircraft unexpectedly unlocked during target dwell"
                )
            self.stop_event.wait(0.1)

        # pointing_takeoff() starts with PROGRAM mode and fc.unlock(); the
        # confirmed locked state above makes this a real re-arm before takeoff.
        self.navi.pointing_takeoff(
            target_point,
            target_height=mission1.CRUISE_HEIGHT,
        )
        self.navi.set_yaw(0)
        if not self.navi.wait_for_yaw():
            raise RuntimeError("Yaw stabilization was not confirmed after retakeoff")
        logger.info(
            "[TEST] Retakeoff reached {}cm cruise height",
            mission1.CRUISE_HEIGHT,
        )

    def _finish_at_takeoff_point(self) -> None:
        """返航后沿用原有定点降落。"""
        if not self.navi.pointing_landing(
            mission1.TAKEOFF_POINT,
            height_timeout=descent_test.LANDING_HEIGHT_TIMEOUT_SECONDS,
        ):
            raise RuntimeError(
                "Failed to land at takeoff point"
            )
        logger.info("[TEST] Pointing landing at takeoff point completed")


def main() -> None:
    fc = FC_Controller()
    radar = LD_Radar()
    stop_event = threading.Event()
    navi: Optional[Navigation] = None
    mission: Optional[StaticTargetVisualLandingMission] = None

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

        descent_test.wait_for_terminal_start_command(
            digital_output_enabled=False
        )

        radar.debug = False
        radar.start()
        logger.info("[TEST] Single radar started")

        navi = descent_test.SingleRadarNavigation(
            fc=fc,
            radar=radar,
            stop_event=stop_event,
        )
        mission = StaticTargetVisualLandingMission(
            fc=fc,
            radar=radar,
            navi=navi,
            stop_event=stop_event,
        )
        mission.run()
    except KeyboardInterrupt:
        logger.warning("[TEST] Interrupted by user")
    except Exception:
        logger.exception("[TEST] Static-target visual landing test failed")
    finally:
        if mission is not None:
            try:
                mission.stop()
            except Exception:
                logger.exception("[TEST] Failed to stop mission")
            try:
                mission.write_visual_descent_log()
            except Exception:
                logger.exception("[TEST] Failed to write visual-landing log")
        elif navi is not None:
            try:
                navi.stop()
            except Exception:
                logger.exception("[TEST] Failed to stop navigation")

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
        logger.info("[TEST] Static-target visual landing test finished")


if __name__ == "__main__":
    main()
