"""
静止目标视觉接地、不锁桨停留和再次起飞测试。

前半段复用 mission1_26_visual_descent_test.py 的单雷达起飞、入口点、
沿 +x 追及和目标发现流程。发现目标后：

1. 视觉修正下降到 20 cm；
2. 以 6 cm/s 继续视觉修正下降；
3. 用低高度、低垂直速度和高度稳定持续时间组合确认接地；
4. 清零控制但不锁桨，在目标平面停留 5 s；
5. 从目标平面再次起飞至 150 cm；
6. 返回起飞点并执行原有定点降落。

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


FINAL_APPROACH_HEIGHT = 20.0
FINAL_DESCENT_SPEED = 6.0
TOUCHDOWN_ALT_THRESHOLD = 12.0
TOUCHDOWN_VERTICAL_SPEED_THRESHOLD = 2.5
TOUCHDOWN_CONFIRM_SECONDS = 0.4
TOUCHDOWN_HEIGHT_RANGE = 1.5
FINAL_DESCENT_TIMEOUT_SECONDS = 8.0
UNLOCKED_DWELL_SECONDS = 5.0


class StaticTargetVisualLandingMission(
    descent_test.StaticTargetVisualDescentMission
):
    """把共用视觉下降控制用于静止目标的不锁桨接地和复飞。"""

    LOG_PREFIX = "mission1_26_visual_target_landing_"

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        # 此测试不控制数字输出；该字段仅用于共用 CSV 结构。
        self._digital_output_enabled = False

    def _perform_target_action(self) -> None:
        self.visual_descent.land_without_lock(
            base_velocity=(0.0, 0.0),
            approach_height=FINAL_APPROACH_HEIGHT,
            final_descent_speed=FINAL_DESCENT_SPEED,
            touchdown_alt_thres=TOUCHDOWN_ALT_THRESHOLD,
            touchdown_vertical_speed_thres=(
                TOUCHDOWN_VERTICAL_SPEED_THRESHOLD
            ),
            touchdown_confirm_time=TOUCHDOWN_CONFIRM_SECONDS,
            touchdown_height_range=TOUCHDOWN_HEIGHT_RANGE,
            final_descent_timeout=FINAL_DESCENT_TIMEOUT_SECONDS,
            dwell_seconds=UNLOCKED_DWELL_SECONDS,
        )

        target_point = self.navi.current_point.copy()
        self._stop_vision_tracker()
        logger.warning(
            "[TEST] Unlocked target dwell completed; take off again at {}",
            target_point,
        )
        self.navi.pointing_takeoff(
            target_point,
            target_height=mission1.CRUISE_HEIGHT,
            takeoff_alt_thres=descent_test.TAKEOFF_ALT_THRESHOLD,
        )
        self.navi.set_yaw(0)
        if not self.navi.wait_for_yaw():
            raise RuntimeError("Yaw stabilization was not confirmed after retakeoff")
        logger.info(
            "[TEST] Retakeoff reached {}cm cruise height",
            mission1.CRUISE_HEIGHT,
        )

    def _finish_at_takeoff_point(self) -> None:
        """返航后使用飞控一键降落，不调用 pointing_landing。"""
        self.navi.navigation_stop_here()
        self.navi.navigation_flag = False
        self.navi.keep_height_flag = False
        self.fc.set_flight_mode(self.fc.PROGRAM_MODE)
        time.sleep(0.1)
        self.fc.stablize()
        self.fc.land()
        if not self.fc.wait_for_lock(timeout_s=20):
            self.fc.land()
            raise RuntimeError(
                "One-key landing at takeoff point did not confirm motor lock"
            )
        logger.info("[TEST] One-key landing at takeoff point completed")


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
