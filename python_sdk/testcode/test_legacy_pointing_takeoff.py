"""
使用旧版 ``Navigation.pointing_takeoff`` 时序执行单雷达定点起飞测试。

飞行流程：

1. 通过 ``FC_Controller`` 直连飞控串口；
2. 启动单雷达定位，并以当前位置校准任务原点；
3. 按旧版时序解锁，固定预热 2 秒；
4. 发送一次 60 cm 一键起飞，固定等待 8 秒；
5. 切换到定点模式，通过高度 PID 上升到 140 cm；
6. 在起飞点悬停 5 秒，然后调用当前安全定点降落流程。

本程序会解锁并驱动真实无人机。运行前必须确认 ``server_ros.py`` 及其他
``FC_Server`` 程序已经关闭、飞行区域净空、雷达定位稳定且急停可用。
未显式传入 ``--confirm-flight`` 时，程序不会连接硬件。
"""

import argparse
import os
import sys
import threading
import time
from typing import Optional


SDK_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if SDK_DIR not in sys.path:
    sys.path.insert(0, SDK_DIR)

from FlightController import FC_Controller  # noqa: E402
from FlightController.Components import LD_Radar  # noqa: E402
from loguru import logger  # noqa: E402

from base_test_tadar import (  # noqa: E402
    SingleRadarNavigation,
    emergency_land,
    wait_for_radar_pose,
)


FC_SERIAL_DEV = "/dev/ttyACM0"
TAKEOFF_POINT = (0.0, 0.0)
FIRST_LIFT_HEIGHT = 60
TARGET_HEIGHT = 140.0
NAVIGATION_SPEED = 15.0
VERTICAL_SPEED = 22.0
MOTOR_WARMUP_SECONDS = 2.0
RAW_TAKEOFF_WAIT_SECONDS = 8.0
HOVER_SECONDS = 5.0
LANDING_HEIGHT_TIMEOUT = 8.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="使用旧版起飞时序执行单雷达定点起飞、悬停和降落测试"
    )
    parser.add_argument(
        "--confirm-flight",
        action="store_true",
        help="确认现场满足真实飞行条件；未提供时不会连接任何硬件",
    )
    parser.add_argument(
        "--fc-port",
        default=FC_SERIAL_DEV,
        help="飞控串口，默认 /dev/ttyACM0",
    )
    parser.add_argument(
        "--radar-port",
        default=None,
        help="雷达串口；默认按雷达 VID:PID 自动搜索",
    )
    return parser.parse_args()


def wait_stage(
    stop_event: threading.Event,
    seconds: float,
    stage_name: str,
) -> None:
    """保持旧版固定延时，同时允许 Ctrl+C/停止事件尽快终止等待。"""
    logger.info("[TEST] {}: wait {:.1f}s", stage_name, seconds)
    if stop_event.wait(seconds):
        raise RuntimeError("{} stopped by external stop event".format(stage_name))


class LegacyTakeoffMission:
    def __init__(
        self,
        fc: FC_Controller,
        radar: LD_Radar,
        navi: SingleRadarNavigation,
        stop_event: threading.Event,
    ):
        self.fc = fc
        self.radar = radar
        self.navi = navi
        self.stop_event = stop_event

    def prepare_navigation(self) -> None:
        """沿用 base_test_tadar.py 的单雷达定位初始化框架。"""
        self.navi.set_navigation_speed(NAVIGATION_SPEED)
        self.navi.set_vertical_speed(VERTICAL_SPEED)
        self.navi.start(mode="radar")
        logger.info("[TEST] Single-radar navigation started")

        wait_for_radar_pose(self.navi, self.radar)
        self.navi.calibrate_basepoint()
        calibration_completed_at = time.monotonic()
        wait_for_radar_pose(
            self.navi,
            self.radar,
            newer_than=calibration_completed_at,
        )
        logger.info("[TEST] Radar basepoint calibrated: {}", self.navi.basepoint)

    def disable_navigation_for_raw_takeoff(self) -> None:
        """旧版阶段 1：关闭导航和高度闭环，避免覆盖飞控一键起飞。"""
        self.navi.navigation_flag = False
        self.navi.keep_height_flag = False
        logger.info("[TEST] Legacy stage 1: navigation loops disabled")

    def enter_program_mode_and_unlock(self) -> None:
        """旧版阶段 2：切 PROGRAM 模式并解锁。"""
        self.fc.set_flight_mode(self.fc.PROGRAM_MODE)
        self.fc.unlock()
        logger.info("[TEST] Legacy stage 2: PROGRAM mode and unlock requested")

    def wait_for_motor_warmup(self) -> None:
        """旧版阶段 3：解锁后固定等待 2 秒。"""
        wait_stage(
            self.stop_event,
            MOTOR_WARMUP_SECONDS,
            "Legacy stage 3 motor warmup",
        )
        state_fresh = bool(
            getattr(self.fc.state, "is_fresh", lambda _age: False)(0.5)
        )
        if (
            not state_fresh
            or self.fc.state.mode.value != self.fc.PROGRAM_MODE
            or not self.fc.state.unlock.value
        ):
            raise RuntimeError(
                "fresh PROGRAM/unlock feedback was not confirmed after motor warmup"
            )
        logger.info("[TEST] Legacy stage 3: PROGRAM/unlock feedback confirmed")

    def send_raw_takeoff_and_wait(self) -> None:
        """旧版阶段 4：发送一次 60 cm 起飞命令并固定等待 8 秒。"""
        self.fc.take_off(FIRST_LIFT_HEIGHT)
        logger.info(
            "[TEST] Legacy stage 4: one-key takeoff requested at {}cm",
            FIRST_LIFT_HEIGHT,
        )
        wait_stage(
            self.stop_event,
            RAW_TAKEOFF_WAIT_SECONDS,
            "Legacy stage 4 raw takeoff",
        )

    def confirm_hovering(self) -> None:
        """旧版阶段 5：等待飞控悬停反馈；失败时不继续切换闭环。"""
        if not self.fc.wait_for_hovering(2):
            raise RuntimeError("hovering was not confirmed after legacy takeoff")
        logger.info("[TEST] Legacy stage 5: hovering confirmed")

    def climb_with_height_pid(self) -> None:
        """旧版阶段 6：切定点模式，用高度 PID 上升到目标高度。"""
        self.fc.set_flight_mode(self.fc.HOLD_POS_MODE)
        self.navi.set_height(TARGET_HEIGHT)
        self.navi.keep_height_flag = True
        if not self.navi.wait_for_height():
            raise RuntimeError("legacy height-PID climb was not confirmed")
        logger.info(
            "[TEST] Legacy stage 6: height PID reached {:.0f}cm",
            TARGET_HEIGHT,
        )

    def enable_position_hold(self) -> None:
        """旧版阶段 7：设置起飞点并开启水平导航闭环。"""
        self.navi.direct_set_waypoint(TAKEOFF_POINT)
        self.navi.switch_pid("hover")
        wait_stage(
            self.stop_event,
            0.1,
            "Legacy stage 7 position-hold handoff",
        )
        self.navi.navigation_flag = True
        logger.info(
            "[TEST] Legacy stage 7: position hold enabled at {}",
            TAKEOFF_POINT,
        )

    def legacy_pointing_takeoff(self) -> None:
        """按旧版函数顺序执行拆分后的七个起飞阶段。"""
        self.disable_navigation_for_raw_takeoff()
        self.enter_program_mode_and_unlock()
        self.wait_for_motor_warmup()
        self.send_raw_takeoff_and_wait()
        self.confirm_hovering()
        self.climb_with_height_pid()
        self.enable_position_hold()

    def run(self) -> None:
        logger.warning("[TEST] Confirmed real-flight mode")
        logger.info(
            "[TEST] Legacy pointing takeoff at {} to {:.0f}cm",
            TAKEOFF_POINT,
            TARGET_HEIGHT,
        )
        self.legacy_pointing_takeoff()

        wait_stage(self.stop_event, HOVER_SECONDS, "Position hover")

        logger.info("[TEST] Hover complete; pointing landing at {}", TAKEOFF_POINT)
        if not self.navi.pointing_landing(
            TAKEOFF_POINT,
            height_timeout=LANDING_HEIGHT_TIMEOUT,
        ):
            raise RuntimeError("pointing landing was not confirmed")
        logger.info("[TEST] Legacy pointing-takeoff test completed")

    def stop(self) -> None:
        self.stop_event.set()
        self.navi.stop()
        logger.info("[TEST] Mission stopped")


def main() -> int:
    args = parse_args()
    if not args.confirm_flight:
        print(
            "[!] 此程序会解锁并驱动真实无人机。"
            "确认 server_ros.py 已关闭且现场安全后，添加 --confirm-flight。"
        )
        return 2

    stop_event = threading.Event()
    fc: Optional[FC_Controller] = None
    radar: Optional[LD_Radar] = None
    navi: Optional[SingleRadarNavigation] = None
    mission: Optional[LegacyTakeoffMission] = None
    takeoff_attempted = False

    try:
        fc = FC_Controller()
        fc.start_listen_serial(
            serial_dev=args.fc_port,
            print_state=False,
        )
        if not fc.wait_for_connection(timeout_s=10):
            raise RuntimeError("flight-controller connection timeout")
        if not fc.state.is_fresh(0.5):
            raise RuntimeError("flight-controller telemetry is stale")
        if fc.state.unlock.value:
            raise RuntimeError(
                "flight controller is already unlocked; test will not take control"
            )
        logger.info("[TEST] Flight controller connected through direct serial")

        radar = LD_Radar()
        radar.debug = False
        radar.start(com=args.radar_port)
        logger.info("[TEST] Single radar started")

        navi = SingleRadarNavigation(
            fc=fc,
            radar=radar,
            stop_event=stop_event,
        )
        mission = LegacyTakeoffMission(
            fc=fc,
            radar=radar,
            navi=navi,
            stop_event=stop_event,
        )
        mission.prepare_navigation()

        takeoff_attempted = True
        mission.run()
        takeoff_attempted = False
        return 0
    except KeyboardInterrupt:
        logger.warning("[TEST] Interrupted by user")
        return 130
    except Exception:
        logger.exception("[TEST] Legacy pointing-takeoff test failed")
        return 1
    finally:
        if mission is not None:
            try:
                mission.stop()
            except Exception:
                logger.exception("[TEST] Failed to stop mission")
        elif navi is not None:
            try:
                navi.stop()
            except Exception:
                logger.exception("[TEST] Failed to stop navigation")

        if (
            takeoff_attempted
            and fc is not None
            and fc.connected
            and fc.state.unlock.value
        ):
            try:
                emergency_land(fc)
            except Exception:
                logger.exception("[TEST] Emergency landing request failed")

        if radar is not None:
            try:
                if radar.running:
                    radar.stop()
            except Exception:
                logger.exception("[TEST] Failed to stop radar")

        if fc is not None:
            try:
                fc.close()
            except Exception:
                logger.exception("[TEST] Failed to close flight controller")


if __name__ == "__main__":
    sys.exit(main())
