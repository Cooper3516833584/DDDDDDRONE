"""
单雷达定位基础导航测试程序。

默认只连接飞控和雷达、建立单雷达定位并持续打印位置，不会解锁或起飞。
只有显式传入 ``--confirm-flight`` 才会执行真实飞行测试：

    (0, 0) 定点起飞至 120 cm
      -> (100, 0)
      -> (100, -100)
      -> (0, -100)
      -> (0, 0)
      -> 定点降落

坐标和高度单位均为 cm；水平坐标系为 x 向前、y 向左。
运行前必须确认 server_ros.py 及其他 FC_Server 程序已经关闭，避免抢占飞控串口。
"""

import argparse
import sys
import threading
import time
from typing import Optional

from loguru import logger

from FlightController import FC_Controller
from FlightController.Components import LD_Radar
from FlightController.Solutions.Navigation import Navigation


FC_SERIAL_DEV = "/dev/ttyACM0"
CRUISE_SPEED = 15.0
CRUISE_HEIGHT = 150.0
VERTICAL_SPEED = 22.0
LANDING_HEIGHT_TIMEOUT = 8.0
TAKEOFF_POINT = (0.0, 0.0)
LANDING_POINT = (150.0, 0.0)
TEST_WAYPOINTS = (
    (150.0, 0.0)
)
RADAR_POSE_READY_TIMEOUT = 15.0
MONITOR_INTERVAL = 1.0

# 保留旧脚本中可能被外部引用的历史拼写。
CURISE_SPEED = CRUISE_SPEED
CUREISE_HEIGHT = CRUISE_HEIGHT


class SingleRadarNavigation(Navigation):
    """
    为单雷达模式补充位姿新鲜度时间戳。

    当前 Navigation._get_radar_pose() 会返回位姿有效标志，但不会像
    T265 路径那样更新 _last_pose_update，导致 wait_for_waypoint() 等
    安全检查始终认为雷达位姿过期。此任务层兼容类不修改底层控制文件，
    只在底层已经确认雷达三轴位姿有效时记录更新时间。
    """

    def _get_radar_pose(self, wait=True):
        pose = super()._get_radar_pose(wait=wait)
        if pose is not None and pose[3]:
            self._last_pose_update = time.monotonic()
        return pose


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="单雷达定位基础导航测试")
    parser.add_argument(
        "--confirm-flight",
        action="store_true",
        help="确认现场满足真实飞行条件；未提供时仅监视定位，不会解锁",
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


def wait_for_radar_pose(
    navi: Navigation,
    radar: LD_Radar,
    timeout: float = RADAR_POSE_READY_TIMEOUT,
    newer_than: float = 0.0,
) -> None:
    """等待雷达数据和三轴位姿均有效，超时则拒绝进入飞行流程。"""
    deadline = time.monotonic() + timeout
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
                "[TEST] Radar pose ready: ({:.1f}, {:.1f}), yaw={:.1f}".format(
                    navi.current_x,
                    navi.current_y,
                    navi.current_yaw,
                )
            )
            return
        time.sleep(0.1)
    raise RuntimeError("single-radar navigation pose was not ready before timeout")


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

    def prepare_navigation(self) -> None:
        """启动单雷达导航、等待有效位姿并以当前位置建立任务原点。"""
        self.navi.set_navigation_speed(CRUISE_SPEED)
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

    def monitor_pose(self) -> None:
        """监视单雷达定位，不解锁、不起飞。"""
        logger.warning(
            "[TEST] Monitor-only mode: the aircraft will not be unlocked; "
            "press Ctrl+C to exit"
        )
        while not self.stop_event.wait(MONITOR_INTERVAL):
            if not self.navi.pose_is_fresh():
                logger.warning("[TEST] Radar navigation pose is stale")
                continue
            logger.info(
                "[TEST] position=({:.1f}, {:.1f})cm yaw={:.1f}deg".format(
                    self.navi.current_x,
                    self.navi.current_y,
                    self.navi.current_yaw,
                )
            )

    def run(self) -> None:
        """执行定点起飞、方形航线、返航和定点降落。"""
        logger.warning("[TEST] Confirmed real-flight mode")
        logger.info(
            "[TEST] Pointing takeoff at {} to {:.0f}cm".format(
                TAKEOFF_POINT,
                CRUISE_HEIGHT,
            )
        )
        self.navi.pointing_takeoff(
            TAKEOFF_POINT,
            target_height=CRUISE_HEIGHT,
        )

        self.navi.set_yaw(0)
        if not self.navi.wait_for_yaw():
            raise RuntimeError("yaw stabilization was not confirmed")

        for waypoint in TEST_WAYPOINTS:
            if self.stop_event.is_set():
                raise RuntimeError("mission stopped before reaching all waypoints")
            logger.info("[TEST] Navigate to waypoint {}", waypoint)
            if not self.navi.navigation_to_waypoint(waypoint, wait=True):
                raise RuntimeError("failed to reach waypoint {}".format(waypoint))

        logger.info("[TEST] Pointing landing at {}", LANDING_POINT)
        if not self.navi.pointing_landing(
            LANDING_POINT,
            height_timeout=LANDING_HEIGHT_TIMEOUT,
        ):
            raise RuntimeError("pointing landing was not confirmed")
        logger.info("[TEST] Single-radar navigation flight completed")

    def stop(self) -> None:
        self.stop_event.set()
        self.navi.stop()
        logger.info("[TEST] Mission stopped")


def emergency_land(fc: FC_Controller) -> None:
    """异常退出时请求降落；未确认落地前不强制锁桨。"""
    logger.warning("[TEST] Flight interrupted; requesting emergency landing")
    fc.set_flight_mode(fc.PROGRAM_MODE)
    time.sleep(0.1)
    fc.stablize()
    fc.land()
    if not fc.wait_for_lock(timeout_s=20):
        logger.error(
            "[TEST] Landing lock was not confirmed; keep landing command active "
            "and refuse airborne force-lock"
        )
        fc.land()


def main() -> int:
    args = parse_args()
    stop_event = threading.Event()
    fc: Optional[FC_Controller] = None
    radar: Optional[LD_Radar] = None
    navi: Optional[Navigation] = None
    mission: Optional[Mission] = None
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
                "flight controller is already unlocked; base test will not take control"
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
        mission = Mission(
            fc=fc,
            radar=radar,
            navi=navi,
            stop_event=stop_event,
        )
        mission.prepare_navigation()

        if args.confirm_flight:
            takeoff_attempted = True
            mission.run()
            takeoff_attempted = False
        else:
            mission.monitor_pose()
        return 0
    except KeyboardInterrupt:
        logger.warning("[TEST] Interrupted by user")
        return 130
    except Exception:
        logger.exception("[TEST] Single-radar base test failed")
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
