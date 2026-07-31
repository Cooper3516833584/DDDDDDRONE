"""
单雷达非定点快速起飞实飞测试。

默认只连接飞控和雷达、建立单雷达定位并持续打印位置，不会解锁或起飞。
只有显式传入 ``--confirm-flight`` 才会执行真实飞行测试：

    非定点垂直起飞（90 cm 一键离地，垂直爬升至 150 cm）
      -> (100, 0)
      -> (0, 0)
      -> 下降至 50 cm
      -> 识别H标记并以30像素阈值完成视觉微调
      -> 飞控一键降落

新起飞函数仅供 mission1_26.py 和 mission2_26.py 的快速任务使用，
不适用于要求定点起飞的常规任务。坐标和高度单位均为 cm；水平坐标系
为 x 向前、y 向左。运行前必须确认 server_ros.py 及其他 FC_Server
程序已经关闭，避免抢占飞控串口。
"""

import argparse
import math
import queue
import sys
import threading
import time
from typing import Optional

from loguru import logger

from FlightController import FC_Controller
from FlightController.Components import LD_Radar
from FlightController.Solutions.Navigation import Navigation
from landing_marker_offset import track_home_h_marker


FC_SERIAL_DEV = "/dev/ttyACM0"
CAMERA_INDEX = 0
CRUISE_SPEED = 15.0
CRUISE_HEIGHT = 150.0
VERTICAL_SPEED = 22.0
LANDING_HEIGHT_TIMEOUT = 8.0
LANDING_HEIGHT = 50.0
LANDING_HEIGHT_TOLERANCE = 8.0
LANDING_PIXEL_THRESHOLD = 30.0
LANDING_CENTER_CONFIRM_FRAMES = 5
LANDING_APPROACH_SPEED = 15.0
LANDING_CONTROL_FREQUENCY = 10.0
LANDING_ALIGNMENT_TIMEOUT = 60.0
LANDING_FRAME_TIMEOUT = 1.0
LANDING_MIN_CONTROL_HEIGHT = 25.0
LANDING_MAX_CONTROL_HEIGHT = 75.0
LANDING_TOUCHDOWN_ALTITUDE = 8.0
LANDING_TOUCHDOWN_TIMEOUT = 12.0
LANDING_LOCK_TIMEOUT = 4.0
TAKEOFF_POINT = (0.0, 0.0)
TEST_WAYPOINT = (100.0, 0.0)
RADAR_POSE_READY_TIMEOUT = 15.0
MONITOR_INTERVAL = 1.0


class SingleRadarNavigation(Navigation):
    """在底层确认雷达三轴有效后记录位姿新鲜度。"""

    def _get_radar_pose(self, wait=True):
        pose = super()._get_radar_pose(wait=wait)
        if pose is not None and pose[3]:
            self._last_pose_update = time.monotonic()
        return pose


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="单雷达非定点快速起飞导航测试"
    )
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
    raise RuntimeError(
        "single-radar navigation pose was not ready before timeout"
    )


class _HomeHOffsetReader:
    """在守护线程中读取相机，向控制线程只提供最新一帧偏移。"""

    def __init__(self, camera_index: int):
        self._offsets = track_home_h_marker(camera_index)
        self._output = queue.Queue(maxsize=1)
        self._stop_event = threading.Event()
        self._thread = threading.Thread(
            target=self._run,
            name="home-h-offset-reader",
            daemon=True,
        )

    def _publish(self, item) -> None:
        try:
            self._output.get_nowait()
        except queue.Empty:
            pass
        try:
            self._output.put_nowait(item)
        except queue.Full:
            pass

    def _run(self) -> None:
        try:
            while not self._stop_event.is_set():
                try:
                    offset = next(self._offsets)
                except StopIteration:
                    self._publish(
                        (
                            "error",
                            RuntimeError("H-marker tracker stopped unexpectedly"),
                        )
                    )
                    return
                except Exception as exc:
                    self._publish(("error", exc))
                    return
                self._publish(("offset", offset))
        finally:
            try:
                self._offsets.close()
            except Exception:
                logger.exception("[HOME-LAND] Failed to close H-marker tracker")

    def start(self) -> None:
        self._thread.start()

    def read(self, timeout: float):
        kind, payload = self._output.get(timeout=timeout)
        if kind == "error":
            raise payload
        return payload

    def close(self) -> None:
        self._stop_event.set()
        self._thread.join(timeout=1.0)
        if self._thread.is_alive():
            logger.warning(
                "[HOME-LAND] Camera reader did not stop promptly; "
                "daemon cleanup deferred until process exit"
            )


def visual_home_h_landing(
    fc: FC_Controller,
    navi: Navigation,
    stop_event: threading.Event,
    camera_index: int,
) -> bool:
    """下降至50 cm，以H标记完成视觉微调，再交给飞控一键降落。"""
    if stop_event.is_set():
        logger.warning("[HOME-LAND] Visual landing stopped before descent")
        return False
    if not navi.running:
        logger.error("[HOME-LAND] Navigation is not running")
        return False
    if not fc.state.is_fresh(0.5) or not fc.state.unlock.value:
        logger.error("[HOME-LAND] Flight state is stale or aircraft is locked")
        return False
    if not navi.pose_is_fresh():
        logger.error("[HOME-LAND] Navigation pose is stale")
        return False

    navi.set_height(LANDING_HEIGHT)
    navi.keep_height_flag = True
    if not navi.wait_for_height(
        height_thres=LANDING_HEIGHT_TOLERANCE,
        timeout=LANDING_HEIGHT_TIMEOUT,
    ):
        logger.error(
            "[HOME-LAND] Failed to reach visual approach height {}cm",
            LANDING_HEIGHT,
        )
        return False
    if (
        stop_event.is_set()
        or not fc.state.is_fresh(0.5)
        or not navi.pose_is_fresh()
    ):
        logger.error("[HOME-LAND] State invalid before visual alignment")
        return False

    logger.info(
        "[HOME-LAND] Reached {}cm; tracking H marker on camera {}",
        LANDING_HEIGHT,
        camera_index,
    )
    offset_reader = _HomeHOffsetReader(camera_index)
    offset_reader.start()
    centered = False
    centered_frames = 0
    period = 1.0 / LANDING_CONTROL_FREQUENCY
    deadline = time.monotonic() + LANDING_ALIGNMENT_TIMEOUT
    try:
        while time.monotonic() < deadline:
            if stop_event.is_set():
                logger.warning("[HOME-LAND] Alignment stopped externally")
                break
            if (
                not fc.state.is_fresh(0.5)
                or not fc.state.unlock.value
                or fc.state.mode.value != fc.HOLD_POS_MODE
                or not navi.pose_is_fresh()
            ):
                logger.error(
                    "[HOME-LAND] Flight or radar state became invalid "
                    "during visual alignment"
                )
                break
            try:
                current_height = float(navi.current_height)
            except Exception:
                current_height = math.nan
            if (
                not math.isfinite(current_height)
                or current_height < LANDING_MIN_CONTROL_HEIGHT
                or current_height > LANDING_MAX_CONTROL_HEIGHT
            ):
                logger.error(
                    "[HOME-LAND] Unsafe height during visual alignment: {}cm",
                    current_height,
                )
                break

            # Stop before waiting for the next camera frame. If camera capture
            # blocks, no previous horizontal velocity remains latched.
            navi.stop_move()
            remaining = deadline - time.monotonic()
            try:
                x_px, y_px = offset_reader.read(
                    timeout=max(
                        0.01,
                        min(LANDING_FRAME_TIMEOUT, remaining),
                    )
                )
            except queue.Empty:
                logger.error(
                    "[HOME-LAND] No new H-marker camera frame within {:.1f}s",
                    LANDING_FRAME_TIMEOUT,
                )
                break

            if (
                time.monotonic() >= deadline
                or stop_event.is_set()
                or not fc.state.is_fresh(0.5)
                or not fc.state.unlock.value
                or fc.state.mode.value != fc.HOLD_POS_MODE
                or not navi.pose_is_fresh()
            ):
                logger.error(
                    "[HOME-LAND] State invalid after waiting for camera frame"
                )
                break
            try:
                current_height = float(navi.current_height)
            except Exception:
                current_height = math.nan
            if (
                not math.isfinite(current_height)
                or current_height < LANDING_MIN_CONTROL_HEIGHT
                or current_height > LANDING_MAX_CONTROL_HEIGHT
            ):
                logger.error(
                    "[HOME-LAND] Unsafe height after camera wait: {}cm",
                    current_height,
                )
                break

            if x_px is None or y_px is None:
                centered_frames = 0
                stop_event.wait(period)
                continue

            x_px = float(x_px)
            y_px = float(y_px)
            if not math.isfinite(x_px) or not math.isfinite(y_px):
                centered_frames = 0
                stop_event.wait(period)
                continue

            distance_px = math.hypot(x_px, y_px)
            if distance_px <= LANDING_PIXEL_THRESHOLD:
                centered_frames += 1
                logger.info(
                    "[HOME-LAND] H marker centered {}/{}: distance={:.1f}px",
                    centered_frames,
                    LANDING_CENTER_CONFIRM_FRAMES,
                    distance_px,
                )
                if centered_frames >= LANDING_CENTER_CONFIRM_FRAMES:
                    centered = True
                    break
                stop_event.wait(period)
                continue

            centered_frames = 0
            direction_deg = math.degrees(math.atan2(y_px, x_px))
            navi.move_by_direction(
                speed=LANDING_APPROACH_SPEED,
                direction_deg=direction_deg,
            )
            stop_event.wait(period)
            navi.stop_move()
    finally:
        navi.stop_move()
        offset_reader.close()

    if not centered:
        logger.error("[HOME-LAND] H-marker alignment was not confirmed")
        return False
    if not fc.state.is_fresh(0.5) or not fc.state.unlock.value:
        logger.error("[HOME-LAND] Flight state invalid after visual alignment")
        return False

    navi.navigation_flag = False
    navi.keep_height_flag = False
    fc.set_flight_mode(fc.PROGRAM_MODE)
    time.sleep(0.1)
    fc.stablize()
    fc.land()

    started_at = time.perf_counter()
    landed = False
    while time.perf_counter() - started_at < LANDING_TOUCHDOWN_TIMEOUT:
        time.sleep(0.1)
        try:
            altitude = float(fc.state.alt_add.value)
        except Exception:
            altitude = 999.0
        if (
            fc.state.is_fresh(0.5)
            and altitude <= LANDING_TOUCHDOWN_ALTITUDE
        ) or not fc.state.unlock.value:
            landed = True
            break

    if not landed:
        logger.error(
            "[HOME-LAND] Landing timeout; keep landing command active and "
            "refuse airborne force-lock"
        )
        fc.land()
        return False

    try:
        locked = fc.wait_for_lock(timeout_s=LANDING_LOCK_TIMEOUT)
    except TypeError:
        locked = fc.wait_for_lock(LANDING_LOCK_TIMEOUT)
    if not locked:
        state_fresh = fc.state.is_fresh(0.5)
        altitude = (
            float(fc.state.alt_add.value) if state_fresh else 999.0
        )
        if state_fresh and altitude <= LANDING_TOUCHDOWN_ALTITUDE:
            fc.lock()
            try:
                locked = fc.wait_for_lock(timeout_s=LANDING_LOCK_TIMEOUT)
            except TypeError:
                locked = fc.wait_for_lock(LANDING_LOCK_TIMEOUT)
            if not locked:
                logger.error(
                    "[HOME-LAND] Lock command was sent but lock feedback "
                    "was not confirmed"
                )
                return False
        else:
            logger.error(
                "[HOME-LAND] Lock not confirmed; refuse lock without "
                "fresh touchdown altitude"
            )
            return False
    return True


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
        logger.info(
            "[TEST] Radar basepoint calibrated: {}", self.navi.basepoint
        )

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
        """执行快速起飞、单航点导航、返航和定点降落。"""
        logger.warning("[TEST] Confirmed real-flight mode")
        logger.info(
            "[TEST] Non-pointing vertical takeoff: "
            "first lift 90cm, target {:.0f}cm",
            CRUISE_HEIGHT,
        )
        self.navi.fast_non_pointing_takeoff(
            target_height=CRUISE_HEIGHT,
        )

        self.navi.set_yaw(0)
        if not self.navi.wait_for_yaw():
            raise RuntimeError("yaw stabilization was not confirmed")

        logger.info("[TEST] Navigate to waypoint {}", TEST_WAYPOINT)
        if not self.navi.navigation_to_waypoint(TEST_WAYPOINT, wait=True):
            raise RuntimeError(
                "failed to reach waypoint {}".format(TEST_WAYPOINT)
            )

        logger.info("[TEST] Return to takeoff point {}", TAKEOFF_POINT)
        if not self.navi.navigation_to_waypoint(TAKEOFF_POINT, wait=True):
            raise RuntimeError("failed to return to takeoff point")

        logger.info(
            "[TEST] Descend to 50cm, align over home H marker and land"
        )
        if not visual_home_h_landing(
            fc=self.fc,
            navi=self.navi,
            stop_event=self.stop_event,
            camera_index=CAMERA_INDEX,
        ):
            raise RuntimeError("visual H-marker landing was not confirmed")
        logger.info("[TEST] Fast takeoff navigation flight completed")

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
        logger.exception("[TEST] Fast takeoff radar test failed")
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
