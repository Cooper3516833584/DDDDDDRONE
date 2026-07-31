"""
单雷达非定点快速起飞实飞测试。

默认只连接飞控和雷达、建立单雷达定位并持续打印位置，不会解锁或起飞。
只有显式传入 ``--confirm-flight`` 才会执行真实飞行测试：

    非定点垂直起飞（90 cm 一键离地，垂直爬升至 150 cm）
      -> (100, 0)
      -> (0, 0)
      -> 下降至 60 cm
      -> 识别H标记并以30像素阈值完成视觉校准
      -> 在该点调用 pointing_landing 定点降落

新起飞函数仅供 mission1_26.py 和 mission2_26.py 的快速任务使用，
不适用于要求定点起飞的常规任务。坐标和高度单位均为 cm；水平坐标系
为 x 向前、y 向左。运行前必须确认 server_ros.py 及其他 FC_Server
程序已经关闭，避免抢占飞控串口。
"""

import argparse
import csv
import math
import queue
import sys
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Optional

from loguru import logger

from FlightController import FC_Controller
from FlightController.Components import LD_Radar
from FlightController.Solutions.Navigation import Navigation
from landing_marker_offset import track_home_h_marker


FC_SERIAL_DEV = "/dev/ttyACM0"
CAMERA_INDEX = 0
VIDEO_OUTPUT_DIR_NAME = "fc_log"
CRUISE_SPEED = 15.0
CRUISE_HEIGHT = 150.0
VERTICAL_SPEED = 22.0
LANDING_HEIGHT_TIMEOUT = 8.0
LANDING_HEIGHT = 60.0
LANDING_HEIGHT_TOLERANCE = 8.0
LANDING_PIXEL_THRESHOLD = 30.0
LANDING_CENTER_CONFIRM_FRAMES = 5
LANDING_APPROACH_SPEED = 15.0
LANDING_CONTROL_FREQUENCY = 10.0
LANDING_ALIGNMENT_TIMEOUT = 60.0
LANDING_FRAME_TIMEOUT = 1.0
CAMERA_START_TIMEOUT = 10.0
LANDING_MIN_CONTROL_HEIGHT = 25.0
LANDING_MAX_CONTROL_HEIGHT = 75.0
LANDING_TOUCHDOWN_ALTITUDE = 8.0
LANDING_TOUCHDOWN_TIMEOUT = 12.0
LANDING_LOCK_TIMEOUT = 4.0
TAKEOFF_POINT = (0.0, 0.0)
TEST_WAYPOINT = (100.0, 0.0)
RADAR_POSE_READY_TIMEOUT = 15.0
MONITOR_INTERVAL = 1.0


def _safe_float(value, default: float = math.nan) -> float:
    try:
        result = float(value)
    except Exception:
        return default
    if not math.isfinite(result):
        return default
    return result


def _fc_mode_value(fc: FC_Controller) -> Optional[int]:
    try:
        return int(fc.state.mode.value)
    except Exception:
        return None


def _fc_unlock_value(fc: FC_Controller) -> Optional[bool]:
    try:
        return bool(fc.state.unlock.value)
    except Exception:
        return None


def _navi_snapshot(navi: Navigation) -> dict:
    pose_fresh = None
    try:
        pose_fresh = bool(navi.pose_is_fresh())
    except Exception:
        pass
    return {
        "x_cm": _safe_float(getattr(navi, "current_x", None)),
        "y_cm": _safe_float(getattr(navi, "current_y", None)),
        "yaw_deg": _safe_float(getattr(navi, "current_yaw", None)),
        "height_cm": _safe_float(getattr(navi, "current_height", None)),
        "pose_fresh": pose_fresh,
    }


class _LandingTelemetryLog:
    """Append-only CSV telemetry log for diagnosing landing accuracy."""

    _FIELDS = [
        "wall_time",
        "monotonic_s",
        "phase",
        "x_px",
        "y_px",
        "pixel_distance_px",
        "height_cm",
        "x_cm",
        "y_cm",
        "yaw_deg",
        "fc_mode",
        "unlock",
        "pose_fresh",
        "direction_deg",
        "speed_cm_s",
        "centered_frames",
        "message",
    ]

    def __init__(self, path: Path):
        self.path = path
        self._lock = threading.Lock()
        self._closed = False
        self._file = path.open("w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._file, fieldnames=self._FIELDS)
        self._writer.writeheader()
        self._file.flush()

    def write(
        self,
        phase: str,
        x_px: Optional[float] = None,
        y_px: Optional[float] = None,
        height_cm: Optional[float] = None,
        x_cm: Optional[float] = None,
        y_cm: Optional[float] = None,
        yaw_deg: Optional[float] = None,
        fc_mode: Optional[int] = None,
        unlock: Optional[bool] = None,
        pose_fresh: Optional[bool] = None,
        direction_deg: Optional[float] = None,
        speed_cm_s: Optional[float] = None,
        centered_frames: Optional[int] = None,
        message: str = "",
    ) -> None:
        pixel_distance_px = (
            math.hypot(x_px, y_px)
            if x_px is not None and y_px is not None
            else None
        )
        row = {
            "wall_time": datetime.now().isoformat(timespec="milliseconds"),
            "monotonic_s": time.monotonic(),
            "phase": phase,
            "x_px": x_px,
            "y_px": y_px,
            "pixel_distance_px": pixel_distance_px,
            "height_cm": height_cm,
            "x_cm": x_cm,
            "y_cm": y_cm,
            "yaw_deg": yaw_deg,
            "fc_mode": fc_mode,
            "unlock": unlock,
            "pose_fresh": pose_fresh,
            "direction_deg": direction_deg,
            "speed_cm_s": speed_cm_s,
            "centered_frames": centered_frames,
            "message": message,
        }
        with self._lock:
            if self._closed:
                return
            try:
                self._writer.writerow(row)
                self._file.flush()
            except Exception:
                logger.exception(
                    "[HOME-LAND] Failed to write telemetry row"
                )

    def close(self) -> None:
        with self._lock:
            if self._closed:
                return
            self._closed = True
            try:
                self._file.flush()
                self._file.close()
            except Exception:
                logger.exception("[HOME-LAND] Failed to close telemetry log")


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

    def __init__(
        self,
        camera_index: int,
        video_output_path: Optional[str] = None,
    ):
        self._offsets = track_home_h_marker(
            camera_index,
            video_output_path=video_output_path,
        )
        self.video_output_path = video_output_path
        self._output = queue.Queue(maxsize=1)
        self._stop_event = threading.Event()
        self._tracking_event = threading.Event()
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

    def _read_and_publish(self) -> bool:
        try:
            offset = next(self._offsets)
        except StopIteration:
            self._publish(
                (
                    "error",
                    RuntimeError("H-marker tracker stopped unexpectedly"),
                )
            )
            return False
        except Exception as exc:
            self._publish(("error", exc))
            return False
        self._publish(("offset", offset))
        return True

    def _run(self) -> None:
        try:
            # The first next() opens the camera, warms it up and verifies that
            # it can produce a frame. H-marker processing remains paused until
            # the aircraft starts returning from the test waypoint.
            if not self._read_and_publish():
                return
            while not self._stop_event.is_set():
                if not self._tracking_event.wait(timeout=0.1):
                    continue
                if self._stop_event.is_set():
                    break
                if not self._read_and_publish():
                    return
        finally:
            try:
                self._offsets.close()
            except Exception:
                logger.exception("[HOME-LAND] Failed to close H-marker tracker")

    def start(self) -> None:
        self._thread.start()

    def enable_tracking(self) -> None:
        self._tracking_event.set()

    def read(self, timeout: float):
        kind, payload = self._output.get(timeout=timeout)
        if kind == "error":
            raise payload
        return payload

    def close(self) -> None:
        self._stop_event.set()
        self._tracking_event.set()
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
    offset_reader: _HomeHOffsetReader,
    camera_index: int,
    telemetry_log: Optional[_LandingTelemetryLog] = None,
) -> bool:
    """下降至60 cm，以H标记完成视觉校准，再在该点调用 pointing_landing 定点降落。"""
    snapshot = _navi_snapshot(navi)
    if telemetry_log is not None:
        telemetry_log.write(
            "visual_landing_start",
            height_cm=snapshot["height_cm"],
            x_cm=snapshot["x_cm"],
            y_cm=snapshot["y_cm"],
            yaw_deg=snapshot["yaw_deg"],
            fc_mode=_fc_mode_value(fc),
            unlock=_fc_unlock_value(fc),
            pose_fresh=snapshot["pose_fresh"],
            message="descend to 60cm and align over home H",
        )
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
    snapshot = _navi_snapshot(navi)
    if telemetry_log is not None:
        telemetry_log.write(
            "visual_approach_height",
            height_cm=snapshot["height_cm"],
            x_cm=snapshot["x_cm"],
            y_cm=snapshot["y_cm"],
            yaw_deg=snapshot["yaw_deg"],
            fc_mode=_fc_mode_value(fc),
            unlock=_fc_unlock_value(fc),
            pose_fresh=snapshot["pose_fresh"],
            message="visual approach height reached",
        )
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
                if telemetry_log is not None:
                    telemetry_log.write(
                        "camera_frame_timeout",
                        height_cm=_safe_float(navi.current_height),
                        x_cm=_safe_float(navi.current_x),
                        y_cm=_safe_float(navi.current_y),
                        yaw_deg=_safe_float(navi.current_yaw),
                        fc_mode=_fc_mode_value(fc),
                        unlock=_fc_unlock_value(fc),
                        pose_fresh=(
                            None
                            if not hasattr(navi, "pose_is_fresh")
                            else navi.pose_is_fresh()
                        ),
                        message=(
                            "no new H-marker camera frame within "
                            f"{LANDING_FRAME_TIMEOUT:.1f}s"
                        ),
                    )
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
                if telemetry_log is not None:
                    telemetry_log.write(
                        "no_h_marker",
                        height_cm=_safe_float(navi.current_height),
                        x_cm=_safe_float(navi.current_x),
                        y_cm=_safe_float(navi.current_y),
                        yaw_deg=_safe_float(navi.current_yaw),
                        fc_mode=_fc_mode_value(fc),
                        unlock=_fc_unlock_value(fc),
                        pose_fresh=(
                            None
                            if not hasattr(navi, "pose_is_fresh")
                            else navi.pose_is_fresh()
                        ),
                        centered_frames=0,
                    )
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
                if telemetry_log is not None:
                    telemetry_log.write(
                        "h_offset",
                        x_px=x_px,
                        y_px=y_px,
                        height_cm=_safe_float(navi.current_height),
                        x_cm=_safe_float(navi.current_x),
                        y_cm=_safe_float(navi.current_y),
                        yaw_deg=_safe_float(navi.current_yaw),
                        fc_mode=_fc_mode_value(fc),
                        unlock=_fc_unlock_value(fc),
                        pose_fresh=(
                            None
                            if not hasattr(navi, "pose_is_fresh")
                            else navi.pose_is_fresh()
                        ),
                        direction_deg=0.0,
                        speed_cm_s=0.0,
                        centered_frames=centered_frames,
                        message="H marker centered",
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
            if telemetry_log is not None:
                telemetry_log.write(
                    "h_offset",
                    x_px=x_px,
                    y_px=y_px,
                    height_cm=_safe_float(navi.current_height),
                    x_cm=_safe_float(navi.current_x),
                    y_cm=_safe_float(navi.current_y),
                    yaw_deg=_safe_float(navi.current_yaw),
                    fc_mode=_fc_mode_value(fc),
                    unlock=_fc_unlock_value(fc),
                    pose_fresh=(
                        None
                        if not hasattr(navi, "pose_is_fresh")
                        else navi.pose_is_fresh()
                    ),
                    direction_deg=direction_deg,
                    speed_cm_s=LANDING_APPROACH_SPEED,
                    centered_frames=0,
                    message="visual correction command",
                )
            stop_event.wait(period)
            navi.stop_move()
    finally:
        navi.stop_move()

    if not centered:
        if telemetry_log is not None:
            telemetry_log.write(
                "alignment_failed",
                height_cm=_safe_float(navi.current_height),
                x_cm=_safe_float(navi.current_x),
                y_cm=_safe_float(navi.current_y),
                yaw_deg=_safe_float(navi.current_yaw),
                fc_mode=_fc_mode_value(fc),
                unlock=_fc_unlock_value(fc),
                pose_fresh=(
                    None
                    if not hasattr(navi, "pose_is_fresh")
                    else navi.pose_is_fresh()
                ),
                message="H-marker alignment was not confirmed",
            )
        logger.error("[HOME-LAND] H-marker alignment was not confirmed")
        return False
    if not fc.state.is_fresh(0.5) or not fc.state.unlock.value:
        logger.error("[HOME-LAND] Flight state invalid after visual alignment")
        return False

    navi.navigation_flag = False
    navi.keep_height_flag = False
    # 视觉校准完成后，以当前位置为落点调用定点降落
    landing_point = navi.current_point
    logger.info(
        "[HOME-LAND] Visual calibration completed at {}cm; "
        "calling pointing_landing at ({:.1f}, {:.1f})",
        LANDING_HEIGHT,
        float(landing_point[0]),
        float(landing_point[1]),
    )
    if telemetry_log is not None:
        telemetry_log.write(
            "landing_command",
            height_cm=_safe_float(navi.current_height),
            x_cm=_safe_float(navi.current_x),
            y_cm=_safe_float(navi.current_y),
            yaw_deg=_safe_float(navi.current_yaw),
            fc_mode=_fc_mode_value(fc),
            unlock=_fc_unlock_value(fc),
            pose_fresh=(
                None
                if not hasattr(navi, "pose_is_fresh")
                else navi.pose_is_fresh()
            ),
            message="calling pointing_landing at calibrated point",
        )
    if not navi.pointing_landing(landing_point):
        if telemetry_log is not None:
            telemetry_log.write(
                "landing_timeout",
                height_cm=_safe_float(navi.current_height),
                fc_mode=_fc_mode_value(fc),
                unlock=_fc_unlock_value(fc),
                message="pointing_landing was not confirmed",
            )
        logger.error("[HOME-LAND] Pointing landing was not confirmed")
        return False
    if telemetry_log is not None:
        telemetry_log.write(
            "locked",
            height_cm=_safe_float(navi.current_height),
            fc_mode=_fc_mode_value(fc),
            unlock=_fc_unlock_value(fc),
            message="landing lock confirmed",
        )
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
        self.camera_reader: Optional[_HomeHOffsetReader] = None
        self.video_path: Optional[Path] = None
        self.telemetry_log: Optional[_LandingTelemetryLog] = None

    def _new_runtime_path(self, prefix: str, suffix: str) -> Path:
        log_dir = Path(__file__).resolve().parent / VIDEO_OUTPUT_DIR_NAME
        log_dir.mkdir(parents=True, exist_ok=True)
        return log_dir / (
            prefix
            + datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            + suffix
        )

    def _log_phase(self, phase: str, message: str = "") -> None:
        if self.telemetry_log is None:
            return
        snapshot = _navi_snapshot(self.navi)
        self.telemetry_log.write(
            phase,
            height_cm=snapshot["height_cm"],
            x_cm=snapshot["x_cm"],
            y_cm=snapshot["y_cm"],
            yaw_deg=snapshot["yaw_deg"],
            fc_mode=_fc_mode_value(self.fc),
            unlock=_fc_unlock_value(self.fc),
            pose_fresh=snapshot["pose_fresh"],
            message=message,
        )

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

    def prepare_camera(self) -> None:
        """Open, warm up and verify the downward camera before takeoff."""
        if self.camera_reader is not None:
            return
        video_path = self._new_runtime_path(
            "home_h_landing_video_",
            ".avi",
        )
        telemetry_path = self._new_runtime_path(
            "home_h_landing_telemetry_",
            ".csv",
        )
        logger.info("[TEST] Video recording path: {}", video_path)
        logger.info("[TEST] Telemetry log path: {}", telemetry_path)
        self.video_path = video_path
        self.telemetry_log = _LandingTelemetryLog(telemetry_path)
        logger.info(
            "[TEST] Starting downward camera {} before takeoff",
            CAMERA_INDEX,
        )
        reader = _HomeHOffsetReader(
            CAMERA_INDEX,
            video_output_path=str(video_path),
        )
        self.camera_reader = reader
        reader.start()
        try:
            reader.read(timeout=CAMERA_START_TIMEOUT)
        except queue.Empty as exc:
            raise RuntimeError(
                "downward camera did not produce its first frame within "
                "{:.1f}s".format(CAMERA_START_TIMEOUT)
            ) from exc
        logger.info(
            "[TEST] Downward camera {} is open and producing frames",
            CAMERA_INDEX,
        )
        logger.info(
            "[TEST] Video recording and telemetry logging started"
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
        self.prepare_camera()
        self._log_phase("camera_ready", message="downward camera verified")
        camera_reader = self.camera_reader
        if camera_reader is None:
            raise RuntimeError("downward camera reader was not initialized")
        logger.info(
            "[TEST] Non-pointing vertical takeoff: "
            "first lift 90cm, target {:.0f}cm",
            CRUISE_HEIGHT,
        )
        self.navi.fast_non_pointing_takeoff(
            target_height=CRUISE_HEIGHT,
        )
        self._log_phase(
            "takeoff_complete",
            message=f"target cruise height {CRUISE_HEIGHT:.0f}cm",
        )

        self.navi.set_yaw(0)
        if not self.navi.wait_for_yaw():
            raise RuntimeError("yaw stabilization was not confirmed")
        self._log_phase("yaw_ready", message="yaw stabilized to 0deg")

        logger.info("[TEST] Navigate to waypoint {}", TEST_WAYPOINT)
        if not self.navi.navigation_to_waypoint(TEST_WAYPOINT, wait=True):
            raise RuntimeError(
                "failed to reach waypoint {}".format(TEST_WAYPOINT)
            )
        self._log_phase(
            "waypoint_reached",
            message=f"waypoint {TEST_WAYPOINT} reached",
        )

        logger.info(
            "[TEST] Start H-marker recognition before returning from {}",
            TEST_WAYPOINT,
        )
        camera_reader.enable_tracking()
        self._log_phase(
            "home_tracking_enabled",
            message="H-marker recognition enabled before return",
        )
        logger.info("[TEST] Return to takeoff point {}", TAKEOFF_POINT)
        if not self.navi.navigation_to_waypoint(TAKEOFF_POINT, wait=True):
            raise RuntimeError("failed to return to takeoff point")
        self._log_phase(
            "returned_home",
            message=f"takeoff point {TAKEOFF_POINT} reached",
        )

        logger.info(
            "[TEST] Descend to 60cm, align over home H marker, "
            "then pointing landing at the calibrated point"
        )
        if not visual_home_h_landing(
            fc=self.fc,
            navi=self.navi,
            stop_event=self.stop_event,
            offset_reader=camera_reader,
            camera_index=CAMERA_INDEX,
            telemetry_log=self.telemetry_log,
        ):
            raise RuntimeError("visual H-marker landing was not confirmed")
        self._log_phase(
            "landing_complete",
            message="visual H-marker landing completed",
        )
        logger.info("[TEST] Fast takeoff navigation flight completed")

    def stop(self) -> None:
        self.stop_event.set()
        self.navi.stop()
        if self.camera_reader is not None:
            self.camera_reader.close()
            self.camera_reader = None
        if self.telemetry_log is not None:
            self.telemetry_log.write(
                "mission_stopped",
                height_cm=_safe_float(self.navi.current_height),
                x_cm=_safe_float(self.navi.current_x),
                y_cm=_safe_float(self.navi.current_y),
                yaw_deg=_safe_float(self.navi.current_yaw),
                fc_mode=_fc_mode_value(self.fc),
                unlock=_fc_unlock_value(self.fc),
                pose_fresh=(
                    None
                    if not hasattr(self.navi, "pose_is_fresh")
                    else self.navi.pose_is_fresh()
                ),
                message="mission stopped",
            )
            self.telemetry_log.close()
            logger.info(
                "[TEST] Telemetry log closed: {}",
                self.telemetry_log.path,
            )
            self.telemetry_log = None
        if self.video_path is not None:
            logger.info("[TEST] Video recording path: {}", self.video_path)
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
