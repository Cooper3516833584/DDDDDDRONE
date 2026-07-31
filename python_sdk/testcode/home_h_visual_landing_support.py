"""仅供独立实飞测试使用的H标记视觉微调和降落支持代码。"""

import math
import time
from typing import Callable, Iterator, Optional, Tuple

from loguru import logger

from landing_marker_offset import track_home_h_marker


APPROACH_HEIGHT_CM = 50.0
HEIGHT_TOLERANCE_CM = 8.0
HEIGHT_TIMEOUT_SECONDS = 8.0
PIXEL_THRESHOLD = 30.0
APPROACH_SPEED_CM_S = 15.0
CONTROL_FREQUENCY_HZ = 10.0
ALIGNMENT_TIMEOUT_SECONDS = 60.0


def _stop_requested(stop_event) -> bool:
    return bool(stop_event is not None and stop_event.is_set())


def _wait_control_period(stop_event, seconds: float) -> None:
    if stop_event is None:
        time.sleep(seconds)
    else:
        stop_event.wait(seconds)


def _align_over_home_h(
    navi,
    stop_event,
    offsets: Iterator[Tuple[Optional[float], Optional[float]]],
    pixel_threshold: float,
    approach_speed: float,
    frequency: float,
    timeout: float,
) -> bool:
    """按H标记像素偏移修正水平位置，视觉丢失时保持当前位置。"""
    period = 1.0 / max(float(frequency), 5.0)
    deadline = time.monotonic() + float(timeout)
    logger.info(
        "[HOME-LAND] H-marker alignment: threshold={}px, speed={}cm/s, "
        "frequency={}Hz, timeout={}s",
        pixel_threshold,
        approach_speed,
        frequency,
        timeout,
    )

    while time.monotonic() < deadline:
        if _stop_requested(stop_event):
            navi.stop_move()
            logger.warning("[HOME-LAND] Alignment stopped externally")
            return False

        try:
            x_px, y_px = next(offsets)
        except StopIteration:
            navi.stop_move()
            logger.error("[HOME-LAND] H-marker tracker stopped unexpectedly")
            return False

        if x_px is None or y_px is None:
            navi.stop_move()
            _wait_control_period(stop_event, period)
            continue

        x_px = float(x_px)
        y_px = float(y_px)
        if not math.isfinite(x_px) or not math.isfinite(y_px):
            navi.stop_move()
            _wait_control_period(stop_event, period)
            continue

        distance_px = math.hypot(x_px, y_px)
        if distance_px <= float(pixel_threshold):
            navi.stop_move()
            logger.info(
                "[HOME-LAND] H marker centered: distance={:.1f}px",
                distance_px,
            )
            return True

        direction_deg = math.degrees(math.atan2(y_px, x_px))
        navi.move_by_direction(
            speed=float(approach_speed),
            direction_deg=direction_deg,
        )
        _wait_control_period(stop_event, period)

    navi.stop_move()
    logger.warning("[HOME-LAND] H-marker alignment timed out")
    return False


def _complete_landing_after_approach(
    fc,
    navi,
    touchdown_alt_thres: float = 8.0,
    touchdown_timeout: float = 12.0,
    lock_timeout: float = 4.0,
) -> bool:
    """复制当前pointing_landing阶段2，仅供隔离测试验证。"""
    navi.navigation_flag = False
    navi.keep_height_flag = False
    fc.set_flight_mode(fc.PROGRAM_MODE)
    time.sleep(0.1)
    fc.stablize()
    fc.land()

    started_at = time.perf_counter()
    landed = False
    altitude_threshold = float(max(3.0, touchdown_alt_thres))
    while (
        time.perf_counter() - started_at
        < max(1.0, float(touchdown_timeout))
    ):
        time.sleep(0.1)
        try:
            altitude = float(fc.state.alt_add.value)
        except Exception:
            altitude = 999.0
        state_fresh = bool(fc.state.is_fresh(0.5))
        if (
            state_fresh and altitude <= altitude_threshold
        ) or not fc.state.unlock.value:
            landed = True
            break

    if not landed:
        logger.error(
            "[HOME-LAND] Landing timeout; keep landing command active, "
            "refuse airborne force-lock"
        )
        fc.land()
        return False

    try:
        locked = fc.wait_for_lock(timeout_s=lock_timeout)
    except TypeError:
        locked = fc.wait_for_lock(lock_timeout)
    if not locked:
        state_fresh = bool(fc.state.is_fresh(0.5))
        altitude = (
            float(fc.state.alt_add.value) if state_fresh else 999.0
        )
        if state_fresh and altitude <= altitude_threshold:
            fc.lock()
        else:
            logger.error(
                "[HOME-LAND] Lock not confirmed; refuse lock without "
                "fresh touchdown altitude"
            )
            return False
    return True


def visual_home_h_landing(
    fc,
    navi,
    stop_event,
    camera_index: int,
    approach_height: float = APPROACH_HEIGHT_CM,
    height_tolerance: float = HEIGHT_TOLERANCE_CM,
    height_timeout: float = HEIGHT_TIMEOUT_SECONDS,
    pixel_threshold: float = PIXEL_THRESHOLD,
    approach_speed: float = APPROACH_SPEED_CM_S,
    frequency: float = CONTROL_FREQUENCY_HZ,
    alignment_timeout: float = ALIGNMENT_TIMEOUT_SECONDS,
    tracker_factory: Callable[
        [int],
        Iterator[Tuple[Optional[float], Optional[float]]],
    ] = track_home_h_marker,
    landing_callback: Optional[Callable[[], bool]] = None,
) -> bool:
    """
    从返航点垂直下降至50 cm，对准H标记后执行一键降落。

    调用前必须已经导航回起飞点，并保持新鲜的飞控遥测和导航位姿。
    256×256画面默认以30像素为视觉对准阈值。
    """
    values = (
        approach_height,
        height_tolerance,
        height_timeout,
        pixel_threshold,
        approach_speed,
        frequency,
        alignment_timeout,
    )
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("visual landing parameters must be finite")
    if (
        float(approach_height) <= 0
        or float(height_tolerance) <= 0
        or float(height_timeout) <= 0
        or float(pixel_threshold) < 0
        or float(approach_speed) <= 0
        or float(frequency) <= 0
        or float(alignment_timeout) <= 0
    ):
        raise ValueError("visual landing parameters are outside valid ranges")
    if _stop_requested(stop_event):
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

    navi.set_height(float(approach_height))
    navi.keep_height_flag = True
    if not navi.wait_for_height(
        height_thres=float(height_tolerance),
        timeout=float(height_timeout),
    ):
        logger.error(
            "[HOME-LAND] Failed to reach visual approach height {}cm",
            approach_height,
        )
        return False
    if _stop_requested(stop_event):
        logger.warning("[HOME-LAND] Visual landing stopped at approach height")
        return False
    if not fc.state.is_fresh(0.5) or not navi.pose_is_fresh():
        logger.error("[HOME-LAND] State became stale before visual alignment")
        return False

    logger.info(
        "[HOME-LAND] Reached {}cm; starting H-marker tracker on camera {}",
        approach_height,
        camera_index,
    )
    offsets = tracker_factory(camera_index)
    try:
        centered = _align_over_home_h(
            navi=navi,
            stop_event=stop_event,
            offsets=offsets,
            pixel_threshold=float(pixel_threshold),
            approach_speed=float(approach_speed),
            frequency=float(frequency),
            timeout=float(alignment_timeout),
        )
    finally:
        try:
            offsets.close()  # type: ignore[attr-defined]
        except AttributeError:
            pass
        except Exception:
            logger.exception("[HOME-LAND] Failed to close H-marker tracker")

    if not centered:
        return False
    if not fc.state.is_fresh(0.5) or not fc.state.unlock.value:
        logger.error("[HOME-LAND] Flight state invalid after visual alignment")
        return False

    if landing_callback is not None:
        return bool(landing_callback())
    return _complete_landing_after_approach(fc, navi)
