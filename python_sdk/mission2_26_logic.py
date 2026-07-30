"""任务二可在无飞控、雷达和相机环境中验证的纯逻辑组件。"""

import math
import time
from dataclasses import dataclass
from typing import Callable, List, Optional, Tuple


TAKEOFF_POINT = (0.0, 0.0)
ENTRY_POINT = (87.5, -37.5)
ARC_CENTER = (237.5, -112.5)
ARC_RADIUS = 75.0
ARC_START = (237.5, -37.5)
ARC_END = (237.5, -187.5)
ROUTE_END = (87.5, -187.5)
ROUTE_GATE_RADIUS = 7.5


def build_pursuit_trajectory(
    altitude: float = 150.0,
    arc_step_degrees: int = 10,
) -> List[Tuple[float, float, float]]:
    """建立直线、右侧顺时针半圆和末段直线组成的追及轨迹。"""
    altitude = float(altitude)
    arc_step_degrees = int(arc_step_degrees)
    if not math.isfinite(altitude):
        raise ValueError("altitude must be finite")
    if arc_step_degrees <= 0 or 180 % arc_step_degrees:
        raise ValueError("arc_step_degrees must be a positive divisor of 180")

    points = [
        (TAKEOFF_POINT[0], TAKEOFF_POINT[1], altitude),
        (ENTRY_POINT[0], ENTRY_POINT[1], altitude),
    ]
    for angle_degrees in range(90, -91, -arc_step_degrees):
        angle = math.radians(float(angle_degrees))
        x = ARC_CENTER[0] + ARC_RADIUS * math.cos(angle)
        y = ARC_CENTER[1] + ARC_RADIUS * math.sin(angle)
        points.append((float(x), float(y), altitude))
    points[2] = (ARC_START[0], ARC_START[1], altitude)
    points[-1] = (ARC_END[0], ARC_END[1], altitude)
    points.append((ROUTE_END[0], ROUTE_END[1], altitude))
    return points


@dataclass
class RoutePassGate:
    """锁存无人机是否从要求方向经过圆弧终点门限。"""

    radius: float = ROUTE_GATE_RADIUS
    passed: bool = False

    def update(self, x: float, y: float) -> bool:
        if self.passed:
            return True
        x = float(x)
        y = float(y)
        if not math.isfinite(x) or not math.isfinite(y):
            return False
        if (
            x < ARC_END[0]
            and math.hypot(x - ARC_END[0], y - ARC_END[1])
            <= float(self.radius)
        ):
            self.passed = True
        return self.passed


def _wait_for_condition(
    stop_event,
    predicate: Callable[[], bool],
    timeout: float,
    error_message: str,
) -> None:
    deadline = time.monotonic() + float(timeout)
    while time.monotonic() < deadline:
        if stop_event.is_set():
            raise RuntimeError("External stop requested during platform retakeoff")
        if predicate():
            return
        stop_event.wait(0.05)
    raise RuntimeError(error_message)


def _wait_duration(stop_event, seconds: float) -> None:
    deadline = time.monotonic() + float(seconds)
    while time.monotonic() < deadline:
        if stop_event.is_set():
            raise RuntimeError("External stop requested during platform retakeoff")
        stop_event.wait(min(0.1, max(0.0, deadline - time.monotonic())))


def retakeoff_from_moving_platform(
    fc,
    navi,
    stop_event,
    target_height: float,
    first_lift_height: int = 30,
    motor_warmup_seconds: float = 2.0,
    mode_timeout: float = 1.5,
    unlock_timeout: float = 3.0,
    takeoff_timeout: float = 8.0,
    hover_timeout: float = 5.0,
    height_timeout: float = 15.0,
    height_tolerance: float = 5.0,
) -> Tuple[float, float]:
    """从运动平台复飞，并以切换定点模式后的实时位置建立悬停点。"""
    navi.navigation_flag = False
    navi.keep_height_flag = False

    fc.set_flight_mode(fc.PROGRAM_MODE)
    _wait_for_condition(
        stop_event,
        lambda: bool(
            fc.state.is_fresh(0.5)
            and fc.state.mode.value == fc.PROGRAM_MODE
        ),
        mode_timeout,
        "Fresh PROGRAM mode was not confirmed before platform retakeoff",
    )
    fc.unlock()
    _wait_for_condition(
        stop_event,
        lambda: bool(
            fc.state.is_fresh(0.5)
            and fc.state.mode.value == fc.PROGRAM_MODE
            and fc.state.unlock.value
        ),
        unlock_timeout,
        "Fresh PROGRAM/unlock feedback was not confirmed",
    )
    _wait_duration(stop_event, motor_warmup_seconds)
    if not (
        fc.state.is_fresh(0.5)
        and fc.state.mode.value == fc.PROGRAM_MODE
        and fc.state.unlock.value
    ):
        raise RuntimeError("Platform retakeoff state became invalid before takeoff")

    fc.take_off(int(first_lift_height))
    if not fc.wait_for_takeoff_done(timeout_s=float(takeoff_timeout)):
        raise RuntimeError("One-key platform retakeoff was not confirmed")
    if not fc.wait_for_hovering(float(hover_timeout)):
        raise RuntimeError("Hovering was not confirmed after platform retakeoff")

    fc.set_flight_mode(fc.HOLD_POS_MODE)
    _wait_for_condition(
        stop_event,
        lambda: bool(
            fc.state.is_fresh(0.5)
            and fc.state.mode.value == fc.HOLD_POS_MODE
            and fc.state.unlock.value
        ),
        mode_timeout,
        "Fresh HOLD_POS feedback was not confirmed after platform retakeoff",
    )
    if not navi.pose_is_fresh():
        raise RuntimeError("Navigation pose is stale after platform retakeoff")

    hold_x = float(navi.current_x)
    hold_y = float(navi.current_y)
    if not math.isfinite(hold_x) or not math.isfinite(hold_y):
        raise RuntimeError("Platform retakeoff hold position is invalid")
    navi.direct_set_waypoint((hold_x, hold_y))
    navi.set_height(max(float(navi.current_height), float(first_lift_height)))
    navi.switch_pid("hover")
    navi.navigation_flag = True
    navi.keep_height_flag = True

    navi.set_height(float(target_height))
    if not navi.wait_for_height(
        height_thres=float(height_tolerance),
        timeout=float(height_timeout),
    ):
        raise RuntimeError("Cruise height was not confirmed after platform retakeoff")
    return hold_x, hold_y


def land_on_target_and_confirm_lock(
    fc,
    navi,
    lock_timeout: float = 20.0,
    mode_settle_seconds: float = 0.1,
) -> None:
    """退出导航后调用一键降落，并要求新鲜遥测确认锁桨。"""
    navi.navigation_stop_here()
    navi.navigation_flag = False
    navi.keep_height_flag = False
    fc.set_flight_mode(fc.PROGRAM_MODE)
    if mode_settle_seconds > 0:
        time.sleep(float(mode_settle_seconds))
    fc.stablize()
    fc.land()
    if not fc.wait_for_lock(timeout_s=float(lock_timeout)):
        fc.land()
        raise RuntimeError("One-key target landing did not confirm motor lock")
    if not fc.state.is_fresh(0.5):
        raise RuntimeError(
            "Flight-controller telemetry became stale after target landing"
        )


def locked_red_led_dwell(
    fc,
    stop_event,
    dwell_seconds: float = 5.0,
    clock: Optional[Callable[[], float]] = None,
) -> None:
    """锁桨后亮红灯停留，任何退出路径均熄灯。"""
    if dwell_seconds < 0:
        raise ValueError("dwell_seconds must be non-negative")
    monotonic = time.monotonic if clock is None else clock
    fc.set_indicator_led(255, 0, 0)
    try:
        deadline = monotonic() + float(dwell_seconds)
        while monotonic() < deadline:
            if stop_event.is_set():
                raise RuntimeError("External stop requested during locked dwell")
            if not fc.state.is_fresh(0.5):
                raise RuntimeError("Flight-controller telemetry became stale")
            if fc.state.unlock.value:
                raise RuntimeError("Aircraft unexpectedly unlocked during dwell")
            stop_event.wait(min(0.1, max(0.0, deadline - monotonic())))
    finally:
        fc.set_indicator_led(0, 0, 0)


__all__ = [
    "ARC_CENTER",
    "ARC_END",
    "ARC_RADIUS",
    "ARC_START",
    "ENTRY_POINT",
    "ROUTE_END",
    "ROUTE_GATE_RADIUS",
    "RoutePassGate",
    "TAKEOFF_POINT",
    "build_pursuit_trajectory",
    "land_on_target_and_confirm_lock",
    "locked_red_led_dwell",
    "retakeoff_from_moving_platform",
]
