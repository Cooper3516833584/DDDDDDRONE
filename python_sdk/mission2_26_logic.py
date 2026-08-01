"""任务二可在无飞控、雷达和相机环境中验证的纯逻辑组件。"""

import math
import time
from dataclasses import dataclass
from typing import Callable, List, Optional, Tuple


TAKEOFF_POINT = (0.0, 0.0)
ENTRY_POINT = (87.5, -37.5)
PURSUIT_SLOWDOWN_POINT = (207.5, -37.5)
ARC_CENTER = (237.5, -112.5)
ARC_RADIUS = 75.0
ARC_START = (237.5, -37.5)
ARC_END = (237.5, -187.5)
ROUTE_END = (87.5, -187.5)
ROUTE_GATE_RADIUS = 7.5


@dataclass(frozen=True)
class ClockwiseArcVelocityPredictor:
    """在已知顺时针圆弧内旋转上一帧目标速度估计。"""

    center: Tuple[float, float] = ARC_CENTER
    radius: float = ARC_RADIUS
    start: Tuple[float, float] = ARC_START
    end: Tuple[float, float] = ARC_END
    position_tolerance: float = 20.0

    def __post_init__(self) -> None:
        values = (
            self.center[0],
            self.center[1],
            self.radius,
            self.start[0],
            self.start[1],
            self.end[0],
            self.end[1],
            self.position_tolerance,
        )
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("Arc predictor parameters must be finite")
        if (
            float(self.radius) <= 0
            or float(self.position_tolerance) < 0
        ):
            raise ValueError(
                "Arc radius must be positive and tolerance non-negative"
            )

    def is_on_arc(self, x: float, y: float) -> bool:
        x = float(x)
        y = float(y)
        if not math.isfinite(x) or not math.isfinite(y):
            return False
        lower_y = min(float(self.start[1]), float(self.end[1]))
        upper_y = max(float(self.start[1]), float(self.end[1]))
        if not lower_y < y < upper_y:
            return False
        radial_distance = math.hypot(
            x - float(self.center[0]),
            y - float(self.center[1]),
        )
        return bool(
            x >= float(self.center[0]) - float(self.position_tolerance)
            and abs(radial_distance - float(self.radius))
            <= float(self.position_tolerance)
        )

    def predict(
        self,
        velocity: Tuple[float, float],
        sample_dt: float,
        x: float,
        y: float,
    ) -> Tuple[float, float]:
        vx = float(velocity[0])
        vy = float(velocity[1])
        dt = float(sample_dt)
        if not all(math.isfinite(value) for value in (vx, vy, dt)):
            raise ValueError("Arc predictor inputs must be finite")
        if dt < 0:
            raise ValueError("sample_dt must be non-negative")
        if dt == 0 or not self.is_on_arc(x, y):
            return vx, vy

        speed = math.hypot(vx, vy)
        if speed == 0:
            return vx, vy
        angle = speed * dt / float(self.radius)
        cosine = math.cos(angle)
        sine = math.sin(angle)
        return (
            cosine * vx + sine * vy,
            -sine * vx + cosine * vy,
        )


@dataclass(frozen=True)
class LowAltitudeTargetOffset:
    """从起始高度到最终高度线性加入视觉目标点偏置。"""

    start_height: float = 50.0
    final_height: float = 25.0
    final_x_px: float = -30.0
    final_y_px: float = 0.0

    def __post_init__(self) -> None:
        values = (
            self.start_height,
            self.final_height,
            self.final_x_px,
            self.final_y_px,
        )
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("Low-altitude target-offset parameters must be finite")
        if float(self.start_height) <= float(self.final_height):
            raise ValueError("start_height must be greater than final_height")

    def offset(self, height: float) -> Tuple[float, float]:
        height = float(height)
        if not math.isfinite(height):
            raise ValueError("height must be finite")
        start_height = float(self.start_height)
        final_height = float(self.final_height)
        progress = (
            (start_height - height)
            / (start_height - final_height)
        )
        progress = min(max(progress, 0.0), 1.0)
        return (
            float(self.final_x_px) * progress,
            float(self.final_y_px) * progress,
        )


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
        (
            PURSUIT_SLOWDOWN_POINT[0],
            PURSUIT_SLOWDOWN_POINT[1],
            altitude,
        ),
    ]
    for angle_degrees in range(90, -91, -arc_step_degrees):
        angle = math.radians(float(angle_degrees))
        x = ARC_CENTER[0] + ARC_RADIUS * math.cos(angle)
        y = ARC_CENTER[1] + ARC_RADIUS * math.sin(angle)
        points.append((float(x), float(y), altitude))
    points[3] = (ARC_START[0], ARC_START[1], altitude)
    points[-1] = (ARC_END[0], ARC_END[1], altitude)
    points.append((ROUTE_END[0], ROUTE_END[1], altitude))
    return points


def straight_return_axis_limits(
    current_x: float,
    current_y: float,
    target_x: float,
    target_y: float,
    speed: float,
) -> Tuple[float, float]:
    """Split a straight-line speed limit into world X/Y axis limits."""
    values = (current_x, current_y, target_x, target_y, speed)
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("Straight-return parameters must be finite")
    speed = float(speed)
    if speed <= 0:
        raise ValueError("Straight-return speed must be positive")

    delta_x = float(target_x) - float(current_x)
    delta_y = float(target_y) - float(current_y)
    distance = math.hypot(delta_x, delta_y)
    if distance <= 1e-9:
        return 0.0, 0.0
    return (
        speed * abs(delta_x) / distance,
        speed * abs(delta_y) / distance,
    )


@dataclass
class PursuitSpeedSchedule:
    """根据轨迹目标和实时位置锁存追及阶段，并给出需要切换的新速度。"""

    initial_speed: float = 40.0
    approach_speed: float = 25.0
    after_slowdown_speed: float = 15.0
    point_tolerance: float = 1e-3
    stage: int = 0

    @property
    def current_speed(self) -> float:
        if self.stage <= 0:
            return float(self.initial_speed)
        if self.stage == 1:
            return float(self.approach_speed)
        return float(self.after_slowdown_speed)

    def update(
        self,
        target_x: float,
        target_y: float,
        current_x: float,
        current_y: float,
    ) -> Optional[float]:
        target_x = float(target_x)
        target_y = float(target_y)
        current_x = float(current_x)
        current_y = float(current_y)
        if not all(
            math.isfinite(value)
            for value in (target_x, target_y, current_x, current_y)
        ):
            return None
        at_slowdown_point = bool(
            math.hypot(
                target_x - PURSUIT_SLOWDOWN_POINT[0],
                target_y - PURSUIT_SLOWDOWN_POINT[1],
            )
            <= float(self.point_tolerance)
        )
        if self.stage == 0 and at_slowdown_point:
            self.stage = 1
            return self.current_speed
        if (
            self.stage == 1
            and current_x >= PURSUIT_SLOWDOWN_POINT[0]
        ):
            self.stage = 2
            return self.current_speed
        return None


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
    "ClockwiseArcVelocityPredictor",
    "ENTRY_POINT",
    "LowAltitudeTargetOffset",
    "PURSUIT_SLOWDOWN_POINT",
    "PursuitSpeedSchedule",
    "ROUTE_END",
    "ROUTE_GATE_RADIUS",
    "RoutePassGate",
    "TAKEOFF_POINT",
    "build_pursuit_trajectory",
    "land_on_target_and_confirm_lock",
    "locked_red_led_dwell",
    "retakeoff_from_moving_platform",
    "straight_return_axis_limits",
]
