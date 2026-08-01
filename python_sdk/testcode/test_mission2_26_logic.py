"""任务二轨迹、门限、平台复飞和锁桨灯光的无硬件测试。"""

import math
import sys
from pathlib import Path


SDK_DIR = Path(__file__).resolve().parents[1]
if str(SDK_DIR) not in sys.path:
    sys.path.insert(0, str(SDK_DIR))

from mission2_26_logic import (  # noqa: E402
    ARC_CENTER,
    ARC_END,
    ARC_RADIUS,
    ARC_START,
    ClockwiseArcVelocityPredictor,
    ENTRY_POINT,
    LowAltitudeTargetOffset,
    PURSUIT_SLOWDOWN_POINT,
    PursuitSpeedSchedule,
    ROUTE_END,
    RoutePassGate,
    TAKEOFF_POINT,
    build_pursuit_trajectory,
    build_pursuit_trajectory_to_b,
    land_on_target_and_confirm_lock,
    locked_red_led_dwell,
    retakeoff_from_moving_platform,
)
from mission2_26_safety import EscortXBoundaryVelocityGuard  # noqa: E402


def test_pursuit_trajectory_geometry() -> None:
    trajectory = build_pursuit_trajectory(
        altitude=150.0,
        arc_step_degrees=10,
    )
    assert trajectory[0] == (TAKEOFF_POINT[0], TAKEOFF_POINT[1], 150.0)
    assert trajectory[1] == (ENTRY_POINT[0], ENTRY_POINT[1], 150.0)
    assert trajectory[2] == (
        PURSUIT_SLOWDOWN_POINT[0],
        PURSUIT_SLOWDOWN_POINT[1],
        150.0,
    )
    assert trajectory[3] == (ARC_START[0], ARC_START[1], 150.0)
    assert trajectory[-2] == (ARC_END[0], ARC_END[1], 150.0)
    assert trajectory[-1] == (ROUTE_END[0], ROUTE_END[1], 150.0)

    arc_points = trajectory[3:-1]
    assert len(arc_points) == 19
    for x, y, height in arc_points:
        assert height == 150.0
        assert x >= ARC_CENTER[0] - 1e-9
        assert math.isclose(
            math.hypot(x - ARC_CENTER[0], y - ARC_CENTER[1]),
            ARC_RADIUS,
            abs_tol=1e-9,
        )
    assert math.isclose(
        max(point[0] for point in arc_points),
        ARC_CENTER[0] + ARC_RADIUS,
        abs_tol=1e-9,
    )
    assert arc_points[1][0] > arc_points[0][0]
    assert arc_points[1][1] < arc_points[0][1]
    assert trajectory[-1][0] < trajectory[-2][0]


def test_task2_pursuit_trajectory_stops_at_b() -> None:
    trajectory = build_pursuit_trajectory_to_b(
        altitude=150.0,
        arc_step_degrees=10,
    )
    assert trajectory[-1] == (ARC_START[0], ARC_START[1], 150.0)
    assert (ARC_END[0], ARC_END[1], 150.0) not in trajectory


def test_clockwise_arc_velocity_prediction() -> None:
    predictor = ClockwiseArcVelocityPredictor()
    assert predictor.predict(
        (10.0, 0.0),
        0.1,
        PURSUIT_SLOWDOWN_POINT[0],
        PURSUIT_SLOWDOWN_POINT[1],
    ) == (10.0, 0.0)

    velocity = (10.0, 0.0)
    sample_dt = math.pi * ARC_RADIUS / 10.0 / 180.0
    for index in range(180):
        position_angle = math.radians(89.5 - index)
        x = ARC_CENTER[0] + ARC_RADIUS * math.cos(position_angle)
        y = ARC_CENTER[1] + ARC_RADIUS * math.sin(position_angle)
        velocity = predictor.predict(velocity, sample_dt, x, y)

    assert math.isclose(velocity[0], -10.0, abs_tol=1e-9)
    assert math.isclose(velocity[1], 0.0, abs_tol=1e-9)
    assert math.isclose(math.hypot(*velocity), 10.0, abs_tol=1e-9)


def test_low_altitude_target_offset() -> None:
    offset = LowAltitudeTargetOffset(
        start_height=50.0,
        final_height=25.0,
        final_x_px=-30.0,
    )
    assert offset.offset(60.0) == (0.0, 0.0)
    assert offset.offset(50.0) == (0.0, 0.0)
    assert offset.offset(37.5) == (-15.0, 0.0)
    assert offset.offset(25.0) == (-30.0, 0.0)
    assert offset.offset(10.0) == (-30.0, 0.0)


def test_escort_x_boundary_velocity_guard() -> None:
    guard = EscortXBoundaryVelocityGuard(max_x=357.5)
    assert guard.apply(357.5, 8, -3) == (8, -3)
    assert guard.apply(357.5001, 8, -3) == (0, -3)
    assert guard.apply(400.0, -8, 3) == (0, 3)

    try:
        guard.apply(math.nan, 8, 3)
    except ValueError as exc:
        assert "current_x" in str(exc)
    else:
        raise AssertionError("Invalid navigation position was accepted")


def test_pursuit_speed_schedule() -> None:
    schedule = PursuitSpeedSchedule()
    assert schedule.current_speed == 40.0
    assert schedule.update(*ENTRY_POINT, *TAKEOFF_POINT) is None
    assert (
        schedule.update(
            *PURSUIT_SLOWDOWN_POINT,
            ENTRY_POINT[0],
            ENTRY_POINT[1],
        )
        == 25.0
    )
    assert schedule.current_speed == 25.0
    assert (
        schedule.update(
            *ARC_START,
            PURSUIT_SLOWDOWN_POINT[0] - 0.1,
            PURSUIT_SLOWDOWN_POINT[1],
        )
        is None
    )
    assert schedule.current_speed == 25.0
    assert (
        schedule.update(
            *ARC_START,
            PURSUIT_SLOWDOWN_POINT[0],
            PURSUIT_SLOWDOWN_POINT[1],
        )
        == 15.0
    )
    assert schedule.current_speed == 15.0
    assert schedule.update(*ARC_END, *ARC_START) is None


def test_route_pass_gate() -> None:
    gate = RoutePassGate(radius=40.0)
    assert not gate.update(ARC_END[0], ARC_END[1])
    assert not gate.update(ARC_END[0] - 40.1, ARC_END[1])
    assert gate.update(ARC_END[0] - 39.9, ARC_END[1])
    assert gate.update(0.0, 0.0)

    invalid_gate = RoutePassGate()
    assert not invalid_gate.update(math.nan, ARC_END[1])


class _Value:
    def __init__(self, value):
        self.value = value


class _State:
    def __init__(self):
        self.mode = _Value(0)
        self.unlock = _Value(False)
        self.alt_add = _Value(0.0)

    @staticmethod
    def is_fresh(_max_age):
        return True


class _FC:
    PROGRAM_MODE = 3
    HOLD_POS_MODE = 2

    def __init__(self):
        self.state = _State()
        self.takeoff_calls = []
        self.land_calls = 0
        self.stabilize_calls = 0
        self.led_calls = []
        self.on_hold_pos = None

    def set_flight_mode(self, mode):
        self.state.mode.value = mode
        if mode == self.HOLD_POS_MODE and self.on_hold_pos is not None:
            self.on_hold_pos()

    def unlock(self):
        self.state.unlock.value = True

    def take_off(self, height):
        self.takeoff_calls.append(height)
        self.state.alt_add.value = float(height)

    @staticmethod
    def wait_for_takeoff_done(timeout_s):
        return timeout_s > 0

    @staticmethod
    def wait_for_hovering(timeout_s):
        return timeout_s > 0

    def set_indicator_led(self, red, green, blue):
        self.led_calls.append((red, green, blue))

    def stablize(self):
        self.stabilize_calls += 1

    def land(self):
        self.land_calls += 1

    def wait_for_lock(self, timeout_s):
        self.state.unlock.value = False
        return timeout_s > 0


class _Navigation:
    def __init__(self):
        self.navigation_flag = True
        self.keep_height_flag = True
        self.current_x = 123.0
        self.current_y = -234.0
        self.current_height = 30.0
        self.direct_waypoints = []
        self.height_targets = []
        self.stop_here_calls = 0

    @staticmethod
    def pose_is_fresh():
        return True

    def direct_set_waypoint(self, point):
        self.direct_waypoints.append(tuple(point))

    def navigation_to_waypoint(self, *_args, **_kwargs):
        raise AssertionError("Platform retakeoff must not navigate to an old point")

    def navigation_stop_here(self):
        self.stop_here_calls += 1

    def set_height(self, height):
        self.height_targets.append(float(height))
        self.current_height = float(height)

    @staticmethod
    def switch_pid(name):
        assert name == "hover"

    @staticmethod
    def wait_for_height(height_thres, timeout):
        return height_thres > 0 and timeout > 0


class _ImmediateStopEvent:
    @staticmethod
    def is_set():
        return False

    @staticmethod
    def wait(_seconds):
        return False


def test_platform_retakeoff_uses_live_hold_point() -> None:
    fc = _FC()
    navi = _Navigation()
    navi.current_x = 10.0
    navi.current_y = -20.0

    def update_position_after_hold_pos():
        navi.current_x = 123.0
        navi.current_y = -234.0

    fc.on_hold_pos = update_position_after_hold_pos
    hold_point = retakeoff_from_moving_platform(
        fc,
        navi,
        _ImmediateStopEvent(),
        target_height=150.0,
        motor_warmup_seconds=0.0,
    )
    assert fc.takeoff_calls == [30]
    assert hold_point == (123.0, -234.0)
    assert navi.direct_waypoints == [(123.0, -234.0)]
    assert navi.height_targets[-1] == 150.0
    assert navi.navigation_flag
    assert navi.keep_height_flag


def test_target_landing_confirms_lock() -> None:
    fc = _FC()
    fc.state.unlock.value = True
    navi = _Navigation()
    land_on_target_and_confirm_lock(
        fc,
        navi,
        lock_timeout=20.0,
        mode_settle_seconds=0.0,
    )
    assert navi.stop_here_calls == 1
    assert not navi.navigation_flag
    assert not navi.keep_height_flag
    assert fc.state.mode.value == fc.PROGRAM_MODE
    assert fc.stabilize_calls == 1
    assert fc.land_calls == 1
    assert not fc.state.unlock.value


class _FakeClock:
    now = 0.0

    @classmethod
    def monotonic(cls):
        return cls.now


class _ClockStopEvent:
    @staticmethod
    def is_set():
        return False

    @staticmethod
    def wait(seconds):
        _FakeClock.now += float(seconds)
        return False


def test_locked_red_led_dwell_and_cleanup() -> None:
    _FakeClock.now = 0.0
    fc = _FC()
    locked_red_led_dwell(
        fc,
        _ClockStopEvent(),
        dwell_seconds=5.0,
        clock=_FakeClock.monotonic,
    )
    assert _FakeClock.now >= 5.0
    assert fc.led_calls == [(255, 0, 0), (0, 0, 0)]

    stale_fc = _FC()
    stale_fc.state.is_fresh = lambda _max_age: False
    try:
        locked_red_led_dwell(
            stale_fc,
            _ClockStopEvent(),
            dwell_seconds=5.0,
            clock=_FakeClock.monotonic,
        )
    except RuntimeError as exc:
        assert "stale" in str(exc)
    else:
        raise AssertionError("Stale telemetry was accepted during locked dwell")
    assert stale_fc.led_calls[-1] == (0, 0, 0)


def main() -> None:
    test_pursuit_trajectory_geometry()
    test_task2_pursuit_trajectory_stops_at_b()
    test_clockwise_arc_velocity_prediction()
    test_low_altitude_target_offset()
    test_escort_x_boundary_velocity_guard()
    test_pursuit_speed_schedule()
    test_route_pass_gate()
    test_platform_retakeoff_uses_live_hold_point()
    test_target_landing_confirms_lock()
    test_locked_red_led_dwell_and_cleanup()
    print("mission2 logic tests passed")


if __name__ == "__main__":
    main()
