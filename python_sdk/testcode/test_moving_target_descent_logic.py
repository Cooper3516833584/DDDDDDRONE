"""移动目标速度估计与视觉控制时序的无硬件逻辑测试。"""

import math
import sys
import types
from pathlib import Path

import numpy as np


SDK_DIR = Path(__file__).resolve().parents[1]
if str(SDK_DIR) not in sys.path:
    sys.path.insert(0, str(SDK_DIR))


class _FakeLogger:
    def __getattr__(self, _name):
        return lambda *args, **kwargs: None


sys.modules.setdefault("loguru", types.SimpleNamespace(logger=_FakeLogger()))

from moving_target_descent import (  # noqa: E402
    MovingTargetDescentConfig,
    TargetVelocityEstimator,
)
import visual_target_descent  # noqa: E402


def _new_estimator() -> TargetVelocityEstimator:
    return TargetVelocityEstimator(
        integral_gain=0.048,
        deadband_px=3.0,
        speed_limit=10.0,
        max_sample_dt=0.2,
    )


def test_estimator() -> None:
    assert MovingTargetDescentConfig().horizontal_command_limit == 15.0
    estimator = _new_estimator()
    estimator.reset((7.2, 0.0))
    assert estimator.velocity == (7.2, 0.0)
    assert estimator.update(0.0, 0.0, 0.1) == (7.2, 0.0)
    assert estimator.update(3.0, -3.0, 0.1) == (7.2, 0.0)

    vx_before, vy_before = estimator.velocity
    vx_after, vy_after = estimator.update(10.0, 20.0, 0.1)
    assert vx_after > vx_before
    assert vy_after > vy_before

    for _ in range(300):
        estimator.update(-30.0, 0.0, 0.2)
    assert estimator.velocity[0] < 0.0

    estimator.reset((0.0, 0.0))
    estimator.update(1000.0, 1000.0, 10.0)
    assert math.isclose(
        math.hypot(*estimator.velocity),
        10.0,
        rel_tol=1e-9,
    )

    slow = _new_estimator()
    capped = _new_estimator()
    slow.reset((0.0, 0.0))
    capped.reset((0.0, 0.0))
    assert np.allclose(
        slow.update(20.0, 0.0, 10.0),
        capped.update(20.0, 0.0, 0.2),
    )

    velocity = estimator.velocity
    assert isinstance(velocity, tuple)
    changed_copy = list(velocity)
    changed_copy[0] = 999.0
    assert estimator.velocity[0] != 999.0

    for invalid in ((1.0,), (1.0, 2.0, 3.0), (math.nan, 0.0)):
        try:
            estimator.reset(invalid)  # type: ignore[arg-type]
        except ValueError:
            pass
        else:
            raise AssertionError("Invalid initial velocity was accepted")

    for invalid in (math.nan, math.inf):
        try:
            estimator.update(invalid, 0.0, 0.1)
        except ValueError:
            pass
        else:
            raise AssertionError("Non-finite estimator input was accepted")

    invalid_configs = (
        {"estimator_integral_gain": -0.1},
        {"estimator_speed_limit": 0.0},
        {"command_filter_alpha": 0.0},
        {"command_filter_alpha": 1.1},
        {"vision_loss_timeout": math.inf},
    )
    for kwargs in invalid_configs:
        try:
            MovingTargetDescentConfig(**kwargs)
        except ValueError:
            pass
        else:
            raise AssertionError("Invalid moving-target config was accepted")


def test_estimator_velocity_predictor() -> None:
    estimator = _new_estimator()
    estimator.reset((10.0, 0.0))
    predictor_calls = []

    def predictor(velocity, sample_dt):
        predictor_calls.append((velocity, sample_dt))
        return 0.0, -10.0

    assert np.allclose(
        estimator.update(
            0.0,
            0.0,
            1.0,
            velocity_predictor=predictor,
        ),
        (0.0, -10.0),
    )
    assert predictor_calls == [((10.0, 0.0), 0.2)]

    try:
        estimator.update(
            0.0,
            0.0,
            0.1,
            velocity_predictor=lambda _velocity, _dt: (math.nan, 0.0),
        )
    except ValueError as exc:
        assert "two finite values" in str(exc)
    else:
        raise AssertionError("Invalid velocity prediction was accepted")


class _Value:
    def __init__(self, value):
        self.value = value


class _State:
    def __init__(self):
        self.unlock = _Value(True)

    @staticmethod
    def is_fresh(_max_age):
        return True


class _FC:
    def __init__(self):
        self.state = _State()


class _FakeTime:
    now = 0.0

    @classmethod
    def monotonic(cls):
        return cls.now


class _StopEvent:
    @staticmethod
    def is_set():
        return False

    @staticmethod
    def wait(seconds):
        _FakeTime.now += float(seconds)
        return False


class _Navigation:
    def __init__(self):
        self.current_height = 150.0
        self.commands = []
        self.height_targets = []
        self.override_stopped = False

    @staticmethod
    def pose_is_fresh():
        return True

    @staticmethod
    def _start_velocity_override(keep_height, require_pose):
        return bool(keep_height and require_pose)

    def _update_velocity_override(
        self,
        vel_x,
        vel_y,
        vel_z=None,
        yaw=0,
        frame="body",
    ):
        self.commands.append((vel_x, vel_y, vel_z, yaw, frame))

    def _stop_velocity_override(self, restore_hover, hover_height=None):
        self.override_stopped = True
        return bool(restore_hover)

    def set_height(self, height):
        self.current_height = float(height)
        self.height_targets.append(float(height))


def _new_visual_controller(latest_sample, navi):
    return visual_target_descent.VisualTargetDescentController(
        fc=_FC(),
        navi=navi,
        stop_event=_StopEvent(),
        latest_vision_sample=latest_sample,
        raise_if_vision_failed=lambda: None,
        correction_gain=0.15,
        correction_deadband_px=3.0,
        horizontal_speed_limit=15.0,
        filter_alpha=0.25,
        control_period=0.05,
        vision_sample_stale_seconds=0.35,
        vision_loss_timeout=1.0,
    )


def test_provider_once_per_sequence_and_command_limit() -> None:
    _FakeTime.now = 0.0
    navi = _Navigation()
    provider_calls = []
    sample = (1, 0.0, 1000.0, 1000.0)
    controller = _new_visual_controller(lambda: sample, navi)

    def provider(x_px, y_px, sample_dt):
        provider_calls.append((x_px, y_px, sample_dt))
        return 7.2, 0.0

    original_time = visual_target_descent.time
    visual_target_descent.time = _FakeTime
    try:
        controller.descend_to_height(
            target_height=100.0,
            hover_seconds=0.0,
            height_confirm_time=0.05,
            timeout=1.0,
            base_velocity_provider=provider,
            pre_descent_follow_seconds=0.1,
            pre_descent_follow_timeout=1.0,
        )
    finally:
        visual_target_descent.time = original_time

    assert len(provider_calls) == 1
    horizontal_speeds = [
        math.hypot(command[0], command[1])
        for command in navi.commands
    ]
    assert horizontal_speeds
    assert max(horizontal_speeds) <= 15.0
    assert navi.override_stopped

    filtered, vel_x, vel_y = controller._next_horizontal_velocity(
        np.zeros(2, dtype=float),
        np.array([10.6, 10.6], dtype=float),
        1000.0,
        1000.0,
    )
    assert math.hypot(*filtered) <= 15.0
    assert math.hypot(vel_x, vel_y) <= 15.0


def test_low_altitude_target_offset_changes_only_control_error() -> None:
    _FakeTime.now = 0.0
    navi = _Navigation()
    recorded_offsets = []
    sample = (1, 0.0, 0.0, 0.0)
    controller = visual_target_descent.VisualTargetDescentController(
        fc=_FC(),
        navi=navi,
        stop_event=_StopEvent(),
        latest_vision_sample=lambda: sample,
        raise_if_vision_failed=lambda: None,
        record_callback=(
            lambda _started, _phase, x_px, y_px, _vx, _vy:
            recorded_offsets.append((x_px, y_px))
        ),
        correction_gain=0.15,
        correction_deadband_px=3.0,
        horizontal_speed_limit=15.0,
        filter_alpha=0.25,
        control_period=0.05,
        vision_sample_stale_seconds=0.35,
        vision_loss_timeout=1.0,
    )
    original_time = visual_target_descent.time
    visual_target_descent.time = _FakeTime
    try:
        controller.descend_to_height(
            target_height=25.0,
            hover_seconds=0.0,
            height_confirm_time=0.05,
            timeout=1.0,
            target_offset_provider=lambda _height: (-30.0, 0.0),
        )
    finally:
        visual_target_descent.time = original_time

    assert recorded_offsets
    assert all(offset == (0.0, 0.0) for offset in recorded_offsets)
    assert any(command[0] < 0 for command in navi.commands)
    assert all(command[1] == 0 for command in navi.commands)


def test_horizontal_command_guard_runs_before_send_and_record() -> None:
    _FakeTime.now = 0.0
    navi = _Navigation()
    guarded_inputs = []
    recorded_commands = []
    sample = (1, 0.0, 100.0, 20.0)
    controller = visual_target_descent.VisualTargetDescentController(
        fc=_FC(),
        navi=navi,
        stop_event=_StopEvent(),
        latest_vision_sample=lambda: sample,
        raise_if_vision_failed=lambda: None,
        record_callback=(
            lambda _started, _phase, _x, _y, vx, vy:
            recorded_commands.append((vx, vy))
        ),
        correction_gain=0.15,
        correction_deadband_px=3.0,
        horizontal_speed_limit=15.0,
        filter_alpha=0.25,
        control_period=0.05,
        vision_sample_stale_seconds=0.35,
        vision_loss_timeout=1.0,
    )

    def guard(velocity_x, velocity_y):
        guarded_inputs.append((velocity_x, velocity_y))
        return 0, velocity_y

    original_time = visual_target_descent.time
    visual_target_descent.time = _FakeTime
    try:
        controller.descend_to_height(
            target_height=100.0,
            hover_seconds=0.0,
            height_confirm_time=0.05,
            timeout=1.0,
            horizontal_command_guard=guard,
        )
    finally:
        visual_target_descent.time = original_time

    assert guarded_inputs
    assert any(command[0] > 0 for command in guarded_inputs)
    assert navi.commands
    assert all(command[0] == 0 for command in navi.commands)
    assert recorded_commands
    assert all(command[0] == 0 for command in recorded_commands)


def test_loss_resets_continuous_follow_timer() -> None:
    _FakeTime.now = 0.0
    navi = _Navigation()
    descent_started_at = []

    def latest_sample():
        now = _FakeTime.now
        if 0.1 <= now < 0.2:
            return None
        return int(now * 1000) + 1, now, 0.0, 0.0

    controller = _new_visual_controller(latest_sample, navi)
    original_time = visual_target_descent.time
    visual_target_descent.time = _FakeTime
    try:
        controller.descend_to_height(
            target_height=100.0,
            hover_seconds=0.0,
            height_confirm_time=0.05,
            timeout=1.0,
            pre_descent_follow_seconds=0.15,
            pre_descent_follow_timeout=1.0,
            on_descent_start=lambda: descent_started_at.append(
                _FakeTime.now
            ),
        )
    finally:
        visual_target_descent.time = original_time

    assert descent_started_at
    assert descent_started_at[0] >= 0.35
    assert any(command[0] == 0 and command[1] == 0 for command in navi.commands)


def test_continuous_follow_total_timeout() -> None:
    _FakeTime.now = 0.0
    navi = _Navigation()

    def latest_sample():
        now = _FakeTime.now
        phase = int(now / 0.05)
        if phase % 2:
            return None
        return phase + 1, now, 0.0, 0.0

    controller = _new_visual_controller(latest_sample, navi)
    original_time = visual_target_descent.time
    visual_target_descent.time = _FakeTime
    try:
        try:
            controller.descend_to_height(
                target_height=100.0,
                hover_seconds=0.0,
                height_confirm_time=0.05,
                timeout=1.0,
                pre_descent_follow_seconds=0.15,
                pre_descent_follow_timeout=0.3,
            )
        except RuntimeError as exc:
            assert "stable before timeout" in str(exc)
        else:
            raise AssertionError("Continuous-follow timeout was not enforced")
    finally:
        visual_target_descent.time = original_time
    assert navi.override_stopped


def test_stability_error_limit_and_external_gate() -> None:
    _FakeTime.now = 0.0
    navi = _Navigation()
    descent_started_at = []

    def latest_sample():
        now = _FakeTime.now
        x_px = 31.0 if 0.1 <= now < 0.2 else 0.0
        return int(now * 1000) + 1, now, x_px, 0.0

    controller = _new_visual_controller(latest_sample, navi)
    original_time = visual_target_descent.time
    visual_target_descent.time = _FakeTime
    try:
        controller.descend_to_height(
            target_height=100.0,
            hover_seconds=0.0,
            height_confirm_time=0.05,
            timeout=1.0,
            pre_descent_follow_seconds=0.2,
            pre_descent_follow_timeout=1.0,
            pre_descent_gate=lambda: _FakeTime.now >= 0.25,
            pre_descent_max_error_px=30.0,
            on_descent_start=lambda: descent_started_at.append(
                _FakeTime.now
            ),
        )
    finally:
        visual_target_descent.time = original_time

    assert descent_started_at
    assert descent_started_at[0] >= 0.4
    assert navi.override_stopped


def test_external_gate_total_timeout() -> None:
    _FakeTime.now = 0.0
    navi = _Navigation()

    def latest_sample():
        now = _FakeTime.now
        return int(now * 1000) + 1, now, 0.0, 0.0

    controller = _new_visual_controller(latest_sample, navi)
    original_time = visual_target_descent.time
    visual_target_descent.time = _FakeTime
    try:
        try:
            controller.descend_to_height(
                target_height=100.0,
                hover_seconds=0.0,
                height_confirm_time=0.05,
                timeout=1.0,
                pre_descent_follow_seconds=0.1,
                pre_descent_follow_timeout=0.3,
                pre_descent_gate=lambda: False,
                pre_descent_max_error_px=30.0,
            )
        except visual_target_descent.PreDescentTimeoutError:
            pass
        else:
            raise AssertionError("External-gate timeout was not enforced")
    finally:
        visual_target_descent.time = original_time
    assert navi.override_stopped


def main() -> None:
    test_estimator()
    test_estimator_velocity_predictor()
    test_provider_once_per_sequence_and_command_limit()
    test_low_altitude_target_offset_changes_only_control_error()
    test_horizontal_command_guard_runs_before_send_and_record()
    test_loss_resets_continuous_follow_timer()
    test_continuous_follow_total_timeout()
    test_stability_error_limit_and_external_gate()
    test_external_gate_total_timeout()
    print("moving-target descent logic tests passed")


if __name__ == "__main__":
    main()
