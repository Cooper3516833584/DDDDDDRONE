"""Exercise the real takeoff wait method with fake telemetry and a fake clock.

AST extraction avoids importing FlightController, opening hardware or ROS.
"""

import ast
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest


SOURCE = Path(__file__).resolve().parents[1] / "FlightController/Application.py"
TREE = ast.parse(SOURCE.read_text(encoding="utf-8"))
CLASS = next(node for node in TREE.body if isinstance(node, ast.ClassDef)
             and node.name == "FC_Application")
METHOD = next(node for node in CLASS.body if isinstance(node, ast.FunctionDef)
              and node.name == "wait_for_takeoff_done")


def simulated_fc(telemetry):
    fc = NS(connected=True, _action_log=Mock())
    fc.state = NS(alt_add=NS(value=8), vel_z=NS(value=0), unlock=NS(value=True))
    flags = NS(fresh=True)
    fc.state.is_fresh = lambda age: flags.fresh
    clock = NS(now=0.0)

    def update():
        values = telemetry(clock.now)
        fc.state.alt_add.value = values.get("height", 8)
        fc.state.vel_z.value = values.get("speed", 0)
        fc.connected = values.get("connected", True)
        fc.state.unlock.value = values.get("unlocked", True)
        flags.fresh = values.get("fresh", True)

    def sleep(seconds):
        clock.now += seconds
        update()

    logger = Mock()
    ns = {"time": NS(perf_counter=lambda: clock.now, sleep=sleep), "logger": logger}
    module = ast.fix_missing_locations(ast.Module(body=[METHOD], type_ignores=[]))
    exec(compile(module, str(SOURCE), "exec"), ns)
    update()
    return fc, clock, logger, ns["wait_for_takeoff_done"]


def test_climb_speed_before_height_does_not_trigger_early_failure():
    def telemetry(seconds):
        if seconds < 3:
            return {"height": 8, "speed": 0}
        if seconds < 3.5:
            return {"height": 9, "speed": 5}
        return {"height": 15, "speed": 6}

    fc, clock, _, wait = simulated_fc(telemetry)
    assert wait(fc) is True
    assert clock.now >= 4.5  # Height confirmation, then existing one-second settle.
    fc._action_log.assert_called_once_with("wait ok", "takeoff done")


@pytest.mark.parametrize("timeout,expected", [(5, False), (8, True)])
def test_slow_liftoff_can_finish_in_eight_second_window(timeout, expected):
    fc, clock, _, wait = simulated_fc(
        lambda t: {"height": 8 if t < 6 else 15, "speed": 0 if t < 3 else 5})
    assert wait(fc, timeout_s=timeout) is expected
    if expected:
        assert 7 <= clock.now < 7.2
    else:
        assert 5 <= clock.now < 5.2


def test_velocity_seen_before_hover_remains_valid_evidence():
    fc, clock, _, wait = simulated_fc(
        lambda t: {"height": 9 if t < 1.5 else 30, "speed": 5 if t < 1.5 else 0})
    assert wait(fc) is True
    assert clock.now >= 2.5


@pytest.mark.parametrize("height,speed", [(8, 5), (9, 5), (30, 0)])
def test_height_and_climb_evidence_are_both_required_until_timeout(height, speed):
    fc, clock, logger, wait = simulated_fc(lambda t: {"height": height, "speed": speed})
    assert wait(fc) is False
    assert 5 <= clock.now < 5.2
    assert logger.warning.called
    fc._action_log.assert_not_called()


@pytest.mark.parametrize("failure", ["connected", "unlocked", "fresh"])
@pytest.mark.parametrize("during_settle", [False, True])
def test_invalid_telemetry_or_lock_aborts_wait_and_settle(failure, during_settle):
    def telemetry(seconds):
        values = {"height": 30 if during_settle else 8, "speed": 5}
        if seconds >= (2 if during_settle else 1.2):
            values[failure] = False
        return values

    fc, clock, _, wait = simulated_fc(telemetry)
    assert wait(fc) is False
    assert clock.now < 3
    fc._action_log.assert_not_called()


def test_height_drop_during_settle_is_not_confirmed():
    fc, _, _, wait = simulated_fc(lambda t: {"height": 30 if t < 2 else 8, "speed": 5})
    assert wait(fc) is False
    fc._action_log.assert_not_called()


def test_normal_takeoff_retains_one_second_start_and_settle_delays():
    fc, clock, _, wait = simulated_fc(lambda t: {"height": 30, "speed": 5})
    assert wait(fc) is True
    assert clock.now == 2


def test_exact_height_and_speed_thresholds_are_accepted():
    fc, _, _, wait = simulated_fc(lambda t: {"height": 10, "speed": 4})
    assert wait(fc) is True


def test_user_interrupt_propagates_to_mission_cleanup():
    fc, _, _, wait = simulated_fc(lambda t: {"height": 8, "speed": 0})

    def interrupt(seconds):
        raise KeyboardInterrupt()

    wait.__globals__["time"].sleep = interrupt
    with pytest.raises(KeyboardInterrupt):
        wait(fc)
    fc._action_log.assert_not_called()


def test_custom_velocity_threshold_is_preserved():
    fc, _, _, wait = simulated_fc(lambda t: {"height": 30, "speed": 5})
    assert wait(fc, z_speed_threshold=6, timeout_s=2) is False


def test_zero_timeout_keeps_legacy_unbounded_wait_semantics():
    fc, clock, _, wait = simulated_fc(
        lambda t: {"height": 8 if t < 6 else 30, "speed": 5})
    assert wait(fc, timeout_s=0) is True
    assert clock.now >= 7
