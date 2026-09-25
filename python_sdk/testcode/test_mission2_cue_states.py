import ast
import math
import types
import unittest
from pathlib import Path
from typing import List, Optional, Tuple


SDK_DIR = Path(__file__).resolve().parents[1]
MISSION2_PATH = SDK_DIR / "mission2_26.py"
BASE_PATH = SDK_DIR / "mission1_26_base.py"


def _class(tree, name):
    return next(
        node
        for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == name
    )


def _method(class_node, name):
    return next(
        node
        for node in class_node.body
        if isinstance(node, ast.FunctionDef) and node.name == name
    )


def _call_name(call):
    func = call.func
    if isinstance(func, ast.Attribute):
        return func.attr
    if isinstance(func, ast.Name):
        return func.id
    return None


class _BaseSignals:
    def __init__(self, mission):
        self._mission = mission

    def _send(self, name, operation_state):
        self._mission.events.append((name, operation_state))

    def send_escort_started(self):
        self._send("escort_started", 5)


class _FakeMission:
    def __init__(self):
        self.events = []


class _Signals:
    def __init__(self, events):
        self.events = events

    def send_target_descent_started(self):
        self.events.append("descent_started")

    def send_target_landing_started(self):
        self.events.append("landing_started")

    def send_target_locked(self):
        self.events.append("target_locked")

    def send_retakeoff_started(self):
        self.events.append("retakeoff_started")

    def send_retakeoff_succeeded(self):
        self.events.append("retakeoff_succeeded")


class _Descent:
    estimated_target_velocity = (9.0, 0.0)

    def __init__(self):
        self.calls = []

    def follow_and_descend(self, **kwargs):
        self.calls.append(kwargs)
        callback = kwargs.get("on_descent_start")
        if callback is not None:
            callback()
        return (9.0, 0.0)


class _Predictor:
    @staticmethod
    def predict(velocity, _sample_dt, _x, _y):
        return velocity


class _Offset:
    @staticmethod
    def offset(_height):
        return 0.0, 0.0


class _Navigation:
    current_x = 0.0
    current_y = 0.0

    @staticmethod
    def set_yaw(_yaw):
        return None

    @staticmethod
    def wait_for_yaw():
        return True


class _ReturnNavigation:
    def __init__(self):
        self.calls = []
        self.current_x = 237.5
        self.current_y = -187.5
        self.navi_x_pid = types.SimpleNamespace(output_limits=None)
        self.navi_y_pid = types.SimpleNamespace(output_limits=None)

    def set_navigation_speed(self, speed):
        self.calls.append(("set_navigation_speed", speed))

    def switch_pid(self, name):
        self.calls.append(("switch_pid", name))

    def direct_set_waypoint(self, point):
        self.calls.append(("direct_set_waypoint", tuple(point)))

    def wait_for_waypoint(self, **kwargs):
        self.calls.append(("wait_for_waypoint", kwargs))
        return True

    def navigation_to_waypoint(self, *_args, **_kwargs):
        raise AssertionError("Return must use horizontal point PID")


class _Logger:
    def __getattr__(self, _name):
        return lambda *_args, **_kwargs: None


class Mission2CueStateTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.mission_tree = ast.parse(
            MISSION2_PATH.read_text(encoding="utf-8")
        )
        cls.base_tree = ast.parse(BASE_PATH.read_text(encoding="utf-8"))

    def test_signal_state_mapping(self):
        signals_node = _class(self.mission_tree, "Mission2Signals")
        module = ast.Module(body=[signals_node], type_ignores=[])
        ast.fix_missing_locations(module)
        states = types.SimpleNamespace(
            TAKEOFF=3,
            ESCORTING=5,
            LANDING_ON_CAR=7,
            ON_CAR=8,
            CRUISING=15,
            RETAKEOFF_SUCCEEDED=16,
        )
        namespace = {
            "mission1": types.SimpleNamespace(
                MissionGroundStationSignals=_BaseSignals
            ),
            "mission_base": types.SimpleNamespace(
                MissionOperationState=states
            ),
        }
        exec(compile(module, str(MISSION2_PATH), "exec"), namespace)
        mission = _FakeMission()
        signals = namespace["Mission2Signals"](mission)

        for method_name in (
            "send_takeoff_succeeded",
            "send_pursuit_started",
            "send_escort_started",
            "send_target_landing_started",
            "send_target_locked",
            "send_retakeoff_started",
            "send_retakeoff_succeeded",
        ):
            getattr(signals, method_name)()

        self.assertEqual(
            [15, 15, 5, 7, 8, 3, 16],
            [state for _name, state in mission.events],
        )

    def test_shared_state_values_are_stable(self):
        states = _class(self.base_tree, "MissionOperationState")
        values = {
            target.id: ast.literal_eval(node.value)
            for node in states.body
            if isinstance(node, ast.Assign)
            for target in node.targets
            if isinstance(target, ast.Name)
        }
        self.assertEqual(7, values["LANDING_ON_CAR"])
        self.assertEqual(8, values["ON_CAR"])
        self.assertEqual(15, values["CRUISING"])
        self.assertEqual(16, values["RETAKEOFF_SUCCEEDED"])

    def test_task2_route_is_local_straight_to_arc_start(self):
        function = next(
            node
            for node in self.mission_tree.body
            if isinstance(node, ast.FunctionDef)
            and node.name == "build_task2_pursuit_trajectory"
        )
        namespace = {
            "math": math,
            "List": List,
            "Tuple": Tuple,
            "TAKEOFF_POINT": (0.0, 0.0),
            "TASK2_ARC_START": (312.5, -112.5),
            "TASK2_PURSUIT_DIRECT_SEGMENTS": 4,
        }
        module = ast.Module(body=[function], type_ignores=[])
        ast.fix_missing_locations(module)
        exec(compile(module, str(MISSION2_PATH), "exec"), namespace)
        trajectory = namespace["build_task2_pursuit_trajectory"](150.0)

        direct = trajectory
        self.assertEqual(5, len(direct))
        self.assertEqual((0.0, 0.0, 150.0), direct[0])
        self.assertEqual((312.5, -112.5, 150.0), direct[-1])
        slope = -112.5 / 312.5
        for x, y, _height in direct[1:]:
            self.assertTrue(math.isclose(y / x, slope, abs_tol=1e-9))

    def test_target_wait_and_escort_thresholds(self):
        values = {
            target.id: ast.literal_eval(node.value)
            for node in self.mission_tree.body
            if isinstance(node, ast.Assign)
            for target in node.targets
            if isinstance(target, ast.Name)
        }
        self.assertEqual(60.0, values["TARGET_WAIT_TIMEOUT_SECONDS"])
        self.assertEqual(80.0, values["ESCORT_ENTRY_PIXEL_RADIUS"])
        self.assertEqual(9.0, values["ESCORT_INITIAL_ESTIMATED_SPEED"])
        self.assertEqual(40.0, values["TARGET_DESCENT_GATE_RADIUS"])
        self.assertEqual(18.0, values["PURSUIT_APPROACH_SPEED"])
        self.assertEqual(5.0, values["LOCKED_DWELL_SECONDS"])
        self.assertEqual(1.0, values["RETAKEOFF_SUCCEEDED_STATE_HOLD_SECONDS"])
        self.assertEqual(30.0, values["TASK2_H_LANDING_HEIGHT"])
        self.assertEqual(5.0, values["TASK2_H_LANDING_HEIGHT_TOLERANCE"])
        self.assertEqual(
            8.0,
            values["TASK2_H_LANDING_ALIGNMENT_TIMEOUT_SECONDS"],
        )
        self.assertEqual(
            90.0,
            values["TASK2_H_LANDING_MAX_CONTROL_HEIGHT"],
        )
        self.assertEqual(13.0, values["TARGET_DIRECT_LOCK_HEIGHT"])
        self.assertEqual(
            0.4,
            values["TARGET_DIRECT_LOCK_CONFIRM_SECONDS"],
        )

    def test_task2_h_landing_height_override_is_restored(self):
        mission = _class(self.mission_tree, "Task2Mission")
        method = _method(mission, "_visual_h_landing_at_takeoff")
        extracted = ast.ClassDef(
            name="ExtractedTask2",
            bases=[ast.Name(id="BaseMission", ctx=ast.Load())],
            keywords=[],
            body=[method],
            decorator_list=[],
        )
        module = ast.Module(body=[extracted], type_ignores=[])
        ast.fix_missing_locations(module)

        class BaseMission:
            def _visual_h_landing_at_takeoff(self):
                self.observed = (
                    descent.H_LANDING_HEIGHT,
                    descent.H_LANDING_HEIGHT_TOLERANCE,
                    descent.H_LANDING_ALIGNMENT_TIMEOUT,
                    descent.H_LANDING_TIMEOUT_FALLBACK_TO_DIRECT_LANDING,
                    descent.H_LANDING_MAX_CONTROL_HEIGHT,
                )
                raise RuntimeError("alignment failed")

        descent = types.SimpleNamespace(
            H_LANDING_HEIGHT=60.0,
            H_LANDING_MAX_CONTROL_HEIGHT=75.0,
            H_LANDING_HEIGHT_TOLERANCE=8.0,
            H_LANDING_ALIGNMENT_TIMEOUT=60.0,
            H_LANDING_TIMEOUT_FALLBACK_TO_DIRECT_LANDING=False,
        )
        namespace = {
            "BaseMission": BaseMission,
            "descent_test": descent,
            "TASK2_H_LANDING_HEIGHT": 30.0,
            "TASK2_H_LANDING_HEIGHT_TOLERANCE": 5.0,
            "TASK2_H_LANDING_ALIGNMENT_TIMEOUT_SECONDS": 8.0,
            "TASK2_H_LANDING_MAX_CONTROL_HEIGHT": 90.0,
        }
        exec(compile(module, str(MISSION2_PATH), "exec"), namespace)
        task = namespace["ExtractedTask2"]()

        with self.assertRaisesRegex(RuntimeError, "alignment failed"):
            task._visual_h_landing_at_takeoff()

        self.assertEqual((30.0, 5.0, 8.0, True, 90.0), task.observed)
        self.assertEqual(60.0, descent.H_LANDING_HEIGHT)
        self.assertEqual(8.0, descent.H_LANDING_HEIGHT_TOLERANCE)
        self.assertEqual(60.0, descent.H_LANDING_ALIGNMENT_TIMEOUT)
        self.assertIs(False, descent.H_LANDING_TIMEOUT_FALLBACK_TO_DIRECT_LANDING)
        self.assertEqual(75.0, descent.H_LANDING_MAX_CONTROL_HEIGHT)

    def test_retakeoff_success_state_is_held_before_return(self):
        mission = _class(self.mission_tree, "Task2Mission")
        run = _method(mission, "run")
        calls = [
            (_call_name(node), node.lineno)
            for node in ast.walk(run)
            if isinstance(node, ast.Call)
            and _call_name(node)
            in {
                "_follow_descend_and_land_on_target",
                "_hold_retakeoff_succeeded_state",
                "_return_home_and_land",
            }
        ]
        call_lines = dict(calls)
        self.assertLess(
            call_lines["_follow_descend_and_land_on_target"],
            call_lines["_hold_retakeoff_succeeded_state"],
        )
        self.assertLess(
            call_lines["_hold_retakeoff_succeeded_state"],
            call_lines["_return_home_and_land"],
        )

    def test_h_alignment_timeout_uses_current_point_landing(self):
        base_mission = _class(
            ast.parse(
                (SDK_DIR / "mission1_26_visual_descent_test.py").read_text(
                    encoding="utf-8"
                )
            ),
            "StaticTargetVisualDescentMission",
        )
        method = _method(base_mission, "_visual_h_landing_at_takeoff")
        extracted = ast.ClassDef(
            name="ExtractedLandingMission",
            bases=[],
            keywords=[],
            body=[method],
            decorator_list=[],
        )
        module = ast.Module(body=[extracted], type_ignores=[])
        ast.fix_missing_locations(module)

        class FakeTime:
            now = 0.0

            @classmethod
            def monotonic(cls):
                return cls.now

        def wait(seconds):
            FakeTime.now += float(seconds)
            return False

        landing_points = []

        namespace = {
            "math": math,
            "time": FakeTime,
            "logger": _Logger(),
            "mission1": types.SimpleNamespace(VISION_SAMPLE_STALE_SECONDS=0.35),
            "H_LANDING_VERTICAL_SPEED": 15.0,
            "H_LANDING_HEIGHT": 30.0,
            "H_LANDING_HEIGHT_TOLERANCE": 5.0,
            "H_LANDING_HEIGHT_TIMEOUT": 8.0,
            "H_LANDING_PIXEL_THRESHOLD": 18.0,
            "H_LANDING_CENTER_CONFIRM_FRAMES": 5,
            "H_LANDING_APPROACH_SPEED": 15.0,
            "H_LANDING_CONTROL_PERIOD": 0.1,
            "H_LANDING_ALIGNMENT_TIMEOUT": 0.2,
            "H_LANDING_TIMEOUT_FALLBACK_TO_DIRECT_LANDING": True,
            "H_LANDING_MIN_CONTROL_HEIGHT": 25.0,
            "H_LANDING_MAX_CONTROL_HEIGHT": 75.0,
        }
        exec(compile(module, str(MISSION2_PATH), "exec"), namespace)
        task = namespace["ExtractedLandingMission"]()
        task.stop_event = types.SimpleNamespace(
            is_set=lambda: False,
            wait=wait,
        )
        task.navi = types.SimpleNamespace(
            running=True,
            current_point=(1.0, 2.0),
            set_vertical_speed=lambda _speed: None,
            set_height=lambda _height: None,
            wait_for_height=lambda **_kwargs: True,
            pose_is_fresh=lambda: True,
            stop_move=lambda: None,
            pointing_landing=lambda point: landing_points.append(point) or True,
        )
        task.fc = types.SimpleNamespace(
            HOLD_POS_MODE=2,
            state=types.SimpleNamespace(
                is_fresh=lambda _age: True,
                unlock=types.SimpleNamespace(value=True),
                mode=types.SimpleNamespace(value=2),
            ),
        )
        task._current_navi_height = lambda: 30.0
        task._latest_vision_sample = lambda: None

        task._visual_h_landing_at_takeoff()

        self.assertEqual([(1.0, 2.0)], landing_points)

    def test_escort_starts_after_arc_start_and_80px_wait(self):
        mission = _class(self.mission_tree, "Task2Mission")
        run = _method(mission, "run")
        calls = {
            _call_name(node): node.lineno
            for node in ast.walk(run)
            if isinstance(node, ast.Call)
            and _call_name(node)
            in {
                "_wait_for_pursuit_to_arc_start",
                "_wait_for_target_to_enter_escort_radius",
                "send_escort_started",
                "_follow_descend_and_land_on_target",
            }
        }
        self.assertLess(
            calls["_wait_for_pursuit_to_arc_start"],
            calls["_wait_for_target_to_enter_escort_radius"],
        )
        self.assertLess(
            calls["_wait_for_target_to_enter_escort_radius"],
            calls["send_escort_started"],
        )
        self.assertLess(
            calls["send_escort_started"],
            calls["_follow_descend_and_land_on_target"],
        )

    def test_straight_pursuit_continues_outside_arc_start_boundary(self):
        mission = _class(self.mission_tree, "Task2Mission")
        method = _method(mission, "_wait_for_pursuit_to_arc_start")
        extracted = ast.ClassDef(
            name="ExtractedTask2",
            bases=[],
            keywords=[],
            body=[method],
            decorator_list=[],
        )
        module = ast.Module(body=[extracted], type_ignores=[])
        ast.fix_missing_locations(module)
        namespace = {
            "math": math,
            "logger": _Logger(),
            "TASK2_ARC_START": (312.5, -112.5),
        }
        exec(compile(module, str(MISSION2_PATH), "exec"), namespace)

        switches = []
        task = namespace["ExtractedTask2"]()
        task.navi = types.SimpleNamespace(
            traj_running_event=types.SimpleNamespace(is_set=lambda: False),
            current_x=250.0,
            current_y=-50.0,
            switch_pid=switches.append,
        )

        task._wait_for_pursuit_to_arc_start()

        self.assertEqual(["hover"], switches)

    def test_target_wait_estimates_before_entering_80px_radius(self):
        mission = _class(self.mission_tree, "Task2Mission")
        method = _method(
            mission,
            "_wait_for_target_to_enter_escort_radius",
        )
        extracted = ast.ClassDef(
            name="ExtractedTargetWait",
            bases=[],
            keywords=[],
            body=[method],
            decorator_list=[],
        )
        module = ast.Module(body=[extracted], type_ignores=[])
        ast.fix_missing_locations(module)

        class FakeTime:
            now = 0.0

            @classmethod
            def monotonic(cls):
                return cls.now

        class StopEvent:
            @staticmethod
            def is_set():
                return False

            @staticmethod
            def wait(seconds):
                FakeTime.now += float(seconds)

        class Estimator:
            def __init__(self):
                self.reset_value = None
                self.updates = []

            def reset(self, value):
                self.reset_value = value

            def update(self, x_px, y_px, sample_dt):
                self.updates.append((x_px, y_px, sample_dt))
                return 9.0, 0.0

        namespace = {
            "Optional": Optional,
            "Tuple": Tuple,
            "math": math,
            "time": FakeTime,
            "logger": _Logger(),
            "ESCORT_INITIAL_ESTIMATED_SPEED": 9.0,
            "TARGET_WAIT_TIMEOUT_SECONDS": 60.0,
            "ESCORT_ENTRY_PIXEL_RADIUS": 80.0,
            "TargetNotFoundError": RuntimeError,
            "mission_base": types.SimpleNamespace(
                VISION_SAMPLE_STALE_SECONDS=0.35,
                ESCORT_CONTROL_PERIOD=0.05,
            ),
        }
        exec(compile(module, str(MISSION2_PATH), "exec"), namespace)
        task = namespace["ExtractedTargetWait"]()
        task.stop_event = StopEvent()
        task._target_wait_estimator = Estimator()
        task._clear_vision_samples = lambda: None
        task._raise_if_vision_failed = lambda: None
        task.fc = types.SimpleNamespace(
            state=types.SimpleNamespace(
                is_fresh=lambda _age: True,
                unlock=types.SimpleNamespace(value=True),
            )
        )
        task.navi = types.SimpleNamespace(pose_is_fresh=lambda: True)
        samples = iter(
            [
                (1, 0.0, 100.0, 0.0),
                (2, 0.05, 79.0, 0.0),
            ]
        )
        task._latest_vision_sample = lambda: next(samples)

        velocity = task._wait_for_target_to_enter_escort_radius()

        self.assertEqual((9.0, 0.0), velocity)
        self.assertEqual((9.0, 0.0), task._target_wait_estimator.reset_value)
        self.assertEqual(2, len(task._target_wait_estimator.updates))
        self.assertEqual(100.0, task._target_wait_estimator.updates[0][0])
        self.assertEqual(79.0, task._target_wait_estimator.updates[1][0])

    def test_landing_reports_bracket_confirmed_lock(self):
        mission = _class(self.mission_tree, "Task2Mission")
        method = _method(mission, "_follow_descend_and_land_on_target")
        calls = sorted(
            (
                node.lineno,
                _call_name(node),
            )
            for node in ast.walk(method)
            if isinstance(node, ast.Call)
            and _call_name(node)
            in {
                "send_target_landing_started",
                "land_on_target_and_confirm_lock",
                "send_target_locked",
            }
        )
        self.assertEqual(
            [
                "send_target_landing_started",
                "land_on_target_and_confirm_lock",
                "send_target_locked",
            ],
            [name for _line, name in calls],
        )

    def test_failed_landing_does_not_report_target_locked(self):
        events = []

        def fail_landing(*_args, **_kwargs):
            events.append("land")
            raise RuntimeError("lock not confirmed")

        task = self._build_extracted_task(events, fail_landing)

        with self.assertRaises(RuntimeError):
            task._follow_descend_and_land_on_target((9.0, 0.0))

        self.assertLess(events.index("landing_started"), events.index("land"))
        self.assertNotIn("target_locked", events)

    def test_successful_landing_reports_locked_after_confirmation(self):
        events = []

        def confirm_landing(*_args, **_kwargs):
            events.append("land")

        task = self._build_extracted_task(events, confirm_landing)

        task._follow_descend_and_land_on_target((9.0, 0.0))

        self.assertEqual(2, len(task.moving_target_descent.calls))
        first_descent, second_descent = task.moving_target_descent.calls
        self.assertEqual(3.0, first_descent["stabilize_seconds"])
        self.assertEqual(100.0, first_descent["target_height"])
        self.assertEqual((9.0, 0.0), first_descent["initial_target_velocity"])
        self.assertEqual(30.0, first_descent["pre_descent_max_error_px"])
        self.assertNotIn("pre_descent_gate", first_descent)

        self.assertEqual(0.0, second_descent["stabilize_seconds"])
        self.assertEqual(25.0, second_descent["target_height"])
        self.assertIn("pre_descent_gate", second_descent)
        self.assertEqual(30.0, second_descent["pre_descent_max_error_px"])
        self.assertIs(False, second_descent["reset_estimator"])

        self.assertLess(events.index("landing_started"), events.index("land"))
        self.assertLess(events.index("land"), events.index("target_locked"))
        self.assertLess(
            events.index("target_locked"),
            events.index("retakeoff_started"),
        )

    def test_return_uses_horizontal_point_pid(self):
        mission = _class(self.mission_tree, "Task2Mission")
        method = _method(mission, "_return_home_and_land")
        extracted = ast.ClassDef(
            name="ExtractedTask2",
            bases=[],
            keywords=[],
            body=[method],
            decorator_list=[],
        )
        module = ast.Module(body=[extracted], type_ignores=[])
        ast.fix_missing_locations(module)
        namespace = {
            "RETURN_SPEED": 30.0,
            "RETURN_POSITION_THRESHOLD": 10.0,
            "RETURN_SETTLE_SECONDS": 0.5,
            "RETURN_TIMEOUT_SECONDS": 45.0,
            "straight_return_axis_limits": (
                lambda current_x, current_y, target_x, target_y, speed: (
                    speed
                    * abs(target_x - current_x)
                    / math.hypot(
                        target_x - current_x,
                        target_y - current_y,
                    ),
                    speed
                    * abs(target_y - current_y)
                    / math.hypot(
                        target_x - current_x,
                        target_y - current_y,
                    ),
                )
            ),
            "math": math,
            "logger": _Logger(),
            "mission_base": types.SimpleNamespace(TAKEOFF_POINT=(0.0, 0.0)),
        }
        exec(compile(module, str(MISSION2_PATH), "exec"), namespace)

        task = namespace["ExtractedTask2"]()
        task.navi = _ReturnNavigation()
        task.signals = types.SimpleNamespace(
            send_return_started=lambda: None,
            send_landing_started=lambda: None,
        )
        task.enable_h_landing_vision = lambda: None
        task._visual_h_landing_at_takeoff = lambda: None

        task._return_home_and_land()

        self.assertEqual(
            [
                ("set_navigation_speed", 30.0),
                ("switch_pid", "navi"),
                ("direct_set_waypoint", (0.0, 0.0)),
                (
                    "wait_for_waypoint",
                    {
                        "time_thres": 0.5,
                        "pos_thres": 10.0,
                        "timeout": 45.0,
                    },
                ),
                ("set_navigation_speed", 30.0),
            ],
            task.navi.calls,
        )
        x_limit = task.navi.navi_x_pid.output_limits[1]
        y_limit = task.navi.navi_y_pid.output_limits[1]
        self.assertTrue(math.isclose(math.hypot(x_limit, y_limit), 30.0))
        self.assertTrue(math.isclose(x_limit / y_limit, 237.5 / 187.5))

    def _build_extracted_task(self, events, landing_function):
        mission = _class(self.mission_tree, "Task2Mission")
        method = _method(mission, "_follow_descend_and_land_on_target")
        extracted = ast.ClassDef(
            name="ExtractedTask2",
            bases=[],
            keywords=[],
            body=[method],
            decorator_list=[],
        )
        module = ast.Module(body=[extracted], type_ignores=[])
        ast.fix_missing_locations(module)
        namespace = {
            "Tuple": Tuple,
            "logger": _Logger(),
            "ESCORT_STABLE_SECONDS": 3.0,
            "ESCORT_STABLE_TIMEOUT_SECONDS": 90.0,
            "TARGET_DESCENT_INTERMEDIATE_HEIGHT": 100.0,
            "TARGET_DETECTION_PIXEL_THRESHOLD": 30.0,
            "TARGET_DESCENT_TIMEOUT_SECONDS": 15.0,
            "TARGET_LANDING_HEIGHT": 25.0,
            "ARC_END": (237.5, -187.5),
            "TARGET_LANDING_LOCK_TIMEOUT_SECONDS": 20.0,
            "TARGET_DIRECT_LOCK_HEIGHT": 13.0,
            "TARGET_DIRECT_LOCK_CONFIRM_SECONDS": 0.4,
            "LOCKED_DWELL_SECONDS": 5.0,
            "CRUISE_HEIGHT": 150.0,
            "PLATFORM_RETAKEOFF_HEIGHT": 30.0,
            "PLATFORM_RETAKEOFF_HEIGHT_TIMEOUT_SECONDS": 15.0,
            "descent_test": types.SimpleNamespace(
                HEIGHT_TOLERANCE=5.0,
                HEIGHT_CONFIRM_SECONDS=1.0,
            ),
            "land_on_target_and_confirm_lock": landing_function,
            "locked_red_led_dwell": (
                lambda *_args, **_kwargs: events.append("locked_dwell")
            ),
            "retakeoff_from_moving_platform": (
                lambda *_args, **_kwargs: (10.0, 20.0)
            ),
        }
        exec(compile(module, str(MISSION2_PATH), "exec"), namespace)
        task = namespace["ExtractedTask2"]()
        task.signals = _Signals(events)
        task.moving_target_descent = _Descent()
        task._arc_velocity_predictor = _Predictor()
        task._low_altitude_target_offset = _Offset()
        task._route_gate_is_open = lambda: True
        task.navi = _Navigation()
        task.fc = object()
        task.stop_event = object()
        task._platform_retakeoff_hold_point = None
        return task


if __name__ == "__main__":
    unittest.main()
