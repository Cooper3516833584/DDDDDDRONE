import ast
import types
import unittest
from pathlib import Path
from typing import Tuple


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
    estimated_target_velocity = (12.0, 0.0)

    def follow_and_descend(self, **kwargs):
        callback = kwargs.get("on_descent_start")
        if callback is not None:
            callback()
        return (12.0, 0.0)


class _Predictor:
    @staticmethod
    def predict(velocity, _sample_dt, _x, _y):
        return velocity


class _BoundaryGuard:
    @staticmethod
    def is_active(_x):
        return False

    @staticmethod
    def apply(_x, velocity_x, velocity_y):
        return velocity_x, velocity_y


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


class _TrajectoryEvent:
    @staticmethod
    def is_set():
        return False


class _StopEvent:
    @staticmethod
    def is_set():
        return False

    @staticmethod
    def wait(_seconds):
        return False


class _FakeClock:
    def __init__(self):
        self.now = 0.0

    def monotonic(self):
        return self.now


class _AdvancingStopEvent:
    def __init__(self, clock):
        self.clock = clock

    @staticmethod
    def is_set():
        return False

    def wait(self, seconds):
        self.clock.now += float(seconds)
        return False


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
            [15, 15, 5, 7, 8, 3, 15],
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

    def test_missing_target_at_b_falls_back_to_c_search(self):
        mission = _class(self.mission_tree, "Task2Mission")
        method = _method(
            mission,
            "_wait_until_target_detected_on_trajectory",
        )
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
            "math": __import__("math"),
            "time": __import__("time"),
            "mission_base": types.SimpleNamespace(
                VISION_SAMPLE_STALE_SECONDS=0.35,
                ESCORT_CONTROL_PERIOD=0.05,
            ),
            "TARGET_DETECTION_PIXEL_THRESHOLD": 30.0,
        }
        exec(compile(module, str(MISSION2_PATH), "exec"), namespace)
        task = namespace["ExtractedTask2"]()
        task.stop_event = _StopEvent()
        task.navi = types.SimpleNamespace(
            traj_running_event=_TrajectoryEvent(),
        )
        task._update_pursuit_speed = lambda: None
        task._route_gate_is_open = lambda: False
        task._raise_if_vision_failed = lambda: None
        task._latest_vision_sample = lambda: None
        task._wait_for_target_at_c = lambda: (4.0, -2.0)

        self.assertEqual(
            (4.0, -2.0),
            task._wait_until_target_detected_on_trajectory(),
        )

    def test_c_search_accepts_valid_visual_target(self):
        clock = _FakeClock()
        task, navigation_calls = self._build_c_search_task(
            clock,
            [(1, clock.now, 4.0, -2.0)],
        )

        self.assertEqual((4.0, -2.0), task._wait_for_target_at_c())
        self.assertEqual([((237.5, -187.5), True)], navigation_calls)
        self.assertTrue(task.samples_cleared)

    def test_c_search_times_out_after_twenty_seconds(self):
        clock = _FakeClock()
        task, navigation_calls = self._build_c_search_task(clock, [])

        with self.assertRaisesRegex(RuntimeError, "20s at C"):
            task._wait_for_target_at_c()

        self.assertGreaterEqual(clock.now, 20.0)
        self.assertEqual([((237.5, -187.5), True)], navigation_calls)

    def _build_c_search_task(self, clock, samples):
        mission = _class(self.mission_tree, "Task2Mission")
        method = _method(mission, "_wait_for_target_at_c")
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
            "ARC_START": (237.5, -37.5),
            "ARC_END": (237.5, -187.5),
            "C_POINT_SEARCH_TIMEOUT_SECONDS": 20.0,
            "TARGET_DETECTION_PIXEL_THRESHOLD": 30.0,
            "TargetNotFoundError": RuntimeError,
            "logger": _Logger(),
            "math": __import__("math"),
            "time": types.SimpleNamespace(monotonic=clock.monotonic),
            "mission_base": types.SimpleNamespace(
                VISION_SAMPLE_STALE_SECONDS=0.35,
                ESCORT_CONTROL_PERIOD=0.05,
            ),
        }
        exec(compile(module, str(MISSION2_PATH), "exec"), namespace)
        task = namespace["ExtractedTask2"]()
        navigation_calls = []

        def navigate(point, wait):
            navigation_calls.append((tuple(point), wait))
            return True

        task.navi = types.SimpleNamespace(
            navigation_to_waypoint=navigate,
        )
        task.stop_event = _AdvancingStopEvent(clock)
        task.samples_cleared = False
        task._clear_vision_samples = lambda: setattr(
            task,
            "samples_cleared",
            True,
        )
        task._raise_if_vision_failed = lambda: None
        sample_queue = list(samples)
        task._latest_vision_sample = lambda: (
            sample_queue.pop(0) if sample_queue else None
        )
        return task, navigation_calls

    def test_failed_landing_does_not_report_target_locked(self):
        events = []

        def fail_landing(*_args, **_kwargs):
            events.append("land")
            raise RuntimeError("lock not confirmed")

        task = self._build_extracted_task(events, fail_landing)

        with self.assertRaises(RuntimeError):
            task._follow_descend_and_land_on_target()

        self.assertLess(events.index("landing_started"), events.index("land"))
        self.assertNotIn("target_locked", events)

    def test_successful_landing_reports_locked_after_confirmation(self):
        events = []

        def confirm_landing(*_args, **_kwargs):
            events.append("land")

        task = self._build_extracted_task(events, confirm_landing)

        task._follow_descend_and_land_on_target()

        self.assertLess(events.index("landing_started"), events.index("land"))
        self.assertLess(events.index("land"), events.index("target_locked"))
        self.assertLess(
            events.index("target_locked"),
            events.index("retakeoff_started"),
        )

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
            "TARGET_DESCENT_INTERMEDIATE_HEIGHT": 100.0,
            "ESCORT_STABLE_SECONDS": 4.0,
            "ESCORT_GATE_TIMEOUT_SECONDS": 90.0,
            "ESCORT_SPEED_MIDPOINT": 12.0,
            "TARGET_DETECTION_PIXEL_THRESHOLD": 30.0,
            "TARGET_DESCENT_TIMEOUT_SECONDS": 15.0,
            "TARGET_LANDING_HEIGHT": 25.0,
            "TARGET_LANDING_LOCK_TIMEOUT_SECONDS": 20.0,
            "LOCKED_DWELL_SECONDS": 5.0,
            "CRUISE_HEIGHT": 150.0,
            "PLATFORM_RETAKEOFF_HEIGHT": 30.0,
            "PLATFORM_RETAKEOFF_HEIGHT_TIMEOUT_SECONDS": 15.0,
            "ESCORT_MAX_X": 357.5,
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
        task._escort_x_boundary_guard = _BoundaryGuard()
        task._escort_x_boundary_active = False
        task._low_altitude_target_offset = _Offset()
        task._route_gate_is_open = lambda: True
        task.navi = _Navigation()
        task.fc = object()
        task.stop_event = object()
        task._platform_retakeoff_hold_point = None
        return task


if __name__ == "__main__":
    unittest.main()
