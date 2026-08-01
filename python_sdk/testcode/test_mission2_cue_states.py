import ast
import math
import types
import unittest
from pathlib import Path
from typing import List, Tuple


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
    def __init__(self):
        self.calls = []

    def follow_and_descend(self, **kwargs):
        self.calls.append(kwargs)
        callback = kwargs.get("on_descent_start")
        if callback is not None:
            callback()
        return (12.0, 0.0)


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

    def test_task2_route_is_local_straight_then_90_degree_arc(self):
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
            "ARC_CENTER": (237.5, -112.5),
            "ARC_RADIUS": 75.0,
            "ARC_END": (237.5, -187.5),
            "ROUTE_END": (87.5, -187.5),
        }
        module = ast.Module(body=[function], type_ignores=[])
        ast.fix_missing_locations(module)
        exec(compile(module, str(MISSION2_PATH), "exec"), namespace)
        trajectory = namespace["build_task2_pursuit_trajectory"](150.0, 10)

        direct = trajectory[:5]
        self.assertEqual((0.0, 0.0, 150.0), direct[0])
        self.assertEqual((312.5, -112.5, 150.0), direct[-1])
        slope = -112.5 / 312.5
        for x, y, _height in direct[1:]:
            self.assertTrue(math.isclose(y / x, slope, abs_tol=1e-9))

        arc = trajectory[4:-1]
        self.assertEqual(10, len(arc))
        self.assertEqual((237.5, -187.5, 150.0), arc[-1])
        for x, y, _height in arc:
            self.assertTrue(
                math.isclose(
                    math.hypot(x - 237.5, y + 112.5),
                    75.0,
                    abs_tol=1e-9,
                )
            )

    def test_target_detection_waits_until_the_arc_is_complete(self):
        mission = _class(self.mission_tree, "Task2Mission")
        method = _method(
            mission,
            "_wait_until_target_detected_on_trajectory",
        )
        detection_ifs = [
            node
            for node in ast.walk(method)
            if isinstance(node, ast.If)
            and any(
                isinstance(child, ast.Call)
                and _call_name(child) == "_stop_pursuit_trajectory"
                for child in ast.walk(node)
            )
        ]
        self.assertTrue(
            any(
                "route_complete"
                in {
                    child.id
                    for child in ast.walk(detection_if.test)
                    if isinstance(child, ast.Name)
                }
                for detection_if in detection_ifs
            )
        )

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
            task._follow_descend_and_land_on_target()

        self.assertLess(events.index("landing_started"), events.index("land"))
        self.assertNotIn("target_locked", events)

    def test_successful_landing_reports_locked_after_confirmation(self):
        events = []

        def confirm_landing(*_args, **_kwargs):
            events.append("land")

        task = self._build_extracted_task(events, confirm_landing)

        task._follow_descend_and_land_on_target()

        self.assertEqual(1, len(task.moving_target_descent.calls))
        descent_call = task.moving_target_descent.calls[0]
        self.assertEqual(3.0, descent_call["stabilize_seconds"])
        self.assertEqual(25.0, descent_call["target_height"])
        self.assertNotIn("pre_descent_gate", descent_call)
        self.assertNotIn("pre_descent_max_error_px", descent_call)
        self.assertNotIn("horizontal_command_guard", descent_call)

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
            "ESCORT_STABLE_SECONDS": 3.0,
            "ESCORT_STABLE_TIMEOUT_SECONDS": 90.0,
            "ESCORT_SPEED_MIDPOINT": 12.0,
            "TARGET_DESCENT_TIMEOUT_SECONDS": 15.0,
            "TARGET_LANDING_HEIGHT": 25.0,
            "TARGET_LANDING_LOCK_TIMEOUT_SECONDS": 20.0,
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
        task.navi = _Navigation()
        task.fc = object()
        task.stop_event = object()
        task._platform_retakeoff_hold_point = None
        return task


if __name__ == "__main__":
    unittest.main()
