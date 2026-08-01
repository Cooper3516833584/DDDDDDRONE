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

    @staticmethod
    def navigation_to_waypoint(*_args, **_kwargs):
        raise AssertionError("Task 2 return must not generate an intermediate trajectory")


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

    def test_return_uses_one_fixed_takeoff_waypoint(self):
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
            "RETURN_SPEED": 20.0,
            "RETURN_POSITION_THRESHOLD": 10.0,
            "RETURN_SETTLE_SECONDS": 0.5,
            "RETURN_TIMEOUT_SECONDS": 45.0,
            "RETURN_MIN_CROSS_TRACK_SPEED": 2.0,
            "math": __import__("math"),
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
                ("set_navigation_speed", 20.0),
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
                ("set_navigation_speed", 20.0),
            ],
            task.navi.calls,
        )
        distance = (237.5 ** 2 + 187.5 ** 2) ** 0.5
        self.assertEqual(
            (
                -20.0 * 237.5 / distance,
                20.0 * 237.5 / distance,
            ),
            task.navi.navi_x_pid.output_limits,
        )
        self.assertEqual(
            (
                -20.0 * 187.5 / distance,
                20.0 * 187.5 / distance,
            ),
            task.navi.navi_y_pid.output_limits,
        )

    def test_fixed_route_search_stops_after_first_segment_detection(self):
        task, calls = self._build_fixed_route_search([(4.0, -2.0)])

        self.assertEqual((4.0, -2.0), task._search_target_on_fixed_route())
        self.assertEqual(
            [
                ("speed", 20.0),
                ("trajectory", [(87.5, -37.5, 150.0),
                                (237.5, -87.5, 150.0)]),
                ("wait", True),
            ],
            calls,
        )

    def test_fixed_route_search_reaches_c_before_waiting(self):
        task, calls = self._build_fixed_route_search(
            [None, None],
            c_target=(1.0, 3.0),
        )

        self.assertEqual((1.0, 3.0), task._search_target_on_fixed_route())
        self.assertEqual(
            [
                ("speed", 20.0),
                ("trajectory", [(87.5, -37.5, 150.0),
                                (237.5, -87.5, 150.0)]),
                ("wait", True),
                ("turn_velocity",),
                ("speed", 20.0),
                ("trajectory", [(237.5, -187.5, 100.0)]),
                ("wait", False),
                ("wait_at_c",),
            ],
            calls,
        )

    def _build_fixed_route_search(self, route_targets, c_target=None):
        mission = _class(self.mission_tree, "Task2Mission")
        method = _method(mission, "_search_target_on_fixed_route")
        extracted = ast.ClassDef(
            name="ExtractedTask2Search",
            bases=[],
            keywords=[],
            body=[method],
            decorator_list=[],
        )
        module = ast.Module(body=[extracted], type_ignores=[])
        ast.fix_missing_locations(module)
        namespace = {"PURSUIT_SPEED": 20.0, "Tuple": Tuple}
        exec(compile(module, str(MISSION2_PATH), "exec"), namespace)

        calls = []
        task = namespace["ExtractedTask2Search"]()
        task._fixed_route = [
            (87.5, -37.5, 150.0),
            (237.5, -87.5, 150.0),
            (237.5, -187.5, 100.0),
        ]
        task.navi = types.SimpleNamespace(
            set_navigation_speed=lambda speed: calls.append(("speed", speed))
        )
        task._start_fixed_trajectory = (
            lambda points: calls.append(("trajectory", points))
        )
        targets = iter(route_targets)

        def wait_for_target_or_end(*, update_speed=False):
            calls.append(("wait", update_speed))
            return next(targets)

        task._wait_for_target_or_trajectory_end = wait_for_target_or_end
        task._wait_for_non_positive_turn_velocity = (
            lambda: calls.append(("turn_velocity",))
        )

        def wait_at_c():
            calls.append(("wait_at_c",))
            return c_target

        task._wait_for_target_at_c = wait_at_c
        return task, calls

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
