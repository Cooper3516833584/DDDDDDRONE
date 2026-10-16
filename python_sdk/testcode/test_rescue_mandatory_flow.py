"""Exercise the rescue mission state machine with fake methods only."""
import ast
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock


SOURCE = Path(__file__).resolve().parents[1] / "rescue_drop_2026.py"


def load_flow():
    tree = ast.parse(SOURCE.read_text(encoding="utf8"))
    state = next(node for node in tree.body
                 if isinstance(node, ast.ClassDef) and node.name == "MissionState")
    deadline = next(node for node in tree.body
                    if isinstance(node, ast.ClassDef) and node.name == "MissionDeadline")
    mission = next(node for node in tree.body
                   if isinstance(node, ast.ClassDef) and node.name == "Mission")
    mission.body = [node for node in mission.body
                    if isinstance(node, ast.FunctionDef) and node.name == "run"]
    future = ast.ImportFrom(module="__future__",
                            names=[ast.alias(name="annotations")], level=0)
    module = ast.fix_missing_locations(ast.Module(
        body=[future, deadline, state, mission], type_ignores=[]))
    from enum import Enum
    namespace = dict(Enum=Enum, logger=Mock(), TAKEOFF_POINT=(0.0, 0.0),
                     CRUISE_HEIGHT=150.0, MANDATORY_COLOR="yellow")
    exec(compile(module, str(SOURCE), "exec"), namespace)
    return namespace


class MandatoryFlowTests(unittest.TestCase):
    def setUp(self):
        self.names = load_flow()
        self.mission = self.names["Mission"]()
        states = self.names["MissionState"]
        self.mission.state = states.ROUTE_1
        self.mission.route_direction = "right"
        self.mission.entry_route = [(100.0, 0.0)]
        self.mission.navi = Mock()
        self.mission.navi.wait_for_height.return_value = True
        self.mission.navi.wait_for_yaw.return_value = True
        self.mission.navi.pose_is_fresh.return_value = True
        self.mission._check = Mock()
        self.mission.start_vision = Mock()
        self.mission.warmup_vision = Mock()
        self.mission._navigate_leg = Mock()
        self.mission._follow_route = Mock(return_value=None)
        self.mission._follow_center_route = Mock(return_value=None)
        self.mission._mandatory_drop = Mock(return_value=True)
        self.mission._navigate_return_anchor = Mock()
        self.mission._return_and_land = Mock()
        self.mission.return_anchor_reached = False
        self.mission.target = None

    def test_no_target_follows_entire_center_then_a_and_o_once(self):
        self.mission.run()
        self.mission._follow_route.assert_called_once_with(
            self.names["MissionState"].ROUTE_1)
        self.mission._follow_center_route.assert_called_once_with()
        self.mission._mandatory_drop.assert_not_called()
        self.mission._navigate_return_anchor.assert_called_once_with()
        self.mission._return_and_land.assert_called_once_with(
            check_deadline=False, protected=True)
        self.assertIs(self.mission.state, self.names["MissionState"].DONE)

    def test_confirmed_yellow_stops_search_and_returns_once(self):
        self.mission.target = SimpleNamespace(color="yellow")
        self.mission._follow_center_route.return_value = self.mission.target
        self.mission.run()
        self.mission._follow_center_route.assert_called_once_with()
        self.mission._mandatory_drop.assert_called_once_with()
        self.mission._navigate_return_anchor.assert_called_once_with()
        self.mission._return_and_land.assert_called_once_with(
            check_deadline=False, protected=True)

    def test_failed_drop_resumes_center_before_return(self):
        target = SimpleNamespace(color="yellow")
        self.mission._follow_center_route.side_effect = [target, None]
        self.mission._mandatory_drop.return_value = False
        self.mission.run()
        self.assertEqual(self.mission._follow_center_route.call_count, 2)
        self.mission._mandatory_drop.assert_called_once_with()
        self.mission._navigate_return_anchor.assert_called_once_with()

    def test_deadline_recovers_through_a_when_pose_is_fresh(self):
        self.mission._follow_center_route.side_effect = self.names["MissionDeadline"]("expired")
        with self.assertRaisesRegex(self.names["MissionDeadline"], "recovery completed"):
            self.mission.run()
        self.mission.navi.navigation_stop_here.assert_called_once_with()
        self.mission._navigate_return_anchor.assert_called_once_with(check_deadline=False)
        self.mission._return_and_land.assert_called_once_with(
            check_deadline=False, protected=True)
        self.assertIsNot(self.mission.state, self.names["MissionState"].DONE)

    def test_deadline_with_stale_pose_never_navigates(self):
        self.mission._follow_center_route.side_effect = self.names["MissionDeadline"]("expired")
        self.mission.navi.pose_is_fresh.return_value = False
        with self.assertRaises(self.names["MissionDeadline"]):
            self.mission.run()
        self.mission._navigate_return_anchor.assert_not_called()
        self.mission._return_and_land.assert_not_called()

    def test_deadline_after_a_does_not_repeat_anchor_leg(self):
        self.mission.return_anchor_reached = True
        self.mission._follow_center_route.side_effect = self.names["MissionDeadline"]("expired")
        with self.assertRaises(self.names["MissionDeadline"]):
            self.mission.run()
        self.mission._navigate_return_anchor.assert_not_called()
        self.mission._return_and_land.assert_called_once_with(
            check_deadline=False, protected=True)


if __name__ == "__main__":
    unittest.main()
