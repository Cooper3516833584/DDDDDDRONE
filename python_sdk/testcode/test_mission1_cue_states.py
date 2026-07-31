import ast
import unittest
from pathlib import Path


SDK_DIR = Path(__file__).resolve().parents[1]
MISSION_PATH = SDK_DIR / "mission1_26.py"
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


class Mission1CueStateTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.mission_tree = ast.parse(
            MISSION_PATH.read_text(encoding="utf-8")
        )
        cls.base_tree = ast.parse(BASE_PATH.read_text(encoding="utf-8"))

    def test_escort_state_follows_target_detection_and_trajectory_stop(self):
        mission = _class(
            self.mission_tree,
            "MovingTargetVisualDescentMission",
        )
        run = _method(mission, "run")
        calls = {
            _call_name(node): node.lineno
            for node in ast.walk(run)
            if isinstance(node, ast.Call)
            and _call_name(node)
            in {
                "navigation_follow_trajectory",
                "send_pursuit_started",
                "_wait_until_target_detected_on_trajectory",
                "send_escort_started",
                "_perform_target_action",
            }
        }
        self.assertLess(
            calls["navigation_follow_trajectory"],
            calls["send_pursuit_started"],
        )
        self.assertLess(
            calls["send_pursuit_started"],
            calls["_wait_until_target_detected_on_trajectory"],
        )
        self.assertLess(
            calls["_wait_until_target_detected_on_trajectory"],
            calls["send_escort_started"],
        )
        self.assertLess(
            calls["send_escort_started"],
            calls["_perform_target_action"],
        )

    def test_drop_completed_uses_mission1_state_14(self):
        states = _class(self.base_tree, "MissionOperationState")
        values = {
            target.id: ast.literal_eval(node.value)
            for node in states.body
            if isinstance(node, ast.Assign)
            for target in node.targets
            if isinstance(target, ast.Name)
        }
        self.assertEqual(14, values["MISSION1_DROP_COMPLETED"])

        signals = _class(self.mission_tree, "MissionGroundStationSignals")
        method = _method(signals, "send_drop_completed")
        attributes = {
            node.attr for node in ast.walk(method) if isinstance(node, ast.Attribute)
        }
        self.assertIn("MISSION1_DROP_COMPLETED", attributes)
        self.assertNotIn("DROP_COMPLETED", attributes)

    def test_pursuit_started_uses_cruising_state_15(self):
        states = _class(self.base_tree, "MissionOperationState")
        values = {
            target.id: ast.literal_eval(node.value)
            for node in states.body
            if isinstance(node, ast.Assign)
            for target in node.targets
            if isinstance(target, ast.Name)
        }
        self.assertEqual(15, values["CRUISING"])

        signals = _class(self.mission_tree, "MissionGroundStationSignals")
        method = _method(signals, "send_pursuit_started")
        attributes = {
            node.attr for node in ast.walk(method) if isinstance(node, ast.Attribute)
        }
        self.assertIn("CRUISING", attributes)

    def test_payload_is_released_before_drop_completed_is_reported(self):
        mission = _class(
            self.mission_tree,
            "MovingTargetVisualDescentMission",
        )
        method = _method(mission, "_disable_output_and_report")
        calls = [
            node
            for node in ast.walk(method)
            if isinstance(node, ast.Call)
        ]
        disable = next(
            node for node in calls if _call_name(node) == "set_digital_output"
        )
        report = next(
            node for node in calls if _call_name(node) == "send_drop_completed"
        )
        self.assertEqual(0, ast.literal_eval(disable.args[0]))
        self.assertIs(False, ast.literal_eval(disable.args[1]))
        self.assertLess(disable.lineno, report.lineno)


if __name__ == "__main__":
    unittest.main()
