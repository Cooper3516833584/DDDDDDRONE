import ast
from pathlib import Path
import unittest


SOURCE_PATH = Path(__file__).resolve().parents[1] / "2026_disaster_survey.py"


class DisasterSurveyElectromagnetTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tree = ast.parse(SOURCE_PATH.read_text(encoding="utf-8"))

    def test_payload_uses_established_channel_three(self):
        assignments = {
            node.targets[0].id: node.value.value
            for node in self.tree.body
            if isinstance(node, ast.Assign)
            and len(node.targets) == 1
            and isinstance(node.targets[0], ast.Name)
            and node.targets[0].id == "ELECTROMAGNET_OUTPUT_CHANNEL"
            and isinstance(node.value, ast.Constant)
        }
        self.assertEqual(3, assignments["ELECTROMAGNET_OUTPUT_CHANNEL"])

    def test_helper_forwards_channel_and_requested_state(self):
        helper = next(
            node
            for node in self.tree.body
            if isinstance(node, ast.FunctionDef)
            and node.name == "set_electromagnet"
        )
        call = helper.body[1].value
        self.assertIsInstance(call, ast.Call)
        self.assertEqual("set_digital_output", call.func.attr)
        self.assertEqual("ELECTROMAGNET_OUTPUT_CHANNEL", call.args[0].id)
        self.assertEqual("engaged", call.args[1].id)

    def test_main_engages_after_prepare_and_before_mission_run(self):
        source = SOURCE_PATH.read_text(encoding="utf-8")
        prepare_branch = source.index(
            "command.command_id == int(CommandId.DRONE_PREPARE_MISSION)"
        )
        engage = source.index("set_electromagnet(fc, True)")
        mission_run = source.rindex("mission.run()")
        self.assertLess(prepare_branch, engage)
        self.assertLess(engage, mission_run)


if __name__ == "__main__":
    unittest.main()
