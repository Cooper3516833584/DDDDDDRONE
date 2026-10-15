"""No FC imports or hardware; verify payload writes and alignment geometry."""
import ast
import math
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, call, patch
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import payload_servos as servos


class PayloadTests(unittest.TestCase):
    def test_open_close_single_and_repeated_open(self):
        fc = Mock()
        payload = servos.DronePayloadServos(fc)
        payload.open_all(); payload.close_all(); payload.open_slot(2); payload.open_all()
        self.assertEqual(fc.set_PWM_output.call_args_list,
            [call(i,60.) for i in range(4)] + [call(i,80.) for i in range(4)] +
            [call(2,60.)] + [call(i,60.) for i in range(4)])

    def test_failure_still_attempts_every_channel(self):
        fc = Mock()
        fc.set_PWM_output.side_effect = [OSError('IPC'),None,None,None]
        with self.assertRaisesRegex(RuntimeError,'channel 0'):
            servos.DronePayloadServos(fc).open_all()
        self.assertEqual(fc.set_PWM_output.call_count,4)

    def test_two_installed_slots_match_the_confirmed_local_positions(self):
        self.assertTrue(servos.validate_drop_plan())
        self.assertEqual(servos.DROP_SERVO_SEQUENCE, (0, 2))
        self.assertEqual(servos.PAYLOAD_COUNT, 2)
        for number, angle in enumerate((0.0, 144.0), 1):
            x, y = servos.payload_offset_body_cm(number)
            self.assertAlmostEqual(x, 8.7 * math.cos(math.radians(angle)))
            self.assertAlmostEqual(y, 8.7 * math.sin(math.radians(angle)))
        with self.assertRaises(ValueError):
            servos.slot_for_drop_attempt(3)

    def test_servo_positions_match_visual_alignment_offsets(self):
        source = Path(__file__).resolve().parents[1] / 'rescue_drop_2026.py'
        tree = ast.parse(source.read_text(encoding='utf8'))
        helpers = [node for node in tree.body if isinstance(node, ast.FunctionDef)
                   and node.name in ('payload_offset_body_cm', 'payload_target_offset_px',
                                     'pixel_offset_to_ground_cm')]
        namespace = dict(math=math, Point=tuple, Tuple=tuple, TOTAL_DROP_COUNT=2,
                         PAYLOAD_RADIUS_CM=servos.PAYLOAD_RADIUS_CM,
                         PAYLOAD_ANGLES_DEG=servos.PAYLOAD_ANGLES_DEG,
                         slot_for_drop_attempt=servos.slot_for_drop_attempt,
                         CAMERA_VIEW_REFERENCE_HEIGHT_CM=160.0,
                         CAMERA_VIEW_LONG_EDGE_CM_AT_REFERENCE_HEIGHT=300.0)
        exec(compile(ast.fix_missing_locations(ast.Module(body=helpers, type_ignores=[])),
                     str(source), 'exec'), namespace)
        for number in range(1, 3):
            expected = namespace['payload_offset_body_cm'](number)
            actual = servos.payload_offset_body_cm(number)
            self.assertAlmostEqual(actual[0], expected[0])
            self.assertAlmostEqual(actual[1], expected[1])
            image_offset = namespace['payload_target_offset_px'](number, 150.0, (640, 480))
            from_image = namespace['pixel_offset_to_ground_cm'](
                image_offset, 150.0, (640, 480))
            self.assertAlmostEqual(actual[0], from_image[0])
            self.assertAlmostEqual(actual[1], from_image[1])

    def test_unknown_and_reused_mapping_fail_closed(self):
        with patch.object(servos,'DROP_SERVO_SEQUENCE',(None,2)):
            self.assertFalse(servos.validate_drop_plan())
            with self.assertRaisesRegex(RuntimeError,'unknown'):
                servos.slot_for_drop_attempt(1)
        with patch.object(servos,'DROP_SERVO_SEQUENCE',(0,0)), \
             patch.object(servos,'SERVO_BODY_POSITIONS_CM',((1,2),)*4):
            self.assertEqual(servos.payload_offset_body_cm(1),(1,2))
            with self.assertRaisesRegex(RuntimeError,'reused'):
                servos.validate_drop_plan()

    def test_invalid_slot_never_writes(self):
        fc=Mock(); payload=servos.DronePayloadServos(fc)
        for slot in (None,True,-1,4,1.0):
            with self.assertRaises(ValueError):
                payload.open_slot(slot)
        fc.set_PWM_output.assert_not_called()
