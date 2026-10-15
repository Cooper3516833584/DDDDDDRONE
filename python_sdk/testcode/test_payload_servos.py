"""No FC imports or hardware; verify all user-output writes and unknown geometry."""
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

    def test_unknown_and_reused_mapping_fail_closed(self):
        self.assertFalse(servos.validate_drop_plan())
        with self.assertRaisesRegex(RuntimeError,'unknown'):
            servos.slot_for_drop_attempt(1)
        with patch.object(servos,'DROP_SERVO_SEQUENCE',(0,1,2,0)), \
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
