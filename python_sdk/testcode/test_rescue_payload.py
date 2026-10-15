"""Extract only ledger/release logic; no FlightController import or hardware."""
import ast
from pathlib import Path
import unittest
from unittest.mock import Mock
from types import SimpleNamespace


class ReleaseTests(unittest.TestCase):
    def setUp(self):
        source=Path(__file__).resolve().parents[1]/'rescue_drop_2026.py'
        tree=ast.parse(source.read_text(encoding='utf8'))
        ledger=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='DropLedger')
        mission=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='Mission')
        mission.body=[n for n in mission.body if isinstance(n,ast.FunctionDef) and n.name in ('_drop','_skip_unconfigured_drop','_free_drop','_mandatory_drop')]
        future=ast.ImportFrom(module='__future__',names=[ast.alias(name='annotations')],level=0)
        tree=ast.fix_missing_locations(ast.Module(body=[future,ledger,mission],type_ignores=[]))
        self.names=dict(TOTAL_DROP_COUNT=4,MANDATORY_DROP_COUNT=1,MANDATORY_COLOR='yellow',
            FREE_COLORS=('red','blue','green'),logger=Mock(),slot_for_drop_attempt=Mock(return_value=2),drop_plan_ready=Mock(return_value=True))
        exec(compile(tree,str(source),'exec'),self.names)
        self.ledger=self.names['DropLedger'](dict(red=1,blue=1,green=1))
        self.mission=self.names['Mission']()
        self.mission.ledger=self.ledger
        self.mission.payload=Mock()
        self.mission._check=Mock()
        self.mission._completed_free_colors=set()

    def test_four_attempts_and_quota_are_preserved(self):
        self.assertEqual([self.ledger.record_attempt(color) for color in
                         ('red','yellow','blue','green')],[1,2,3,4])
        with self.assertRaisesRegex(RuntimeError,'quota'):
            self.ledger.record_attempt('red')

    def test_release_writes_only_mapped_slot(self):
        self.mission._drop('blue','target',True)
        self.mission.payload.open_slot.assert_called_once_with(2)
        self.mission.payload.open_all.assert_not_called()
        self.assertEqual(self.ledger.remaining['blue'],0)
        with self.assertRaisesRegex(RuntimeError,'quota'):
            self.mission._drop('blue','target',True)
        self.mission.payload.open_slot.assert_called_once()

    def test_unknown_mapping_does_not_claim_a_drop(self):
        self.names['drop_plan_ready'].return_value=False
        self.mission._drop('blue','target',True)
        self.assertEqual(self.ledger.next_drop_number,2)
        self.assertEqual(self.ledger.remaining['blue'],0)
        self.assertEqual(self.ledger.skipped,[(1,'blue')])
        self.mission.payload.open_slot.assert_not_called()

    def test_failed_ipc_consumes_attempt_instead_of_claiming_retry_cargo(self):
        self.mission.payload.open_slot.side_effect=RuntimeError('IPC failed')
        with self.assertRaisesRegex(RuntimeError,'IPC failed'):
            self.mission._drop('blue','target',True)
        self.assertEqual(self.ledger.next_drop_number,2)
        self.assertEqual(self.ledger.remaining['blue'],0)

    def test_unknown_free_target_skips_approach_descent_and_release(self):
        self.names['drop_plan_ready'].return_value=False
        self.mission.target=SimpleNamespace(color='blue')
        self.mission.free_origin='route'
        self.mission._resume_route=Mock()
        self.mission._approach_target=Mock()
        self.mission._set_height=Mock()
        self.mission._free_drop()
        self.mission._resume_route.assert_called_once_with('route')
        self.mission._approach_target.assert_not_called()
        self.mission._set_height.assert_not_called()
        self.mission.payload.open_slot.assert_not_called()

    def test_unknown_yellow_skips_drop_and_continues_to_center_exit(self):
        self.names['drop_plan_ready'].return_value=False
        self.mission.target=SimpleNamespace(color='yellow')
        self.mission._navigate_center_exit=Mock()
        self.mission._mandatory_boundary_if_needed=Mock()
        self.assertTrue(self.mission._mandatory_drop())
        self.mission._mandatory_boundary_if_needed.assert_not_called()
        self.mission._navigate_center_exit.assert_called_once()
        self.assertEqual(self.ledger.skipped,[(1,'yellow')])


class RecognitionFilterTests(unittest.TestCase):
    def test_car_color_never_reaches_drone_tracking(self):
        source=Path(__file__).resolve().parents[1]/'rescue_drop_2026.py'
        tree=ast.parse(source.read_text(encoding='utf8'))
        vision=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='VisionInterface')
        vision.body=[n for n in vision.body if isinstance(n,ast.FunctionDef) and
                     n.name in ('__init__','set_allocation','poll')]
        future=ast.ImportFrom(module='__future__',names=[ast.alias(name='annotations')],level=0)
        module=ast.fix_missing_locations(ast.Module(body=[future,vision],type_ignores=[]))
        names=dict(FREE_COLORS=('red','blue','green'),MANDATORY_COLOR='yellow',logger=Mock(),
                   TARGET_CAMERA_READ_FAILURES=10,time=SimpleNamespace(monotonic=lambda:1.,time=lambda:1.))
        exec(compile(module,str(source),'exec'),names)
        instance=names['VisionInterface']()
        instance.set_allocation(dict(red=2,blue=0,green=1))
        instance._camera=Mock()
        instance._camera.read.return_value=(True,object())
        instance._detector=Mock()
        instance._detector.detect.return_value=[SimpleNamespace(color=color) for color in
                                              ('red','blue','green','yellow')]
        instance._track=Mock(return_value=())
        instance.poll()
        self.assertEqual([d.color for d in instance._track.call_args.args[0]],['red','green','yellow'])
