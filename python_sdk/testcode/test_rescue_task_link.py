"""Car-controlled quota/start reception without FC, ROS, camera or relay imports."""
from pathlib import Path
import sys
import threading
import unittest
from unittest.mock import Mock, patch
import struct
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from fleet_bus.command_queue import AirCommandQueue
from fleet_bus.drop_task_protocol import DropTask, encode_drop_task, encode_drop_start
from fleet_bus.models import AirFleetState, AckReason, AckStatus, CommandId, CommandPayload, Frame, MessageKind, NodeId
from fleet_bus.protocol import decode_ack, encode_command, unpack_frame
from fleet_bus.rescue_task import RescueFleetNode, RescueTaskReceiver


class RescueLinkTests(unittest.TestCase):
    def setUp(self):
        self.stop_event = threading.Event()
        self.payload = Mock()
        self.fc = SimpleNamespace(connected=True, state=SimpleNamespace(
            is_fresh=lambda age: True, unlock=SimpleNamespace(value=False)))
        self.navi = Mock(running=True)
        self.receiver = RescueTaskReceiver(self.stop_event, allow_start=True, payload=self.payload,
            fc=self.fc, navigation=self.navi, validate_plan=lambda: None)
        self.node = RescueFleetNode(Mock(), lambda: AirFleetState(0, 0), AirCommandQueue(),
                                   self.stop_event, receiver=self.receiver)
        self.seq = 0

    def request(self, command, body=b"", *, source=NodeId.CAR, session=123, seq=None):
        self.seq += 1
        frame = Frame(1, source, NodeId.DRONE, MessageKind.COMMAND, 0, session,
                      self.seq if seq is None else seq, encode_command(CommandPayload(command, command_body=body)))
        response = self.node._handle(frame)
        if response is None:
            return None
        frame = unpack_frame(response)
        self.assertEqual(frame.dst, source)
        return decode_ack(frame.payload)

    def ready(self):
        self.assertEqual(self.receiver.close_payload(123, 7)[0], AckStatus.COMPLETED)
        self.receiver.remap_state = 'ready'

    def test_counts_saved_before_start_no_localization_required(self):
        self.ready()
        task = DropTask(42, 0, 0, 1)
        self.assertEqual(self.request(CommandId.DRONE_SET_DROP_TASK, encode_drop_task(task)).status, AckStatus.COMPLETED)
        self.assertEqual(self.request(CommandId.DRONE_START_DROP_TASK, encode_drop_start(42)).status, AckStatus.ACCEPTED)
        self.assertEqual(self.receiver.wait_for_start(.01).allocation(), dict(red=0, blue=0, green=1))

    def test_missing_mismatched_task_or_controller_session_never_starts(self):
        for session, task_id in ((123, 42), (124, 41)):
            ack = self.request(CommandId.DRONE_START_DROP_TASK, encode_drop_start(task_id), session=session)
            self.assertEqual((ack.status, ack.reason), (AckStatus.REJECTED, AckReason.NOT_READY))
        self.request(CommandId.DRONE_SET_DROP_TASK, encode_drop_task(DropTask(41, 1, 0, 0)))
        self.assertEqual(self.request(CommandId.DRONE_START_DROP_TASK, encode_drop_start(42)).status, AckStatus.REJECTED)
        self.assertEqual(self.request(CommandId.DRONE_START_DROP_TASK, encode_drop_start(41), session=124).status, AckStatus.REJECTED)
        with self.assertRaises(TimeoutError):
            self.receiver.wait_for_start(.001)

    def test_ground_station_cannot_directly_supply_tasks(self):
        self.assertIsNone(self.request(CommandId.DRONE_SET_DROP_TASK,
                         encode_drop_task(DropTask(42, 0, 1, 0)), source=NodeId.GROUND))

    def test_duplicate_task_and_start_do_not_reinitialize_active_task(self):
        self.ready()
        body = encode_drop_task(DropTask(42, 0, 1, 0))
        ack1 = self.request(CommandId.DRONE_SET_DROP_TASK, body, seq=10)
        self.assertEqual(self.request(CommandId.DRONE_SET_DROP_TASK, body, seq=10), ack1)
        self.request(CommandId.DRONE_START_DROP_TASK, encode_drop_start(42))
        task = self.receiver.wait_for_start(.01)
        self.assertEqual(self.request(CommandId.DRONE_START_DROP_TASK, encode_drop_start(42)).status, AckStatus.ACCEPTED)
        self.assertIs(self.receiver.wait_for_start(.01), task)
        self.assertEqual(self.request(CommandId.DRONE_SET_DROP_TASK,
                         encode_drop_task(DropTask(43, 1, 0, 0))).reason, AckReason.BUSY)

    def test_bad_data_and_same_id_changed_quota_rejected(self):
        for data in (b"", struct.pack("<BI3B",1,42,0,0,1),
                     struct.pack("<BI3B",2,42,2,0,1), struct.pack("<BI3B",2,42,0,0,0)):
            self.assertEqual(self.request(CommandId.DRONE_SET_DROP_TASK, data).reason, AckReason.BAD_PAYLOAD)
        self.request(CommandId.DRONE_SET_DROP_TASK, encode_drop_task(DropTask(42, 1, 0, 0)))
        self.assertEqual(self.request(CommandId.DRONE_SET_DROP_TASK,
                         encode_drop_task(DropTask(42, 0, 1, 0))).reason, AckReason.BAD_PAYLOAD)

    def test_stop_before_launch_cancels_and_monitor_mode_refuses_start(self):
        self.request(CommandId.DRONE_SET_DROP_TASK, encode_drop_task(DropTask(42, 1, 0, 0)))
        self.request(CommandId.DRONE_START_DROP_TASK, encode_drop_start(42))
        self.assertEqual(self.request(CommandId.TARGETED_STOP).status, AckStatus.ACCEPTED)
        with self.assertRaises(RuntimeError):
            self.receiver.wait_for_start(.01)
        receiver = RescueTaskReceiver(threading.Event())
        receiver.set_task(1, DropTask(1, 0, 1, 0))
        self.assertEqual(receiver.request_start(1, 1), (AckStatus.REJECTED, AckReason.NOT_READY))

    def test_run_body_authorization_and_late_close_after_stop(self):
        for command in (CommandId.DRONE_PAYLOAD_CLOSE, CommandId.DRONE_REBUILD_MAP):
            for body in (b'', b'\x01', struct.pack('<BI', 2, 7), struct.pack('<BI', 1, 0)):
                self.assertEqual(self.request(command, body).reason, AckReason.BAD_PAYLOAD)
            self.assertIsNone(self.request(command, struct.pack('<BI', 1, 7), source=NodeId.GROUND))
        self.assertEqual(self.request(CommandId.DRONE_REBUILD_MAP, struct.pack('<BI',1,7)).status,
                         AckStatus.REJECTED)
        self.assertEqual(self.request(CommandId.DRONE_PAYLOAD_CLOSE, struct.pack('<BI',1,7)).status,
                         AckStatus.COMPLETED)
        self.request(CommandId.TARGETED_STOP)
        self.payload.open_all.assert_not_called()
        self.assertEqual(self.request(CommandId.DRONE_PAYLOAD_CLOSE, struct.pack('<BI',1,7)).status,
                         AckStatus.REJECTED)
        self.assertEqual(self.payload.close_all.call_count,1)

    def test_one_async_remap_cached_ack_and_new_seq_status_queries(self):
        body = struct.pack('<BI',1,7)
        self.request(CommandId.DRONE_PAYLOAD_CLOSE,body)
        entered, release = threading.Event(), threading.Event()
        def rebuild(*args, **kwargs):
            entered.set()
            self.assertTrue(release.wait(1))
            return dict(needs_calibration=False, pose={'stamp_ns':123})
        with patch('fastlio_control.restart_and_calibrate_fastlio',side_effect=rebuild) as restart:
            ack = self.request(CommandId.DRONE_REBUILD_MAP,body,seq=100)
            self.assertEqual(ack.status,AckStatus.ACCEPTED)
            self.assertTrue(entered.wait(1))
            self.assertEqual(self.request(CommandId.DRONE_REBUILD_MAP,body).status,AckStatus.ACCEPTED)
            release.set()
            self.assertTrue(self.receiver.join_remap(1))
            self.assertEqual(self.request(CommandId.DRONE_REBUILD_MAP,body).status,AckStatus.COMPLETED)
            self.assertEqual(self.request(CommandId.DRONE_REBUILD_MAP,body,seq=100),ack)
            restart.assert_called_once_with(self.fc,self.navi,timeout=45)

    def test_operator_preflight_car_commands_only_acknowledge_completed_work(self):
        self.payload.connected = True
        self.payload.get_states.return_value = {channel: False for channel in range(1, 5)}
        self.navi.pose_is_fresh.return_value = True
        self.navi.height_is_fresh.return_value = True
        result = dict(needs_calibration=False, pose={'stamp_ns': 123})
        self.receiver = RescueTaskReceiver(self.stop_event, allow_start=True,
            payload=self.payload, fc=self.fc, navigation=self.navi,
            validate_plan=lambda: None, preflight_result=result)
        self.node.receiver = self.receiver
        body = struct.pack('<BI', 1, 7)
        self.assertEqual(self.request(CommandId.DRONE_SET_DROP_TASK,
                         encode_drop_task(DropTask(42, 1, 0, 0))).status, AckStatus.COMPLETED)
        with patch('fastlio_control.restart_and_calibrate_fastlio') as restart:
            self.assertEqual(self.request(CommandId.DRONE_PAYLOAD_CLOSE, body).status,
                             AckStatus.COMPLETED)
            self.assertEqual(self.request(CommandId.DRONE_REBUILD_MAP, body).status,
                             AckStatus.ACCEPTED)
            self.assertEqual(self.request(CommandId.DRONE_REBUILD_MAP, body).status,
                             AckStatus.COMPLETED)
            self.assertEqual(self.request(CommandId.DRONE_START_DROP_TASK,
                             encode_drop_start(42)).status, AckStatus.ACCEPTED)
            restart.assert_not_called()
        self.payload.close_all.assert_not_called()
        self.payload.open_all.assert_not_called()
        self.payload.get_states.assert_called_once_with()
        self.assertIs(self.receiver.remap_result, result)

    def test_operator_preflight_refuses_open_payload_or_stale_pose(self):
        self.payload.connected = True
        self.payload.get_states.return_value = {1: True, 2: False, 3: False, 4: False}
        self.navi.pose_is_fresh.return_value = True
        self.navi.height_is_fresh.return_value = True
        self.receiver = RescueTaskReceiver(self.stop_event, allow_start=True,
            payload=self.payload, fc=self.fc, navigation=self.navi,
            validate_plan=lambda: None,
            preflight_result=dict(needs_calibration=False, pose={'stamp_ns': 123}))
        self.node.receiver = self.receiver
        self.receiver.set_task(123, DropTask(42, 1, 0, 0))
        self.assertEqual(self.receiver.close_payload(123, 7)[0], AckStatus.FAILED)
        self.assertEqual(self.receiver.request_start(123, 42)[0], AckStatus.REJECTED)
        self.payload.get_states.return_value = {channel: False for channel in range(1, 5)}
        self.receiver = RescueTaskReceiver(self.stop_event, allow_start=True,
            payload=self.payload, fc=self.fc, navigation=self.navi,
            validate_plan=lambda: None,
            preflight_result=dict(needs_calibration=False, pose={'stamp_ns': 123}))
        self.receiver.set_task(123, DropTask(42, 1, 0, 0))
        self.assertEqual(self.receiver.close_payload(123, 7)[0], AckStatus.COMPLETED)
        self.navi.pose_is_fresh.return_value = False
        self.assertEqual(self.receiver.rebuild_map(123, 7),
                         (AckStatus.REJECTED, AckReason.NOT_READY))
        self.navi.pose_is_fresh.side_effect = RuntimeError('pose unavailable')
        self.assertEqual(self.receiver.rebuild_map(123, 7),
                         (AckStatus.FAILED, AckReason.INTERNAL_ERROR))
        self.navi.pose_is_fresh.side_effect = None
        self.navi.pose_is_fresh.return_value = True
        self.assertEqual(self.receiver.rebuild_map(123, 7)[0], AckStatus.ACCEPTED)
        self.navi.height_is_fresh.return_value = False
        self.assertEqual(self.receiver.request_start(123, 42)[0], AckStatus.REJECTED)

    def test_stop_during_remap_cannot_become_ready_or_close_again(self):
        self.ready()
        self.receiver.remap_state='idle'
        entered, release=threading.Event(),threading.Event()
        def rebuild(*args,**kwargs):
            entered.set(); release.wait(1)
            return dict(needs_calibration=False,pose={'stamp_ns':123})
        with patch('fastlio_control.restart_and_calibrate_fastlio',side_effect=rebuild):
            self.receiver.rebuild_map(123,7)
            self.assertTrue(entered.wait(1))
            self.receiver.stop()
            release.set()
            self.assertTrue(self.receiver.join_remap(1))
        self.assertEqual(self.receiver.remap_state,'cancelled')
        self.assertEqual(self.receiver.close_payload(123,7)[0],AckStatus.REJECTED)

    def test_fastlio_instance_or_calibration_failure_never_becomes_ready(self):
        for failure in (RuntimeError('new FAST-LIO instance unavailable'),
                        RuntimeError('calibration timed out')):
            with self.subTest(failure=str(failure)):
                receiver = RescueTaskReceiver(threading.Event(), allow_start=True,
                    payload=self.payload, fc=self.fc, navigation=self.navi,
                    validate_plan=lambda: None)
                self.assertEqual(receiver.close_payload(123, 7)[0], AckStatus.COMPLETED)
                receiver.set_task(123, DropTask(42, 1, 0, 0))
                with patch('fastlio_control.restart_and_calibrate_fastlio',
                           side_effect=failure):
                    self.assertEqual(receiver.rebuild_map(123, 7)[0], AckStatus.ACCEPTED)
                    self.assertTrue(receiver.join_remap(1))
                self.assertEqual(receiver.remap_state, 'failed')
                self.assertEqual(receiver.request_start(123, 42),
                                 (AckStatus.REJECTED, AckReason.NOT_READY))

    def test_start_requires_current_run_ready_and_physical_mapping(self):
        self.receiver.set_task(123,DropTask(42,1,0,0))
        self.assertEqual(self.receiver.request_start(123,42)[0],AckStatus.REJECTED)
        self.ready()
        self.receiver._validate_plan=Mock(side_effect=RuntimeError('mapping unknown'))
        self.assertEqual(self.receiver.request_start(123,42)[0],AckStatus.REJECTED)
        self.receiver._validate_plan=lambda:None
        self.assertEqual(self.receiver.request_start(123,42)[0],AckStatus.ACCEPTED)

    def test_unknown_installation_allows_start_with_release_skipped(self):
        self.ready()
        self.receiver.set_task(123,DropTask(42,1,0,0))
        self.receiver._validate_plan=lambda:False
        self.assertEqual(self.receiver.request_start(123,42)[0],AckStatus.ACCEPTED)

    def test_close_failure_and_stale_run_never_report_ready(self):
        self.payload.close_all.side_effect=RuntimeError('IPC failed')
        self.assertEqual(self.receiver.close_payload(123,7)[0],AckStatus.FAILED)
        self.assertEqual(self.receiver.rebuild_map(123,7)[0],AckStatus.REJECTED)
        self.assertEqual(self.receiver.close_payload(124,8)[0],AckStatus.REJECTED)


if __name__ == "__main__":
    unittest.main()
