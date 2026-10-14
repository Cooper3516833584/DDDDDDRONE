"""Car-controlled quota/start reception without FC, ROS, camera or relay imports."""
from pathlib import Path
import sys
import threading
import unittest
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from fleet_bus.command_queue import AirCommandQueue
from fleet_bus.drop_task_protocol import DropTask, encode_drop_task, encode_drop_start
from fleet_bus.models import AirFleetState, AckReason, AckStatus, CommandId, CommandPayload, Frame, MessageKind, NodeId
from fleet_bus.protocol import decode_ack, encode_command, unpack_frame
from fleet_bus.rescue_task import RescueFleetNode, RescueTaskReceiver


class RescueLinkTests(unittest.TestCase):
    def setUp(self):
        self.stop_event = threading.Event()
        self.receiver = RescueTaskReceiver(self.stop_event, allow_start=True)
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

    def test_counts_saved_before_start_no_localization_required(self):
        task = DropTask(42, 0, 1, 3)
        self.assertEqual(self.request(CommandId.DRONE_SET_DROP_TASK, encode_drop_task(task)).status, AckStatus.COMPLETED)
        self.assertEqual(self.request(CommandId.DRONE_START_DROP_TASK, encode_drop_start(42)).status, AckStatus.ACCEPTED)
        self.assertEqual(self.receiver.wait_for_start(.01).allocation(), dict(red=0, blue=1, green=3))

    def test_missing_mismatched_task_or_controller_session_never_starts(self):
        for session, task_id in ((123, 42), (124, 41)):
            ack = self.request(CommandId.DRONE_START_DROP_TASK, encode_drop_start(task_id), session=session)
            self.assertEqual((ack.status, ack.reason), (AckStatus.REJECTED, AckReason.NOT_READY))
        self.request(CommandId.DRONE_SET_DROP_TASK, encode_drop_task(DropTask(41, 2, 1, 1)))
        self.assertEqual(self.request(CommandId.DRONE_START_DROP_TASK, encode_drop_start(42)).status, AckStatus.REJECTED)
        self.assertEqual(self.request(CommandId.DRONE_START_DROP_TASK, encode_drop_start(41), session=124).status, AckStatus.REJECTED)
        with self.assertRaises(TimeoutError):
            self.receiver.wait_for_start(.001)

    def test_ground_station_cannot_directly_supply_tasks(self):
        self.assertIsNone(self.request(CommandId.DRONE_SET_DROP_TASK,
                         encode_drop_task(DropTask(42, 1, 2, 1)), source=NodeId.GROUND))

    def test_duplicate_task_and_start_do_not_reinitialize_active_task(self):
        body = encode_drop_task(DropTask(42, 1, 2, 1))
        ack1 = self.request(CommandId.DRONE_SET_DROP_TASK, body, seq=10)
        self.assertEqual(self.request(CommandId.DRONE_SET_DROP_TASK, body, seq=10), ack1)
        self.request(CommandId.DRONE_START_DROP_TASK, encode_drop_start(42))
        task = self.receiver.wait_for_start(.01)
        self.assertEqual(self.request(CommandId.DRONE_START_DROP_TASK, encode_drop_start(42)).status, AckStatus.ACCEPTED)
        self.assertIs(self.receiver.wait_for_start(.01), task)
        self.assertEqual(self.request(CommandId.DRONE_SET_DROP_TASK,
                         encode_drop_task(DropTask(43, 2, 1, 1))).reason, AckReason.BUSY)

    def test_bad_data_and_same_id_changed_quota_rejected(self):
        for data in (b"", b"\x01\x2a\0\0\0\x01\x01\x01", b"\x02\x2a\0\0\0\x02\x01\x01"):
            self.assertEqual(self.request(CommandId.DRONE_SET_DROP_TASK, data).reason, AckReason.BAD_PAYLOAD)
        self.request(CommandId.DRONE_SET_DROP_TASK, encode_drop_task(DropTask(42, 2, 1, 1)))
        self.assertEqual(self.request(CommandId.DRONE_SET_DROP_TASK,
                         encode_drop_task(DropTask(42, 1, 2, 1))).reason, AckReason.BAD_PAYLOAD)

    def test_stop_before_launch_cancels_and_monitor_mode_refuses_start(self):
        self.request(CommandId.DRONE_SET_DROP_TASK, encode_drop_task(DropTask(42, 2, 1, 1)))
        self.request(CommandId.DRONE_START_DROP_TASK, encode_drop_start(42))
        self.assertEqual(self.request(CommandId.TARGETED_STOP).status, AckStatus.ACCEPTED)
        with self.assertRaises(RuntimeError):
            self.receiver.wait_for_start(.01)
        receiver = RescueTaskReceiver(threading.Event())
        receiver.set_task(1, DropTask(1, 1, 1, 2))
        self.assertEqual(receiver.request_start(1, 1), (AckStatus.REJECTED, AckReason.NOT_READY))


if __name__ == "__main__":
    unittest.main()
