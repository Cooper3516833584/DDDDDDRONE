import threading
import time
import unittest

from fleet_bus.air_node import AirFleetNode
from fleet_bus.command_queue import AirCommandQueue
from fleet_bus.models import (
    AckReason,
    AckStatus,
    AirFleetState,
    CommandId,
    CommandPayload,
    DroneGotoCommand,
    Frame,
    MessageKind,
    NodeFlags,
    NodeId,
    NodeTiming,
)
from fleet_bus.pose_provider import NavigationAirStateProvider
from fleet_bus.protocol import (
    decode_ack,
    decode_report,
    encode_command,
    encode_drone_goto,
    pack_frame,
    unpack_frame,
)


class FakeTransport:
    def __init__(self):
        self.started = False
        self.writes = []
        self.write_threads = []

    def start(self):
        self.started = True

    def stop(self):
        self.started = False

    def write(self, data):
        self.writes.append(data)
        self.write_threads.append(threading.current_thread().name)


def request(seq, command=None, session=10):
    kind = MessageKind.POLL
    payload = b"\x07\x00"
    if command is not None:
        kind = MessageKind.COMMAND
        payload = encode_command(command)
    return pack_frame(
        Frame(1, NodeId.GROUND, NodeId.DRONE, kind, 0, session, seq, payload)
    )


class AirFleetNodeTests(unittest.TestCase):
    def setUp(self):
        self.transport = FakeTransport()
        self.commands = AirCommandQueue()
        self.flight_stop = threading.Event()
        self.state = AirFleetState(
            int(NodeFlags.POSE_VALID | NodeFlags.READY),
            123,
            100,
            -200,
            300,
            9000,
            battery_cV=1234,
            pose_quality=4,
        )
        self.node = AirFleetNode(
            self.transport,
            lambda: self.state,
            self.commands,
            self.flight_stop,
            NodeTiming(turnaround_s=0.01),
        )
        self.node.start()

    def tearDown(self):
        self.node.close()

    def wait_for_writes(self, count):
        deadline = time.monotonic() + 1
        while len(self.transport.writes) < count and time.monotonic() < deadline:
            time.sleep(0.005)
        self.assertEqual(count, len(self.transport.writes))

    def test_poll_report_is_written_by_worker_after_turnaround(self):
        started = time.monotonic()
        self.node.feed_bytes(request(1))
        self.assertEqual([], self.transport.writes)
        self.wait_for_writes(1)
        self.assertGreaterEqual(time.monotonic() - started, 0.008)
        self.assertEqual(["fleet-air-node"], self.transport.write_threads)
        report = decode_report(unpack_frame(self.transport.writes[0]).payload)
        self.assertEqual(
            (10, 1, 100, -200, 300),
            (
                report.request_session,
                report.request_seq,
                report.x_cm,
                report.y_cm,
                report.z_cm,
            ),
        )

    def test_goto_is_queued_without_flight_action(self):
        body = encode_drone_goto(DroneGotoCommand(20, 30, 100, 4500))
        self.node.feed_bytes(
            request(2, CommandPayload(CommandId.DRONE_GOTO, 0, body))
        )
        self.wait_for_writes(1)
        ack = decode_ack(unpack_frame(self.transport.writes[0]).payload)
        self.assertEqual(AckStatus.ACCEPTED, ack.status)
        queued = self.commands.receive(timeout=0.1)
        self.assertEqual(20, queued.command_body.x_cm)
        self.assertFalse(self.flight_stop.is_set())

    def test_targeted_stop_only_sets_event_and_enqueues(self):
        self.node.feed_bytes(request(3, CommandPayload(CommandId.TARGETED_STOP)))
        self.wait_for_writes(1)
        self.assertTrue(self.flight_stop.is_set())
        self.assertEqual(
            CommandId.TARGETED_STOP, self.commands.receive(0.1).command_id
        )

    def test_duplicate_request_replays_identical_ack(self):
        packet = request(4, CommandPayload(CommandId.DRONE_HOLD))
        self.node.feed_bytes(packet)
        self.wait_for_writes(1)
        first = self.transport.writes[0]
        self.node.feed_bytes(packet)
        self.wait_for_writes(2)
        self.assertEqual(first, self.transport.writes[1])
        self.assertIsNotNone(self.commands.receive(0.1))
        self.assertIsNone(self.commands.receive(0.01))

    def test_new_ground_session_resets_deduplication(self):
        self.node.feed_bytes(request(8, CommandPayload(CommandId.DRONE_HOLD), 10))
        self.wait_for_writes(1)
        self.node.feed_bytes(request(8, CommandPayload(CommandId.DRONE_HOLD), 11))
        self.wait_for_writes(2)
        self.assertIsNotNone(self.commands.receive(0.1))
        self.assertIsNotNone(self.commands.receive(0.1))

    def test_close_during_turnaround_prevents_write(self):
        self.node.close()
        self.transport = FakeTransport()
        self.node = AirFleetNode(
            self.transport,
            lambda: self.state,
            self.commands,
            self.flight_stop,
            NodeTiming(turnaround_s=0.2),
        )
        self.node.start()
        self.node.feed_bytes(request(9))
        time.sleep(0.02)
        self.node.close()
        time.sleep(0.22)
        self.assertEqual([], self.transport.writes)

    def test_invalid_pose_rejects_goto_but_allows_stop(self):
        self.state = AirFleetState(int(NodeFlags.READY), 10)
        body = encode_drone_goto(DroneGotoCommand(1, 2, 3))
        self.node.feed_bytes(
            request(10, CommandPayload(CommandId.DRONE_GOTO, 0, body))
        )
        self.wait_for_writes(1)
        ack = decode_ack(unpack_frame(self.transport.writes[0]).payload)
        self.assertEqual(AckReason.LOCALIZATION_INVALID, ack.reason)
        self.node.feed_bytes(
            request(11, CommandPayload(CommandId.TARGETED_STOP))
        )
        self.wait_for_writes(2)
        self.assertTrue(self.flight_stop.is_set())

    def test_readonly_rejects_goto(self):
        self.node.close()
        self.transport = FakeTransport()
        self.node = AirFleetNode(
            self.transport,
            lambda: self.state,
            self.commands,
            self.flight_stop,
            NodeTiming(turnaround_s=0),
            readonly=True,
        )
        self.node.start()
        body = encode_drone_goto(DroneGotoCommand(1, 2, 3))
        self.node.feed_bytes(
            request(5, CommandPayload(CommandId.DRONE_GOTO, 0, body))
        )
        self.wait_for_writes(1)
        ack = decode_ack(unpack_frame(self.transport.writes[0]).payload)
        self.assertEqual(AckStatus.REJECTED, ack.status)
        self.assertEqual(AckReason.UNSUPPORTED, ack.reason)


class PoseProviderTests(unittest.TestCase):
    class Value:
        def __init__(self, value):
            self.value = value

    class State:
        bat = None
        mode = None
        unlock = None

    class FC:
        state = None

    class Navigation:
        current_x = 125
        current_y = -250
        current_height = 75
        current_yaw = 90

        def pose_is_fresh(self):
            return True

    def test_units_and_heading_direction(self):
        state = self.State()
        state.bat = self.Value(12.34)
        state.mode = self.Value(3)
        state.unlock = self.Value(1)
        fc = self.FC()
        fc.state = state
        result = NavigationAirStateProvider(fc, self.Navigation())()
        self.assertEqual((125, -250, 75), (result.x_cm, result.y_cm, result.z_cm))
        self.assertEqual(27000, result.heading_cdeg)
        self.assertEqual(1234, result.battery_cV)
        self.assertTrue(result.node_flags & int(NodeFlags.ARMED_OR_MOTOR_ACTIVE))

    def test_stale_pose_is_not_reported_as_valid(self):
        navigation = self.Navigation()
        navigation.pose_is_fresh = lambda: False
        fc = self.FC()
        fc.state = self.State()
        result = NavigationAirStateProvider(fc, navigation)()
        self.assertEqual(
            (0, 0, 0, 0),
            (result.x_cm, result.y_cm, result.z_cm, result.pose_quality),
        )
        self.assertFalse(result.node_flags & int(NodeFlags.POSE_VALID))


if __name__ == "__main__":
    unittest.main()
