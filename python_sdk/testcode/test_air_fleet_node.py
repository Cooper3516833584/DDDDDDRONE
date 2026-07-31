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
    SurveyState,
    TraceReportFlags,
    TraceRequestPayload,
    TraceSample,
)
from fleet_bus.pose_provider import NavigationAirStateProvider
from fleet_bus.protocol import (
    ProtocolError,
    decode_ack,
    decode_report,
    decode_survey_report,
    decode_trace_report,
    encode_command,
    encode_drone_goto,
    encode_trace_request,
    pack_frame,
    unpack_frame,
)
from fleet_bus.trace_buffer import TraceSamplingOptions


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


class FailOnceTransport(FakeTransport):
    def __init__(self):
        super().__init__()
        self.failures_remaining = 1

    def write(self, data):
        if self.failures_remaining:
            self.failures_remaining -= 1
            raise RuntimeError("simulated FC ACK timeout")
        super().write(data)


def request(seq, command=None, session=10):
    kind = MessageKind.POLL
    payload = b"\x07\x00"
    if command is not None:
        kind = MessageKind.COMMAND
        payload = encode_command(command)
    return pack_frame(
        Frame(1, NodeId.GROUND, NodeId.DRONE, kind, 0, session, seq, payload)
    )


def trace_request(seq, value=None, session=10, payload=None):
    if payload is None:
        payload = encode_trace_request(
            value or TraceRequestPayload(0, 0, 15, 0)
        )
    return pack_frame(
        Frame(
            1,
            NodeId.GROUND,
            NodeId.DRONE,
            MessageKind.TRACE_REQUEST,
            0,
            session,
            seq,
            payload,
        )
    )


class AirFleetNodeTests(unittest.TestCase):
    def test_default_turnaround_supports_dense_pose_polling(self):
        self.assertAlmostEqual(0.10, NodeTiming().turnaround_s)

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

    def test_transport_write_failure_does_not_stop_reply_worker(self):
        self.node.close()
        self.transport = FailOnceTransport()
        self.node = AirFleetNode(
            self.transport,
            lambda: self.state,
            self.commands,
            self.flight_stop,
            NodeTiming(turnaround_s=0),
        )
        self.node.start()

        self.node.feed_bytes(request(30))
        deadline = time.monotonic() + 1
        while self.node.write_failures < 1 and time.monotonic() < deadline:
            time.sleep(0.005)
        self.assertEqual(1, self.node.write_failures)

        self.node.feed_bytes(request(31))
        self.wait_for_writes(1)
        report = decode_report(unpack_frame(self.transport.writes[0]).payload)
        self.assertEqual(31, report.request_seq)

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

        self.node.feed_bytes(
            request(6, CommandPayload(CommandId.TARGETED_STOP))
        )
        self.wait_for_writes(2)
        stop_ack = decode_ack(unpack_frame(self.transport.writes[1]).payload)
        self.assertEqual(AckStatus.COMPLETED, stop_ack.status)
        self.assertTrue(self.flight_stop.is_set())

    def test_readonly_task_can_explicitly_allow_start_without_pose(self):
        self.node.close()
        self.state = AirFleetState(int(NodeFlags.READY), 10)
        self.transport = FakeTransport()
        self.node = AirFleetNode(
            self.transport,
            lambda: self.state,
            self.commands,
            self.flight_stop,
            NodeTiming(turnaround_s=0),
            readonly=True,
            allow_start_mission=True,
        )
        self.node.start()
        self.node.feed_bytes(
            request(7, CommandPayload(CommandId.DRONE_START_MISSION))
        )
        self.wait_for_writes(1)
        ack = decode_ack(unpack_frame(self.transport.writes[0]).payload)
        self.assertEqual(AckStatus.ACCEPTED, ack.status)
        queued = self.commands.receive(timeout=0.1)
        self.assertEqual(CommandId.DRONE_START_MISSION, queued.command_id)

    def test_readonly_task_can_prepare_payload_without_pose(self):
        self.node.close()
        self.state = AirFleetState(int(NodeFlags.READY), 10)
        self.transport = FakeTransport()
        self.node = AirFleetNode(
            self.transport,
            lambda: self.state,
            self.commands,
            self.flight_stop,
            NodeTiming(turnaround_s=0),
            readonly=True,
            allow_start_mission=True,
        )
        self.node.start()
        self.node.feed_bytes(
            request(13, CommandPayload(CommandId.DRONE_PREPARE_MISSION))
        )
        self.wait_for_writes(1)
        ack = decode_ack(unpack_frame(self.transport.writes[0]).payload)
        self.assertEqual(AckStatus.ACCEPTED, ack.status)
        queued = self.commands.receive(timeout=0.1)
        self.assertEqual(CommandId.DRONE_PREPARE_MISSION, queued.command_id)

    def test_survey_request_returns_task_snapshot(self):
        self.node.close()
        self.transport = FakeTransport()
        terrain = (1,) * 14 + (7,)
        positions = tuple(
            (115 + 70 * col, 175 + 70 * row)
            for row in range(3)
            for col in range(5)
        )
        self.node = AirFleetNode(
            self.transport,
            lambda: self.state,
            self.commands,
            self.flight_stop,
            NodeTiming(turnaround_s=0),
            survey_provider=lambda: SurveyState(
                survey_revision=9,
                survey_flags=3,
                wildfire_event_id=3,
                wildfire_row=2,
                wildfire_col=4,
                terrain_codes=terrain,
                cell_positions_cm=positions,
            ),
        )
        self.node.start()
        self.node.feed_bytes(
            pack_frame(
                Frame(
                    1, NodeId.GROUND, NodeId.DRONE,
                    MessageKind.SURVEY_REQUEST, 0, 10, 12, b"",
                )
            )
        )
        self.wait_for_writes(1)
        survey = decode_survey_report(
            unpack_frame(self.transport.writes[0]).payload
        )
        self.assertEqual((10, 12, 9, 3, 2, 4), (
            survey.request_session,
            survey.request_seq,
            survey.survey_revision,
            survey.wildfire_event_id,
            survey.wildfire_row,
            survey.wildfire_col,
        ))
        self.assertEqual((395, 315), survey.cell_positions_cm[-1])

    def test_trace_request_returns_legal_empty_report(self):
        self.node.feed_bytes(trace_request(20))
        self.wait_for_writes(1)
        frame = unpack_frame(self.transport.writes[0])
        self.assertEqual(MessageKind.TRACE_REPORT, frame.kind)
        report = decode_trace_report(frame.payload)
        self.assertEqual((10, 20), (report.request_session, report.request_seq))
        self.assertNotEqual(0, report.trace_session)
        self.assertEqual(
            (0, 0, 0, ()),
            (
                report.oldest_available_seq,
                report.first_sample_seq,
                report.latest_available_seq,
                report.samples,
            ),
        )

    def test_trace_request_batches_without_control_and_replays_cached_bytes(self):
        self.node.trace_buffer.record(
            TraceSample(100, 1, 2, 3, 400, 4, 1)
        )
        self.node.trace_buffer.record(
            TraceSample(200, 5, 6, 7, 500, 3, 1)
        )
        packet = trace_request(21)
        self.node.feed_bytes(packet)
        self.wait_for_writes(1)
        first_response = self.transport.writes[0]
        report = decode_trace_report(unpack_frame(first_response).payload)
        self.assertEqual(2, len(report.samples))
        self.assertTrue(
            report.report_flags & int(TraceReportFlags.CURSOR_RESET)
        )
        self.assertIsNone(self.commands.receive(0.01))
        self.assertFalse(self.flight_stop.is_set())

        self.node.trace_buffer.record(
            TraceSample(300, 9, 10, 11, 600, 2, 1)
        )
        self.node.feed_bytes(packet)
        self.wait_for_writes(2)
        self.assertEqual(first_response, self.transport.writes[1])

    def test_invalid_trace_sample_is_rejected_before_it_can_poison_buffer(self):
        with self.assertRaises(ProtocolError):
            self.node.trace_buffer.record(
                TraceSample(100, 1, 2, 3, 36000, 4, 1)
            )
        self.assertEqual(0, self.node.trace_buffer.sample_count)
        self.node.trace_buffer.record(
            TraceSample(200, 1, 2, 3, 100, 4, 1)
        )
        self.node.feed_bytes(trace_request(24))
        self.wait_for_writes(1)
        report = decode_trace_report(
            unpack_frame(self.transport.writes[0]).payload
        )
        self.assertEqual(1, len(report.samples))

    def test_invalid_trace_request_does_not_stop_worker(self):
        self.node.feed_bytes(trace_request(22, payload=b""))
        self.node.feed_bytes(request(23))
        self.wait_for_writes(1)
        response = unpack_frame(self.transport.writes[0])
        self.assertEqual(MessageKind.REPORT, response.kind)

    def test_enabled_trace_sampler_starts_once_and_stops_on_close(self):
        self.node.close()
        self.transport = FakeTransport()
        sample_number = [0]

        def changing_state():
            sample_number[0] += 1
            return AirFleetState(
                int(NodeFlags.POSE_VALID | NodeFlags.READY),
                sample_number[0] * 10,
                sample_number[0] * 2,
                0,
                0,
                0,
                pose_quality=4,
            )

        self.node = AirFleetNode(
            self.transport,
            changing_state,
            self.commands,
            self.flight_stop,
            NodeTiming(turnaround_s=0),
            trace_options=TraceSamplingOptions(
                enabled=True,
                sample_interval_s=0.01,
                buffer_capacity=20,
            ),
        )
        self.node.start()
        sampler = self.node.trace_sampler
        self.assertIsNotNone(sampler)
        deadline = time.monotonic() + 0.5
        while self.node.trace_buffer.sample_count < 2 and time.monotonic() < deadline:
            time.sleep(0.005)
        self.assertGreaterEqual(self.node.trace_buffer.sample_count, 2)
        self.node.start()
        self.assertTrue(sampler.running)
        self.node.close()
        self.assertFalse(sampler.running)
        self.node.close()


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

    def test_task_transform_reports_field_absolute_pose(self):
        fc = self.FC()
        fc.state = self.State()
        result = NavigationAirStateProvider(
            fc,
            self.Navigation(),
            position_transform=lambda x, y: (75 - y, 75 + x),
            heading_offset_deg=90,
        )()
        self.assertEqual((325, 200), (result.x_cm, result.y_cm))
        self.assertEqual(0, result.heading_cdeg)


if __name__ == "__main__":
    unittest.main()
