import threading
import time
import unittest

from FlightController.Components.GroundStationLink import (
    CommandId,
    FCStatePayload,
    GroundLinkMode,
    GroundStationLink,
    LEDControl,
    LEDMode,
    MessageType,
)
from FlightController.Components.GroundStationLink.models import Command
from FlightController.Components.GroundStationLink.protocol import (
    FrameParser,
    pack_frame,
)
from FlightController.Components.GroundStationLink.transport import FCWirelessTransport


KEY = bytes.fromhex("00112233445566778899aabbccddeeff")


class FakeTransport:
    def __init__(self, **kwargs):
        self.on_bytes = kwargs["on_bytes"]
        self.on_connected = kwargs["on_connected"]
        self.on_disconnected = kwargs["on_disconnected"]
        self.writes = []
        self.connected = True

    def start(self):
        self.on_connected()

    def stop(self):
        self.connected = False

    def write(self, data):
        self.writes.append(data)


class FakeFC:
    def __init__(self):
        self.connected = True
        self.callback = None
        self.writes = []

    def register_wireless_callback(self, callback, threaded=False):
        self.callback = callback

    def send_to_wireless(self, data):
        self.writes.append(data)


class GroundStationLinkTests(unittest.TestCase):
    def make_link(self, stop_event=None):
        link = GroundStationLink(
            fc=None,
            key=KEY,
            stop_event=stop_event,
            turnaround_seconds=0.0,
            transport_factory=FakeTransport,
        )
        link._on_connected()
        link._on_bytes(
            pack_frame(MessageType.HEARTBEAT, b"\x01", 55, 1, KEY)
        )
        return link

    def test_fc_state_payload_contains_compact_core(self):
        state = FCStatePayload(
            1000, -2000, 11.85, 3, True,
        )
        payload = state.to_payload()
        self.assertEqual(len(payload), 13)
        self.assertEqual(FCStatePayload.from_payload(payload), state)

    def test_fc_wireless_transport_uses_existing_fc_bridge(self):
        fc = FakeFC()
        received = []
        connected = threading.Event()
        transport = FCWirelessTransport(
            fc=fc,
            on_bytes=received.append,
            on_connected=connected.set,
            monitor_seconds=0.001,
        )
        transport.start()
        try:
            self.assertTrue(connected.wait(0.1))
            transport.write(b"ground-link-frame")
            self.assertEqual(fc.writes, [b"ground-link-frame"])
            fc.callback(b"ground-link-reply")
            self.assertEqual(received, [b"ground-link-reply"])
        finally:
            transport.stop()

    def test_fc_wireless_transport_rejects_oversized_payload(self):
        fc = FakeFC()
        transport = FCWirelessTransport(fc=fc, on_bytes=lambda data: None)
        transport.start()
        try:
            deadline = time.monotonic() + 0.1
            while not transport.connected and time.monotonic() < deadline:
                time.sleep(0.001)
            with self.assertRaises(ValueError):
                transport.write(b"x" * 256)
        finally:
            transport.stop()

    def test_duplicate_command_is_queued_once(self):
        link = self.make_link()
        frame = pack_frame(
            MessageType.COMMAND,
            Command(CommandId.SET_TARGETS, 2, 9).to_payload(),
            55,
            7,
            KEY,
        )
        link._on_bytes(frame[:3])
        link._on_bytes(frame[3:])
        link._on_bytes(frame)
        command = link.get_command(timeout=0.01)
        self.assertIsNotNone(command)
        self.assertEqual(command.command.command_id, CommandId.SET_TARGETS)
        self.assertIsNone(link.get_command(timeout=0.01))

    def test_stop_different_sequences_sets_event_and_queues_once(self):
        stop_event = threading.Event()
        link = self.make_link(stop_event)
        for seq in (8, 9, 10):
            link._on_bytes(
                pack_frame(
                    MessageType.COMMAND,
                    Command(CommandId.STOP_MISSION).to_payload(),
                    55,
                    seq,
                    KEY,
                )
            )
        self.assertTrue(stop_event.is_set())
        self.assertEqual(link.get_command(timeout=0.01).seq, 8)
        self.assertIsNone(link.get_command(timeout=0.01))

    def test_corrupted_frame_is_rejected(self):
        parser = FrameParser(KEY)
        frame = bytearray(pack_frame(MessageType.HEARTBEAT, b"ok", 1, 2, KEY))
        frame[15] ^= 0x40
        self.assertEqual(parser.feed(bytes(frame)), [])
        self.assertEqual(parser.stats.checksum_failures, 1)

    def test_frame_uses_base_outer_format(self):
        frame = pack_frame(MessageType.HEARTBEAT, b"ok", 1, 2, KEY)
        self.assertEqual(frame[:2], b"\xAA\x22")
        self.assertEqual(frame[2], MessageType.HEARTBEAT)
        self.assertEqual(frame[3], len(frame) - 5)
        self.assertEqual(frame[-1], sum(frame[:-1]) & 0xFF)

    def test_default_telemetry_rate_is_twenty_hz(self):
        link = GroundStationLink(
            fc=None,
            key=KEY,
            transport_factory=FakeTransport,
            turnaround_seconds=0.0,
        )
        self.assertAlmostEqual(link._telemetry_period, 0.05)

    def test_command_mode_accepts_commands_but_does_not_transmit_telemetry(self):
        link = self.make_link()
        link._state_provider = lambda: FCStatePayload(
            1000, -2000, 11.85, 3, True
        ).to_payload()
        self.assertEqual(link.mode, GroundLinkMode.COMMAND_RX)
        self.assertFalse(link.send_fc_state_now())
        self.assertFalse(
            link.report_mission_status(
                4, progress=10, message="must not send before takeoff"
            )
        )

    def test_telemetry_mode_sends_state_ignores_ping_but_accepts_stop(self):
        stop_event = threading.Event()
        link = self.make_link(stop_event)
        link._state_provider = lambda: FCStatePayload(
            1000, -2000, 11.85, 3, True
        ).to_payload()
        link.enable_telemetry_transmission()
        self.assertEqual(link.mode, GroundLinkMode.TELEMETRY_TX)
        for _ in range(3):
            self.assertTrue(link.send_fc_state_now())
        parser = FrameParser(KEY)
        states = parser.feed(link._transport.writes[-1])
        self.assertEqual(len(states), 3)
        state = states[-1]
        self.assertEqual(state.msg_type, MessageType.FC_STATE)
        self.assertEqual(state.session, link.session)
        self.assertEqual(FCStatePayload.from_payload(state.payload).pos_x_cm, 1000)

        before = len(link._transport.writes)
        link._on_bytes(
            pack_frame(
                MessageType.COMMAND,
                Command(CommandId.PING).to_payload(),
                55,
                10,
                KEY,
            )
        )
        self.assertEqual(len(link._transport.writes), before)
        self.assertIsNone(link.get_command(timeout=0.01))

        link._on_bytes(
            pack_frame(
                MessageType.COMMAND,
                Command(CommandId.STOP_MISSION).to_payload(),
                55,
                11,
                KEY,
            )
        )
        self.assertTrue(stop_event.is_set())
        self.assertEqual(
            link.get_command(timeout=0.01).command.command_id,
            CommandId.STOP_MISSION,
        )

    def test_switching_back_to_command_mode_discards_partial_telemetry_batch(self):
        link = self.make_link()
        link._state_provider = lambda: FCStatePayload(
            1000, -2000, 11.85, 3, True
        ).to_payload()
        link.enable_telemetry_transmission()
        link.send_fc_state_now()
        link.send_fc_state_now()
        self.assertEqual(link._transport.writes, [])

        link.enable_command_reception()
        link.enable_telemetry_transmission()
        for _ in range(3):
            link.send_fc_state_now()
        frames = FrameParser(KEY).feed(link._transport.writes[-1])
        self.assertEqual(len(frames), 3)

    def test_led_control_requires_telemetry_mode(self):
        link = self.make_link()
        control = LEDControl(LEDMode.PIXELS, brightness=4, pixels=((0, 255, 0),) * 7)
        self.assertFalse(link.report_led_control(control))
        link.enable_telemetry_transmission()
        self.assertTrue(link.report_led_control(control))
        frame = FrameParser(KEY).feed(link._transport.writes[-1])[0]
        self.assertEqual(frame.msg_type, MessageType.LED_CONTROL)


if __name__ == "__main__":
    unittest.main()
