import threading
import time
import unittest

from FlightController.Components.GroundStationLink import (
    CommandId,
    FCStatePayload,
    FLAG_UPLINK_WINDOW,
    GroundStationLink,
    MessageType,
)
from FlightController.Components.GroundStationLink.models import Command
from FlightController.Components.GroundStationLink.protocol import (
    FrameParser,
    pack_frame,
)


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
        self.assertEqual(parser.stats.crc_failures, 1)

    def test_default_telemetry_rate_is_ten_hz(self):
        link = GroundStationLink(
            fc=None,
            key=KEY,
            transport_factory=FakeTransport,
            turnaround_seconds=0.0,
        )
        self.assertAlmostEqual(link._telemetry_period, 0.1)

    def test_any_uplink_byte_pauses_telemetry_before_protocol_parse(self):
        link = self.make_link()
        before = time.monotonic()
        link._on_bytes(b"\x00")
        self.assertGreaterEqual(link._telemetry_pause_until, before + 1.4)

    def test_only_scheduled_state_advertises_uplink_window(self):
        link = self.make_link()
        link._state_provider = lambda: FCStatePayload(
            1000, -2000, 11.85, 3, True
        ).to_payload()
        link.send_fc_state_now()
        link.send_fc_state_now(uplink_window=True)
        parser = FrameParser(KEY)
        regular = parser.feed(link._transport.writes[-2])[0]
        window = parser.feed(link._transport.writes[-1])[0]
        self.assertEqual(regular.flags & FLAG_UPLINK_WINDOW, 0)
        self.assertEqual(window.flags & FLAG_UPLINK_WINDOW, FLAG_UPLINK_WINDOW)
        self.assertGreater(link._telemetry_pause_until, time.monotonic())


if __name__ == "__main__":
    unittest.main()
