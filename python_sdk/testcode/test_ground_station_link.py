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
    FastTelemetryParser,
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

    def test_telemetry_mode_sends_state_and_ignores_commands(self):
        link = self.make_link()
        link._state_provider = lambda: FCStatePayload(
            1000, -2000, 11.85, 3, True
        ).to_payload()
        link.enable_telemetry_transmission()
        self.assertEqual(link.mode, GroundLinkMode.TELEMETRY_TX)
        for _ in range(3):
            self.assertTrue(link.send_fc_state_now())
        parser = FastTelemetryParser()
        states = parser.feed(link._transport.writes[-1])
        self.assertEqual(len(states), 3)
        state = states[-1]
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
        frames = FastTelemetryParser().feed(link._transport.writes[-1])
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
