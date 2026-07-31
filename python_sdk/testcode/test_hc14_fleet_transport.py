import os
import unittest
from unittest.mock import patch

from fleet_bus.hc14_transport import (
    DEFAULT_AIR_HC14_BAUDRATE,
    DEFAULT_AIR_HC14_PORT,
    HC14BridgeCodec,
    HC14FleetTransport,
    resolve_hc14_settings,
)


class FakeSerial:
    def __init__(self):
        self.writes = []
        self.flushes = 0

    def write(self, data):
        self.writes.append(bytes(data))

    def flush(self):
        self.flushes += 1


class HC14BridgeCodecTests(unittest.TestCase):
    def test_fragmented_and_consecutive_frames_round_trip(self):
        first = HC14BridgeCodec.encode(b"first")
        second = HC14BridgeCodec.encode(b"second")
        codec = HC14BridgeCodec()

        self.assertEqual([], codec.feed(b"noise" + first[:4]))
        self.assertEqual(
            [b"first", b"second"],
            codec.feed(first[4:] + second),
        )

    def test_full_fleetbus_limit_fits_one_bridge_frame(self):
        payload = b"x" * 255
        encoded = HC14BridgeCodec.encode(payload)

        self.assertEqual(258, len(encoded))
        self.assertEqual([payload], HC14BridgeCodec().feed(encoded))

    def test_empty_and_oversized_frames_are_rejected(self):
        for payload in (b"", b"x" * 256):
            with self.subTest(length=len(payload)), self.assertRaises(ValueError):
                HC14BridgeCodec.encode(payload)


class HC14FleetTransportTests(unittest.TestCase):
    def test_defaults_and_environment_overrides(self):
        with patch.dict(os.environ, {}, clear=True):
            self.assertEqual(
                (DEFAULT_AIR_HC14_PORT, DEFAULT_AIR_HC14_BAUDRATE),
                resolve_hc14_settings(),
            )
        with patch.dict(
            os.environ,
            {"D_TASK_HC14_PORT": "/dev/test-hc14", "D_TASK_HC14_BAUDRATE": "57600"},
            clear=True,
        ):
            self.assertEqual(
                ("/dev/test-hc14", 57600),
                resolve_hc14_settings(),
            )

    def test_write_adds_car_compatible_bridge_envelope(self):
        transport = HC14FleetTransport(
            port="/dev/test-hc14",
            baudrate=115200,
            on_bytes=lambda data: None,
        )
        serial_obj = FakeSerial()
        transport._serial = serial_obj

        transport.write(b"fleet-frame")

        self.assertEqual(
            [HC14BridgeCodec.encode(b"fleet-frame")],
            serial_obj.writes,
        )
        self.assertEqual(1, serial_obj.flushes)

    def test_serial_open_uses_115200_8n1_and_disables_control_lines(self):
        class SerialPort:
            def __init__(self):
                self.opened = False
                self.dtr_values = []
                self.rts_values = []

            def open(self):
                self.opened = True

            def setDTR(self, value):
                self.dtr_values.append(value)

            def setRTS(self, value):
                self.rts_values.append(value)

        class SerialModule:
            EIGHTBITS = 8
            PARITY_NONE = "N"
            STOPBITS_ONE = 1

            def __init__(self):
                self.instance = SerialPort()

            def Serial(self):
                return self.instance

        module = SerialModule()
        transport = HC14FleetTransport(
            port="/dev/test-hc14",
            baudrate=115200,
            on_bytes=lambda data: None,
        )

        result = transport._open_serial(module)

        self.assertIs(result, module.instance)
        self.assertEqual("/dev/test-hc14", result.port)
        self.assertEqual(115200, result.baudrate)
        self.assertEqual((8, "N", 1), (result.bytesize, result.parity, result.stopbits))
        self.assertFalse(result.dsrdtr)
        self.assertFalse(result.rtscts)
        self.assertFalse(result.dtr)
        self.assertFalse(result.rts)
        self.assertEqual([False], result.dtr_values)
        self.assertEqual([False], result.rts_values)
        self.assertTrue(result.opened)


if __name__ == "__main__":
    unittest.main()
