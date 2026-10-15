"""Pure logic: all serial handles are fakes; no FC imports or hardware I/O."""
import contextlib
import io
import os
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import stm32_servos as servo


class FakeSerial:
    def __init__(self):
        self.writes = []
        self.opened = False
        self.closed = False
        self.received = bytearray()
        self.states = [False] * 4
        self.overrides = {}
        self.partial = ""
        self.short_write = False

    def open(self):
        self.opened = True
        self.open_signals = (self.dtr, self.rts)

    def close(self):
        self.closed = True

    def reset_input_buffer(self):
        self.received.clear()

    def write(self, data):
        self.writes.append(data)
        if self.short_write:
            return len(data) - 1
        if data in self.overrides:
            result = self.overrides[data]
            if isinstance(result, Exception):
                raise result
            self.received.extend(result)
            return len(data)
        text = self.partial + data.decode("ascii")
        lines = text.split("\n")
        self.partial = lines.pop()
        for line in lines:
            if line == "PING":
                response = "PONG"
            elif line == "STATUS":
                response = "STATE " + " ".join("%d=%d" % (i + 1, v) for i, v in enumerate(self.states))
            elif line in ("ALL OPEN", "ALL CLOSE"):
                self.states[:] = [line.endswith("OPEN")] * 4
                response = "OK " + line
            elif line in tuple("%d %s" % (ch, state) for ch in range(1, 5)
                               for state in ("OPEN", "CLOSE")):
                self.states[int(line[0]) - 1] = line.endswith("OPEN")
                response = "OK " + line
            else:
                response = "ERR COMMAND"
            self.received.extend((response + "\r\n").encode("ascii"))
        return len(data)

    def read(self, count):
        if not self.received:
            time.sleep(0.001)
            return b""
        result = bytes(self.received[:count])
        del self.received[:count]
        return result


class STM32ServoTests(unittest.TestCase):
    def setUp(self):
        self.link = FakeSerial()
        self.api = SimpleNamespace(Serial=Mock(return_value=self.link), EIGHTBITS=8,
                                   PARITY_NONE="N", STOPBITS_ONE=1)
        self.api_patch = patch.object(servo, "_serial_api", return_value=self.api)
        self.api_patch.start()
        self.addCleanup(self.api_patch.stop)
        self.env_patch = patch.dict(os.environ, {}, clear=True)
        self.env_patch.start()
        self.addCleanup(self.env_patch.stop)

    def client(self, **kwargs):
        client = servo.STM32Servos("FAKE", **kwargs)
        self.addCleanup(client.close)
        return client

    def test_construct_and_close_do_not_open_or_move_hardware(self):
        client = self.client()
        self.api.Serial.assert_not_called()
        self.assertFalse(client.connected)
        client.close()
        self.assertEqual([], self.link.writes)

    def test_port_override_environment_and_unique_detection(self):
        with patch.object(servo, "list_ch340_ports", return_value=[SimpleNamespace(device="USB3")]) as ports:
            self.assertEqual("USB3", servo.resolve_port())
            with patch.dict(os.environ, {servo.PORT_ENV: "ENV"}):
                self.assertEqual("ENV", servo.resolve_port())
                self.assertEqual("EXPLICIT", servo.resolve_port("EXPLICIT"))
            self.assertEqual(1, ports.call_count)

    def test_zero_or_multiple_devices_never_open_any_port(self):
        for candidates in ([], [SimpleNamespace(device="USB0"), SimpleNamespace(device="USB1")]):
            with patch.object(servo, "list_ch340_ports", return_value=candidates):
                with self.assertRaisesRegex(RuntimeError, "exactly one CH340"):
                    servo.STM32Servos().connect()
        self.api.Serial.assert_not_called()

    def test_enumeration_filters_by_usb_id_and_never_opens_ports(self):
        ports = [SimpleNamespace(device=name, vid=vid, pid=pid)
                 for name, vid, pid in (("USB2", 0x1A86, 0x7523),
                                        ("ACM0", 0x66CC, 0x2233),
                                        ("USB1", 0x10C4, 0xEA60),
                                        ("USB0", 0x1A86, 0x7523),
                                        ("BT0", None, None))]
        list_ports = SimpleNamespace(comports=Mock(return_value=ports))
        tools = SimpleNamespace(list_ports=list_ports)
        with patch.dict(sys.modules, {"serial": SimpleNamespace(tools=tools), "serial.tools": tools}):
            self.assertEqual(["USB0", "USB2"], [p.device for p in servo.list_ch340_ports()])
        self.api.Serial.assert_not_called()

    def test_automatic_reconnect_reenumerates_the_unique_ch340(self):
        with patch.object(servo, "list_ch340_ports", return_value=[SimpleNamespace(device="USB0")]) as ports:
            client = servo.STM32Servos().connect()
            self.addCleanup(client.close)
            self.assertEqual("USB0", client.port)
            client.close()
            ports.return_value = [SimpleNamespace(device="USB2")]
            client.connect()
            self.assertEqual("USB2", client.port)

    def test_handshake_and_serial_settings_never_move_any_channel(self):
        client = self.client().connect()
        self.assertTrue(client.connected)
        self.assertEqual([b"!\nPING\n", b"STATUS\n"], self.link.writes)
        self.assertEqual((False, False), self.link.open_signals)
        self.assertEqual((115200, 8, "N", 1),
                         (self.link.baudrate, self.link.bytesize, self.link.parity, self.link.stopbits))
        self.assertEqual([False] * 4, self.link.states)
        client.connect()
        self.assertEqual(1, self.api.Serial.call_count)

    def test_abandoned_open_command_is_invalidated_before_newline(self):
        self.link.partial = "1 OPEN"
        self.client().connect()
        self.assertEqual([False] * 4, self.link.states)

    def test_startup_greeting_and_oversize_old_command_recovery(self):
        self.link.overrides[b"!\nPING\n"] = (servo.READY + "\r\nERR LINE\r\nPONG\r\n").encode()
        self.assertTrue(self.client().connect().connected)

    def test_wrong_device_or_invalid_status_is_rejected_without_motion(self):
        for key, value in ((b"!\nPING\n", b"OTHER\r\n"),
                           (b"STATUS\n", b"STATE 1=0 2=0 3=0\r\n"),
                           (b"STATUS\n", b"STATE 1=0 2=0 3=0 4=2\r\n")):
            self.link.overrides = {key: value}
            with self.assertRaises(servo.ServoProtocolError):
                self.client().connect()
            self.assertTrue(self.link.closed)
            self.assertFalse(any(b" OPEN\n" in cmd or b" CLOSE\n" in cmd for cmd in self.link.writes))

    def test_all_four_individual_channels_and_all_commands(self):
        client = self.client().connect()
        for channel in range(1, 5):
            client.open_servo(channel)
            self.assertTrue(client.get_states()[channel])
            self.assertEqual(1, sum(client.get_states().values()))
            client.close_servo(channel)
            self.assertFalse(any(client.get_states().values()))
        client.open_all()
        self.assertEqual({i: True for i in range(1, 5)}, client.get_states())
        client.close_all()
        self.assertEqual({i: False for i in range(1, 5)}, client.get_states())

    def test_invalid_channels_never_write(self):
        client = self.client().connect()
        baseline = list(self.link.writes)
        for channel in (True, False, 0, 5, -1, 1.0, "1", None):
            for method in (client.open_servo, client.close_servo):
                with self.assertRaises(ValueError):
                    method(channel)
        self.assertEqual(baseline, self.link.writes)

    def test_timeout_error_reset_corruption_and_wrong_ack_invalidate_link(self):
        failures = (b"", b"ERR UART\r\n", b"OK 2 OPEN\r\n",
                    (servo.READY + "\r\n").encode(), b"X" * 129, b"\xff\n",
                    OSError("unplugged"))
        for response in failures:
            self.link.overrides = {}
            client = self.client(timeout=0.02).connect()
            self.link.overrides[b"1 OPEN\n"] = response
            before = len(self.link.writes)
            with self.assertRaises((OSError, servo.ServoProtocolError, UnicodeError)):
                client.open_servo(1)
            self.assertFalse(client.connected)
            self.assertTrue(self.link.closed)
            self.assertEqual(before + 1, len(self.link.writes))  # no motion retry
            with self.assertRaisesRegex(RuntimeError, "connect"):
                client.close_servo(1)
            self.assertEqual(before + 1, len(self.link.writes))

    def test_partial_write_and_failed_open_release_handle(self):
        client = self.client().connect()
        self.link.short_write = True
        with self.assertRaisesRegex(servo.ServoProtocolError, "Incomplete"):
            client.close_all()
        self.assertFalse(client.connected)
        self.link.short_write = False
        with patch.object(self.link, "open", side_effect=OSError("busy")):
            with self.assertRaisesRegex(OSError, "busy"):
                self.client().connect()
        self.assertTrue(self.link.closed)

    def test_context_exit_releases_port_without_changing_last_target(self):
        with self.client() as client:
            client.open_servo(4)
        self.assertTrue(self.link.closed)
        self.assertEqual([False, False, False, True], self.link.states)
        self.assertEqual(b"4 OPEN\n", self.link.writes[-1])

    def test_bad_status_after_connect_disconnects_instead_of_returning_old_state(self):
        client = self.client().connect()
        self.link.overrides[b"STATUS\n"] = b"STATE 1=1 2=0 3=0\r\n"
        with self.assertRaisesRegex(servo.ServoProtocolError, "Invalid STM32 status"):
            client.get_states()
        self.assertFalse(client.connected)

    def test_concurrent_callers_have_serialized_command_reply_pairs(self):
        client = self.client().connect()
        errors = []
        def worker(channel):
            try:
                for _ in range(10):
                    client.open_servo(channel)
                    client.close_servo(channel)
            except Exception as exc:
                errors.append(exc)
        threads = [threading.Thread(target=worker, args=(i,)) for i in range(1, 5)]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join(2)
        self.assertTrue(all(not thread.is_alive() for thread in threads))
        self.assertEqual([], errors)
        self.assertEqual([False] * 4, self.link.states)

    def test_bad_timeout_or_unconnected_operation_never_writes(self):
        for timeout in (0, -1, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                self.client(timeout=timeout)
        with self.assertRaisesRegex(RuntimeError, "connect"):
            self.client().open_all()
        self.assertEqual([], self.link.writes)

    def test_cli_list_and_help_do_not_open_ports(self):
        with patch.object(servo, "list_ch340_ports", return_value=[]), contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(0, servo.main(["--list"]))
            with self.assertRaises(SystemExit) as result:
                servo.main(["--help"])
        self.assertEqual(0, result.exception.code)
        self.api.Serial.assert_not_called()

    def test_cli_open_uses_correct_channel_and_closes_port(self):
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            self.assertEqual(0, servo.main(["--port", "FAKE", "open", "3"]))
        self.assertEqual("STATE 1=0 2=0 3=1 4=0\n", output.getvalue())
        self.assertEqual([b"!\nPING\n", b"STATUS\n", b"3 OPEN\n", b"STATUS\n"], self.link.writes)
        self.assertTrue(self.link.closed)


if __name__ == "__main__":
    unittest.main()
