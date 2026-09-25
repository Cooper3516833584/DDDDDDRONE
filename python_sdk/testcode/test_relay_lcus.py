"""LCUS 8 路 USB 继电器驱动的纯逻辑测试 (不访问真实串口/硬件)。

运行::

    python3 testcode/test_relay_lcus.py

测试用 ``sys.modules`` 注入的假 serial 模块替换 pyserial, 因此不会打开任何真实设备。

风险与边界 (不能替代实机验证):

- 全部用例都在假串口上跑, 只验证帧构造、解析、参数校验和控制流程分支;
  **不能**证明真实板子会动作, 也**不能**验证波特率/时序/电气特性。
- 假串口的 ``read()`` 不会阻塞, 因此"查询超时"类用例只覆盖逻辑分支,
  真实超时时间取决于 ``query_timeout`` 和板子响应速度。
- 真机上"控制帧后约 50ms 内 FF 回读仍是旧状态"的现象, 在本文件中体现为
  ``test_set_channel_verify_retries_then_fails`` 覆盖的重发分支;
  想确认真机行为必须用 ``relay_lcus_terminal.py`` 在硬件上实测。
- 这些用例不会吸合任何继电器; 反之, 通过本文件也不代表继电器可以安全带负载使用。
"""

import os
import sys
import types
import unittest
from unittest import mock

SDK_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if SDK_DIR not in sys.path:
    sys.path.insert(0, SDK_DIR)

from FlightController.Components.relay_lcus import (  # noqa: E402
    DEFAULT_BAUDRATE,
    LCUSRelay,
    RELAY_PORT_ENV,
    build_channel_command,
    format_states,
    parse_status_response,
)


def status_reply(states):
    """按说明书格式拼出 FF 查询的返回文本: 每路 10 字节。"""
    text = ""
    for channel in sorted(states):
        text += "CH{0}: {1}\r\n".format(channel, "ON " if states[channel] else "OFF")
    return text.encode("ascii")


def full_status(on_channels):
    return {channel: channel in on_channels for channel in range(1, 9)}


class FakeSerial(object):
    """最小串口替身, 只实现 LCUSRelay 用到的方法。"""

    def __init__(self, responder=None):
        self.writes = []
        self.buffer = bytearray()
        self.responder = responder
        self.closed = False
        self.port = None
        self.baudrate = None
        self.bytesize = None
        self.parity = None
        self.stopbits = None
        self.timeout = None
        self.write_timeout = None
        self.xonxoff = None
        self.rtscts = None
        self.dsrdtr = None
        self.dtr = None
        self.rts = None
        self.set_dtr_calls = []
        self.set_rts_calls = []

    def open(self):
        self.closed = False

    def close(self):
        self.closed = True

    def setDTR(self, value):
        self.dtr = value
        self.set_dtr_calls.append(value)

    def setRTS(self, value):
        self.rts = value
        self.set_rts_calls.append(value)

    def reset_input_buffer(self):
        self.buffer = bytearray()

    def write(self, data):
        payload = bytes(data)
        self.writes.append(payload)
        if self.responder is not None:
            reply = self.responder(payload)
            if reply:
                self.buffer.extend(reply)
        return len(payload)

    def flush(self):
        pass

    @property
    def in_waiting(self):
        return len(self.buffer)

    def read(self, size=1):
        if not self.buffer:
            return b""
        chunk = bytes(self.buffer[:size])
        del self.buffer[:size]
        return chunk


class FakeSerialFactory(object):
    """假 serial 模块: 记录创建的串口对象。"""

    EIGHTBITS = 8
    PARITY_NONE = "N"
    STOPBITS_ONE = 1

    def __init__(self, responder=None):
        self.responder = responder
        self.created = []

    def Serial(self):
        serial_obj = FakeSerial(self.responder)
        self.created.append(serial_obj)
        return serial_obj


class ProtocolTests(unittest.TestCase):
    """控制帧与 FF 返回解析的纯逻辑测试。"""

    def test_manual_examples(self):
        self.assertEqual(build_channel_command(1, True), bytes.fromhex("A00101A2"))
        self.assertEqual(build_channel_command(1, False), bytes.fromhex("A00100A1"))

    def test_all_eight_channels_prefix_channel_state_and_checksum(self):
        for channel in range(1, 9):
            for on in (True, False):
                command = build_channel_command(channel, on)
                self.assertEqual(len(command), 4)
                self.assertEqual(command[0], 0xA0)
                self.assertEqual(command[1], channel)
                self.assertEqual(command[2], 0x01 if on else 0x00)
                self.assertEqual(command[3], (0xA0 + channel + command[2]) & 0xFF)

    def test_on_and_off_differ_only_in_state_and_checksum(self):
        for channel in range(1, 9):
            self.assertEqual(
                build_channel_command(channel, True)[:2],
                build_channel_command(channel, False)[:2],
            )
            self.assertNotEqual(
                build_channel_command(channel, True)[2:],
                build_channel_command(channel, False)[2:],
            )

    def test_invalid_channel_rejected(self):
        for channel in (0, 9, -1):
            with self.assertRaises(ValueError):
                build_channel_command(channel, True)
        with self.assertRaises(ValueError):
            build_channel_command("1", True)
        with self.assertRaises(ValueError):
            build_channel_command(True, True)

    def test_channel_count_limits_accepted_range(self):
        self.assertEqual(len(build_channel_command(4, True, channel_count=4)), 4)
        with self.assertRaises(ValueError):
            build_channel_command(5, True, channel_count=4)

    def test_parse_manual_four_channel_example(self):
        raw = b"CH1: ON \r\nCH2: ON \r\nCH3: OFF\r\nCH4: OFF\r\n"
        self.assertEqual(
            parse_status_response(raw),
            {1: True, 2: True, 3: False, 4: False},
        )

    def test_parse_eight_channel_reply(self):
        raw = status_reply(full_status([1, 3, 5, 7, 8]))
        self.assertEqual(
            parse_status_response(raw),
            {1: True, 2: False, 3: True, 4: False, 5: True, 6: False, 7: True, 8: True},
        )

    def test_parse_tolerates_noise_and_padding(self):
        raw = b"\x00\xffCH1:ON\r\njunk CH2:  off \r\n\nCH9: ON\r\n"
        self.assertEqual(parse_status_response(raw), {1: True, 2: False})

    def test_parse_empty_and_garbage(self):
        self.assertEqual(parse_status_response(b""), {})
        self.assertEqual(parse_status_response(b"no status here"), {})

    def test_format_states(self):
        self.assertEqual(format_states({2: False, 1: True}), "CH1=ON CH2=OFF")
        self.assertEqual(format_states({}), "(空)")


class DriverTests(unittest.TestCase):
    """驱动层测试: 用假 serial 模块验证帧发送、回读确认和异常路径。"""

    def setUp(self):
        self._patcher = None

    def tearDown(self):
        if self._patcher is not None:
            self._patcher.stop()
            self._patcher = None

    def install_fake_serial(self, responder=None):
        fake_module = types.ModuleType("serial")
        factory = FakeSerialFactory(responder)
        fake_module.Serial = factory.Serial
        fake_module.EIGHTBITS = factory.EIGHTBITS
        fake_module.PARITY_NONE = factory.PARITY_NONE
        fake_module.STOPBITS_ONE = factory.STOPBITS_ONE
        self._patcher = mock.patch.dict(sys.modules, {"serial": fake_module})
        self._patcher.start()
        return factory

    def make_relay(self, responder=None, **kwargs):
        factory = self.install_fake_serial(responder)
        relay = LCUSRelay(port="/dev/fake-relay", query_timeout=0.05, **kwargs)
        relay.open()
        return relay, factory.created[0]

    def test_open_configures_port_and_disables_dtr_rts(self):
        relay, serial_obj = self.make_relay()
        self.assertEqual(serial_obj.port, "/dev/fake-relay")
        self.assertEqual(serial_obj.baudrate, DEFAULT_BAUDRATE)
        self.assertFalse(serial_obj.dtr)
        self.assertFalse(serial_obj.rts)
        self.assertEqual(serial_obj.set_dtr_calls, [False])
        self.assertEqual(serial_obj.set_rts_calls, [False])
        self.assertTrue(relay.connected)
        self.assertEqual(relay.port, "/dev/fake-relay")

    def test_set_channel_writes_expected_frame(self):
        relay, serial_obj = self.make_relay()
        self.assertTrue(relay.set_channel(3, True))
        self.assertEqual(serial_obj.writes, [bytes.fromhex("A00301A4")])
        self.assertTrue(relay.turn_off(8))
        self.assertEqual(serial_obj.writes[-1], bytes.fromhex("A00800A8"))

    def test_set_channel_verify_confirms_via_ff_query(self):
        def responder(command):
            if command == b"\xff":
                return status_reply(full_status([2]))
            return b""

        relay, serial_obj = self.make_relay(responder)
        self.assertTrue(relay.set_channel(2, True, verify=True))
        self.assertEqual(serial_obj.writes[0], bytes.fromhex("A00201A3"))
        self.assertEqual(serial_obj.writes[1], b"\xff")

    def test_set_channel_verify_retries_then_fails(self):
        def responder(command):
            if command == b"\xff":
                return status_reply(full_status([]))  # 回读显示仍然全关
            return b""

        relay, serial_obj = self.make_relay(responder)
        self.assertFalse(relay.set_channel(1, True, verify=True, retries=1))
        self.assertEqual(serial_obj.writes, [bytes.fromhex("A00101A2"), b"\xff"] * 2)

    def test_set_channel_verify_without_reply_fails(self):
        relay, serial_obj = self.make_relay()
        self.assertFalse(relay.set_channel(1, True, verify=True, retries=0))
        self.assertEqual(serial_obj.writes, [bytes.fromhex("A00101A2"), b"\xff"])

    def test_query_status_returns_states(self):
        def responder(command):
            return status_reply(full_status([1, 4])) if command == b"\xff" else b""

        relay, serial_obj = self.make_relay(responder)
        self.assertEqual(
            relay.query_status(),
            {1: True, 2: False, 3: False, 4: True, 5: False, 6: False, 7: False, 8: False},
        )
        self.assertEqual(serial_obj.writes, [b"\xff"])

    def test_query_status_timeout_returns_none(self):
        relay, _ = self.make_relay()
        self.assertIsNone(relay.query_status())
        self.assertIsNone(relay.get_channel_state(1))

    def test_detect_channel_count_on_four_channel_board(self):
        def responder(command):
            return status_reply({c: False for c in range(1, 5)}) if command == b"\xff" else b""

        relay, serial_obj = self.make_relay(responder)  # 默认按 8 路配置
        self.assertEqual(relay.detect_channel_count(), 4)
        self.assertEqual(serial_obj.writes, [b"\xff"])  # 只发查询帧, 不发控制帧

    def test_detect_channel_count_on_eight_channel_board(self):
        def responder(command):
            return status_reply(full_status([2, 8])) if command == b"\xff" else b""

        relay, _ = self.make_relay(responder, channel_count=4)  # 按 4 路配置也能识别 8 路板
        self.assertEqual(relay.detect_channel_count(), 8)

    def test_detect_channel_count_without_reply_returns_none(self):
        relay, serial_obj = self.make_relay()
        self.assertIsNone(relay.detect_channel_count())
        self.assertEqual(serial_obj.writes, [b"\xff"])

    def test_get_channel_state_reports_single_channel(self):
        def responder(command):
            return status_reply(full_status([5])) if command == b"\xff" else b""

        relay, _ = self.make_relay(responder)
        self.assertTrue(relay.get_channel_state(5))
        self.assertFalse(relay.get_channel_state(6))

    def test_all_off_sends_every_channel_off_frame(self):
        relay, serial_obj = self.make_relay()
        self.assertTrue(relay.all_off())
        self.assertEqual(
            serial_obj.writes,
            [build_channel_command(channel, False) for channel in range(1, 9)],
        )

    def test_channel_out_of_range_rejected_before_serial_write(self):
        relay, serial_obj = self.make_relay()
        with self.assertRaises(ValueError):
            relay.set_channel(9, True)
        with self.assertRaises(ValueError):
            relay.get_channel_state(0)
        self.assertEqual(serial_obj.writes, [])

    def test_operations_require_open_serial(self):
        self.install_fake_serial()
        relay = LCUSRelay(port="/dev/fake-relay")
        self.assertFalse(relay.connected)
        with self.assertRaises(RuntimeError):
            relay.set_channel(1, True)
        with self.assertRaises(RuntimeError):
            relay.query_status()

    def test_context_manager_closes_serial(self):
        factory = self.install_fake_serial()
        with LCUSRelay(port="/dev/fake-relay") as relay:
            self.assertTrue(relay.connected)
        self.assertTrue(factory.created[0].closed)
        self.assertFalse(relay.connected)

    def test_port_from_environment_variable(self):
        self.install_fake_serial()
        with mock.patch.dict(os.environ, {RELAY_PORT_ENV: "/dev/env-relay"}):
            relay = LCUSRelay()
            self.assertEqual(relay.port, "/dev/env-relay")

    def test_explicit_port_overrides_environment_variable(self):
        self.install_fake_serial()
        with mock.patch.dict(os.environ, {RELAY_PORT_ENV: "/dev/env-relay"}):
            self.assertEqual(LCUSRelay(port="/dev/arg-relay").port, "/dev/arg-relay")

    def test_missing_port_raises_value_error(self):
        self.install_fake_serial()
        with mock.patch.dict(os.environ, {}, clear=True):
            with self.assertRaises(ValueError):
                LCUSRelay()

    def test_invalid_channel_count_rejected(self):
        self.install_fake_serial()
        for channel_count in (0, 9, "8"):
            with self.assertRaises(ValueError):
                LCUSRelay(port="/dev/fake-relay", channel_count=channel_count)

    def test_four_channel_board_stops_after_channel_four(self):
        relay, serial_obj = self.make_relay(channel_count=4)
        self.assertTrue(relay.all_off())
        self.assertEqual(
            serial_obj.writes,
            [build_channel_command(channel, False, channel_count=4) for channel in range(1, 5)],
        )


if __name__ == "__main__":
    unittest.main(verbosity=2)
