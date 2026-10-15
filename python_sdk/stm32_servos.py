"""Temporary task-layer client for the separate STM32 four-SG90 controller.

Importing/constructing this module does not open hardware. Call connect(), or
use a context manager. Channel numbers are 1..4 (PA6, PA7, PB0, PB1).
"""
import argparse
import logging
import math
import os
import re
import threading
import time

LOG = logging.getLogger(__name__)
PORT_ENV = "STM32_SERVO_PORT"
CH340_USB_ID = (0x1A86, 0x7523)
READY = "READY SG90x4 CLOSE=0 OPEN=90"
STATE_PATTERN = re.compile(r"STATE 1=([01]) 2=([01]) 3=([01]) 4=([01])")


class ServoProtocolError(RuntimeError):
    """No confirmed reply; the requested mechanical state is unknown."""


def _serial_api():
    try:
        import serial
        return serial
    except ImportError as exc:
        raise RuntimeError("STM32 servos require pyserial (already in requirements.txt)") from exc


def list_ch340_ports():
    """Enumerate USB IDs only; never open/probe a port."""
    _serial_api()
    from serial.tools import list_ports
    return sorted((p for p in list_ports.comports()
                   if (p.vid, p.pid) == CH340_USB_ID), key=lambda p: p.device)


def resolve_port(port=None):
    configured = port or os.environ.get(PORT_ENV)
    if configured:
        return configured
    candidates = list_ch340_ports()
    if len(candidates) != 1:
        devices = ", ".join(p.device for p in candidates) or "none"
        raise RuntimeError("Expected exactly one CH340 (1a86:7523), found %d: %s; "
                           "set %s or pass port explicitly" %
                           (len(candidates), devices, PORT_ENV))
    return candidates[0].device


def _channel(channel):
    if isinstance(channel, bool) or not isinstance(channel, int) or not 1 <= channel <= 4:
        raise ValueError("STM32 servo channel must be an integer 1..4")
    return channel


class STM32Servos:
    """115200/8N1, CLOSE=logical 0 deg, OPEN=logical 90 deg.

    Each command waits for the exact acknowledgement; timeout/error closes the
    connection and never retries motion. STATUS reports PWM targets, not shaft
    feedback. close() only releases the port and does not move the servos.
    """

    def __init__(self, port=None, timeout=1.0):
        self.timeout = float(timeout)
        if not math.isfinite(self.timeout) or self.timeout <= 0:
            raise ValueError("timeout must be finite and positive")
        self.port = port
        self._configured_port = port
        self._serial = None
        self._ready = False
        self._lock = threading.RLock()

    @property
    def connected(self):
        return self._ready

    def connect(self):
        with self._lock:
            if self._ready:
                return self
            port = resolve_port(self._configured_port)
            serial_api = _serial_api()
            link = serial_api.Serial()
            link.port = port
            link.baudrate = 115200
            link.bytesize = serial_api.EIGHTBITS
            link.parity = serial_api.PARITY_NONE
            link.stopbits = serial_api.STOPBITS_ONE
            link.timeout = min(self.timeout, 0.05)
            link.write_timeout = self.timeout
            link.rtscts = False
            link.dsrdtr = False
            link.xonxoff = False
            link.dtr = False
            link.rts = False
            if os.name == "posix":
                link.exclusive = True
            self._serial = link
            try:
                link.open()
                link.reset_input_buffer()
                # Invalidate an abandoned partial command before terminating
                # it. A bare newline could execute an old partial OPEN command.
                self._request("!\nPING", "PONG", startup=True)
                self._states(startup=True)
                self.port = port
                self._ready = True
            except Exception:
                self.close()
                raise
            LOG.info("STM32 four-servo controller connected on %s", port)
            return self

    def close(self):
        with self._lock:
            link, self._serial = self._serial, None
            self._ready = False
            if link is not None:
                link.close()

    def __enter__(self):
        return self.connect()

    def __exit__(self, *_exc):
        self.close()

    def _request(self, command, expected=None, startup=False):
        """Caller holds the lock. Any uncertain exchange invalidates this link."""
        try:
            if self._serial is None:
                raise RuntimeError("Call connect() before controlling STM32 servos")
            deadline = time.monotonic() + self.timeout
            data = (command + "\n").encode("ascii")
            if self._serial.write(data) != len(data):
                raise ServoProtocolError("Incomplete STM32 serial write")
            line = bytearray()
            sync_error_pending = startup and command == "!\nPING"
            while time.monotonic() < deadline:
                byte = self._serial.read(1)
                if not byte:
                    continue
                line.extend(byte)
                if len(line) > 128:
                    raise ServoProtocolError("Oversize STM32 reply")
                if byte != b"\n":
                    continue
                response = line.decode("ascii").rstrip("\r\n")
                line.clear()
                if not response or (startup and response == READY):
                    continue
                if sync_error_pending and response in ("ERR COMMAND", "ERR LINE"):
                    sync_error_pending = False
                    continue
                if response.startswith("ERR "):
                    raise ServoProtocolError(response)
                if response == READY:
                    raise ServoProtocolError("STM32 reset during command; servo targets reset")
                if expected is not None and response != expected:
                    raise ServoProtocolError("Expected %r, received %r" % (expected, response))
                return response
            raise TimeoutError("STM32 reply timed out for %r; command outcome unknown" % command)
        except Exception:
            self.close()
            raise

    def _states(self, startup=False):
        response = self._request("STATUS", startup=startup)
        match = STATE_PATTERN.fullmatch(response)
        if match is None:
            self.close()
            raise ServoProtocolError("Invalid STM32 status: %r" % response)
        return {index: value == "1" for index, value in enumerate(match.groups(), 1)}

    def get_states(self):
        with self._lock:
            self._require_ready()
            return self._states()

    def _require_ready(self):
        if not self._ready:
            raise RuntimeError("Call connect() before controlling STM32 servos")

    def _set(self, channel, opened):
        state = "OPEN" if opened else "CLOSE"
        with self._lock:
            self._require_ready()
            self._request("%s %s" % (channel, state), "OK %s %s" % (channel, state))

    def open_servo(self, channel):
        self._set(_channel(channel), True)

    def close_servo(self, channel):
        self._set(_channel(channel), False)

    def open_all(self):
        self._set("ALL", True)

    def close_all(self):
        self._set("ALL", False)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="explicit COM/by-path device; default: unique CH340")
    parser.add_argument("--list", action="store_true", help="list CH340s without opening ports")
    parser.add_argument("--timeout", type=float, default=1.0, help="reply deadline in seconds")
    commands = parser.add_subparsers(dest="command")
    commands.add_parser("status")
    for name in ("open", "close"):
        commands.add_parser(name).add_argument("channel", type=int, choices=range(1, 5))
    for name in ("open-all", "close-all"):
        commands.add_parser(name)
    args = parser.parse_args(argv)
    if args.list and args.command:
        parser.error("--list cannot be combined with a command")
    if not args.list and args.command is None:
        parser.error("choose status/open/close/open-all/close-all, or --list")
    try:
        if args.list:
            for port in list_ch340_ports():
                print("%s  %s  %s" % (port.device, port.description, port.hwid))
            return 0
        with STM32Servos(args.port, timeout=args.timeout) as servos:
            if args.command == "open":
                servos.open_servo(args.channel)
            elif args.command == "close":
                servos.close_servo(args.channel)
            elif args.command == "open-all":
                servos.open_all()
            elif args.command == "close-all":
                servos.close_all()
            states = servos.get_states()
            print("STATE " + " ".join("%d=%d" % (ch, states[ch]) for ch in range(1, 5)))
    except (OSError, RuntimeError, ValueError) as exc:
        parser.exit(1, "%s\n" % exc)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
