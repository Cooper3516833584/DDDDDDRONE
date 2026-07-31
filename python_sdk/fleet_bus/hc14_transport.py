"""Direct CH340/HC-14 transport for the airborne FleetBus endpoint."""

from __future__ import annotations

import logging
import os
import threading
from typing import Callable, Optional


LOG = logging.getLogger("fleet-air-hc14")

DEFAULT_AIR_HC14_PORT = (
    "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
)
DEFAULT_AIR_HC14_BAUDRATE = 115200
HC14_PORT_ENV = "D_TASK_HC14_PORT"
HC14_BAUDRATE_ENV = "D_TASK_HC14_BAUDRATE"
BRIDGE_HEADER = b"\xBB\x33"
BRIDGE_MAX_PAYLOAD = 255


def resolve_hc14_settings(
    port: Optional[str] = None,
    baudrate: Optional[int] = None,
):
    resolved_port = port or os.environ.get(HC14_PORT_ENV, DEFAULT_AIR_HC14_PORT)
    raw_baudrate = (
        baudrate
        if baudrate is not None
        else os.environ.get(HC14_BAUDRATE_ENV, DEFAULT_AIR_HC14_BAUDRATE)
    )
    try:
        resolved_baudrate = int(raw_baudrate)
    except (TypeError, ValueError) as exc:
        raise ValueError("HC-14 baudrate must be an integer") from exc
    if not resolved_port:
        raise ValueError("HC-14 serial port must not be empty")
    if resolved_baudrate <= 0:
        raise ValueError("HC-14 baudrate must be positive")
    return resolved_port, resolved_baudrate


class HC14BridgeCodec:
    """Encode/decode the envelope shared with the car and ground station."""

    def __init__(self) -> None:
        self._buffer = bytearray()

    @staticmethod
    def encode(data: bytes) -> bytes:
        payload = bytes(data)
        if not payload:
            raise ValueError("FleetBus frame must not be empty")
        if len(payload) > BRIDGE_MAX_PAYLOAD:
            raise ValueError("FleetBus frame exceeds HC-14 bridge limit")
        return BRIDGE_HEADER + bytes((len(payload),)) + payload

    def reset(self) -> None:
        self._buffer.clear()

    def feed(self, data: bytes):
        if data:
            self._buffer.extend(data)
        payloads = []
        while True:
            index = self._buffer.find(BRIDGE_HEADER)
            if index < 0:
                keep = 1 if self._buffer[-1:] == BRIDGE_HEADER[:1] else 0
                if keep:
                    del self._buffer[:-1]
                else:
                    self._buffer.clear()
                return payloads
            if index:
                del self._buffer[:index]
            if len(self._buffer) < 3:
                return payloads
            payload_length = self._buffer[2]
            if payload_length == 0:
                del self._buffer[0]
                continue
            frame_length = 3 + payload_length
            if len(self._buffer) < frame_length:
                return payloads
            payloads.append(bytes(self._buffer[3:frame_length]))
            del self._buffer[:frame_length]


class HC14FleetTransport:
    """Own the airborne CH340 and exchange framed FleetBus packets directly."""

    def __init__(
        self,
        *,
        port: str,
        baudrate: int,
        on_bytes: Callable[[bytes], None],
        on_connected: Optional[Callable[[], None]] = None,
        on_disconnected: Optional[Callable[[Optional[Exception]], None]] = None,
        reconnect_seconds: float = 1.0,
    ) -> None:
        self._port, self._baudrate = resolve_hc14_settings(port, baudrate)
        self._on_bytes = on_bytes
        self._on_connected = on_connected
        self._on_disconnected = on_disconnected
        self._reconnect_seconds = reconnect_seconds
        self._codec = HC14BridgeCodec()
        self._stop = threading.Event()
        self._thread = None  # type: Optional[threading.Thread]
        self._serial = None
        self._lock = threading.Lock()

    @property
    def connected(self) -> bool:
        with self._lock:
            return self._serial is not None

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run,
            name="fleet-air-hc14",
            daemon=True,
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        with self._lock:
            serial_obj = self._serial
            self._serial = None
        if serial_obj is not None:
            try:
                serial_obj.close()
            except Exception:
                pass
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=2.0)
        self._thread = None

    def write(self, data: bytes) -> None:
        outbound = HC14BridgeCodec.encode(data)
        with self._lock:
            serial_obj = self._serial
            if serial_obj is None:
                raise RuntimeError("airborne HC-14 link is not connected")
            serial_obj.write(outbound)
            serial_obj.flush()

    def _open_serial(self, serial_module):
        serial_obj = serial_module.Serial()
        serial_obj.port = self._port
        serial_obj.baudrate = self._baudrate
        serial_obj.bytesize = serial_module.EIGHTBITS
        serial_obj.parity = serial_module.PARITY_NONE
        serial_obj.stopbits = serial_module.STOPBITS_ONE
        serial_obj.timeout = 0.1
        serial_obj.write_timeout = 0.5
        serial_obj.dsrdtr = False
        serial_obj.rtscts = False
        serial_obj.dtr = False
        serial_obj.rts = False
        serial_obj.open()
        serial_obj.setDTR(False)
        serial_obj.setRTS(False)
        return serial_obj

    def _run(self) -> None:
        try:
            import serial
        except ImportError as exc:
            self._notify_disconnected(exc)
            return

        while not self._stop.is_set():
            error = None  # type: Optional[Exception]
            try:
                serial_obj = self._open_serial(serial)
                self._codec.reset()
                with self._lock:
                    self._serial = serial_obj
                if self._on_connected is not None:
                    self._on_connected()
                self._read_loop(serial_obj)
            except Exception as exc:
                error = exc
            finally:
                with self._lock:
                    serial_obj = self._serial
                    self._serial = None
                if serial_obj is not None:
                    try:
                        serial_obj.close()
                    except Exception:
                        pass
                if error is not None or not self._stop.is_set():
                    self._notify_disconnected(error)
            if not self._stop.is_set():
                self._stop.wait(self._reconnect_seconds)

    def _read_loop(self, serial_obj) -> None:
        while not self._stop.is_set():
            with self._lock:
                if self._serial is not serial_obj:
                    return
                waiting = serial_obj.in_waiting
                data = serial_obj.read(waiting) if waiting else b""
            if data:
                for payload in self._codec.feed(data):
                    self._on_bytes(payload)
            else:
                self._stop.wait(0.005)

    def _notify_disconnected(self, error: Optional[Exception]) -> None:
        if self._on_disconnected is not None:
            self._on_disconnected(error)
        elif error is not None:
            LOG.warning("Airborne HC-14 disconnected: %s", error)
