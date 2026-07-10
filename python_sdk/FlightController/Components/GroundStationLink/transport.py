import threading
import time
from typing import Callable, Optional


class HC14SerialTransport:
    """HC-14 USB serial transport with safe control-line defaults."""

    def __init__(
        self,
        port: str,
        on_bytes: Callable[[bytes], None],
        baudrate: int = 9600,
        on_connected: Optional[Callable[[], None]] = None,
        on_disconnected: Optional[Callable[[Optional[Exception]], None]] = None,
        reconnect_seconds: float = 1.0,
    ):
        self._port = port
        self._baudrate = baudrate
        self._on_bytes = on_bytes
        self._on_connected = on_connected
        self._on_disconnected = on_disconnected
        self._reconnect_seconds = reconnect_seconds
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
            target=self._run, name="ground-station-hc14", daemon=True
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
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def write(self, data: bytes) -> None:
        with self._lock:
            serial_obj = self._serial
            if serial_obj is None:
                raise RuntimeError("HC-14 link is not connected")
            serial_obj.write(data)
            serial_obj.flush()

    def _open_serial(self):
        import serial

        serial_obj = serial.Serial()
        serial_obj.port = self._port
        serial_obj.baudrate = self._baudrate
        serial_obj.bytesize = serial.EIGHTBITS
        serial_obj.parity = serial.PARITY_NONE
        serial_obj.stopbits = serial.STOPBITS_ONE
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
            import serial  # noqa: F401
        except ImportError as exc:
            if self._on_disconnected is not None:
                self._on_disconnected(exc)
            return

        while not self._stop.is_set():
            error = None  # type: Optional[Exception]
            try:
                serial_obj = self._open_serial()
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
                if self._on_disconnected is not None:
                    self._on_disconnected(error)
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
                self._on_bytes(data)
            else:
                self._stop.wait(0.005)
