from dataclasses import dataclass
import itertools
from queue import Empty, Full, PriorityQueue
import threading
import time
from typing import Callable, Optional

from loguru import logger

from .models import (
    AckStatus,
    Alarm,
    Command,
    CommandAck,
    CommandId,
    FCStatePayload,
    GroundLinkMode,
    LEDControl,
    MessageType,
    MissionState,
    MissionStatus,
    RejectReason,
)
from .protocol import (
    Frame,
    FrameParser,
    RecentResponseCache,
    new_session,
    pack_frame,
)
from .transport import FCWirelessTransport


DEFAULT_HC14_PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"


@dataclass(frozen=True)
class MissionCommand:
    session: int
    seq: int
    command: Command
    received_at: float


class GroundStationLink:
    """Reliable HC-14 adapter between the ground station and mission scheduler.

    The serial receive thread only validates frames, sends protocol ACKs, sets the
    optional stop event, and queues MissionCommand objects. Flight actions must be
    performed by the task thread that consumes get_command().
    """

    def __init__(
        self,
        fc,
        key: bytes,
        port: str = DEFAULT_HC14_PORT,
        baudrate: int = 9600,
        telemetry_hz: float = 20.0,
        turnaround_seconds: float = 0.2,
        stop_event: Optional[threading.Event] = None,
        state_provider: Optional[Callable[[], Optional[bytes]]] = None,
        queue_size: int = 32,
        transport_factory=None,
    ):
        if not key:
            raise ValueError("HMAC key is required")
        if telemetry_hz <= 0:
            raise ValueError("telemetry_hz must be positive")
        self._fc = fc
        self._key = key
        self._session = new_session()
        self._parser = FrameParser(key)
        self._peer_session = None  # type: Optional[int]
        self._telemetry_period = 1.0 / telemetry_hz
        self._turnaround_seconds = turnaround_seconds
        self._stop_event = stop_event
        self._state_provider = state_provider or self._snapshot_fc_state
        self._external_state_provider = state_provider is not None
        self._batch_buffer = bytearray()
        self._batch_count = 0
        self._batch_lock = threading.Lock()
        self._queue = PriorityQueue(maxsize=queue_size)
        self._queue_counter = itertools.count()
        self._recent = RecentResponseCache(max_items=64)
        self._recent_lock = threading.Lock()
        self._seq_lock = threading.Lock()
        self._next_seq = 1
        self._stop = threading.Event()
        self._telemetry_thread = None  # type: Optional[threading.Thread]
        self._mode = GroundLinkMode.COMMAND_RX
        self._mode_lock = threading.Lock()
        self._mode_changed = threading.Event()
        self._last_rx_time = 0.0
        self._stop_lock = threading.Lock()
        self._stop_latched = False
        self._stop_status = None  # type: Optional[AckStatus]
        self._stop_aliases = {}  # type: dict
        if transport_factory is None:
            self._transport = FCWirelessTransport(
                fc=fc,
                on_bytes=self._on_bytes,
                on_connected=self._on_connected,
                on_disconnected=self._on_disconnected,
            )
        else:
            self._transport = transport_factory(
                port=port,
                baudrate=baudrate,
                on_bytes=self._on_bytes,
                on_connected=self._on_connected,
                on_disconnected=self._on_disconnected,
            )

    @property
    def session(self) -> int:
        return self._session

    @property
    def connected(self) -> bool:
        return self._transport.connected

    @property
    def mode(self) -> GroundLinkMode:
        with self._mode_lock:
            return self._mode

    def set_mode(self, mode: GroundLinkMode) -> None:
        """Switch between pre-flight command reception and in-flight telemetry."""
        try:
            mode = GroundLinkMode(mode)
        except ValueError as exc:
            raise ValueError("invalid ground link mode") from exc
        with self._mode_lock:
            self._mode = mode
        if mode == GroundLinkMode.COMMAND_RX:
            self._clear_telemetry_batch()
        self._mode_changed.set()

    def enable_command_reception(self) -> None:
        self.set_mode(GroundLinkMode.COMMAND_RX)

    def enable_telemetry_transmission(self) -> None:
        self.set_mode(GroundLinkMode.TELEMETRY_TX)

    def start(self) -> None:
        if self._telemetry_thread is not None and self._telemetry_thread.is_alive():
            return
        self._stop.clear()
        self._transport.start()
        self._telemetry_thread = threading.Thread(
            target=self._telemetry_loop,
            name="ground-station-telemetry",
            daemon=True,
        )
        self._telemetry_thread.start()

    def close(self) -> None:
        self._stop.set()
        self._clear_telemetry_batch()
        self._transport.stop()
        if self._telemetry_thread is not None:
            self._telemetry_thread.join(timeout=2.0)

    def get_command(self, timeout: Optional[float] = None) -> Optional[MissionCommand]:
        try:
            _, _, command = self._queue.get(timeout=timeout)
            return command
        except Empty:
            return None

    def task_done(self) -> None:
        self._queue.task_done()

    def accept(self, command: MissionCommand) -> None:
        self._send_ack(command, AckStatus.ACCEPTED)

    def reject(self, command: MissionCommand, reason: RejectReason) -> None:
        if reason == RejectReason.NONE:
            raise ValueError("rejected command requires a reason")
        self._send_ack(command, AckStatus.REJECTED, reason)

    def complete(self, command: MissionCommand) -> None:
        if command.command.command_id == CommandId.STOP_MISSION:
            self._finish_stop(AckStatus.COMPLETED, RejectReason.NONE)
            return
        self._send_ack(command, AckStatus.COMPLETED)

    def fail(self, command: MissionCommand, reason: RejectReason) -> None:
        if reason == RejectReason.NONE:
            raise ValueError("failed command requires a reason")
        if command.command.command_id == CommandId.STOP_MISSION:
            self._finish_stop(AckStatus.FAILED, reason)
            return
        self._send_ack(command, AckStatus.FAILED, reason)

    def reset_stop_latch_for_new_mission(self) -> None:
        """Arm STOP for a newly started mission; call from the mission scheduler."""
        with self._stop_lock:
            self._stop_latched = False
            self._stop_status = None
            self._stop_aliases.clear()

    def report_mission_status(
        self,
        state: MissionState,
        target1: Optional[int] = None,
        target2: Optional[int] = None,
        progress: int = 0,
        error_code: int = 0,
        message: str = "",
    ) -> bool:
        if self.mode != GroundLinkMode.TELEMETRY_TX:
            return False
        status = MissionStatus(
            state=state,
            target1=target1,
            target2=target2,
            progress=progress,
            error_code=error_code,
            message=message,
        )
        self._send_message(MessageType.MISSION_STATUS, status.to_payload())
        return True

    def report_alarm(self, code: int, message: str) -> bool:
        if self.mode != GroundLinkMode.TELEMETRY_TX:
            return False
        self._send_message(MessageType.ALARM, Alarm(code, message).to_payload())
        return True

    def report_led_control(self, control: LEDControl) -> bool:
        """Send a low-rate, authenticated GPIO18 indicator update to the ground."""
        if self.mode != GroundLinkMode.TELEMETRY_TX:
            return False
        self._send_message(MessageType.LED_CONTROL, control.to_payload())
        return True

    def send_fc_state_now(self) -> bool:
        if self.mode != GroundLinkMode.TELEMETRY_TX:
            return False
        payload = self._state_provider()
        if payload is None:
            return False
        FCStatePayload.from_payload(payload)
        frame_bytes = pack_frame(
            MessageType.FC_STATE,
            payload,
            self._session,
            self._next_frame_seq(),
            self._key,
        )
        batch = None
        with self._batch_lock:
            self._batch_buffer.extend(frame_bytes)
            self._batch_count += 1
            if self._batch_count >= 3:
                batch = bytes(self._batch_buffer)
                self._batch_buffer.clear()
                self._batch_count = 0
        if batch is not None:
            self._write(batch)
        return True

    def _clear_telemetry_batch(self) -> None:
        with self._batch_lock:
            self._batch_buffer.clear()
            self._batch_count = 0

    def _on_connected(self) -> None:
        self._peer_session = None
        logger.info("[GroundLink] HC-14 connected")

    def _on_disconnected(self, error: Optional[Exception]) -> None:
        self._peer_session = None
        if error is not None:
            logger.warning("[GroundLink] HC-14 disconnected: {}", error)

    def _on_bytes(self, data: bytes) -> None:
        now = time.monotonic()
        self._last_rx_time = now
        for frame in self._parser.feed(data):
            if frame.msg_type == MessageType.HEARTBEAT:
                self._peer_session = frame.session
            elif frame.msg_type == MessageType.COMMAND:
                if self._peer_session is None:
                    self._peer_session = frame.session
                self._handle_command(frame)

    def _handle_command(self, frame: Frame) -> None:
        try:
            command = Command.from_payload(frame.payload)
        except ValueError:
            logger.warning("[GroundLink] Invalid command payload")
            return
        # In telemetry mode START/target commands remain disabled, but STOP must
        # stay available throughout flight.  The receive thread only latches an
        # event and queues the command; it never performs a flight action.
        if (
            self.mode != GroundLinkMode.COMMAND_RX
            and command.command_id != CommandId.STOP_MISSION
        ):
            return
        with self._recent_lock:
            cached = self._recent.get(frame.session, frame.seq)
        if cached is not None:
            self._write(cached)
            return

        mission_command = MissionCommand(
            session=frame.session,
            seq=frame.seq,
            command=command,
            received_at=time.monotonic(),
        )
        self._send_ack(mission_command, AckStatus.RECEIVED)
        if self._peer_session != frame.session:
            self._send_ack(mission_command, AckStatus.REJECTED, RejectReason.LINK_DOWN)
            return
        if command.command_id == CommandId.SET_TARGETS and not self._valid_targets(command):
            self._send_ack(mission_command, AckStatus.REJECTED, RejectReason.BAD_TARGETS)
            return
        if command.command_id == CommandId.PING:
            self._send_ack(mission_command, AckStatus.ACCEPTED)
            self._send_ack(mission_command, AckStatus.COMPLETED)
            return
        if command.command_id == CommandId.STOP_MISSION:
            self._handle_stop(mission_command)
            return
        try:
            self._queue.put_nowait((10, next(self._queue_counter), mission_command))
        except Full:
            self._send_ack(mission_command, AckStatus.REJECTED, RejectReason.TASK_BUSY)

    def _handle_stop(self, command: MissionCommand) -> None:
        with self._stop_lock:
            self._stop_aliases[(command.session, command.seq)] = command
            if len(self._stop_aliases) > 64:
                first_key = next(iter(self._stop_aliases))
                del self._stop_aliases[first_key]
            if self._stop_latched:
                status = self._stop_status or AckStatus.ACCEPTED
                self._send_ack(command, status)
                return
            self._stop_latched = True
            self._stop_status = AckStatus.ACCEPTED
            if self._stop_event is not None:
                self._stop_event.set()
            try:
                self._queue.put_nowait((0, next(self._queue_counter), command))
            except Full:
                logger.warning("[GroundLink] Command queue full while STOP was latched")
            self._send_ack(command, AckStatus.ACCEPTED)

    def _finish_stop(self, status: AckStatus, reason: RejectReason) -> None:
        with self._stop_lock:
            self._stop_status = status
            aliases = list(self._stop_aliases.values())
        for command in aliases:
            self._send_ack(command, status, reason)

    @staticmethod
    def _valid_targets(command: Command) -> bool:
        return (
            command.target1 is not None
            and command.target2 is not None
            and 1 <= command.target1 <= 12
            and 1 <= command.target2 <= 12
            and command.target1 != command.target2
        )

    def _send_ack(
        self,
        command: MissionCommand,
        status: AckStatus,
        reason: RejectReason = RejectReason.NONE,
    ) -> None:
        msg_type = (
            MessageType.COMMAND_RESULT
            if status in (AckStatus.COMPLETED, AckStatus.FAILED)
            else MessageType.COMMAND_ACK
        )
        payload = CommandAck(
            command_id=command.command.command_id,
            seq=command.seq,
            status=status,
            reason=reason,
        ).to_payload()
        response = self._build_frame(msg_type, payload)
        with self._recent_lock:
            self._recent.put(command.session, command.seq, response)
        self._write(response)

    def _send_message(
        self, msg_type: MessageType, payload: bytes, flags: int = 0
    ) -> None:
        self._write(self._build_frame(msg_type, payload, flags=flags))

    def _build_frame(
        self, msg_type: MessageType, payload: bytes, flags: int = 0
    ) -> bytes:
        return pack_frame(
            msg_type,
            payload,
            self._session,
            self._next_frame_seq(),
            self._key,
            flags=flags,
        )

    def _next_frame_seq(self) -> int:
        with self._seq_lock:
            seq = self._next_seq
            self._next_seq = 1 if seq >= 0xFFFF else seq + 1
        return seq

    def _write(self, data: bytes) -> None:
        delay = self._turnaround_seconds - (time.monotonic() - self._last_rx_time)
        if delay > 0:
            self._stop.wait(delay)
        try:
            self._transport.write(data)
        except RuntimeError:
            logger.debug("[GroundLink] Drop outbound frame while HC-14 is offline")

    def _telemetry_loop(self) -> None:
        next_state = 0.0
        while not self._stop.is_set():
            if self.mode != GroundLinkMode.TELEMETRY_TX:
                self._mode_changed.wait(0.05)
                self._mode_changed.clear()
                next_state = 0.0
                continue
            now = time.monotonic()
            if not self.connected:
                self._stop.wait(0.01)
                continue
            if now >= next_state:
                if self._external_state_provider or bool(
                    self._fc is not None and getattr(self._fc, "connected", False)
                ):
                    try:
                        if self.send_fc_state_now():
                            pass
                    except (ValueError, TypeError) as exc:
                        logger.warning("[GroundLink] FC state snapshot failed: {}", exc)
                next_state = now + self._telemetry_period
            remaining = next_state - time.monotonic()
            self._stop.wait(max(0.0, min(0.01, remaining)))

    def _snapshot_fc_state(self) -> Optional[bytes]:
        if self._fc is None:
            return None
        state = self._fc.state
        first = self._read_state_values(state)
        for _ in range(2):
            second = self._read_state_values(state)
            if first == second:
                break
            first = second
        return FCStatePayload(*first).to_payload()

    @staticmethod
    def _read_state_values(state):
        return (
            state.pos_x.value,
            state.pos_y.value,
            state.bat.value,
            state.mode.value,
            state.unlock.value,
        )
