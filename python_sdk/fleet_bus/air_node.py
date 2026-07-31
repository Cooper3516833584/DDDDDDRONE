"""FleetBus V1 drone endpoint with no direct flight-control operations."""

import logging
import queue
import threading
import time
from typing import Callable, Optional

from .command_queue import AirCommandQueue
from .models import (
    AckPayload,
    AckReason,
    AckStatus,
    AirCommand,
    CommandId,
    CommandPayload,
    Frame,
    MessageKind,
    NodeId,
    NodeTiming,
    ReportPayload,
    SurveyReportPayload,
    SurveyState,
    TraceRequestPayload,
)
from .protocol import (
    FrameParser,
    ProtocolError,
    RecentResponseCache,
    decode_command,
    decode_drone_goto,
    decode_trace_request,
    encode_ack,
    encode_report,
    encode_survey_report,
    encode_trace_report,
    decode_drone_select_mission,
    new_session,
    pack_frame,
)
from .trace_buffer import (
    PoseTraceBuffer,
    PoseTraceSampler,
    TraceSamplingOptions,
    air_state_to_trace_sample,
)


LOG = logging.getLogger("fleet-air-node")


class AirFleetNode:
    """Parse in the transport callback and process/reply only in a worker thread."""

    def __init__(
        self,
        transport: object,
        state_provider: Callable[[], object],
        command_queue: AirCommandQueue,
        stop_event: threading.Event,
        timing: NodeTiming = NodeTiming(),
        readonly: bool = False,
        allow_start_mission: bool = False,
        survey_provider: Optional[Callable[[], SurveyState]] = None,
        wait: Callable[[float], None] = time.sleep,
        trace_options: TraceSamplingOptions = TraceSamplingOptions(),
        allowed_readonly_command_ids=frozenset(),
    ) -> None:
        self._transport = transport
        self._state_provider = state_provider
        self._commands = command_queue
        self._flight_stop_event = stop_event
        self._timing = timing
        self._readonly = readonly
        self._allow_start_mission = allow_start_mission
        self._allowed_readonly_command_ids = frozenset(
            int(command_id) for command_id in allowed_readonly_command_ids
        )
        self._survey_provider = survey_provider
        self._wait = wait
        self._parser = FrameParser(int(NodeId.DRONE))
        self._inbox = queue.Queue(timing.queue_size)
        self._cache = RecentResponseCache()
        self._session = new_session()
        self._stop = threading.Event()
        self._worker = None  # type: Optional[threading.Thread]
        self.write_failures = 0
        self._trace_buffer = PoseTraceBuffer(trace_options)
        self._trace_progress = threading.Condition()
        self._observed_trace_session = 0
        self._observed_after_sample_seq = 0
        self._trace_sampler = (
            PoseTraceSampler(
                state_provider=state_provider,
                trace_buffer=self._trace_buffer,
                options=trace_options,
                state_adapter=air_state_to_trace_sample,
            )
            if trace_options.enabled
            else None
        )  # type: Optional[PoseTraceSampler]

    @property
    def parser(self) -> FrameParser:
        return self._parser

    @property
    def command_queue(self) -> AirCommandQueue:
        return self._commands

    @property
    def trace_buffer(self) -> PoseTraceBuffer:
        return self._trace_buffer

    @property
    def trace_sampler(self) -> Optional[PoseTraceSampler]:
        return self._trace_sampler

    def start(self) -> None:
        if self._worker is not None:
            return
        if self._trace_sampler is not None:
            self._trace_sampler.start()
        self._stop.clear()
        self._worker = threading.Thread(
            target=self._run, name="fleet-air-node", daemon=True
        )
        self._worker.start()
        try:
            self._transport.start()
        except Exception:
            self._stop.set()
            self._worker.join(timeout=1.0)
            self._worker = None
            if self._trace_sampler is not None:
                self._trace_sampler.close()
            raise

    def close(self) -> None:
        if self._trace_sampler is not None:
            self._trace_sampler.close()
        self._transport.stop()
        self._stop.set()
        with self._trace_progress:
            self._trace_progress.notify_all()
        worker = self._worker
        if worker is not None:
            worker.join(timeout=1.0)
        self._worker = None

    def wait_for_trace_drain(self, timeout_s: float) -> bool:
        """Freeze sampling and wait until ground confirms the final cursor."""
        if timeout_s < 0:
            raise ValueError("timeout_s must not be negative")
        if self._trace_sampler is not None:
            self._trace_sampler.close()
        trace_session, latest_sample_seq = self._trace_buffer.latest_cursor()
        if latest_sample_seq == 0:
            return True

        deadline = time.monotonic() + timeout_s
        with self._trace_progress:
            while True:
                if (
                    self._observed_trace_session == trace_session
                    and self._observed_after_sample_seq >= latest_sample_seq
                ):
                    return True
                if self._stop.is_set():
                    return False
                remaining_s = deadline - time.monotonic()
                if remaining_s <= 0:
                    return False
                self._trace_progress.wait(min(remaining_s, 0.1))

    def feed_bytes(self, data: bytes) -> None:
        """Transport callback: validate/address/queue, without waiting or writing."""
        for frame in self._parser.feed(data):
            try:
                self._inbox.put_nowait(frame)
            except queue.Full:
                continue

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                frame = self._inbox.get(timeout=0.1)
            except queue.Empty:
                continue
            try:
                response = self._handle(frame)
            except ProtocolError:
                continue
            if response is None or self._stop.is_set():
                continue
            self._wait(self._timing.turnaround_s)
            if not self._stop.is_set():
                try:
                    self._transport.write(response)
                except Exception:
                    self.write_failures += 1
                    LOG.exception(
                        "FleetBus response write failed; keeping reply worker active"
                    )

    def _handle(self, frame: Frame) -> Optional[bytes]:
        if frame.src != int(NodeId.GROUND):
            return None
        self._cache.begin_ground_session(frame.session)
        trace_request = None
        if frame.kind == int(MessageKind.TRACE_REQUEST):
            trace_request = decode_trace_request(frame.payload)
            self._observe_trace_cursor(trace_request)
        cached = self._cache.get(frame.session, frame.seq)
        if cached is not None:
            return cached
        if frame.kind == int(MessageKind.POLL):
            response = self._report(frame)
        elif frame.kind == int(MessageKind.SURVEY_REQUEST):
            response = self._survey_report(frame)
        elif frame.kind == int(MessageKind.TRACE_REQUEST):
            assert trace_request is not None
            response = self._trace_report(frame, trace_request)
        elif frame.kind == int(MessageKind.COMMAND):
            response = self._command(frame)
        else:
            return None
        self._cache.put(frame.session, frame.seq, response)
        return response

    def _report(self, request: Frame) -> bytes:
        state = self._state_provider()
        command_status = self._commands.status()
        payload = encode_report(
            ReportPayload(
                request.session,
                request.seq,
                state.node_flags,
                state.node_uptime_ms,
                state.x_cm,
                state.y_cm,
                state.z_cm,
                state.heading_cdeg,
                state.vx_cm_s,
                state.vy_cm_s,
                state.vz_cm_s,
                state.battery_cV,
                state.operation_state,
                state.pose_quality,
                command_status.active_command_seq,
                command_status.status,
                state.error_code or command_status.error_code,
            )
        )
        return self._response(request, MessageKind.REPORT, payload)

    def _survey_report(self, request: Frame) -> bytes:
        state = SurveyState() if self._survey_provider is None else self._survey_provider()
        payload = encode_survey_report(
            SurveyReportPayload(
                request.session,
                request.seq,
                state.survey_revision,
                state.survey_flags,
                state.wildfire_event_id,
                state.wildfire_row,
                state.wildfire_col,
                state.debris_event_id,
                state.debris_row,
                state.debris_col,
                state.terrain_codes,
                state.cell_positions_cm,
            )
        )
        return self._response(request, MessageKind.SURVEY_REPORT, payload)

    def _observe_trace_cursor(self, trace_request: TraceRequestPayload) -> None:
        trace_session, _ = self._trace_buffer.latest_cursor()
        if trace_request.known_trace_session != trace_session:
            return
        with self._trace_progress:
            if self._observed_trace_session != trace_session:
                self._observed_trace_session = trace_session
                self._observed_after_sample_seq = 0
            self._observed_after_sample_seq = max(
                self._observed_after_sample_seq,
                trace_request.after_sample_seq,
            )
            self._trace_progress.notify_all()

    def _trace_report(
        self,
        request: Frame,
        trace_request: TraceRequestPayload,
    ) -> bytes:
        report = self._trace_buffer.build_report(
            request.session,
            request.seq,
            trace_request,
        )
        payload = encode_trace_report(report)
        return self._response(request, MessageKind.TRACE_REPORT, payload)

    def _command(self, request: Frame) -> bytes:
        try:
            command = decode_command(request.payload)
            command_body = self._decode_command_body(command)
        except ProtocolError:
            command_id = request.payload[0] if request.payload else 0
            return self._ack(
                request, command_id, AckStatus.REJECTED, AckReason.BAD_PAYLOAD
            )

        if command.command_id == int(CommandId.PING):
            return self._ack(
                request, command.command_id, AckStatus.COMPLETED, AckReason.NONE
            )

        if (
            command.command_id == int(CommandId.DRONE_SELECT_MISSION)
            and command.command_id not in self._allowed_readonly_command_ids
        ):
            return self._ack(
                request, command.command_id, AckStatus.REJECTED, AckReason.UNSUPPORTED
            )

        if (
            self._readonly
            and command.command_id == int(CommandId.TARGETED_STOP)
        ):
            self._flight_stop_event.set()
            return self._ack(
                request, command.command_id, AckStatus.COMPLETED, AckReason.NONE
            )

        if self._readonly and not (
            command.command_id in self._allowed_readonly_command_ids
            or (self._allow_start_mission and command.command_id in (
                int(CommandId.DRONE_START_MISSION),
                int(CommandId.DRONE_PREPARE_MISSION),
            ))
        ):
            return self._ack(
                request, command.command_id, AckStatus.REJECTED, AckReason.UNSUPPORTED
            )

        if command.command_id != int(CommandId.TARGETED_STOP):
            state = self._state_provider()
            if (
                command.command_id not in (
                    int(CommandId.DRONE_START_MISSION),
                    int(CommandId.DRONE_PREPARE_MISSION),
                    int(CommandId.DRONE_SELECT_MISSION),
                )
                and not state.node_flags & 0x0001
            ):
                return self._ack(
                    request,
                    command.command_id,
                    AckStatus.REJECTED,
                    AckReason.LOCALIZATION_INVALID,
                )

        queued = AirCommand(
            request.session, request.seq, command.command_id, command_body
        )
        if command.command_id == int(CommandId.TARGETED_STOP):
            self._flight_stop_event.set()
        if not self._commands.put(queued):
            return self._ack(
                request, command.command_id, AckStatus.REJECTED, AckReason.BUSY
            )
        self._commands.accept(queued)
        return self._ack(
            request, command.command_id, AckStatus.ACCEPTED, AckReason.NONE
        )

    @staticmethod
    def _decode_command_body(command: CommandPayload) -> object:
        if command.command_id == int(CommandId.DRONE_GOTO):
            return decode_drone_goto(command.command_body)
        if command.command_id == int(CommandId.DRONE_SELECT_MISSION):
            return decode_drone_select_mission(command.command_body)
        if command.command_id in (
            int(CommandId.PING),
            int(CommandId.TARGETED_STOP),
            int(CommandId.DRONE_HOLD),
            int(CommandId.CANCEL_TASK),
            int(CommandId.DRONE_START_MISSION),
            int(CommandId.DRONE_PREPARE_MISSION),
        ):
            if command.command_body:
                raise ProtocolError("payload", "command body must be empty")
            return None
        raise ProtocolError("payload", "unsupported command")

    def _ack(
        self,
        request: Frame,
        command_id: int,
        status: AckStatus,
        reason: AckReason,
    ) -> bytes:
        return self._response(
            request,
            MessageKind.ACK,
            encode_ack(
                AckPayload(
                    request.session,
                    request.seq,
                    command_id,
                    int(status),
                    int(reason),
                )
            ),
        )

    def _response(self, request: Frame, kind: MessageKind, payload: bytes) -> bytes:
        return pack_frame(
            Frame(
                version=1,
                src=int(NodeId.DRONE),
                dst=int(NodeId.GROUND),
                kind=int(kind),
                flags=0,
                session=self._session,
                seq=request.seq,
                payload=payload,
            )
        )


def attach_air_fleet_node(
    fc: object,
    navigation: object,
    stop_event: threading.Event,
    readonly: bool = False,
    allow_start_mission: bool = False,
    survey_provider: Optional[Callable[[], SurveyState]] = None,
    position_transform: Optional[Callable] = None,
    heading_offset_deg: float = 0.0,
    state_provider: Optional[Callable[[], object]] = None,
    trace_options: TraceSamplingOptions = TraceSamplingOptions(),
    allowed_readonly_command_ids=frozenset(),
    hc14_port: Optional[str] = None,
    hc14_baudrate: Optional[int] = None,
    transport_factory=None,
) -> AirFleetNode:
    """Create the airborne FleetBus endpoint on its direct CH340/HC-14 link.

    ``state_provider`` lets a task add task-defined report fields without
    changing the navigation pose conversion shared by other missions.
    """
    from .hc14_transport import (
        HC14FleetTransport,
        resolve_hc14_settings,
    )

    from .pose_provider import NavigationAirStateProvider

    holder = {}
    port, baudrate = resolve_hc14_settings(hc14_port, hc14_baudrate)
    factory = HC14FleetTransport if transport_factory is None else transport_factory
    transport = factory(
        port=port,
        baudrate=baudrate,
        on_bytes=lambda data: holder["node"].feed_bytes(data),
        on_connected=lambda: LOG.info(
            "Airborne HC-14 connected directly on %s at %s baud",
            port,
            baudrate,
        ),
        on_disconnected=lambda error: LOG.warning(
            "Airborne HC-14 direct link disconnected: %s",
            error,
        ),
    )
    commands = AirCommandQueue()
    default_state_provider = NavigationAirStateProvider(
        fc,
        navigation,
        position_transform=position_transform,
        heading_offset_deg=heading_offset_deg,
    )
    node = AirFleetNode(
        transport,
        default_state_provider if state_provider is None else state_provider,
        commands,
        stop_event,
        readonly=readonly,
        allow_start_mission=allow_start_mission,
        allowed_readonly_command_ids=allowed_readonly_command_ids,
        survey_provider=survey_provider,
        trace_options=trace_options,
    )
    holder["node"] = node
    node.start()
    return node
