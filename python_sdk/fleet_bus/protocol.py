"""Hardware-free FleetBus V1 framing and payload codecs."""

from collections import OrderedDict
import secrets
import struct
from typing import List, Optional, Tuple

from .models import (
    AckPayload,
    CarNavigateCommand,
    CommandPayload,
    CoordinateFrameCommand,
    DisasterRescueCommand,
    DroneGotoCommand,
    Frame,
    GoalFlags,
    MapReportPayload,
    ParserStats,
    PathReportPayload,
    PollPayload,
    ReportPayload,
    SurveyReportPayload,
    TraceReportFlags,
    TraceReportPayload,
    TraceRequestPayload,
    TraceSample,
    MissionId,
)


MAGIC = b"\xD3\x91"
TAIL = b"\x1D\x0F"
VERSION = 1
MAX_PAYLOAD_LEN = 220
MAX_INNER_FRAME_LEN = 239

HEADER = struct.Struct("<2sBBBBBIHH")
CRC = struct.Struct("<H")
POLL = struct.Struct("<H")
REPORT = struct.Struct("<IHHIiiiHhhhHBBHBB")
ACK_HEADER = struct.Struct("<IHBBBB")
COORDINATE_FRAME = struct.Struct("<iiH")
CAR_NAVIGATE = struct.Struct("<Bii")
DRONE_GOTO = struct.Struct("<Biii")
HEADING = struct.Struct("<H")
POINT_REPORT_HEADER = struct.Struct("<IHIB")
POINT = struct.Struct("<ii")
SURVEY_REPORT_HEADER = struct.Struct("<IHHBHBBHBB")
DISASTER_RESCUE_HEADER = struct.Struct("<HBB")
SURVEY_CELL_COUNT = 15
TRACE_REQUEST = struct.Struct("<IIBB")
TRACE_REPORT_HEADER = struct.Struct("<IHIIIIBB")
TRACE_SAMPLE_ABSOLUTE = struct.Struct("<IiiiHBB")
TRACE_SAMPLE_DELTA = struct.Struct("<HhhhHBB")
TRACE_MAX_SAMPLES = 15
TRACE_REPORT_FLAG_MASK = int(
    TraceReportFlags.MORE_PENDING
    | TraceReportFlags.CURSOR_RESET
    | TraceReportFlags.BUFFER_OVERRUN
)

FIXED_HEADER_LEN = HEADER.size
FRAME_OVERHEAD = FIXED_HEADER_LEN + CRC.size + len(TAIL)


class ProtocolError(ValueError):
    def __init__(self, code: str, message: str) -> None:
        super().__init__(message)
        self.code = code


def _require_range(name: str, value: int, minimum: int, maximum: int) -> int:
    if not isinstance(value, int) or not minimum <= value <= maximum:
        raise ProtocolError(
            "range", "{} must be between {} and {}".format(name, minimum, maximum)
        )
    return value


def _require_u8(name: str, value: int) -> int:
    return _require_range(name, value, 0, 0xFF)


def _require_u16(name: str, value: int) -> int:
    return _require_range(name, value, 0, 0xFFFF)


def _require_u32(name: str, value: int) -> int:
    return _require_range(name, value, 0, 0xFFFFFFFF)


def _require_i16(name: str, value: int) -> int:
    return _require_range(name, value, -0x8000, 0x7FFF)


def _require_i32(name: str, value: int) -> int:
    return _require_range(name, value, -0x80000000, 0x7FFFFFFF)


def _require_heading(value: int) -> int:
    return _require_range("heading_cdeg", value, 0, 35999)


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for byte in bytes(data):
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def new_session() -> int:
    return secrets.randbits(32)


class SequenceCounter:
    """Allocate 1..65535 and wrap without ever returning the reserved value 0."""

    def __init__(self, initial: int = 0) -> None:
        self._value = _require_u16("initial", initial)

    def next(self) -> int:
        self._value = (self._value % 0xFFFF) + 1
        return self._value


class RecentResponseCache:
    """LRU response cache keyed by the current ground session and request seq."""

    def __init__(self, max_items: int = 64) -> None:
        if max_items <= 0:
            raise ValueError("max_items must be positive")
        self._max_items = max_items
        self._ground_session = None  # type: Optional[int]
        self._items = OrderedDict()  # type: OrderedDict[Tuple[int, int], bytes]

    def begin_ground_session(self, session: int) -> None:
        session = _require_u32("session", session)
        if session != self._ground_session:
            self._ground_session = session
            self._items.clear()

    def get(self, session: int, seq: int) -> Optional[bytes]:
        key = (_require_u32("session", session), _require_u16("seq", seq))
        value = self._items.get(key)
        if value is not None:
            self._items.move_to_end(key)
        return value

    def put(self, session: int, seq: int, response: bytes) -> None:
        self.begin_ground_session(session)
        key = (session, _require_u16("seq", seq))
        self._items[key] = bytes(response)
        self._items.move_to_end(key)
        while len(self._items) > self._max_items:
            self._items.popitem(last=False)


def pack_frame(frame: Frame) -> bytes:
    payload = bytes(frame.payload)
    if len(payload) > MAX_PAYLOAD_LEN:
        raise ProtocolError(
            "oversize",
            "payload too large: {} > {}".format(len(payload), MAX_PAYLOAD_LEN),
        )
    header = HEADER.pack(
        MAGIC,
        _require_u8("version", frame.version),
        _require_u8("src", frame.src),
        _require_u8("dst", frame.dst),
        _require_u8("kind", frame.kind),
        _require_u8("flags", frame.flags),
        _require_u32("session", frame.session),
        _require_u16("seq", frame.seq),
        len(payload),
    )
    protected = header[2:] + payload
    packed = header + payload + CRC.pack(crc16_ccitt_false(protected)) + TAIL
    if len(packed) > MAX_INNER_FRAME_LEN:
        raise ProtocolError("oversize", "inner frame exceeds FleetBus V1 limit")
    return packed


def unpack_frame(frame_bytes: bytes) -> Frame:
    data = bytes(frame_bytes)
    if len(data) < FRAME_OVERHEAD:
        raise ProtocolError("truncated", "frame is too short")
    try:
        magic, version, src, dst, kind, flags, session, seq, payload_len = (
            HEADER.unpack_from(data)
        )
    except struct.error as exc:
        raise ProtocolError("truncated", "frame header is incomplete") from exc
    if magic != MAGIC:
        raise ProtocolError("magic", "invalid FleetBus magic")
    if payload_len > MAX_PAYLOAD_LEN:
        raise ProtocolError("oversize", "payload length exceeds FleetBus V1 limit")
    expected_len = FRAME_OVERHEAD + payload_len
    if len(data) != expected_len:
        raise ProtocolError("length", "frame length does not match payload_len")
    if data[-len(TAIL) :] != TAIL:
        raise ProtocolError("tail", "invalid FleetBus tail")
    payload_end = FIXED_HEADER_LEN + payload_len
    expected_crc = CRC.unpack_from(data, payload_end)[0]
    actual_crc = crc16_ccitt_false(data[2:payload_end])
    if expected_crc != actual_crc:
        raise ProtocolError("crc", "FleetBus CRC mismatch")
    if version != VERSION:
        raise ProtocolError("version", "unsupported FleetBus version")
    return Frame(
        version=version,
        src=src,
        dst=dst,
        kind=kind,
        flags=flags,
        session=session,
        seq=seq,
        payload=data[FIXED_HEADER_LEN:payload_end],
    )


class FrameParser:
    def __init__(self, local_node: Optional[int] = None) -> None:
        self._buffer = bytearray()
        self._local_node = (
            None if local_node is None else _require_u8("local_node", local_node)
        )
        self.stats = ParserStats()

    def feed(self, data: bytes) -> List[Frame]:
        self._buffer.extend(data)
        frames = []  # type: List[Frame]
        while True:
            start = self._buffer.find(MAGIC)
            if start < 0:
                keep = 1 if self._buffer.endswith(MAGIC[:1]) else 0
                discarded = len(self._buffer) - keep
                self.stats.discarded_bytes += discarded
                if keep:
                    del self._buffer[:-keep]
                else:
                    self._buffer.clear()
                return frames
            if start:
                self.stats.discarded_bytes += start
                del self._buffer[:start]
            if len(self._buffer) < FIXED_HEADER_LEN:
                return frames
            payload_len = int.from_bytes(self._buffer[13:15], "little")
            if payload_len > MAX_PAYLOAD_LEN:
                self.stats.oversize_frames += 1
                self._discard_candidate_start()
                continue
            total_len = FRAME_OVERHEAD + payload_len
            if len(self._buffer) < total_len:
                return frames
            candidate = bytes(self._buffer[:total_len])
            try:
                frame = unpack_frame(candidate)
            except ProtocolError as exc:
                self._record_failure(exc.code)
                self._discard_candidate_start()
                continue
            del self._buffer[:total_len]
            if self._local_node is not None and frame.dst != self._local_node:
                self.stats.address_drops += 1
                continue
            frames.append(frame)

    def _discard_candidate_start(self) -> None:
        del self._buffer[0]
        self.stats.discarded_bytes += 1

    def _record_failure(self, code: str) -> None:
        if code == "crc":
            self.stats.crc_failures += 1
        elif code == "tail":
            self.stats.tail_failures += 1
        elif code == "version":
            self.stats.version_failures += 1
        elif code == "oversize":
            self.stats.oversize_frames += 1


def encode_poll(payload: PollPayload) -> bytes:
    return POLL.pack(_require_u16("request_flags", payload.request_flags))


def decode_poll(data: bytes) -> PollPayload:
    if len(data) != POLL.size:
        raise ProtocolError("payload", "POLL payload must be 2 bytes")
    return PollPayload(POLL.unpack(data)[0])


def encode_report(payload: ReportPayload) -> bytes:
    return REPORT.pack(
        _require_u32("request_session", payload.request_session),
        _require_u16("request_seq", payload.request_seq),
        _require_u16("node_flags", payload.node_flags),
        _require_u32("node_uptime_ms", payload.node_uptime_ms),
        _require_i32("x_cm", payload.x_cm),
        _require_i32("y_cm", payload.y_cm),
        _require_i32("z_cm", payload.z_cm),
        _require_heading(payload.heading_cdeg),
        _require_i16("vx_cm_s", payload.vx_cm_s),
        _require_i16("vy_cm_s", payload.vy_cm_s),
        _require_i16("vz_cm_s", payload.vz_cm_s),
        _require_u16("battery_cV", payload.battery_cV),
        _require_u8("operation_state", payload.operation_state),
        _require_range("pose_quality", payload.pose_quality, 0, 4),
        _require_u16("active_command_seq", payload.active_command_seq),
        _require_u8("active_command_status", payload.active_command_status),
        _require_u8("error_code", payload.error_code),
    )


def decode_report(data: bytes) -> ReportPayload:
    if len(data) != REPORT.size:
        raise ProtocolError(
            "payload", "REPORT payload must be {} bytes".format(REPORT.size)
        )
    payload = ReportPayload(*REPORT.unpack(data))
    _require_heading(payload.heading_cdeg)
    _require_range("pose_quality", payload.pose_quality, 0, 4)
    return payload


def encode_trace_request(payload: TraceRequestPayload) -> bytes:
    max_samples = _require_range(
        "max_samples", payload.max_samples, 1, TRACE_MAX_SAMPLES
    )
    flags = _require_u8("flags", payload.flags)
    if flags:
        raise ProtocolError("payload", "TRACE_REQUEST flags must be zero")
    return TRACE_REQUEST.pack(
        _require_u32("known_trace_session", payload.known_trace_session),
        _require_u32("after_sample_seq", payload.after_sample_seq),
        max_samples,
        flags,
    )


def decode_trace_request(data: bytes) -> TraceRequestPayload:
    if len(data) != TRACE_REQUEST.size:
        raise ProtocolError(
            "payload",
            "TRACE_REQUEST payload must be {} bytes".format(TRACE_REQUEST.size),
        )
    payload = TraceRequestPayload(*TRACE_REQUEST.unpack(data))
    encode_trace_request(payload)
    return payload


def validate_trace_sample(sample: TraceSample) -> None:
    _require_u32("uptime_ms", sample.uptime_ms)
    _require_i32("x_cm", sample.x_cm)
    _require_i32("y_cm", sample.y_cm)
    _require_i32("z_cm", sample.z_cm)
    _require_heading(sample.heading_cdeg)
    _require_range("quality", sample.quality, 0, 4)
    _require_u8("sample_flags", sample.flags)


def _validate_trace_report_header(
    payload: TraceReportPayload, sample_count: int
) -> None:
    _require_u32("request_session", payload.request_session)
    _require_u16("request_seq", payload.request_seq)
    _require_u32("trace_session", payload.trace_session)
    _require_u32("oldest_available_seq", payload.oldest_available_seq)
    _require_u32("first_sample_seq", payload.first_sample_seq)
    _require_u32("latest_available_seq", payload.latest_available_seq)
    report_flags = _require_u8("report_flags", payload.report_flags)
    if report_flags & ~TRACE_REPORT_FLAG_MASK:
        raise ProtocolError("payload", "TRACE_REPORT contains unknown flags")
    if not 0 <= sample_count <= TRACE_MAX_SAMPLES:
        raise ProtocolError(
            "payload",
            "TRACE_REPORT sample count must be between 0 and {}".format(
                TRACE_MAX_SAMPLES
            ),
        )
    if sample_count == 0:
        if payload.first_sample_seq != 0:
            raise ProtocolError(
                "payload", "empty TRACE_REPORT must have first_sample_seq zero"
            )
        return
    if payload.first_sample_seq == 0:
        raise ProtocolError(
            "payload", "non-empty TRACE_REPORT must have a nonzero first sample"
        )
    last_sample_seq = payload.first_sample_seq + sample_count - 1
    if last_sample_seq > 0xFFFFFFFF:
        raise ProtocolError("range", "TRACE_REPORT sample sequence overflows uint32")
    if payload.latest_available_seq < last_sample_seq:
        raise ProtocolError(
            "payload", "TRACE_REPORT samples exceed latest_available_seq"
        )


def encode_trace_report(payload: TraceReportPayload) -> bytes:
    samples = tuple(payload.samples)
    sample_count = len(samples)
    _validate_trace_report_header(payload, sample_count)
    encoded = bytearray(
        TRACE_REPORT_HEADER.pack(
            payload.request_session,
            payload.request_seq,
            payload.trace_session,
            payload.oldest_available_seq,
            payload.first_sample_seq,
            payload.latest_available_seq,
            sample_count,
            payload.report_flags,
        )
    )
    if samples:
        first = samples[0]
        validate_trace_sample(first)
        encoded.extend(
            TRACE_SAMPLE_ABSOLUTE.pack(
                first.uptime_ms,
                first.x_cm,
                first.y_cm,
                first.z_cm,
                first.heading_cdeg,
                first.quality,
                first.flags,
            )
        )
        previous = first
        for sample in samples[1:]:
            validate_trace_sample(sample)
            dt_ms = _require_range(
                "dt_ms", sample.uptime_ms - previous.uptime_ms, 1, 0xFFFF
            )
            dx_cm = _require_i16("dx_cm", sample.x_cm - previous.x_cm)
            dy_cm = _require_i16("dy_cm", sample.y_cm - previous.y_cm)
            dz_cm = _require_i16("dz_cm", sample.z_cm - previous.z_cm)
            encoded.extend(
                TRACE_SAMPLE_DELTA.pack(
                    dt_ms,
                    dx_cm,
                    dy_cm,
                    dz_cm,
                    sample.heading_cdeg,
                    sample.quality,
                    sample.flags,
                )
            )
            previous = sample
    if len(encoded) > MAX_PAYLOAD_LEN:
        raise ProtocolError("oversize", "TRACE_REPORT exceeds FleetBus V1 limit")
    return bytes(encoded)


def decode_trace_report(data: bytes) -> TraceReportPayload:
    payload_data = bytes(data)
    if len(payload_data) > MAX_PAYLOAD_LEN:
        raise ProtocolError("oversize", "TRACE_REPORT exceeds FleetBus V1 limit")
    if len(payload_data) < TRACE_REPORT_HEADER.size:
        raise ProtocolError("payload", "TRACE_REPORT payload is too short")
    (
        request_session,
        request_seq,
        trace_session,
        oldest_available_seq,
        first_sample_seq,
        latest_available_seq,
        sample_count,
        report_flags,
    ) = TRACE_REPORT_HEADER.unpack_from(payload_data)
    expected_size = TRACE_REPORT_HEADER.size
    if sample_count:
        expected_size += TRACE_SAMPLE_ABSOLUTE.size
        expected_size += (sample_count - 1) * TRACE_SAMPLE_DELTA.size
    if len(payload_data) != expected_size:
        raise ProtocolError(
            "payload", "TRACE_REPORT sample count/length mismatch"
        )
    provisional = TraceReportPayload(
        request_session,
        request_seq,
        trace_session,
        oldest_available_seq,
        first_sample_seq,
        latest_available_seq,
        report_flags,
        (),
    )
    _validate_trace_report_header(provisional, sample_count)
    if not sample_count:
        return provisional

    values = TRACE_SAMPLE_ABSOLUTE.unpack_from(
        payload_data, TRACE_REPORT_HEADER.size
    )
    first = TraceSample(*values)
    validate_trace_sample(first)
    samples = [first]
    previous = first
    offset = TRACE_REPORT_HEADER.size + TRACE_SAMPLE_ABSOLUTE.size
    for _ in range(1, sample_count):
        (
            dt_ms,
            dx_cm,
            dy_cm,
            dz_cm,
            heading_cdeg,
            quality,
            sample_flags,
        ) = TRACE_SAMPLE_DELTA.unpack_from(payload_data, offset)
        _require_range("dt_ms", dt_ms, 1, 0xFFFF)
        sample = TraceSample(
            _require_u32("uptime_ms", previous.uptime_ms + dt_ms),
            _require_i32("x_cm", previous.x_cm + dx_cm),
            _require_i32("y_cm", previous.y_cm + dy_cm),
            _require_i32("z_cm", previous.z_cm + dz_cm),
            heading_cdeg,
            quality,
            sample_flags,
        )
        validate_trace_sample(sample)
        samples.append(sample)
        previous = sample
        offset += TRACE_SAMPLE_DELTA.size
    return TraceReportPayload(
        request_session,
        request_seq,
        trace_session,
        oldest_available_seq,
        first_sample_seq,
        latest_available_seq,
        report_flags,
        tuple(samples),
    )


def encode_command(payload: CommandPayload) -> bytes:
    body = bytes(payload.command_body)
    encoded = bytes(
        (
            _require_u8("command_id", payload.command_id),
            _require_u8("command_flags", payload.command_flags),
        )
    ) + body
    if len(encoded) > MAX_PAYLOAD_LEN:
        raise ProtocolError("oversize", "COMMAND payload exceeds FleetBus V1 limit")
    return encoded


def decode_command(data: bytes) -> CommandPayload:
    if len(data) < 2:
        raise ProtocolError("payload", "COMMAND payload is too short")
    return CommandPayload(data[0], data[1], bytes(data[2:]))


def encode_drone_select_mission(mission_id: int) -> bytes:
    try:
        return bytes((int(MissionId(mission_id)),))
    except (TypeError, ValueError, OverflowError) as exc:
        raise ProtocolError("payload", "DRONE_SELECT_MISSION mission id must be 1 or 2") from exc


def decode_drone_select_mission(data: bytes) -> MissionId:
    if len(data) != 1:
        raise ProtocolError("payload", "DRONE_SELECT_MISSION body must be one byte")
    try:
        return MissionId(data[0])
    except ValueError as exc:
        raise ProtocolError("payload", "unknown DRONE_SELECT_MISSION mission id") from exc


def encode_ack(payload: AckPayload) -> bytes:
    detail = payload.detail.encode("utf-8")
    if len(detail) > 0xFF:
        raise ProtocolError("payload", "ACK detail exceeds 255 UTF-8 bytes")
    encoded = ACK_HEADER.pack(
        _require_u32("request_session", payload.request_session),
        _require_u16("request_seq", payload.request_seq),
        _require_u8("command_id", payload.command_id),
        _require_u8("status", payload.status),
        _require_u8("reason", payload.reason),
        len(detail),
    ) + detail
    if len(encoded) > MAX_PAYLOAD_LEN:
        raise ProtocolError("oversize", "ACK payload exceeds FleetBus V1 limit")
    return encoded


def decode_ack(data: bytes) -> AckPayload:
    if len(data) < ACK_HEADER.size:
        raise ProtocolError("payload", "ACK payload is too short")
    values = ACK_HEADER.unpack_from(data)
    detail_len = values[-1]
    if len(data) != ACK_HEADER.size + detail_len:
        raise ProtocolError("payload", "ACK detail length mismatch")
    try:
        detail = data[ACK_HEADER.size :].decode("utf-8")
    except UnicodeDecodeError as exc:
        raise ProtocolError("payload", "ACK detail is not valid UTF-8") from exc
    return AckPayload(*values[:-1], detail=detail)


def encode_coordinate_frame(command: CoordinateFrameCommand) -> bytes:
    return COORDINATE_FRAME.pack(
        _require_i32("origin_x_cm", command.origin_x_cm),
        _require_i32("origin_y_cm", command.origin_y_cm),
        _require_heading(command.startup_x_heading_cdeg),
    )


def decode_coordinate_frame(data: bytes) -> CoordinateFrameCommand:
    if len(data) != COORDINATE_FRAME.size:
        raise ProtocolError(
            "payload",
            "SET_COORDINATE_FRAME body must be {} bytes".format(
                COORDINATE_FRAME.size
            ),
        )
    command = CoordinateFrameCommand(*COORDINATE_FRAME.unpack(data))
    _require_heading(command.startup_x_heading_cdeg)
    return command


def encode_car_navigate(command: CarNavigateCommand) -> bytes:
    flags = 0
    body = CAR_NAVIGATE.pack(
        0,
        _require_i32("x_cm", command.x_cm),
        _require_i32("y_cm", command.y_cm),
    )
    if command.heading_cdeg is not None:
        flags = int(GoalFlags.HAS_FINAL_HEADING)
        body = bytes((flags,)) + body[1:] + HEADING.pack(
            _require_heading(command.heading_cdeg)
        )
    return body


def decode_car_navigate(data: bytes) -> CarNavigateCommand:
    if len(data) not in (CAR_NAVIGATE.size, CAR_NAVIGATE.size + HEADING.size):
        raise ProtocolError("payload", "CAR_NAVIGATE_TO body has invalid length")
    flags, x_cm, y_cm = CAR_NAVIGATE.unpack_from(data)
    if flags & ~int(GoalFlags.HAS_FINAL_HEADING):
        raise ProtocolError("payload", "CAR_NAVIGATE_TO contains unknown flags")
    has_heading = bool(flags & int(GoalFlags.HAS_FINAL_HEADING))
    if has_heading != (len(data) == CAR_NAVIGATE.size + HEADING.size):
        raise ProtocolError(
            "payload", "CAR_NAVIGATE_TO heading flag/length mismatch"
        )
    heading = None
    if has_heading:
        heading = HEADING.unpack_from(data, CAR_NAVIGATE.size)[0]
        _require_heading(heading)
    return CarNavigateCommand(x_cm, y_cm, heading)


def encode_drone_goto(command: DroneGotoCommand) -> bytes:
    flags = 0
    body = DRONE_GOTO.pack(
        0,
        _require_i32("x_cm", command.x_cm),
        _require_i32("y_cm", command.y_cm),
        _require_i32("z_cm", command.z_cm),
    )
    if command.heading_cdeg is not None:
        flags = int(GoalFlags.HAS_FINAL_HEADING)
        body = bytes((flags,)) + body[1:] + HEADING.pack(
            _require_heading(command.heading_cdeg)
        )
    return body


def decode_drone_goto(data: bytes) -> DroneGotoCommand:
    if len(data) not in (DRONE_GOTO.size, DRONE_GOTO.size + HEADING.size):
        raise ProtocolError("payload", "DRONE_GOTO body has invalid length")
    flags, x_cm, y_cm, z_cm = DRONE_GOTO.unpack_from(data)
    if flags & ~int(GoalFlags.HAS_FINAL_HEADING):
        raise ProtocolError("payload", "DRONE_GOTO contains unknown flags")
    has_heading = bool(flags & int(GoalFlags.HAS_FINAL_HEADING))
    if has_heading != (len(data) == DRONE_GOTO.size + HEADING.size):
        raise ProtocolError("payload", "DRONE_GOTO heading flag/length mismatch")
    heading = None
    if has_heading:
        heading = HEADING.unpack_from(data, DRONE_GOTO.size)[0]
        _require_heading(heading)
    return DroneGotoCommand(x_cm, y_cm, z_cm, heading)


def _encode_terrain_codes(terrain_codes: Tuple[int, ...]) -> bytes:
    if len(terrain_codes) != SURVEY_CELL_COUNT:
        raise ProtocolError("payload", "terrain grid must contain 15 cells")
    return bytes(_require_range("terrain_code", value, 0, 7) for value in terrain_codes)


def encode_disaster_rescue(command: DisasterRescueCommand) -> bytes:
    return DISASTER_RESCUE_HEADER.pack(
        _require_range("event_id", command.event_id, 1, 0xFFFF),
        _require_range("wildfire_row", command.wildfire_row, 0, 2),
        _require_range("wildfire_col", command.wildfire_col, 0, 4),
    ) + _encode_terrain_codes(command.terrain_codes)


def decode_disaster_rescue(data: bytes) -> DisasterRescueCommand:
    expected = DISASTER_RESCUE_HEADER.size + SURVEY_CELL_COUNT
    if len(data) != expected:
        raise ProtocolError("payload", "CAR_DISASTER_RESCUE body has invalid length")
    event_id, row, col = DISASTER_RESCUE_HEADER.unpack_from(data)
    if row > 2 or col > 4:
        raise ProtocolError("payload", "wildfire cell is outside the 3x5 survey grid")
    return DisasterRescueCommand(event_id, row, col, tuple(data[DISASTER_RESCUE_HEADER.size:]))


def encode_survey_report(payload: SurveyReportPayload) -> bytes:
    for name, event_id, row, col in (
        ("wildfire", payload.wildfire_event_id, payload.wildfire_row, payload.wildfire_col),
        ("debris", payload.debris_event_id, payload.debris_row, payload.debris_col),
    ):
        _require_u16(name + "_event_id", event_id)
        if event_id:
            _require_range(name + "_row", row, 0, 2)
            _require_range(name + "_col", col, 0, 4)
        elif row != 0xFF or col != 0xFF:
            raise ProtocolError("payload", name + " event cell must be 255 when absent")
    encoded = SURVEY_REPORT_HEADER.pack(
        _require_u32("request_session", payload.request_session),
        _require_u16("request_seq", payload.request_seq),
        _require_u16("survey_revision", payload.survey_revision),
        _require_u8("survey_flags", payload.survey_flags),
        payload.wildfire_event_id,
        payload.wildfire_row,
        payload.wildfire_col,
        payload.debris_event_id,
        payload.debris_row,
        payload.debris_col,
    ) + _encode_terrain_codes(payload.terrain_codes)
    has_positions = bool(payload.survey_flags & 0x02)
    if has_positions != bool(payload.cell_positions_cm):
        raise ProtocolError(
            "payload", "survey absolute-position flag/data mismatch"
        )
    if has_positions:
        if len(payload.cell_positions_cm) != SURVEY_CELL_COUNT:
            raise ProtocolError(
                "payload", "survey positions must contain 15 cells"
            )
        encoded += b"".join(
            POINT.pack(
                _require_i32("cell_x_cm", x_cm),
                _require_i32("cell_y_cm", y_cm),
            )
            for x_cm, y_cm in payload.cell_positions_cm
        )
    return encoded


def decode_survey_report(data: bytes) -> SurveyReportPayload:
    base_size = SURVEY_REPORT_HEADER.size + SURVEY_CELL_COUNT
    if len(data) < base_size:
        raise ProtocolError("payload", "SURVEY_REPORT payload has invalid length")
    values = SURVEY_REPORT_HEADER.unpack_from(data)
    has_positions = bool(values[3] & 0x02)
    expected = base_size + (SURVEY_CELL_COUNT * POINT.size if has_positions else 0)
    if len(data) != expected:
        raise ProtocolError("payload", "SURVEY_REPORT payload has invalid length")
    positions = tuple(
        POINT.unpack_from(data, base_size + index * POINT.size)
        for index in range(SURVEY_CELL_COUNT)
    ) if has_positions else ()
    payload = SurveyReportPayload(
        *values,
        tuple(data[SURVEY_REPORT_HEADER.size:base_size]),
        positions,
    )
    encode_survey_report(payload)
    return payload


def _encode_point_report(
    request_session: int,
    request_seq: int,
    revision: int,
    points: Tuple[Tuple[int, int], ...],
) -> bytes:
    if len(points) > 0xFF:
        raise ProtocolError("payload", "point count exceeds u8")
    encoded = POINT_REPORT_HEADER.pack(
        _require_u32("request_session", request_session),
        _require_u16("request_seq", request_seq),
        _require_u32("revision", revision),
        len(points),
    )
    encoded += b"".join(
        POINT.pack(_require_i32("x_cm", x_cm), _require_i32("y_cm", y_cm))
        for x_cm, y_cm in points
    )
    if len(encoded) > MAX_PAYLOAD_LEN:
        raise ProtocolError("oversize", "point report exceeds FleetBus V1 limit")
    return encoded


def _decode_point_report(
    data: bytes,
) -> Tuple[int, int, int, Tuple[Tuple[int, int], ...]]:
    if len(data) < POINT_REPORT_HEADER.size:
        raise ProtocolError("payload", "point report is too short")
    request_session, request_seq, revision, count = POINT_REPORT_HEADER.unpack_from(
        data
    )
    expected_len = POINT_REPORT_HEADER.size + count * POINT.size
    if len(data) != expected_len:
        raise ProtocolError("payload", "point report count/length mismatch")
    points = tuple(
        POINT.unpack_from(data, POINT_REPORT_HEADER.size + index * POINT.size)
        for index in range(count)
    )
    return request_session, request_seq, revision, points


def encode_map_report(payload: MapReportPayload) -> bytes:
    if len(payload.corners) not in (0, 4):
        raise ProtocolError("payload", "MAP_REPORT must contain zero or four corners")
    return _encode_point_report(
        payload.request_session,
        payload.request_seq,
        payload.map_revision,
        payload.corners,
    )


def decode_map_report(data: bytes) -> MapReportPayload:
    request_session, request_seq, revision, points = _decode_point_report(data)
    if len(points) not in (0, 4):
        raise ProtocolError("payload", "MAP_REPORT must contain zero or four corners")
    return MapReportPayload(request_session, request_seq, revision, points)


def encode_path_report(payload: PathReportPayload) -> bytes:
    return _encode_point_report(
        payload.request_session,
        payload.request_seq,
        payload.path_revision,
        payload.points,
    )


def decode_path_report(data: bytes) -> PathReportPayload:
    request_session, request_seq, revision, points = _decode_point_report(data)
    return PathReportPayload(request_session, request_seq, revision, points)
