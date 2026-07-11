from dataclasses import dataclass
from enum import IntEnum
import struct
from typing import Optional, Tuple


PROTOCOL_VERSION = 1
MAX_PAYLOAD_LEN = 128
FLAG_UPLINK_WINDOW = 0x01


class MessageType(IntEnum):
    HEARTBEAT = 1
    FC_STATE = 2
    MISSION_STATUS = 3
    COMMAND = 4
    COMMAND_ACK = 5
    COMMAND_RESULT = 6
    ALARM = 7
    LED_CONTROL = 8


class CommandId(IntEnum):
    PING = 1
    SET_TARGETS = 2
    START_MISSION = 3
    START_VISION_ACQUIRE = 4
    STOP_MISSION = 5


class AckStatus(IntEnum):
    RECEIVED = 1
    ACCEPTED = 2
    REJECTED = 3
    COMPLETED = 4
    FAILED = 5


class RejectReason(IntEnum):
    NONE = 0
    BAD_PAYLOAD = 1
    BAD_TARGETS = 2
    FC_OFFLINE = 3
    TASK_BUSY = 4
    TARGETS_NOT_READY = 5
    CAMERA_UNAVAILABLE = 6
    STALE_TELEMETRY = 7
    LINK_DOWN = 8
    UNKNOWN_COMMAND = 9


class MissionState(IntEnum):
    IDLE = 0
    ACQUIRING = 1
    READY = 2
    COUNTDOWN = 3
    RUNNING = 4
    STOPPING = 5
    LANDING = 6
    COMPLETED = 7
    FAILED = 8


class GroundLinkMode(IntEnum):
    """Aircraft radio role for one flight phase."""

    COMMAND_RX = 1
    TELEMETRY_TX = 2


class LEDMode(IntEnum):
    FLOW = 0
    PIXELS = 1


FC_STATE_LAYOUT_V1 = 0x81
FC_STATE_STRUCT = struct.Struct("<BiiHBB")
LEGACY_FC_STATE_STRUCT = struct.Struct("<hhhii hhh ii H B ? B B B")
EXTENSION_HEADER = struct.Struct("BB")
MISSION_STATUS_HEADER = struct.Struct(">BBBBB")
ACK_STRUCT = struct.Struct(">BBHBB")
ALARM_HEADER = struct.Struct(">B")
LED_CONTROL_HEADER = struct.Struct(">BBB")


@dataclass(frozen=True)
class TelemetryExtension:
    field_id: int
    data: bytes

    def __post_init__(self) -> None:
        if not 1 <= self.field_id <= 255:
            raise ValueError("telemetry extension field_id must be between 1 and 255")
        if len(self.data) > 255:
            raise ValueError("telemetry extension data is too long")


def _encode_extensions(extensions: Tuple[TelemetryExtension, ...]) -> bytes:
    encoded = bytearray()
    seen = set()
    for extension in extensions:
        if extension.field_id in seen:
            raise ValueError("duplicate telemetry extension field_id")
        seen.add(extension.field_id)
        encoded.extend(EXTENSION_HEADER.pack(extension.field_id, len(extension.data)))
        encoded.extend(extension.data)
    return bytes(encoded)


def _decode_extensions(payload: bytes) -> Tuple[TelemetryExtension, ...]:
    extensions = []
    offset = 0
    seen = set()
    while offset < len(payload):
        if len(payload) - offset < EXTENSION_HEADER.size:
            raise ValueError("truncated telemetry extension header")
        field_id, length = EXTENSION_HEADER.unpack_from(payload, offset)
        offset += EXTENSION_HEADER.size
        end = offset + length
        if end > len(payload):
            raise ValueError("truncated telemetry extension data")
        if field_id == 0 or field_id in seen:
            raise ValueError("invalid telemetry extension field_id")
        seen.add(field_id)
        extensions.append(TelemetryExtension(field_id, payload[offset:end]))
        offset = end
    return tuple(extensions)


@dataclass(frozen=True)
class FCStatePayload:
    pos_x_cm: int
    pos_y_cm: int
    battery_v: float
    mode: int
    unlock: bool
    extensions: Tuple[TelemetryExtension, ...] = ()

    def to_payload(self) -> bytes:
        payload = FC_STATE_STRUCT.pack(
            FC_STATE_LAYOUT_V1,
            self.pos_x_cm,
            self.pos_y_cm,
            round(self.battery_v / 0.01),
            self.mode,
            self.unlock,
        )
        payload += _encode_extensions(self.extensions)
        if len(payload) > MAX_PAYLOAD_LEN:
            raise ValueError("FC_STATE payload exceeds protocol limit")
        return payload

    @classmethod
    def from_payload(cls, payload: bytes) -> "FCStatePayload":
        if payload and payload[0] == FC_STATE_LAYOUT_V1:
            if len(payload) < FC_STATE_STRUCT.size:
                raise ValueError("FC_STATE payload is too short")
            _, pos_x, pos_y, battery, mode, unlock = FC_STATE_STRUCT.unpack_from(payload)
            return cls(
                pos_x,
                pos_y,
                battery * 0.01,
                mode,
                bool(unlock),
                _decode_extensions(payload[FC_STATE_STRUCT.size :]),
            )
        if len(payload) == LEGACY_FC_STATE_STRUCT.size:
            values = LEGACY_FC_STATE_STRUCT.unpack(payload)
            return cls(values[8], values[9], values[10] * 0.01, values[11], values[12])
        raise ValueError("unsupported FC_STATE payload layout")

    def extension(self, field_id: int) -> Optional[bytes]:
        for extension in self.extensions:
            if extension.field_id == field_id:
                return extension.data
        return None


@dataclass(frozen=True)
class Command:
    command_id: CommandId
    target1: Optional[int] = None
    target2: Optional[int] = None

    def to_payload(self) -> bytes:
        if self.command_id == CommandId.SET_TARGETS:
            if self.target1 is None or self.target2 is None:
                raise ValueError("SET_TARGETS requires target1 and target2")
            return bytes([self.command_id, self.target1, self.target2])
        return bytes([self.command_id])

    @classmethod
    def from_payload(cls, payload: bytes) -> "Command":
        if not payload:
            raise ValueError("empty command payload")
        try:
            command_id = CommandId(payload[0])
        except ValueError as exc:
            raise ValueError("unknown command id") from exc
        if command_id == CommandId.SET_TARGETS:
            if len(payload) != 3:
                raise ValueError("SET_TARGETS payload must be 3 bytes")
            return cls(command_id, payload[1], payload[2])
        if len(payload) != 1:
            raise ValueError("command payload has extra bytes")
        return cls(command_id)


@dataclass(frozen=True)
class CommandAck:
    command_id: CommandId
    seq: int
    status: AckStatus
    reason: RejectReason = RejectReason.NONE

    def to_payload(self) -> bytes:
        return ACK_STRUCT.pack(
            MessageType.COMMAND,
            self.command_id,
            self.seq,
            self.status,
            self.reason,
        )


@dataclass(frozen=True)
class MissionStatus:
    state: MissionState
    target1: Optional[int] = None
    target2: Optional[int] = None
    progress: int = 0
    error_code: int = 0
    message: str = ""

    def to_payload(self) -> bytes:
        if not 0 <= self.progress <= 100:
            raise ValueError("progress must be between 0 and 100")
        if not 0 <= self.error_code <= 255:
            raise ValueError("error_code must fit u8")
        text = self.message.encode("utf-8")
        if len(text) > MAX_PAYLOAD_LEN - MISSION_STATUS_HEADER.size:
            raise ValueError("mission status message is too long")
        return MISSION_STATUS_HEADER.pack(
            self.state,
            self.target1 or 0,
            self.target2 or 0,
            self.progress,
            self.error_code,
        ) + text


@dataclass(frozen=True)
class Alarm:
    code: int
    message: str

    def to_payload(self) -> bytes:
        if not 0 <= self.code <= 255:
            raise ValueError("alarm code must fit u8")
        text = self.message.encode("utf-8")
        if len(text) > MAX_PAYLOAD_LEN - ALARM_HEADER.size:
            raise ValueError("alarm message is too long")
        return ALARM_HEADER.pack(self.code) + text


@dataclass(frozen=True)
class LEDControl:
    mode: LEDMode
    brightness: int = 3
    pixels: Tuple[Tuple[int, int, int], ...] = ()

    def to_payload(self) -> bytes:
        if not 0 <= self.brightness <= 20:
            raise ValueError("LED brightness must be between 0 and 20")
        if self.mode == LEDMode.FLOW:
            if self.pixels:
                raise ValueError("FLOW LED control cannot contain pixels")
            return LED_CONTROL_HEADER.pack(self.mode, self.brightness, 0)
        if self.mode != LEDMode.PIXELS or len(self.pixels) != 7:
            raise ValueError("PIXELS LED control requires exactly 7 RGB values")
        data = bytearray(LED_CONTROL_HEADER.pack(self.mode, self.brightness, 7))
        for red, green, blue in self.pixels:
            if not all(0 <= value <= 255 for value in (red, green, blue)):
                raise ValueError("LED RGB values must be between 0 and 255")
            data.extend((red, green, blue))
        return bytes(data)
