from collections import OrderedDict
from dataclasses import dataclass
import hashlib
import hmac
import secrets
import struct
from typing import List, Optional, Tuple

from .models import MAX_PAYLOAD_LEN, PROTOCOL_VERSION, MessageType


MAGIC = b"\xA5\x5A"
HEADER_STRUCT = struct.Struct(">2sBBB I H H")
CRC_STRUCT = struct.Struct(">H")
HMAC_LEN = 8
HEADER_LEN = HEADER_STRUCT.size
CRC_LEN = CRC_STRUCT.size
MIN_FRAME_LEN = HEADER_LEN + CRC_LEN + HMAC_LEN


class ProtocolError(ValueError):
    pass


@dataclass(frozen=True)
class Frame:
    version: int
    msg_type: MessageType
    flags: int
    session: int
    seq: int
    payload: bytes


@dataclass
class ParserStats:
    crc_failures: int = 0
    hmac_failures: int = 0
    oversize_frames: int = 0
    version_failures: int = 0
    discarded_bytes: int = 0


def new_session() -> int:
    return secrets.randbits(32)


def crc16_ccitt(data: bytes, init: int = 0xFFFF) -> int:
    crc = init
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def _tag(data: bytes, key: bytes) -> bytes:
    if not key:
        raise ProtocolError("HMAC key is required")
    return hmac.new(key, data, hashlib.sha256).digest()[:HMAC_LEN]


def pack_frame(
    msg_type: MessageType,
    payload: bytes,
    session: int,
    seq: int,
    key: bytes,
    flags: int = 0,
    version: int = PROTOCOL_VERSION,
) -> bytes:
    if len(payload) > MAX_PAYLOAD_LEN:
        raise ProtocolError("payload too large: {} > {}".format(len(payload), MAX_PAYLOAD_LEN))
    header = HEADER_STRUCT.pack(
        MAGIC,
        version,
        int(msg_type),
        flags & 0xFF,
        session & 0xFFFFFFFF,
        seq & 0xFFFF,
        len(payload),
    )
    protected = header + payload
    crc = CRC_STRUCT.pack(crc16_ccitt(protected))
    return protected + crc + _tag(protected + crc, key)


def unpack_frame(frame_bytes: bytes, key: bytes) -> Frame:
    if len(frame_bytes) < MIN_FRAME_LEN:
        raise ProtocolError("frame too short")
    magic, version, msg_type, flags, session, seq, length = HEADER_STRUCT.unpack(
        frame_bytes[:HEADER_LEN]
    )
    if magic != MAGIC:
        raise ProtocolError("bad magic")
    if version != PROTOCOL_VERSION:
        raise ProtocolError("unsupported version")
    if length > MAX_PAYLOAD_LEN:
        raise ProtocolError("payload too large")
    expected_len = HEADER_LEN + length + CRC_LEN + HMAC_LEN
    if len(frame_bytes) != expected_len:
        raise ProtocolError("wrong frame length")
    crc_offset = HEADER_LEN + length
    expected_crc = CRC_STRUCT.unpack(frame_bytes[crc_offset : crc_offset + CRC_LEN])[0]
    if expected_crc != crc16_ccitt(frame_bytes[:crc_offset]):
        raise ProtocolError("crc mismatch")
    expected_tag = frame_bytes[crc_offset + CRC_LEN :]
    actual_tag = _tag(frame_bytes[: crc_offset + CRC_LEN], key)
    if not hmac.compare_digest(expected_tag, actual_tag):
        raise ProtocolError("hmac mismatch")
    try:
        typed_msg = MessageType(msg_type)
    except ValueError as exc:
        raise ProtocolError("unknown message type") from exc
    payload = frame_bytes[HEADER_LEN : HEADER_LEN + length]
    return Frame(version, typed_msg, flags, session, seq, payload)


class FrameParser:
    def __init__(self, key: bytes):
        if not key:
            raise ProtocolError("HMAC key is required")
        self._key = key
        self._buffer = bytearray()
        self.stats = ParserStats()

    def feed(self, data: bytes) -> List[Frame]:
        self._buffer.extend(data)
        frames = []
        while True:
            start = self._buffer.find(MAGIC)
            if start < 0:
                keep = 1 if self._buffer.endswith(MAGIC[:1]) else 0
                self.stats.discarded_bytes += len(self._buffer) - keep
                if keep:
                    del self._buffer[:-keep]
                else:
                    self._buffer.clear()
                return frames
            if start:
                self.stats.discarded_bytes += start
                del self._buffer[:start]
            if len(self._buffer) < HEADER_LEN:
                return frames
            _, version, _, _, _, _, length = HEADER_STRUCT.unpack(
                self._buffer[:HEADER_LEN]
            )
            if version != PROTOCOL_VERSION:
                self.stats.version_failures += 1
                del self._buffer[0]
                continue
            if length > MAX_PAYLOAD_LEN:
                self.stats.oversize_frames += 1
                del self._buffer[0]
                continue
            total_len = HEADER_LEN + length + CRC_LEN + HMAC_LEN
            if len(self._buffer) < total_len:
                return frames
            candidate = bytes(self._buffer[:total_len])
            del self._buffer[:total_len]
            try:
                frames.append(unpack_frame(candidate, self._key))
            except ProtocolError as exc:
                if "crc" in str(exc):
                    self.stats.crc_failures += 1
                elif "hmac" in str(exc):
                    self.stats.hmac_failures += 1
                else:
                    self.stats.discarded_bytes += 1


class RecentResponseCache:
    def __init__(self, max_items: int = 64):
        self._items = OrderedDict()  # type: OrderedDict[Tuple[int, int], bytes]
        self._max_items = max_items

    def get(self, session: int, seq: int) -> Optional[bytes]:
        key = (session, seq)
        value = self._items.get(key)
        if value is not None:
            self._items.move_to_end(key)
        return value

    def put(self, session: int, seq: int, response: bytes) -> None:
        key = (session, seq)
        self._items[key] = response
        self._items.move_to_end(key)
        while len(self._items) > self._max_items:
            self._items.popitem(last=False)
