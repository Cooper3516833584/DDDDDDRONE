from collections import OrderedDict
from dataclasses import dataclass
import hashlib
import hmac
import secrets
import struct
from typing import List, Optional, Tuple

from .models import MAX_PAYLOAD_LEN, PROTOCOL_VERSION, MessageType


MAGIC = b"\xAA\x22"
HEADER_STRUCT = struct.Struct("<2sBB")
METADATA_STRUCT = struct.Struct(">BBIH")
HMAC_LEN = 8
HEADER_LEN = HEADER_STRUCT.size
METADATA_LEN = METADATA_STRUCT.size
MIN_DATA_LEN = METADATA_LEN + HMAC_LEN
MAX_DATA_LEN = METADATA_LEN + MAX_PAYLOAD_LEN + HMAC_LEN
CHECKSUM_LEN = 1
MIN_FRAME_LEN = HEADER_LEN + MIN_DATA_LEN + CHECKSUM_LEN


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
    checksum_failures: int = 0
    hmac_failures: int = 0
    oversize_frames: int = 0
    version_failures: int = 0
    discarded_bytes: int = 0

    @property
    def crc_failures(self) -> int:
        """Compatibility alias for callers that used the previous counter name."""
        return self.checksum_failures


def new_session() -> int:
    return secrets.randbits(32)


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
    metadata = METADATA_STRUCT.pack(
        version,
        flags & 0xFF,
        session & 0xFFFFFFFF,
        seq & 0xFFFF,
    )
    data_len = len(metadata) + len(payload) + HMAC_LEN
    header = HEADER_STRUCT.pack(MAGIC, int(msg_type), data_len)
    protected = header + metadata + payload
    frame_without_checksum = protected + _tag(protected, key)
    checksum = (sum(frame_without_checksum) & 0xFF).to_bytes(1, "little")
    return frame_without_checksum + checksum


def unpack_frame(frame_bytes: bytes, key: bytes) -> Frame:
    if len(frame_bytes) < MIN_FRAME_LEN:
        raise ProtocolError("frame too short")
    magic, msg_type, data_len = HEADER_STRUCT.unpack(frame_bytes[:HEADER_LEN])
    if magic != MAGIC:
        raise ProtocolError("bad magic")
    if data_len < MIN_DATA_LEN:
        raise ProtocolError("frame data too short")
    if data_len > MAX_DATA_LEN:
        raise ProtocolError("payload too large")
    expected_len = HEADER_LEN + data_len + CHECKSUM_LEN
    if len(frame_bytes) != expected_len:
        raise ProtocolError("wrong frame length")
    if frame_bytes[-1] != (sum(frame_bytes[:-1]) & 0xFF):
        raise ProtocolError("checksum mismatch")
    tag_offset = len(frame_bytes) - CHECKSUM_LEN - HMAC_LEN
    expected_tag = frame_bytes[tag_offset:-CHECKSUM_LEN]
    actual_tag = _tag(frame_bytes[:tag_offset], key)
    if not hmac.compare_digest(expected_tag, actual_tag):
        raise ProtocolError("hmac mismatch")
    metadata_end = HEADER_LEN + METADATA_LEN
    version, flags, session, seq = METADATA_STRUCT.unpack(
        frame_bytes[HEADER_LEN:metadata_end]
    )
    if version != PROTOCOL_VERSION:
        raise ProtocolError("unsupported version")
    try:
        typed_msg = MessageType(msg_type)
    except ValueError as exc:
        raise ProtocolError("unknown message type") from exc
    payload = frame_bytes[metadata_end:tag_offset]
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
            _, _, data_len = HEADER_STRUCT.unpack(self._buffer[:HEADER_LEN])
            if data_len < MIN_DATA_LEN or data_len > MAX_DATA_LEN:
                self.stats.oversize_frames += 1
                del self._buffer[0]
                continue
            total_len = HEADER_LEN + data_len + CHECKSUM_LEN
            if len(self._buffer) < total_len:
                return frames
            candidate = bytes(self._buffer[:total_len])
            del self._buffer[:total_len]
            try:
                frames.append(unpack_frame(candidate, self._key))
            except ProtocolError as exc:
                if "checksum" in str(exc):
                    self.stats.checksum_failures += 1
                elif "hmac" in str(exc):
                    self.stats.hmac_failures += 1
                elif "version" in str(exc):
                    self.stats.version_failures += 1
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
