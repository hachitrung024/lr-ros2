"""Encode bounded LRFP v1 snapshots in MAVLink TUNNEL payload type 32769."""

# Kept byte-identical to lr_mavros_bridge/path_protocol.py so the copied host
# tool remains independent of ROS. All timestamps are integer boot nanoseconds.

from dataclasses import dataclass
import math
import struct
import time
import uuid
import zlib


PAYLOAD_TYPE = 32769
HEADER = struct.Struct('<4sB3x16sQQHHHHI')
POINT = struct.Struct('<Qiii4f')
POINTS_PER_FRAGMENT = 2
MAX_POINTS = 128
MAX_HORIZON_NS = 120_000_000_000


@dataclass(frozen=True)
class PathPoint:
    stamp_ns: int
    lat_e7: int
    lon_e7: int
    alt_msl_mm: int
    quaternion: tuple


@dataclass(frozen=True)
class Snapshot:
    session_id: str
    sequence: int
    stamp_ns: int
    points: tuple


def _validate(points, stamp_ns):
    previous = stamp_ns - 1
    for point in points:
        if not stamp_ns <= point.stamp_ns <= stamp_ns + MAX_HORIZON_NS:
            raise ValueError('Path point outside reference/horizon')
        if point.stamp_ns <= previous:
            raise ValueError('Path point timestamps must increase')
        if not -900000000 <= point.lat_e7 <= 900000000 or not (
                -1800000000 <= point.lon_e7 <= 1800000000):
            raise ValueError('Invalid path latitude/longitude')
        norm = sum(v * v for v in point.quaternion)
        if not all(math.isfinite(v) for v in point.quaternion) or abs(norm - 1.) > .01:
            raise ValueError('Invalid path orientation')
        previous = point.stamp_ns


def fragments(snapshot):
    """Encode a complete replacement, including an explicit empty path."""
    count = len(snapshot.points)
    if not 0 <= count <= MAX_POINTS or snapshot.stamp_ns <= 0:
        raise ValueError('Invalid path size/reference')
    _validate(snapshot.points, snapshot.stamp_ns)
    session = uuid.UUID(hex=snapshot.session_id).bytes
    body = b''.join(POINT.pack(p.stamp_ns, p.lat_e7, p.lon_e7, p.alt_msl_mm,
                               *p.quaternion) for p in snapshot.points)
    crc = zlib.crc32(body)
    total = max(1, (count + POINTS_PER_FRAGMENT - 1) // POINTS_PER_FRAGMENT)
    width = POINT.size * POINTS_PER_FRAGMENT
    return [HEADER.pack(b'LRFP', 1, session, snapshot.sequence, snapshot.stamp_ns,
                        index, total, count, 0, crc) + body[index * width:(index + 1) * width]
            for index in range(total)]


class Assembler:
    """Accept reordered fragments; never expose partial, expired or old paths."""

    def __init__(self, *, timeout=.5, monotonic=time.monotonic):
        self.timeout, self.monotonic = timeout, monotonic
        self.reset()

    def reset(self):
        self.session = ''
        self.sequence = -1
        self.parts = {}
        self.key = None
        self.started = 0.
        self.finished = True

    def accept(self, payload, session_id):
        payload = bytes(payload)
        if not HEADER.size <= len(payload) <= 128:
            raise ValueError('Invalid path fragment size')
        magic, version, session, seq, stamp, index, total, count, reserved, crc = (
            HEADER.unpack_from(payload))
        identifier = uuid.UUID(bytes=session).hex
        expected_total = max(1, (count + POINTS_PER_FRAGMENT - 1) // POINTS_PER_FRAGMENT)
        expected_count = min(POINTS_PER_FRAGMENT, count - index * POINTS_PER_FRAGMENT)
        if magic != b'LRFP' or version != 1 or payload[5:8] != b'\0\0\0' or reserved:
            raise ValueError('Invalid path fragment version/reserved fields')
        if count > MAX_POINTS or stamp <= 0 or total != expected_total or index >= total:
            raise ValueError('Invalid path fragment bounds')
        if len(payload) != HEADER.size + expected_count * POINT.size:
            raise ValueError('Invalid path fragment point count')
        if identifier != session_id:
            return None
        if identifier != self.session:
            self.reset()
            self.session = identifier
        if seq < self.sequence or (seq == self.sequence and self.finished):
            return None
        key = (identifier, seq, stamp, total, count, crc)
        if seq > self.sequence:
            self.sequence, self.key = seq, key
            self.parts, self.started, self.finished = {}, self.monotonic(), False
        if self.monotonic() - self.started > self.timeout or key != self.key:
            self.finished = True
            self.parts.clear()
            return None
        part = payload[HEADER.size:]
        if index in self.parts and self.parts[index] != part:
            self.finished = True
            self.parts.clear()
            raise ValueError('Conflicting path fragment')
        self.parts[index] = part
        if len(self.parts) != total:
            return None
        body = b''.join(self.parts[i] for i in range(total))
        self.finished = True
        self.parts.clear()
        if zlib.crc32(body) != crc:
            raise ValueError('Path snapshot CRC mismatch')
        points = tuple(PathPoint(*values[:4], tuple(values[4:]))
                       for values in POINT.iter_unpack(body))
        _validate(points, stamp)
        return Snapshot(identifier, seq, stamp, points)
