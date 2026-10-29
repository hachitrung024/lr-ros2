"""Wire failures and look-ahead must not compromise causal telemetry."""

from dataclasses import replace
import sqlite3
import uuid

import pytest

from host_replay.future_path import FuturePath
from host_replay.live import LiveTime
from host_replay.path_protocol import (
    Assembler, HEADER, MAX_POINTS, PathPoint, Snapshot, fragments,
)
from host_replay.records import Recording
from host_replay.runner import arguments
from host_replay.telemetry import TelemetrySender
from .test_live import receive
from .test_runner import database, endpoint


def snapshot(sequence=0, session=None, count=7, stamp=1_783_700_660_123_456_789):
    return Snapshot(session or uuid.uuid4().hex, sequence, stamp, tuple(
        PathPoint(stamp + i * 100_000_001, 340000000, -1180000000 + i, 500000,
                  (0., 0., 0., 1.)) for i in range(count)))


def test_integer_precision_reordering_duplicates_and_new_session():
    source = snapshot()
    packets = fragments(source)
    assert all(len(packet) <= 128 for packet in packets)
    receiver = Assembler()
    assert receiver.accept(packets[-1], source.session_id) is None
    assert receiver.accept(packets[-1], source.session_id) is None
    assert receiver.accept(packets[1], source.session_id) is None
    assert receiver.accept(packets[0], source.session_id) is None
    assert receiver.accept(packets[2], source.session_id) == source
    assert receiver.accept(packets[0], source.session_id) is None
    newer = replace(source, sequence=1)
    for packet in reversed(fragments(newer)):
        result = receiver.accept(packet, newer.session_id)
    assert result == newer
    assert all(receiver.accept(p, newer.session_id) is None for p in packets)
    restarted = snapshot(stamp=5_000_000_000)
    for packet in fragments(restarted):
        result = receiver.accept(packet, restarted.session_id)
    assert result == restarted
    assert all(receiver.accept(p, restarted.session_id) is None for p in packets)


def test_loss_expiration_malformed_and_corrupt_snapshot():
    now = [0.]
    source = snapshot()
    packets = fragments(source)
    receiver = Assembler(monotonic=lambda: now[0])
    for packet in packets[:-1]:
        assert receiver.accept(packet, source.session_id) is None
    now[0] = .6
    assert receiver.accept(packets[-1], source.session_id) is None
    for packet in fragments(replace(source, sequence=1)):
        result = receiver.accept(packet, source.session_id)
    assert result == replace(source, sequence=1)
    with pytest.raises(ValueError):
        receiver.accept(b'bad', source.session_id)
    malformed = bytearray(packets[0])
    malformed[5] = 1
    with pytest.raises(ValueError):
        receiver.accept(malformed, source.session_id)
    corrupt = fragments(replace(source, sequence=2))
    last = bytearray(corrupt[-1])
    last[-1] ^= 1
    for packet in corrupt[:-1]:
        assert receiver.accept(packet, source.session_id) is None
    with pytest.raises(ValueError, match='CRC'):
        receiver.accept(last, source.session_id)


def test_empty_and_maximum_snapshots_and_conflicting_fragments():
    source = snapshot(count=0)
    assert len(fragments(source)[0]) == HEADER.size
    assert Assembler().accept(fragments(source)[0], source.session_id) == source
    source = snapshot(count=MAX_POINTS)
    receiver = Assembler()
    for packet in reversed(fragments(source)):
        result = receiver.accept(packet, source.session_id)
    assert result == source
    receiver.reset()
    packet = fragments(source)[0]
    assert receiver.accept(packet, source.session_id) is None
    conflict = bytearray(packet)
    conflict[-1] ^= 1
    with pytest.raises(ValueError, match='Conflicting'):
        receiver.accept(conflict, source.session_id)
    with pytest.raises(ValueError):
        fragments(snapshot(count=MAX_POINTS + 1))


def test_path_on_same_udp_socket_and_telemetry_never_contains_future(tmp_path):
    path = database(tmp_path / 'moving.db')
    with sqlite3.connect(path) as db:
        db.execute('UPDATE gps SET lon=lon + id * 0.0000001')
    recording = Recording(path)
    live = LiveTime(1_000_000_000, monotonic_ns=lambda: 5_000_000_123)
    lookahead = FuturePath(recording, step_m=.005)
    sink, reservation = endpoint(), endpoint()
    bind_port = reservation.getsockname()[1]
    reservation.close()
    sender = TelemetrySender(recording, '127.0.0.1', bind_port, '127.0.0.1',
                             sink.getsockname()[1], live=live, future_path=lookahead)
    try:
        frame = 1_055_123_456
        sender.send_frame(frame)
        initial_cursor = recording.cursor
        assert len(sender.path_pending) > 1
        for _ in range(20):
            before = sender.path_packets_sent
            sender.service()
            assert sender.path_packets_sent - before <= 4
        assert recording.cursor == initial_cursor
        messages = receive(sink)
        receiver, result = Assembler(), None
        for message in messages:
            if message.get_type() == 'TUNNEL' and message.payload_type == 32769:
                result = receiver.accept(message.payload[:message.payload_length], live.session_id.hex)
            elif message.get_type() in ('GPS_RAW_INT', 'ATTITUDE', 'HIGHRES_IMU'):
                ns = (message.time_boot_ms * 1_000_000 if message.get_type() == 'ATTITUDE'
                      else message.time_usec * 1000)
                assert ns <= live.boot(frame)
        assert result is not None and len(result.points) > 2
        assert result.stamp_ns == live.boot(frame)
        assert result.points[-1].stamp_ns > live.boot(frame)
        assert lookahead.prepare(frame + 1, live) is None
    finally:
        sender.close()
        sink.close()


def test_path_cli_rejects_replay_and_invalid_rates():
    args = ['--svo-path', 'x', '--mavlink-db-path', 'y', '--stream-port', '32000',
            '--mavlink-bind-address', '127.0.0.1', '--mavlink-bind-port', '14550',
            '--mavlink-destination', '127.0.0.1', '--mavlink-destination-port', '14540']
    assert arguments(args + ['--time-mode', 'live', '--future-path']).future_path
    with pytest.raises(SystemExit):
        arguments(args + ['--future-path'])
    for rate in ('nan', '0', '6'):
        with pytest.raises(SystemExit):
            arguments(args + ['--time-mode', 'live', '--future-path-rate-hz', rate])
