"""Exercise live calibration and standard telemetry on real loopback sockets."""

import socket

from pymavlink.dialects.v20 import common as mavlink
import pytest

from host_replay.live import LiveTime, SESSION
from host_replay.records import Recording
from host_replay.runner import arguments
from host_replay.telemetry import TelemetrySender
from .test_runner import database, endpoint


def receive(udp):
    parser = mavlink.MAVLink(None)
    messages = []
    udp.settimeout(.05)
    while True:
        try:
            messages.extend(parser.parse_buffer(udp.recv(65535)) or [])
        except socket.timeout:
            return messages


def test_live_wire_calibration_timesync_and_no_future_samples(tmp_path):
    sink, reservation = endpoint(), endpoint()
    port = reservation.getsockname()[1]
    reservation.close()
    boot = [5_000_000_123]
    live = LiveTime(1_000_000_000, monotonic_ns=lambda: boot[0],
                    wall_ns=lambda: 1_780_000_000_000_000_000)
    sender = TelemetrySender(Recording(database(tmp_path / 'wire.db')), '127.0.0.1', port,
                             '127.0.0.1', sink.getsockname()[1], live=live)
    try:
        sender.send_frame(1_055_123_456)
        messages = receive(sink)
        descriptor = next(m for m in messages if m.get_type() == 'TUNNEL')
        magic, version, session, sequence, record, anchor = SESSION.unpack(
            bytes(descriptor.payload[:descriptor.payload_length]))
        assert (magic, version, sequence, record, anchor) == (
            b'LRST', 1, 0, 1_000_000_000, 5_000_000_123)
        assert session == live.session_id.bytes
        assert descriptor.payload_type == 32768
        system = next(m for m in messages if m.get_type() == 'SYSTEM_TIME')
        assert system.time_unix_usec == 1_780_000_000_055_123
        assert system.time_boot_ms == 5055
        sensors = [m for m in messages if m.get_type() in ('GPS_RAW_INT', 'ATTITUDE', 'HIGHRES_IMU')]
        assert sensors
        for message in sensors:
            ns = (message.time_boot_ms * 1_000_000 if message.get_type() == 'ATTITUDE'
                  else message.time_usec * 1000)
            assert live.boot_ns - 1_000_000 < ns <= live.boot(1_055_123_456)
        imu = next(m for m in sensors if m.get_type() == 'HIGHRES_IMU')
        assert imu.zacc == pytest.approx(-9.80665, abs=1e-6)
        boot[0] += 123_456_789
        request = mavlink.MAVLink(None, srcSystem=255, srcComponent=240)
        sink.sendto(request.timesync_encode(0, 1234567).pack(request), ('127.0.0.1', port))
        sender.service()
        reply = next(m for m in receive(sink) if m.get_type() == 'TIMESYNC')
        assert (reply.tc1, reply.ts1) == (boot[0], 1234567)
    finally:
        sender.close()
        sink.close()


def test_live_cli_needs_no_clock_and_rejects_clock_endpoints():
    args = ['--time-mode', 'live', '--svo-path', 'x', '--mavlink-db-path', 'y',
            '--stream-port', '32000', '--mavlink-bind-address', '127.0.0.1',
            '--mavlink-bind-port', '14550', '--mavlink-destination', '127.0.0.1',
            '--mavlink-destination-port', '14540']
    assert arguments(args).clock_destination is None
    with pytest.raises(SystemExit):
        arguments(args + ['--clock-destination', '127.0.0.1'])
    with pytest.raises(SystemExit):
        arguments(args[2:])
