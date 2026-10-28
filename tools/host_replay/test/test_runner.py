"""Check one scheduler, wire timing and a copied tool with ROS imports denied."""

import json
import os
from pathlib import Path
import shutil
import socket
import sqlite3
import subprocess
import sys
from types import SimpleNamespace

import pytest
from pymavlink.dialects.v20 import common as mavlink

from host_replay.clock import ClockSender
from host_replay.records import Recording
from host_replay.runner import arguments, run
from host_replay.telemetry import TelemetrySender, validate_recording


def database(path):
    with sqlite3.connect(path) as db:
        db.executescript('''
            CREATE TABLE gps (id INTEGER, t_wall_epoch_us INTEGER, lat REAL,
                              lon REAL, alt REAL, fix_type INTEGER);
            CREATE TABLE attitude (id INTEGER, t_wall_epoch_us INTEGER,
                                   roll REAL, pitch REAL, yaw REAL);
            CREATE TABLE imu (id INTEGER, t_wall_epoch_us INTEGER, acc_z REAL);
        ''')
        for i in range(20):
            stamp = 1_000_000 + 10_000 * i
            db.execute('INSERT INTO gps VALUES (?,?,?,?,?,?)', (i, stamp, 34., -118., 500., 4))
            db.execute('INSERT INTO attitude VALUES (?,?,?,?,?)', (i, stamp, 0., 0., 0.))
            db.execute('INSERT INTO imu VALUES (?,?,?)', (i, stamp, -1000.))
    return path


def endpoint():
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp.bind(('127.0.0.1', 0))
    return udp


def test_scheduler_clock_before_telemetry_and_no_duplicate_eof(sdk, svo):
    from host_replay.source import SvoSource
    source = SvoSource(svo, 32000, sdk=sdk)
    events = []
    now = [0.0]
    telemetry = SimpleNamespace(
        send_frame=lambda stamp: events.append(('telemetry', stamp)), service=lambda: None,
    )
    clock = SimpleNamespace(send=lambda stamp: events.append(('clock', stamp)))
    try:
        frames = run(source, telemetry, clock, monotonic=lambda: now[0],
                     sleep=lambda duration: now.__setitem__(0, now[0] + duration))
        assert frames == 4
        assert [kind for kind, _ in events] == ['clock', 'telemetry'] * 4
        assert all(events[i][1] == events[i + 1][1] for i in range(0, 8, 2))
        assert now[0] >= 4 * source.period
    finally:
        source.close()


def test_clock_udp_retains_nanoseconds_and_session():
    receiver = endpoint()
    receiver.settimeout(1)
    sender = ClockSender('127.0.0.1', '127.0.0.1', receiver.getsockname()[1])
    stamp = 1783700660123456789
    try:
        sender.send(stamp)
        sender.send(stamp + 1)
        packets = [json.loads(receiver.recv(4096)) for _ in range(2)]
        assert [p['stamp_ns'] for p in packets] == [stamp, stamp + 1]
        assert [p['sequence'] for p in packets] == [0, 1]
        assert packets[0]['session_id'] == packets[1]['session_id'] == sender.session_id
    finally:
        sender.close()
        receiver.close()


def test_udp_system_time_and_causal_sensor_stamps(tmp_path):
    path = database(tmp_path / 'telemetry.db')
    assert validate_recording(path) == (1_000_000_000, 1_190_000_000)
    receiver = endpoint()
    receiver.settimeout(.1)
    reservation = endpoint()
    bind_port = reservation.getsockname()[1]
    reservation.close()
    sender = TelemetrySender(Recording(path), '127.0.0.1', bind_port,
                             '127.0.0.1', receiver.getsockname()[1])
    try:
        stamp = 1_055_123_456
        sender.send_frame(stamp)
        messages = []
        parser = mavlink.MAVLink(None)
        while True:
            try:
                messages.extend(parser.parse_buffer(receiver.recv(65535)) or [])
            except socket.timeout:
                break
        epoch = sender.recording.epoch_us
        system_time = next(m for m in messages if m.get_type() == 'SYSTEM_TIME')
        assert system_time.time_unix_usec - system_time.time_boot_ms * 1000 == epoch
        sensors = [m for m in messages if m.get_type() in ('GPS_RAW_INT', 'ATTITUDE', 'HIGHRES_IMU')]
        assert len(sensors) == 6
        for message in sensors:
            time_us = message.time_boot_ms * 1000 if message.get_type() == 'ATTITUDE' else message.time_usec
            assert (epoch + time_us) * 1000 <= stamp
    finally:
        sender.close()
        receiver.close()


@pytest.mark.parametrize('mode', ['replay', 'live'])
def test_copied_tool_replays_without_workspace_or_ros(tmp_path, mode):
    copied = tmp_path / 'standalone'
    shutil.copytree(Path(__file__).resolve().parents[1], copied,
                    ignore=shutil.ignore_patterns('__pycache__', '.pytest_cache'))
    db = database(tmp_path / 'telemetry.db')
    svo = tmp_path / 'test.svo2'
    svo.touch()
    reservation, mav_sink, clock_sink = endpoint(), endpoint(), endpoint()
    bind_port = reservation.getsockname()[1]
    reservation.close()
    script = '''
import builtins, sys, types
sys.path.insert(0, sys.argv.pop(1))
original_import = builtins.__import__
def guarded(name, *args, **kwargs):
    if name.startswith(('rclpy', 'ament_', 'launch', 'rosgraph_msgs', 'zed_msgs', 'lr_')):
        raise AssertionError('ROS/workspace import: ' + name)
    return original_import(name, *args, **kwargs)
builtins.__import__ = guarded
from test.conftest import fake_sdk
pyzed = types.ModuleType('pyzed')
pyzed.sl = fake_sdk()
sys.modules['pyzed'] = pyzed
sys.modules['pyzed.sl'] = pyzed.sl
from host_replay.runner import main
raise SystemExit(main(sys.argv[1:]))
'''
    args = [sys.executable, '-I', '-c', script, str(copied),
            '--svo-path', str(svo), '--mavlink-db-path', str(db), '--stream-port', '32000',
            '--mavlink-bind-address', '127.0.0.1', '--mavlink-bind-port', str(bind_port),
            '--mavlink-destination', '127.0.0.1',
            '--mavlink-destination-port', str(mav_sink.getsockname()[1]), '--time-mode', mode]
    if mode == 'replay':
        args += ['--clock-destination', '127.0.0.1',
                 '--clock-destination-port', str(clock_sink.getsockname()[1])]
    try:
        result = subprocess.run(args, cwd=tmp_path, capture_output=True, text=True, timeout=10,
                                env={k: v for k, v in os.environ.items() if k != 'PYTHONPATH'})
        assert result.returncode == 0, result.stdout + result.stderr
        assert 'EOF: 4 frames' in result.stdout
    finally:
        mav_sink.close()
        clock_sink.close()


def test_no_network_endpoint_defaults_and_invalid_arguments():
    with pytest.raises(SystemExit):
        arguments([])


@pytest.mark.parametrize('failure, expected', [(RuntimeError('capture failed'), 1),
                                              (KeyboardInterrupt(), 130)])
def test_runtime_failure_and_interrupt_close_all_resources(monkeypatch, failure, expected):
    from host_replay import runner
    closed = []
    monkeypatch.setattr(runner, 'arguments', lambda _argv: SimpleNamespace(
        mavlink_db_path='unused', svo_path='unused', stream_port=32000, play_from_frame=0,
        mavlink_bind_address='127.0.0.1', mavlink_bind_port=14550,
        mavlink_destination='127.0.0.1', mavlink_destination_port=14540,
        clock_destination='127.0.0.1', clock_destination_port=30010,
        time_mode='replay',
    ))
    monkeypatch.setattr(runner.signal, 'signal', lambda *_args: None)
    monkeypatch.setattr(runner, 'validate_recording', lambda _: (1, 2_000_000_000))
    monkeypatch.setattr(runner, 'Recording', lambda _: SimpleNamespace(epoch_us=0))
    monkeypatch.setattr(runner, 'SvoSource', lambda *_args: SimpleNamespace(
        close=lambda: closed.append('source'), stamp_ns=1_000_000_000, fps=30,
        frame_id=0, total_frames=4,
    ))
    monkeypatch.setattr(runner, 'TelemetrySender', lambda *_args: SimpleNamespace(
        close=lambda: closed.append('telemetry'), sent=0,
    ))
    monkeypatch.setattr(runner, 'ClockSender', lambda *_args: SimpleNamespace(
        close=lambda: closed.append('clock'), session_id='test',
    ))

    def fail(*_args):
        raise failure

    monkeypatch.setattr(runner, 'run', fail)
    assert runner.main([]) == expected
    assert closed == ['clock', 'telemetry', 'source']
