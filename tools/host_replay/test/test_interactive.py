"""Seek boundaries, pause transport cleanup and nonblocking terminal input."""

import os
from types import SimpleNamespace

import pytest

from host_replay import interactive
from host_replay.interactive import LivePlayback, TerminalCommands
from host_replay.live import SESSION
from host_replay.path_protocol import HEADER
from host_replay.records import Recording
from host_replay.runner import arguments, run
from host_replay.source import SvoSource
from .conftest import fake_sdk
from .test_live import receive
from .test_runner import database, endpoint


@pytest.fixture
def playback(tmp_path, svo, monkeypatch):
    sink, reservation = endpoint(), endpoint()
    port = reservation.getsockname()[1]
    reservation.close()
    args = arguments([
        '--time-mode', 'live', '--interactive', '--future-path',
        '--svo-path', str(svo), '--mavlink-db-path', str(database(tmp_path / 'wire.db')),
        '--stream-port', '32000', '--mavlink-bind-address', '127.0.0.1',
        '--mavlink-bind-port', str(port), '--mavlink-destination', '127.0.0.1',
        '--mavlink-destination-port', str(sink.getsockname()[1]),
    ])
    monkeypatch.setattr(interactive, 'SvoSource', lambda path, port, frame: SvoSource(
        path, port, frame, sdk=fake_sdk()))
    player = LivePlayback(args, Recording(args.mavlink_db_path), (1_000_000_000, 1_190_000_000))
    try:
        player.start(0)
        yield player, sink
    finally:
        player.close()
        sink.close()


def test_forward_and_backward_seek_retire_transport_and_reanchor_all_data(playback):
    player, sink = playback
    sessions = []
    for frame in (3, 0, 2):
        previous, transport = player.source, player.telemetry
        sessions.append(transport.live.session_id)
        transport.send_frame(previous.stamp_ns)
        transport.path_pending.extend(player.future_path.prepare(
            player.future_path.next_stamp, transport.live))
        assert player.recording.cursor is not None
        assert player.future_path.next_stamp > 0
        assert transport.path_pending
        receive(sink)
        player.command(f'seek {frame}')
        assert previous.closed and transport.socket.fileno() == -1
        assert not player.telemetry.path_pending
        assert player.recording.cursor is None
        assert player.future_path.next_stamp == player.future_path.sequence == 0
        assert player.source.frame_id == frame
        assert player.telemetry.live.record_ns == player.source.stamp_ns
        player.telemetry.send_frame(player.source.stamp_ns)
        player.telemetry.service()
        messages = receive(sink)
        descriptor = next(m for m in messages if m.get_type() == 'TUNNEL'
                          and m.payload_type == 32768)
        _, _, session, _, stamp, boot = SESSION.unpack(
            bytes(descriptor.payload[:descriptor.payload_length]))
        assert session == player.telemetry.live.session_id.bytes
        assert (stamp, boot) == (player.source.stamp_ns, player.telemetry.live.boot_ns)
        path = next(m for m in messages if m.get_type() == 'TUNNEL' and m.payload_type == 32769)
        assert HEADER.unpack(bytes(path.payload[:HEADER.size]))[2] == session
        for message in messages:
            if message.get_type() in ('GPS_RAW_INT', 'ATTITUDE', 'HIGHRES_IMU'):
                sensor_ns = (message.time_boot_ms * 1_000_000 if (
                    message.get_type() == 'ATTITUDE') else message.time_usec * 1000)
                assert boot - 1_000_000 < sensor_ns <= boot
    assert len(set(sessions + [player.telemetry.live.session_id])) == 4


def test_pause_releases_source_and_socket_resume_uses_selected_frame(playback):
    player, sink = playback
    previous, transport = player.source, player.telemetry
    player.command('pause')
    receive(sink)
    assert player.source is None and player.telemetry is None
    assert previous.closed and transport.socket.fileno() == -1
    assert player.next_frame == 1
    player.command('resume')
    assert player.source.frame_id == 1
    assert player.telemetry.live.session_id != transport.live.session_id
    player.command('pause')
    receive(sink)
    player.command('seek 3')
    player.command('skip -0.1')
    assert player.state == 'paused' and player.next_frame == 0
    assert not receive(sink)
    player.command('resume')
    assert player.source.frame_id == 0
    assert player.telemetry.live.session_id != transport.live.session_id
    player.command('skip 1e308')
    assert player.source.frame_id == 3
    player.command('replay')
    assert player.source.frame_id == 0


@pytest.mark.parametrize('command', [
    'seek -1', 'seek 4', 'seek abc', 'skip nan', 'skip inf', 'pause extra', 'seek', 'unknown',
])
def test_invalid_commands_leave_playback_running(playback, command):
    player, _ = playback
    source, telemetry = player.source, player.telemetry
    with pytest.raises(ValueError):
        player.command(command)
    assert player.source is source and player.telemetry is telemetry and not source.closed


def test_eof_can_seek_and_replay_without_exiting(playback):
    player, _ = playback
    while player.source.tick():
        pass
    player.stop(ended=True)
    assert player.state == 'ended' and not player.quitting
    with pytest.raises(ValueError, match='End of SVO'):
        player.command('resume')
    player.command('seek 1')
    assert player.source is None and player.next_frame == 1
    player.command('resume')
    assert player.source.frame_id == 1
    player.command('quit')
    assert player.quitting and player.source is None


def test_pause_after_the_last_frame_does_not_replay_it_on_resume(playback):
    player, _ = playback
    player.command('seek 3')
    player.command('pause')
    assert player.state == 'ended' and player.next_frame == 4
    with pytest.raises(ValueError, match='End of SVO'):
        player.command('resume')


def test_sdk_eof_at_advertised_last_frame_keeps_controls_available(playback, monkeypatch):
    player, _ = playback
    original = player.source
    sdk = fake_sdk()
    sdk.Camera().grab = lambda _runtime: sdk.ERROR_CODE.END_OF_SVOFILE_REACHED
    monkeypatch.setattr(interactive, 'SvoSource', lambda path, port, frame: SvoSource(
        path, port, frame, sdk=sdk))
    player.command('seek 3')
    assert original.closed and sdk.Camera().closed
    assert player.source is None and player.state == 'ended' and player.next_frame == 4
    player.command('seek 0')
    assert player.next_frame == 0 and player.state == 'paused'
    assert not player.quitting


def test_failed_restart_closes_both_transports_and_allows_retry(playback, monkeypatch):
    player, _ = playback
    previous, transport = player.source, player.telemetry
    original = interactive.TelemetrySender

    def fail(*_args, **_kwargs):
        raise OSError('port unavailable')

    monkeypatch.setattr(interactive, 'TelemetrySender', fail)
    with pytest.raises(OSError):
        player.command('seek 2')
    assert previous.closed and transport.socket.fileno() == -1
    assert player.source is None and player.state == 'paused'
    monkeypatch.setattr(interactive, 'TelemetrySender', original)
    player.command('resume')
    assert player.source.frame_id == 2


def test_terminal_handles_fragmented_lines_buffered_commands_and_eof():
    read_fd, write_fd = os.pipe()
    with os.fdopen(read_fd) as stream:
        commands = TerminalCommands(stream)
        try:
            assert commands.poll() is None
            os.write(write_fd, b'sta')
            assert commands.poll() is None
            os.write(write_fd, b'tus\npause\nseek 2')
            assert commands.poll() == 'status'
            assert commands.poll() == 'pause'
        finally:
            os.close(write_fd)
        assert commands.poll() == 'seek 2'
        assert commands.poll() == 'quit'


def test_scheduler_returns_before_using_closed_transport(playback):
    player, _ = playback
    source = player.source
    now = [0.]
    commands = SimpleNamespace(poll=lambda: 'pause')
    frames = run(source, player.telemetry, monotonic=lambda: now[0],
                 sleep=lambda duration: now.__setitem__(0, now[0] + duration),
                 poll=lambda: player.poll(commands))
    assert frames == 1 and source.closed and player.source is None


def test_interactive_cli_rejects_recorded_time_replay():
    with pytest.raises(SystemExit):
        arguments(['--interactive', '--svo-path', 'x', '--mavlink-db-path', 'y',
                   '--stream-port', '32000', '--mavlink-bind-address', '127.0.0.1',
                   '--mavlink-bind-port', '14550', '--mavlink-destination', '127.0.0.1',
                   '--mavlink-destination-port', '14540', '--clock-destination', '127.0.0.1',
                   '--clock-destination-port', '30010'])
