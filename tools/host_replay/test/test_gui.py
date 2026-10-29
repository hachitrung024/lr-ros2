"""Exercise real Tk widgets and the single SDK worker on an available X display."""

import time
from threading import Event
from types import SimpleNamespace

import pytest

tk = pytest.importorskip('tkinter')

from host_replay import interactive
from host_replay.gui import ControlChannel, PlaybackWindow, time_label
from host_replay.records import Recording
from host_replay.runner import arguments, run_controlled
from host_replay.source import SvoSource
from .conftest import fake_sdk
from .test_runner import database, endpoint


def until(ui, predicate, timeout=3):
    end = time.monotonic() + timeout
    while not predicate() and time.monotonic() < end:
        ui.root.update()
        time.sleep(.005)
    assert predicate(), ui.window.state
    assert not ui.errors


@pytest.fixture
def ui(tmp_path, svo, monkeypatch):
    try:
        root = tk.Tk()
    except tk.TclError:
        pytest.skip('GUI tests require an X display (for example Xvfb)')
    sink, reservation = endpoint(), endpoint()
    port = reservation.getsockname()[1]
    reservation.close()
    args = arguments([
        '--time-mode', 'live', '--gui', '--svo-path', str(svo),
        '--mavlink-db-path', str(database(tmp_path / 'wire.db')),
        '--stream-port', '32000', '--mavlink-bind-address', '127.0.0.1',
        '--mavlink-bind-port', str(port), '--mavlink-destination', '127.0.0.1',
        '--mavlink-destination-port', str(sink.getsockname()[1]),
    ])
    calls, cameras, errors = [], [], []
    gate = Event()
    gate.set()

    def source(path, stream_port, frame):
        calls.append(frame)
        assert gate.wait(3), 'test did not release SDK startup'
        sdk = fake_sdk()
        camera = sdk.Camera()
        camera.get_svo_number_of_frames = lambda: 1000

        def read():
            if camera.cursor >= 1000:
                return 1
            camera.frame = camera.cursor
            camera.cursor += 1
            camera.sent.append(camera.frame)
            return 0

        camera.read = read
        cameras.append(camera)
        return SvoSource(path, stream_port, frame, sdk=sdk)

    monkeypatch.setattr(interactive, 'SvoSource', source)
    root.report_callback_exception = lambda kind, value, trace: errors.append(value)
    window = PlaybackWindow(root, args, Recording(args.mavlink_db_path),
                            (1_000_000_000, 1_190_000_000), run_controlled)
    instance = SimpleNamespace(root=root, window=window, calls=calls, cameras=cameras,
                               errors=errors, gate=gate)
    try:
        yield instance
    finally:
        gate.set()
        if window.worker.ident is not None:
            window.close()
            until(instance, window.channel.finished.is_set)
            window.worker.join(timeout=1)
            assert not window.worker.is_alive()
        try:
            root.destroy()
        except tk.TclError:
            pass
        sink.close()


def start(ui):
    ui.window.start()
    until(ui, lambda: ui.window.state['state'] == 'playing' and not ui.window.state['busy'])


def ready(ui):
    return not ui.window.state['busy'] and not ui.window.pending


def test_slider_seeks_once_on_release_and_stays_at_preview_while_dragging(ui):
    start(ui)
    window = ui.window
    session = window.channel.player.telemetry.live.session_id
    window.scale.event_generate('<ButtonPress-1>', x=10, y=8)
    for frame in (30, 60, 90):
        window.scale.set(frame)
        ui.root.update()
    assert ui.calls == [0]
    assert window.dragging and window.frame_value.get() == '90'
    window.scale.event_generate('<ButtonRelease-1>', x=10, y=8)
    until(ui, lambda: ui.calls == [0, 90] and ready(ui))
    assert window.channel.player.telemetry.live.session_id != session
    assert not window.dragging


def test_pause_frame_entry_resume_and_replay_buttons(ui):
    start(ui)
    window = ui.window
    window.play.invoke()
    until(ui, lambda: window.state['state'] == 'paused' and ready(ui))
    assert window.channel.player.source is None
    window.frame_value.set('120')
    window.go.invoke()
    until(ui, lambda: window.state['frame'] == 120 and ready(ui))
    assert ui.calls == [0]  # Selecting while paused does not open the encoder.
    window.play.invoke()
    until(ui, lambda: ui.calls == [0, 120] and ready(ui))
    window.replay.invoke()
    until(ui, lambda: ui.calls == [0, 120, 0] and ready(ui))


def test_skip_buttons_and_invalid_frame_input(ui):
    start(ui)
    window = ui.window
    window.forward.invoke()
    until(ui, lambda: len(ui.calls) == 2 and ready(ui))
    assert 300 <= ui.calls[-1] < 330
    window.back.invoke()
    until(ui, lambda: len(ui.calls) == 3 and ready(ui))
    assert 0 <= ui.calls[-1] < 30
    window.frame_value.set('-1')
    window.go.invoke()
    assert '0 to 999' in window.message.get()
    assert not window.pending and len(ui.calls) == 3


def test_close_during_sdk_reopen_keeps_tk_responsive_and_closes_transport(ui):
    start(ui)
    window = ui.window
    ui.gate.clear()
    window.forward.invoke()
    until(ui, lambda: len(ui.calls) == 2 and window.state['busy'])
    assert window.play.instate(['disabled'])
    beats = []
    ui.root.after(30, lambda: beats.append(1))
    until(ui, lambda: bool(beats))
    window.close()
    assert window.channel.stopping.is_set() and window.worker.is_alive()
    ui.gate.set()
    until(ui, window.channel.finished.is_set)
    assert all(camera.closed for camera in ui.cameras)
    assert window.channel.player.source is None


def test_worker_failure_is_visible_and_window_can_close(ui, monkeypatch):
    def fail(*args):
        raise RuntimeError('test source cannot open')

    monkeypatch.setattr(interactive, 'SvoSource', fail)
    ui.window.start()
    until(ui, lambda: ui.window.state['state'] == 'failed')
    assert 'test source cannot open' in ui.window.message.get()
    assert ui.window.play.instate(['disabled'])
    assert ui.window.result == 1


def test_command_acknowledgement_and_close_priority():
    channel = ControlChannel()
    assert channel.send('seek 10')
    assert channel.poll() == 'seek 10'
    assert not channel.send('seek 20')  # The previous command is still running.
    assert channel.poll() is None  # Returning to the scheduler acknowledges it.
    assert channel.send('seek 20')
    channel.stopping.set()
    assert channel.poll() == 'quit'


def test_position_labels():
    assert time_label(300) == '05:00.0'
    assert time_label(3661.5) == '1:01:01.5'
