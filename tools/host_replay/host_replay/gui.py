"""Native playback window; all SVO and MAVLink operations stay on one worker."""

from pathlib import Path
from queue import Empty, Full, Queue
import sys
from threading import Event, Thread
import time

from .interactive import LivePlayback


def time_label(seconds):
    """Format a recording position as minutes, seconds and tenths."""
    minutes, seconds = divmod(max(0., seconds), 60)
    hours, minutes = divmod(int(minutes), 60)
    return (f'{hours:d}:{minutes:02d}:{seconds:04.1f}' if hours else
            f'{minutes:02d}:{seconds:04.1f}')


class ControlChannel:
    """Pass commands and the latest status between Tk and the SDK worker."""

    def __init__(self):
        """Keep pending work bounded and make window closure take priority."""
        self.commands, self.updates = Queue(maxsize=1), Queue(maxsize=1)
        self.stopping, self.finished, self.pending = Event(), Event(), Event()
        self.player = None
        self.inflight = False
        self.next_update = 0.

    def publish(self, state):
        """Coalesce status changes so a slow display cannot delay playback."""
        try:
            self.updates.get_nowait()
        except Empty:
            pass
        self.updates.put_nowait(state)

    def send(self, command):
        """Queue a single UI action; reject more work until it is consumed."""
        if self.stopping.is_set() or self.pending.is_set():
            return False
        self.pending.set()
        try:
            self.commands.put_nowait(command)
            return True
        except Full:
            self.pending.clear()
            return False

    def poll(self):
        """Refresh the display at 10 Hz without moving SDK objects across threads."""
        if self.stopping.is_set():
            return 'quit'
        if self.inflight:
            # The previous command has returned to the scheduler. An older
            # display snapshot must not re-enable buttons before this point.
            self.inflight = False
            self.pending.clear()
            self.next_update = 0.
        now = time.monotonic()
        if self.player is not None and now >= self.next_update:
            self.publish(self.player.snapshot())
            self.next_update = now + .1
        try:
            command = self.commands.get_nowait()
            self.inflight = True
            return command
        except Empty:
            return None


class PlaybackWindow:
    """A seek timeline, playback buttons and frame entry for the host source."""

    def __init__(self, root, args, recording, coverage, play_loop):
        """Build widgets on the main thread before starting the SDK worker."""
        import tkinter as tk
        from tkinter import ttk

        self.root, self.args = root, args
        self.recording, self.coverage, self.play_loop = recording, coverage, play_loop
        self.channel = ControlChannel()
        self.worker = Thread(target=self._work, name='svo-playback')
        self.result = 0
        self.local_error = ''
        self.closing = self.dragging = self.pending = False
        self.state = {'state': 'paused', 'busy': True, 'frame': args.play_from_frame,
                      'total_frames': None, 'fps': None, 'error': ''}
        root.title('Host SVO Playback')
        root.geometry('820x330')
        root.minsize(720, 330)
        root.configure(background='#f4f6fa')
        root.protocol('WM_DELETE_WINDOW', self.close)

        style = ttk.Style(root)
        style.theme_use('clam')
        style.configure('TFrame', background='#f4f6fa')
        style.configure('TLabel', background='#f4f6fa', foreground='#18243b')
        style.configure('Title.TLabel', font=('DejaVu Sans', 18, 'bold'))
        style.configure('Muted.TLabel', foreground='#56647b')
        style.configure('Status.TLabel', foreground='#2563a6')
        style.configure('TButton', padding=(12, 8))
        style.configure('Accent.TButton', background='#2563a6', foreground='white')
        style.map('Accent.TButton', background=[('active', '#1e508a'),
                                                ('disabled', '#b8c7da')])

        body = ttk.Frame(root, padding=(24, 20))
        body.pack(fill='both', expand=True)
        ttk.Label(body, text='SVO Playback', style='Title.TLabel').pack(anchor='w')
        ttk.Label(body, text=Path(args.svo_path).name, style='Muted.TLabel').pack(
            anchor='w', pady=(4, 18))
        position_row = ttk.Frame(body)
        position_row.pack(fill='x')
        self.position = tk.StringVar(value='Opening recording…')
        self.frames = tk.StringVar(value='')
        ttk.Label(position_row, textvariable=self.position).pack(side='left')
        ttk.Label(position_row, textvariable=self.frames, style='Muted.TLabel').pack(side='right')
        self.scale = ttk.Scale(body, from_=0, to=1, command=self._preview)
        self.scale.pack(fill='x', pady=(6, 18))
        self.scale.bind('<ButtonPress-1>', self._begin_drag)
        self.scale.bind('<ButtonRelease-1>', self._end_drag)
        # Keyboard changes also commit once, when the key is released.
        self.scale.bind('<KeyPress-Left>', self._begin_drag)
        self.scale.bind('<KeyPress-Right>', self._begin_drag)
        self.scale.bind('<KeyRelease-Left>', self._end_drag)
        self.scale.bind('<KeyRelease-Right>', self._end_drag)

        actions = ttk.Frame(body)
        actions.pack(fill='x')
        self.back = ttk.Button(actions, text='−10 s', width=5,
                               command=lambda: self.submit('skip -10'))
        self.back.pack(side='left')
        self.play = ttk.Button(actions, text='Pause', width=9,
                               style='Accent.TButton', command=self.toggle)
        self.play.pack(side='left', padx=8)
        self.forward = ttk.Button(actions, text='+10 s', width=5,
                                  command=lambda: self.submit('skip 10'))
        self.forward.pack(side='left')
        self.replay = ttk.Button(actions, text='Replay', width=6,
                                 command=lambda: self.submit('replay'))
        self.replay.pack(side='left', padx=(8, 0))
        self.go = ttk.Button(actions, text='Go', width=3, command=self.seek_frame)
        self.go.pack(side='right')
        self.frame_value = tk.StringVar()
        self.frame_entry = ttk.Entry(actions, textvariable=self.frame_value, width=9)
        self.frame_entry.pack(side='right', padx=8)
        self.frame_entry.bind('<Return>', lambda _event: self.seek_frame())
        ttk.Label(actions, text='Frame').pack(side='right')

        self.message = tk.StringVar(value='Opening recording…')
        self.status_label = ttk.Label(body, textvariable=self.message, style='Status.TLabel',
                                      wraplength=750)
        self.status_label.pack(anchor='w', pady=(18, 8))
        ttk.Label(body, text='Seeking restarts the stream. Allow time for Jetson to reconnect.',
                  style='Muted.TLabel').pack(anchor='w')
        self.controls = (self.scale, self.back, self.play, self.forward, self.replay,
                         self.frame_entry, self.go)
        self._enable(False)
        root.update_idletasks()
        root.minsize(max(720, body.winfo_reqwidth()), max(330, body.winfo_reqheight()))
        root.bind('<space>', self._space)
        root.after(50, self._refresh)

    def start(self):
        """Start playback after the window is ready to report startup progress."""
        self.worker.start()

    def _work(self):
        player = None
        try:
            player = LivePlayback(self.args, self.recording, self.coverage,
                                  on_update=self.channel.publish)
            self.channel.player = player
            try:
                self.play_loop(player, self.channel)
            finally:
                player.close()
        except Exception as error:
            self.result = 1
            state = player.snapshot() if player is not None else dict(self.state)
            state.update(state='failed', busy=False, error=str(error))
            self.channel.publish(state)
            print(f'Replay failed: {error}', file=sys.stderr, flush=True)
        finally:
            self.channel.finished.set()

    def _enable(self, enabled):
        for widget in self.controls:
            widget.state(['!disabled'] if enabled else ['disabled'])

    def submit(self, command):
        """Disable actions until the worker acknowledges a playback change."""
        if (self.closing or self.pending or self.state['busy'] or
                self.channel.finished.is_set()):
            return False
        if not self.channel.send(command):
            return False
        self.pending = True
        self.local_error = ''
        self.dragging = False
        self._enable(False)
        self.message.set('Updating playback…')
        return True

    def toggle(self):
        """Pause a playing source or resume/replay a stopped one."""
        state = self.state['state']
        return self.submit('pause' if state == 'playing' else
                           'replay' if state == 'ended' else 'resume')

    def seek_frame(self):
        """Accept an exact frame index without sending malformed commands."""
        total = self.state['total_frames']
        if total is None:
            return False
        try:
            frame = int(self.frame_value.get())
            if not 0 <= frame < total:
                raise ValueError
        except ValueError:
            self.local_error = f'Enter a frame from 0 to {total - 1}.'
            self.message.set(self.local_error)
            return False
        return self.submit(f'seek {frame}')

    def _space(self, _event):
        if self.root.focus_get() is not self.frame_entry:
            self.toggle()
            return 'break'

    def _begin_drag(self, _event):
        if not self.state['busy'] and not self.pending:
            self.dragging = True

    def _end_drag(self, _event):
        if self.dragging:
            self.root.after_idle(self._seek_selected)

    def _seek_selected(self):
        if self.dragging:
            frame = round(self.scale.get())
            self.dragging = False
            self.submit(f'seek {frame}')

    def _preview(self, value):
        if self.dragging and self.state['fps']:
            frame = round(float(value))
            self._position(frame)
            self.frame_value.set(str(frame))

    def _position(self, frame):
        fps, total = self.state['fps'], self.state['total_frames']
        if fps and total:
            self.position.set(f'{time_label(frame / fps)} / {time_label((total - 1) / fps)}')
            self.frames.set(f'Frame {frame:,} / {total - 1:,}')

    def _refresh(self):
        try:
            state = self.channel.updates.get_nowait()
        except Empty:
            state = None
        if state is not None and not self.closing:
            self.state = state
            self.pending = self.channel.pending.is_set()
            available = bool(state['total_frames']) and state['state'] != 'failed'
            self._enable(available and not state['busy'] and not self.pending)
            if available:
                self.scale.configure(to=max(1, state['total_frames'] - 1))
                if not self.dragging:
                    frame = min(state['frame'], state['total_frames'] - 1)
                    self.scale.set(frame)
                    self._position(frame)
                    if self.root.focus_get() is not self.frame_entry:
                        self.frame_value.set(str(frame))
            self.play.configure(text='Pause' if state['state'] == 'playing' else
                                'Play again' if state['state'] == 'ended' else 'Play')
            error = self.local_error or state['error']
            self.message.set(error or ('Updating playback…' if state['busy'] else {
                'playing': 'Playing', 'paused': 'Paused — choose a position, then press Play.',
                'ended': 'End of recording — choose a position or press Replay.',
            }.get(state['state'], 'Playback stopped.')))
            self.status_label.configure(foreground='#b42318' if error else '#2563a6')
        if self.closing and self.channel.finished.is_set():
            self.root.destroy()
        else:
            self.root.after(50, self._refresh)

    def close(self):
        """Keep the window responsive while the worker releases its SDK/socket."""
        self.closing = True
        self.channel.stopping.set()
        self._enable(False)
        self.message.set('Closing playback…')


def run_gui(args, recording, coverage, play_loop):
    """Open a native window and wait for clean source shutdown before returning."""
    try:
        import tkinter as tk
    except ImportError as error:
        raise RuntimeError('GUI requires Tk; install python3-tk, or use --interactive') from error
    try:
        root = tk.Tk()
    except tk.TclError as error:
        raise RuntimeError(
            'Cannot open the GUI display; check DISPLAY and the X11 mount') from error
    window = PlaybackWindow(root, args, recording, coverage, play_loop)
    window.start()
    try:
        root.mainloop()
    except KeyboardInterrupt:
        window.close()
        while not window.channel.finished.is_set():
            root.update()
            time.sleep(.02)
        raise
    finally:
        window.channel.stopping.set()
        window.worker.join()
        try:
            root.destroy()
        except tk.TclError:
            pass
    return window.result
