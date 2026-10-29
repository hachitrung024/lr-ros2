"""Terminal controls for live playback, with a fresh NET/MAVLink session per seek."""

from collections import deque
from contextlib import ExitStack, contextmanager
import math
import os
import select
import sys

from .future_path import FuturePath
from .live import LiveTime
from .source import SvoEnd, SvoSource
from .telemetry import TelemetrySender


HELP = ('Commands: seek FRAME | skip SECONDS (signed) | pause | resume | replay | '
        'status | help | quit; press Enter after each command.')


class TerminalCommands:
    """Read complete lines without blocking the frame/TIMESYNC scheduler."""

    def __init__(self, stream=None):
        """Accept terminal or piped input without a reader thread."""
        self.fd = (sys.stdin if stream is None else stream).fileno()
        self.buffer = b''
        self.pending = deque()
        self.eof = False

    def poll(self):
        """Return one command, None when idle, or quit when input closes."""
        if not self.pending and not self.eof and select.select([self.fd], [], [], 0)[0]:
            chunk = os.read(self.fd, 4096)
            if not chunk:
                self.eof = True
                if self.buffer:
                    self.pending.append(self.buffer.decode('utf-8', errors='replace'))
                    self.buffer = b''
            else:
                lines = (self.buffer + chunk).split(b'\n')
                self.buffer = lines.pop()
                self.pending.extend(line.decode('utf-8', errors='replace') for line in lines)
                if len(self.buffer) > 4096:
                    self.pending.append('invalid-command-too-long')
                    self.buffer = b''
        if self.pending:
            return self.pending.popleft().strip()
        return 'quit' if self.eof else None


class LivePlayback:
    """Serialize SDK changes and retire all telemetry/path state at each restart."""

    def __init__(self, args, recording, coverage, *, on_update=None):
        """Retain decoded recording data while transport sessions are replaced."""
        self.args, self.recording, self.coverage = args, recording, coverage
        self.resources = ExitStack()
        self.source = self.telemetry = None
        self.total_frames = self.fps = None
        self.next_frame = args.play_from_frame
        self.state = 'paused'
        self.quitting = False
        self.busy, self.error = False, ''
        self.on_update = on_update
        self.future_path = (FuturePath(
            recording, radius_m=args.future_path_radius_m,
            step_m=args.future_path_step_m, rate_hz=args.future_path_rate_hz)
            if args.future_path else None)

    def close(self):
        """Release the current socket and NET encoder, including partial startup."""
        try:
            self.resources.close()
        finally:
            self.source = self.telemetry = None

    def start(self, frame):
        """Open a fresh source and calibration at the selected frame."""
        self.check_frame(frame)
        with self.changing():
            self._start(frame)

    def _start(self, frame):
        self.close()
        self.next_frame, self.state = frame, 'paused'
        with ExitStack() as resources:
            try:
                source = SvoSource(self.args.svo_path, self.args.stream_port, frame)
            except SvoEnd as end:
                self.total_frames, self.fps = end.total_frames, end.fps
                self.stop(ended=True)
                return
            resources.callback(source.close)
            if not self.coverage[0] - 60_000_000_000 <= source.stamp_ns <= (
                    self.coverage[1] + 60_000_000_000):
                raise ValueError('The selected MAVLink DB does not cover the target SVO timestamp')
            self.recording.reset()
            if self.future_path is not None:
                self.future_path.reset()
            live = LiveTime(source.stamp_ns)
            telemetry = TelemetrySender(
                self.recording, self.args.mavlink_bind_address, self.args.mavlink_bind_port,
                self.args.mavlink_destination, self.args.mavlink_destination_port,
                live=live, future_path=self.future_path)
            resources.callback(telemetry.close)
            self.source, self.telemetry = source, telemetry
            self.total_frames, self.fps = source.total_frames, source.fps
            self.state = 'playing'
            self.resources = resources.pop_all()
        self.status()

    def check_frame(self, frame):
        """Reject invalid targets before disturbing a running source."""
        if frame < 0 or (self.total_frames is not None and frame >= self.total_frames):
            raise ValueError(f'Frame must be in [0, {self.total_frames - 1}]' if (
                self.total_frames is not None) else 'Frame must be non-negative')

    def stop(self, *, ended=False):
        """Remember the next frame and stop transmitting while waiting for input."""
        with self.changing():
            if ended:
                self.next_frame = self.total_frames
            elif self.source is not None:
                self.next_frame = self.source.frame_id + 1
            self.close()
            self.state = 'ended' if self.next_frame == self.total_frames else 'paused'
            self.status()

    @contextmanager
    def changing(self):
        """Report blocking SDK changes without calling UI code on the worker."""
        previous = self.busy
        self.busy, self.error = True, ''
        self.notify()
        try:
            yield
        finally:
            self.busy = previous
            self.notify()

    def snapshot(self):
        """Copy the current position for a display without exposing SDK objects."""
        return {
            'state': self.state, 'busy': self.busy, 'error': self.error,
            'frame': self.source.frame_id if self.source is not None else self.next_frame,
            'total_frames': self.total_frames, 'fps': self.fps,
        }

    def notify(self):
        """Publish a status snapshot to an optional display queue."""
        if self.on_update is not None:
            self.on_update(self.snapshot())

    def status(self):
        """Print the source position and current session identity."""
        frame = self.source.frame_id if self.source is not None else self.next_frame
        session = str(self.telemetry.live.session_id) if self.telemetry is not None else '-'
        print(f'{self.state}: frame={frame}/{self.total_frames} '
              f'position={frame / self.fps:.3f}s session={session}', flush=True)
        self.notify()

    def command(self, line):
        """Apply one complete command on the SDK scheduler thread."""
        words = line.split()
        if not words:
            return
        self.error = ''
        name = words[0].lower()
        if name in ('seek', 'skip') and len(words) == 2:
            if name == 'seek':
                frame = int(words[1])
            else:
                seconds = float(words[1])
                if not math.isfinite(seconds):
                    raise ValueError('skip requires a finite number of seconds')
                current = self.source.frame_id if self.source is not None else self.next_frame
                frame = round(max(0, min(self.total_frames - 1, current + seconds * self.fps)))
            self.check_frame(frame)
            if self.source is not None:
                self.start(frame)
            else:
                self.next_frame, self.state = frame, 'paused'
                self.status()
        elif len(words) == 1 and name == 'pause':
            if self.source is not None:
                self.stop()
            else:
                self.status()
        elif len(words) == 1 and name == 'resume':
            if self.source is None:
                if self.state == 'ended':
                    raise ValueError('End of SVO; use seek FRAME or replay')
                self.start(self.next_frame)
            else:
                self.status()
        elif len(words) == 1 and name == 'replay':
            self.start(0)
        elif len(words) == 1 and name == 'status':
            self.status()
        elif len(words) == 1 and name == 'help':
            print(HELP, flush=True)
        elif len(words) == 1 and name == 'quit':
            self.quitting = True
            self.close()
        else:
            raise ValueError(HELP)

    def poll(self, commands):
        """Handle a command and tell the scheduler when its transport was replaced."""
        previous = self.source
        line = commands.poll()
        if line is not None:
            try:
                self.command(line)
            except (ValueError, RuntimeError, OSError) as error:
                self.error = str(error)
                self.notify()
                print(f'Control failed: {error}', file=sys.stderr, flush=True)
        return self.source is not previous or self.quitting
