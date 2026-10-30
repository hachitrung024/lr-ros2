"""Terminal/GUI playback using a standard, independently scheduled FCU."""

from contextlib import ExitStack
import json
from pathlib import Path
import time

from .interactive import LivePlayback
from .native_worker import NativeProcess
from .source import SvoEnd, SvoSource


class NativePlayback(LivePlayback):
    def __init__(self, args, recording, coverage, **kwargs):
        super().__init__(args, recording, coverage, **kwargs)
        self.fcu = NativeProcess(args)
        self.net_enabled = True

    def _start(self, frame):
        self.resources.close()
        self.source = self.telemetry = None
        self.next_frame, self.state = frame, 'paused'
        with ExitStack() as resources:
            try:
                source = SvoSource(self.args.svo_path, self.args.stream_port, frame)
            except SvoEnd as end:
                self.total_frames, self.fps = end.total_frames, end.fps
                self.stop(ended=True)
                return
            resources.callback(source.close)
            if not self.coverage[0]-60_000_000_000 <= source.stamp_ns <= self.coverage[1]+60_000_000_000:
                raise ValueError('The MAVLink DB does not cover the target SVO timestamp')
            self.fcu.send(dict(action='anchor', record_ns=source.stamp_ns,
                               monotonic_ns=time.monotonic_ns()))
            self.source, self.telemetry = source, self.fcu
            self.total_frames, self.fps = source.total_frames, source.fps
            self.net_enabled = True
            self.state = 'playing'
            self.resources = resources.pop_all()
        self.status()

    def stop(self, *, ended=False):
        with self.changing():
            if ended:
                self.next_frame = self.total_frames
            elif self.source is not None:
                self.next_frame = self.source.frame_id+1
            self.resources.close()
            self.source = self.telemetry = None
            self.fcu.send(dict(action='pause'))
            self.state = 'ended' if self.next_frame == self.total_frames else 'paused'
            self.status()

    def status(self):
        frame = self.source.frame_id if self.source else self.next_frame
        print(f'{self.state}: frame={frame}/{self.total_frames} '
              f'position={frame/self.fps:.3f}s protocol=native', flush=True)
        self.notify()

    def poll(self, commands):
        replaced = super().poll(commands)
        if not self.quitting:
            self.fcu.service()
        if self.source is not None and self.args.control_file:
            try:
                enabled = json.loads(Path(self.args.control_file).read_text()).get('camera_enabled', True)
                if enabled != self.net_enabled:
                    if enabled:
                        self.source._check(self.source.camera.enable_streaming(
                            self.source.streaming), 'resume NET')
                    else:
                        self.source.camera.disable_streaming()
                    self.net_enabled = enabled
            except (FileNotFoundError, ValueError):
                pass
        return replaced

    def close(self):
        super().close()
        if hasattr(self, 'fcu'):
            self.fcu.close()
