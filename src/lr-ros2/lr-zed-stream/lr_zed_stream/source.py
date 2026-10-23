"""Serialized SDK operations, independent of ROS for playback testing."""

from dataclasses import dataclass
import math
from pathlib import Path


@dataclass(frozen=True)
class SourceOptions:
    """Configuration of one SDK source and its encoder."""

    svo_path: str = 'live'
    serial_number: int = 0
    camera_id: int = -1
    resolution: str = 'HD720'
    fps: int = 60
    port: int = 30000
    codec: str = 'H264'
    bitrate: int = 20000
    gop_size: int = 1
    replay_rate: float = 1.0
    start_paused: bool = False
    play_from_frame: int = 0

    def validate(self):
        """Fail before opening a camera or binding the encoder port."""
        if self.svo_path != 'live':
            path = Path(self.svo_path)
            if not path.is_absolute() or not path.is_file():
                raise ValueError('svo_path must be an existing absolute SVO/SVO2 path, or live')
        if not 2 <= self.port <= 65534 or self.port % 2:
            raise ValueError('port must be even and in [2, 65534]')
        if self.codec not in ('H264', 'H265'):
            raise ValueError('codec must be H264 or H265')
        if not 1000 <= self.bitrate <= 60000:
            raise ValueError('bitrate must be in [1000, 60000] Kbit/s')
        if self.gop_size != -1 and not 1 <= self.gop_size <= 256:
            raise ValueError('gop_size must be -1 or in [1, 256]')
        if not math.isfinite(self.replay_rate) or not 0.1 <= self.replay_rate <= 5.0:
            raise ValueError('replay_rate must be in [0.1, 5.0]')
        if self.fps <= 0 or self.serial_number < 0 or self.camera_id < -1:
            raise ValueError('invalid fps, serial_number or camera_id')
        if self.serial_number and self.camera_id >= 0:
            raise ValueError('select a camera by serial_number or camera_id, not both')
        if self.play_from_frame < 0:
            raise ValueError('play_from_frame must be non-negative')
        if self.svo_path == 'live' and (self.start_paused or self.play_from_frame):
            raise ValueError('start_paused and play_from_frame require an SVO file')


class StreamSource:
    """Own the camera; callers must serialize tick, seek, toggle and close."""

    def __init__(self, options, sdk=None):
        """Open the selected input and encoder, closing the SDK on startup failure."""
        options.validate()
        if sdk is None:
            import pyzed.sl as sdk
        self.sdk = sdk
        self.options = options
        self.svo_mode = options.svo_path != 'live'
        self.paused = options.start_paused
        self.ended = False
        self.frame_id = 0
        self.stamp_ns = 0
        self.total_frames = 0
        self.camera = sdk.Camera()
        self.runtime = sdk.RuntimeParameters()
        self.runtime.enable_depth = False
        try:
            init = sdk.InitParameters()
            init.depth_mode = sdk.DEPTH_MODE.NONE
            # Pace on the ROS steady timer. Every source frame remains seekable,
            # and pause does not depend on a specially compiled ROS wrapper.
            init.svo_real_time_mode = False
            if self.svo_mode:
                init.set_from_svo_file(options.svo_path)
            else:
                init.camera_resolution = getattr(sdk.RESOLUTION, options.resolution)
                init.camera_fps = options.fps
                if options.serial_number:
                    init.set_from_serial_number(options.serial_number)
                elif options.camera_id >= 0:
                    init.set_from_camera_id(options.camera_id)
            self._check(self.camera.open(init), 'open source')
            self.fps = float(self.camera.get_camera_information().camera_configuration.fps)
            if not math.isfinite(self.fps) or self.fps <= 0:
                raise RuntimeError('SDK returned an invalid source frame rate')
            if self.svo_mode:
                self.total_frames = self.camera.get_svo_number_of_frames()
                if not 0 <= options.play_from_frame < self.total_frames:
                    raise ValueError('play_from_frame is outside the SVO frame range')
                self._check(
                    self.camera.set_svo_position(options.play_from_frame), 'set start frame'
                )
            stream = sdk.StreamingParameters()
            stream.port = options.port
            stream.codec = getattr(sdk.STREAMING_CODEC, options.codec)
            stream.bitrate = options.bitrate
            stream.gop_size = options.gop_size
            stream.adaptative_bitrate = False
            self._check(self.camera.enable_streaming(stream), 'enable streaming')
            # Establish a valid frame even when starting paused.
            self._read()
        except BaseException:
            self.camera.close()
            raise

    @property
    def period(self):
        """Use native capture rate for live, requested playback rate for SVO."""
        return 1.0 / (self.fps * (self.options.replay_rate if self.svo_mode else 1.0))

    def _check(self, result, operation):
        if result != self.sdk.ERROR_CODE.SUCCESS:
            raise RuntimeError(f'Cannot {operation}: {result}')

    def _read(self):
        # Use the SDK's standard streaming path with depth explicitly disabled.
        result = self.camera.grab(self.runtime)
        if self.svo_mode and result == self.sdk.ERROR_CODE.END_OF_SVOFILE_REACHED:
            self.ended = True
            return
        self._check(result, 'read source frame')
        if self.svo_mode:
            self.frame_id = self.camera.get_svo_position()
        self.stamp_ns = self.camera.get_timestamp(self.sdk.TIME_REFERENCE.IMAGE).get_nanoseconds()

    def tick(self):
        """Grab a new frame, or request the held frame for experimental controls."""
        if self.svo_mode and (self.paused or self.ended):
            # Preserve IMAGE time. SDK 5.4.1 can deduplicate these grabs on NET;
            # they do not guarantee receiver liveness or delivery during pause.
            self._check(self.camera.set_svo_position(self.frame_id), 'hold SVO frame')
        self._read()

    def toggle(self):
        """Toggle playback; keep the last frame available at EOF."""
        if not self.svo_mode:
            raise ValueError('Playback controls require an SVO source')
        if self.ended:
            self.seek(0)
        else:
            self.paused = not self.paused
        return 'SVO is paused' if self.paused else 'SVO is playing'

    def seek(self, frame):
        """Read and send the target once, preserving pause except during Replay."""
        if not self.svo_mode:
            raise ValueError('Playback controls require an SVO source')
        if not 0 <= frame < self.total_frames:
            raise ValueError(f'Frame must be in [0, {self.total_frames - 1}]')
        was_ended = self.ended
        self._check(self.camera.set_svo_position(frame), 'seek SVO')
        self.ended = False
        self._read()
        if self.ended:
            raise RuntimeError(f'SDK could not read SVO frame {frame}')
        if was_ended:
            self.paused = False

    def close(self):
        """Release the stream and camera, including on ROS shutdown."""
        self.camera.disable_streaming()
        self.camera.close()
