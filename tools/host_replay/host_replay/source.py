"""Sequential SVO playback using only the ZED SDK."""

import math
from pathlib import Path


class SvoEnd(RuntimeError):
    """The SDK reports EOF at a requested position inside its advertised range."""

    def __init__(self, total_frames, fps):
        """Retain metadata so the terminal can remain available at EOF."""
        super().__init__('SVO has no frame at the selected position')
        self.total_frames, self.fps = total_frames, fps


class SvoSource:
    """Own one SVO and NET encoder; never seek after the selected start frame."""

    def __init__(self, svo_path, port, play_from_frame=0, sdk=None):
        path = Path(svo_path).expanduser().resolve(strict=True)
        if not path.is_file():
            raise ValueError('svo_path must be a file')
        if not 2 <= port <= 65534 or port % 2:
            raise ValueError('stream_port must be even and in [2, 65534]')
        if play_from_frame < 0:
            raise ValueError('play_from_frame must be non-negative')
        if sdk is None:
            import pyzed.sl as sdk
        self.sdk = sdk
        self.camera = sdk.Camera()
        self.closed = False
        self.ended = False
        self.frame_id = 0
        self.stamp_ns = 0
        try:
            init = sdk.InitParameters()
            init.depth_mode = sdk.DEPTH_MODE.NONE
            init.svo_real_time_mode = False
            init.set_from_svo_file(str(path))
            self._check(self.camera.open(init), 'open SVO')
            self.total_frames = self.camera.get_svo_number_of_frames()
            if not 0 <= play_from_frame < self.total_frames:
                raise ValueError('play_from_frame is outside the SVO frame range')
            self.fps = float(self.camera.get_camera_information().camera_configuration.fps)
            if not math.isfinite(self.fps) or self.fps <= 0:
                raise RuntimeError('SDK returned an invalid source frame rate')
            self.period = 1.0 / self.fps
            self._check(self.camera.set_svo_position(play_from_frame), 'set start frame')
            streaming = sdk.StreamingParameters()
            streaming.port = port
            streaming.codec = sdk.STREAMING_CODEC.H264
            streaming.bitrate = 20000
            streaming.gop_size = 1
            streaming.adaptative_bitrate = False
            self._check(self.camera.enable_streaming(streaming), 'enable NET streaming')
            self.runtime = sdk.RuntimeParameters()
            self.runtime.enable_depth = False
            if not self.tick():
                raise SvoEnd(self.total_frames, self.fps)
        except BaseException:
            self.close()
            raise

    def _check(self, result, operation):
        if result != self.sdk.ERROR_CODE.SUCCESS:
            raise RuntimeError(f'Cannot {operation}: {result}')

    def tick(self):
        """Grab once; stop at EOF without resending or looping the last frame."""
        if self.ended:
            return False
        result = self.camera.grab(self.runtime)
        if result == self.sdk.ERROR_CODE.END_OF_SVOFILE_REACHED:
            self.ended = True
            return False
        self._check(result, 'read source frame')
        stamp = self.camera.get_timestamp(self.sdk.TIME_REFERENCE.IMAGE).get_nanoseconds()
        if stamp <= 0 or stamp <= self.stamp_ns:
            raise RuntimeError('SVO IMAGE timestamps must be positive and strictly increasing')
        self.stamp_ns = stamp
        self.frame_id = self.camera.get_svo_position()
        return True

    def close(self):
        """Release the camera even if encoder setup or shutdown fails."""
        if not self.closed:
            self.closed = True
            try:
                self.camera.disable_streaming()
            finally:
                self.camera.close()
