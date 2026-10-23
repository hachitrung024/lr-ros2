"""Playback contracts with a small SDK double; no GPU or recording required."""

from types import SimpleNamespace

import pytest

from lr_zed_stream.source import SourceOptions


class FakeCamera:
    """Model the SDK frame cursor and retain actual streaming operations."""

    def __init__(self):
        self.cursor = 0
        self.frame = -1
        self.sent = []
        self.closed = False

    def open(self, init):
        self.init = init
        return 0

    def get_camera_information(self):
        return SimpleNamespace(camera_configuration=SimpleNamespace(fps=30))

    def get_svo_number_of_frames(self):
        return 4

    def set_svo_position(self, frame):
        self.cursor = frame
        return 0

    def get_svo_position(self):
        return self.frame

    def enable_streaming(self, options):
        self.stream = options
        return 0

    def read(self):
        if self.cursor >= 4:
            return 1
        self.frame = self.cursor
        self.cursor += 1
        self.sent.append(self.frame)
        return 0

    def grab(self, runtime):
        assert runtime.enable_depth is False
        return self.read()

    def get_timestamp(self, _reference):
        return SimpleNamespace(get_nanoseconds=lambda: 1_000_000_000 + self.frame * 33_333_333)

    def disable_streaming(self):
        self.stream = None

    def close(self):
        self.closed = True


class FakeInit:
    """Retain source selection for live and SVO assertions."""

    def set_from_svo_file(self, path):
        self.path = path

    def set_from_serial_number(self, serial):
        self.serial = serial

    def set_from_camera_id(self, camera_id):
        self.camera_id = camera_id


@pytest.fixture
def sdk():
    camera = FakeCamera()
    return SimpleNamespace(
        Camera=lambda: camera, InitParameters=FakeInit,
        StreamingParameters=SimpleNamespace,
        RuntimeParameters=SimpleNamespace,
        ERROR_CODE=SimpleNamespace(SUCCESS=0, END_OF_SVOFILE_REACHED=1),
        TIME_REFERENCE=SimpleNamespace(IMAGE=0), DEPTH_MODE=SimpleNamespace(NONE=0),
        RESOLUTION=SimpleNamespace(HD720=0), STREAMING_CODEC=SimpleNamespace(H264=0, H265=1),
    )


@pytest.fixture
def svo(tmp_path):
    path = tmp_path / 'test.svo2'
    path.touch()
    return SourceOptions(svo_path=str(path))
