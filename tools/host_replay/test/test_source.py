"""Verify sequential SDK semantics without ROS, camera hardware or a GPU."""

from types import SimpleNamespace

import pytest

from host_replay.source import SvoSource


def test_all_frames_once_and_no_eof_hold(sdk, svo):
    source = SvoSource(svo, 32000, sdk=sdk)
    camera = source.camera
    stamps = [source.stamp_ns]
    assert camera.init.depth_mode == sdk.DEPTH_MODE.NONE
    assert camera.init.svo_real_time_mode is False
    assert (camera.stream.codec, camera.stream.bitrate, camera.stream.gop_size) == (0, 20000, 1)
    while source.tick():
        stamps.append(source.stamp_ns)
    assert camera.sent == [0, 1, 2, 3]
    assert len(stamps) == 4 and stamps == sorted(set(stamps))
    assert not source.tick()
    assert camera.sent == [0, 1, 2, 3]
    source.close()
    source.close()
    assert camera.closed


def test_start_frame_and_sdk_failure_cleanup(sdk, svo):
    source = SvoSource(svo, 32000, 2, sdk=sdk)
    assert source.frame_id == 2
    assert source.camera.sent == [2]
    source.close()
    with pytest.raises(ValueError, match='outside'):
        SvoSource(svo, 32000, 4, sdk=sdk)
    assert source.camera.closed


def test_open_failure_closes_camera(sdk, svo):
    camera = sdk.Camera()
    camera.open = lambda _init: 99
    with pytest.raises(RuntimeError, match='open SVO'):
        SvoSource(svo, 32000, sdk=sdk)
    assert camera.closed


def test_non_increasing_image_timestamp_fails(sdk, svo):
    source = SvoSource(svo, 32000, sdk=sdk)
    source.camera.get_timestamp = lambda _: SimpleNamespace(
        get_nanoseconds=lambda: source.stamp_ns,
    )
    with pytest.raises(RuntimeError, match='strictly increasing'):
        source.tick()
    source.close()


@pytest.mark.parametrize('port', [0, 30001, 65535, 65536])
def test_invalid_stream_port_never_opens_sdk(sdk, svo, port):
    with pytest.raises(ValueError, match='stream_port'):
        SvoSource(svo, port, sdk=sdk)
    assert not sdk.Camera().sent
