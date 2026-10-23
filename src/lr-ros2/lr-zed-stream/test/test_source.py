"""Verify source playback without a camera or GPU."""

from dataclasses import replace

import pytest

from lr_zed_stream.source import SourceOptions, StreamSource


def test_pause_holds_sdk_frame_without_advancing_clock(sdk, svo):
    source = StreamSource(svo, sdk)
    source.tick()
    stamp, frame = source.stamp_ns, source.frame_id
    source.toggle()
    # This checks source semantics, not delivery by the SDK's NET encoder.
    for _ in range(2000):
        source.tick()
    assert source.stamp_ns == stamp
    assert source.camera.sent[-2000:] == [frame] * 2000
    source.toggle()
    source.tick()
    assert source.frame_id == frame + 1


def test_paused_seek_sends_exact_target_and_keeps_pause(sdk, svo):
    source = StreamSource(replace(svo, start_paused=True), sdk)
    source.seek(2)
    assert source.paused
    assert source.frame_id == 2
    stamp = source.stamp_ns
    source.tick()
    assert source.frame_id == 2
    assert source.stamp_ns == stamp
    source.seek(0)
    assert source.stamp_ns < stamp
    assert source.paused


def test_eof_holds_last_frame_and_replay_resumes(sdk, svo):
    source = StreamSource(svo, sdk)
    for _ in range(4):
        source.tick()
    assert source.ended
    stamp = source.stamp_ns
    for _ in range(10):
        source.tick()
        assert source.ended
        assert source.frame_id == 3
        assert source.stamp_ns == stamp
    source.seek(0)
    assert not source.ended
    assert not source.paused
    source.tick()
    assert source.frame_id == 1


@pytest.mark.parametrize('frame', [-1, 4])
def test_invalid_seek_does_not_change_playback(sdk, svo, frame):
    source = StreamSource(svo, sdk)
    with pytest.raises(ValueError):
        source.seek(frame)
    source.tick()
    assert source.frame_id == 1


def test_live_selects_camera_and_disables_depth(sdk):
    source = StreamSource(SourceOptions(serial_number=123), sdk)
    assert source.camera.init.serial == 123
    assert source.camera.init.depth_mode == sdk.DEPTH_MODE.NONE
    assert source.camera.stream.bitrate == 20000
    assert source.camera.stream.gop_size == 1
    with pytest.raises(ValueError):
        source.seek(0)
    with pytest.raises(ValueError):
        source.toggle()
    source.close()
    assert source.camera.closed


@pytest.mark.parametrize('options', [
    SourceOptions(port=30001), SourceOptions(bitrate=0),
    SourceOptions(gop_size=0), SourceOptions(replay_rate=float('nan')),
    SourceOptions(serial_number=1, camera_id=0), SourceOptions(start_paused=True),
    SourceOptions(svo_path='relative.svo2'),
])
def test_invalid_configuration_fails_before_camera_open(options):
    with pytest.raises(ValueError):
        options.validate()
