"""Synchronization regressions for delayed masks and bounded replay history."""

from types import SimpleNamespace
import pytest

from lr_segmentation.image_pair_buffer import ImagePairBuffer


def message(ms):
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=ms // 1000, nanosec=(ms % 1000) * 1_000_000)
        )
    )


def test_delayed_mask_matches_older_depth_instead_of_latest():
    pairs = ImagePairBuffer(50_000_000)
    early = message(1000)
    pairs.add('depth', early)
    pairs.add('depth', message(1100))
    pairs.add('depth', message(1200))
    mask = message(1000)
    pairs.add('mask', mask)
    assert pairs.pop_pair() == (mask, early)
    pairs.add('mask', mask)
    assert pairs.pop_pair() is None


def test_nearest_pair_tolerance_and_each_depth_used_once():
    pairs = ImagePairBuffer(50_000_000)
    for stamp in (990, 1030):
        pairs.add('depth', message(stamp))
    mask = message(1000)
    pairs.add('mask', mask)
    assert pairs.pop_pair()[1].header.stamp.nanosec == 990_000_000
    pairs.add('mask', message(1200))
    assert pairs.pop_pair() is None


def test_buffers_expire_and_reset_after_seek():
    pairs = ImagePairBuffer(50_000_000)
    for stamp in range(1000, 2000, 10):
        pairs.add('depth', message(stamp))
    assert len(pairs.depths) == 30
    pairs.add('mask', message(1000))
    assert pairs.pop_pair() is None
    pairs.add('depth', message(3000))
    assert len(pairs.depths) == 1
    assert not pairs.masks
    pairs.reset()
    pairs.add('mask', message(500))
    assert pairs.pop_pair() is None
    pairs.add('depth', message(500))
    assert pairs.pop_pair() is not None


@pytest.mark.parametrize('kind', ['mask', 'depth'])
def test_inflight_pre_seek_input_cannot_poison_new_timeline(kind):
    pairs = ImagePairBuffer(50_000_000)
    pairs.add('depth', message(50_000), now_ns=50_000_000_000)
    pairs.reset()
    # Simulate the previous timeline's inference finishing after the clock reset.
    pairs.add(kind, message(50_000), now_ns=10_000_000_000)
    assert pairs.latest_stamp is None
    assert pairs.future_inputs == 1
    current = message(10_100)
    pairs.add('depth', current, now_ns=10_100_000_000)
    pairs.add('mask', current, now_ns=10_200_000_000)
    assert pairs.pop_pair() == (current, current)
    assert pairs.pair_count == 1


def test_small_clock_delivery_skew_is_tolerated():
    pairs = ImagePairBuffer(50_000_000)
    current = message(10_100)
    pairs.add('mask', current, now_ns=10_000_000_000)
    pairs.add('depth', current, now_ns=10_000_000_000)
    assert pairs.pop_pair() == (current, current)
