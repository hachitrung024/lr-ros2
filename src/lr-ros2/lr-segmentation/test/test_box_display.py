"""Check bounded presentation retention without changing measured object output."""

import numpy as np
from builtin_interfaces.msg import Time
from std_msgs.msg import Header
from visualization_msgs.msg import Marker

from lr_segmentation.box_display import BoxDisplayBuffer
from lr_segmentation.box_geometry import TrackedBox


STYLE = dict(line_width_m=.07, fill_alpha=.12, label_scale_m=.28,
             footprint_z_offset_m=.03, prism_fill_alpha=.18, show_boxes=False)


def box():
    return TrackedBox(
        track_id=1, center=np.array([1., 0., 1.]), size=np.ones(3),
        orientation=np.array([0., 0., 0., 1.]),
        footprint_xy=np.array([[0., 0.], [1., 0.], [1., 1.], [0., 1.]]),
        minimum_z=.5, maximum_z=1.5,
    )


def test_missing_observation_retains_position_and_source_stamp_then_expires():
    buffer = BoxDisplayBuffer(.5)
    header = Header(frame_id='map', stamp=Time(sec=10))
    buffer.update([box()], header, 10_000_000_000)
    initial = buffer.markers(10_000_000_000, **STYLE)
    buffer.update([], Header(frame_id='map', stamp=Time(sec=10, nanosec=100_000_000)),
                  10_100_000_000)
    held = buffer.markers(10_100_000_000, **STYLE)
    assert held.markers[1].header.stamp == header.stamp
    assert held.markers[1].points == initial.markers[1].points
    assert held.markers[1].color.a < initial.markers[1].color.a
    assert held.markers[-1].text.endswith('(last seen)')
    assert buffer.markers(10_100_000_000, **STYLE) is None  # Frozen ROS time.
    expired = buffer.markers(10_600_000_000, **STYLE)
    assert len(expired.markers) == 1
    assert expired.markers[0].action == Marker.DELETEALL
    assert buffer.markers(11_000_000_000, **STYLE) is None


def test_recovery_updates_and_reset_discards_all_held_footprints():
    buffer = BoxDisplayBuffer(.5)
    header = Header(frame_id='map', stamp=Time(sec=10))
    buffer.update([box()], header, 10_000_000_000)
    buffer.markers(10_000_000_000, **STYLE)
    buffer.update([], header, 10_100_000_000)
    buffer.markers(10_100_000_000, **STYLE)
    buffer.update([box()], header, 10_200_000_000)
    recovered = buffer.markers(10_200_000_000, **STYLE)
    assert '(last seen)' not in recovered.markers[-1].text
    buffer.reset()
    assert buffer.markers(5_000_000_000, **STYLE) is None


def test_zero_hold_draws_measured_frame_but_clears_the_next_empty_frame():
    buffer = BoxDisplayBuffer(0.)
    header = Header(frame_id='map', stamp=Time(sec=10))
    buffer.update([box()], header, 10_000_000_000)
    assert len(buffer.markers(10_000_000_000, **STYLE).markers) > 1
    buffer.update([], header, 10_100_000_000)
    assert len(buffer.markers(10_100_000_000, **STYLE).markers) == 1
