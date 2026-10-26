"""Regression tests for radius-based, smooth and temporally stable path output."""

import numpy as np
import pytest

from lr_future_path.path_processing import prepare_path, StablePathProcessor


def process(points, *, stamps=None, **kwargs):
    """Process a synthetic route with identity attitudes."""
    points = np.asarray(points, dtype=float)
    if stamps is None:
        stamps = np.arange(len(points), dtype=np.int64) * 10 ** 9
    return prepare_path(
        stamps, points, np.tile([0., 0., 0., 1.], (len(points), 1)),
        radius_m=kwargs.pop('radius_m', 15.), step_m=kwargs.pop('step_m', .2), **kwargs,
    )


def test_geometry_is_independent_of_speed_and_long_stops():
    """Retiming the same path never changes the displayed spatial geometry."""
    points = [[0, 0, 0], [3, 0, 0], [10, 0, 0], [20, 0, 0]]
    fast = process(points)
    slow = process(points, stamps=np.array([0, 10, 100, 500]) * 10 ** 9)
    np.testing.assert_allclose(fast[1], slow[1])
    assert len(slow[1]) == 76
    np.testing.assert_allclose(slow[1][-1], [15, 0, 0])
    assert slow[0][-1] > 20 * 10 ** 9
    assert np.all(np.diff(slow[0]) > 0)


def test_first_exit_interpolation_and_turns_use_radius_not_arc_length():
    """Clip at the circle, even when a later segment returns inside it."""
    _, points, _ = process([[0, 0, 1], [3, 0, 1], [3, 6, 1], [0, 0, 1]], radius_m=5.)
    np.testing.assert_allclose(points[-1], [3, 4, 1])
    assert np.all(np.linalg.norm(points[:, :2], axis=1) <= 5. + 1e-9)
    assert np.linalg.norm(np.diff(points[:, :2], axis=0), axis=1).sum() > 5.


def test_smoothing_reduces_jitter_but_preserves_anchor_and_limits_deviation():
    """Suppress lateral sensor noise without moving the current pose or cutting far corners."""
    x = np.linspace(0, 10, 101)
    raw = np.column_stack((x, .06 * np.sin(x * 20), x * 0))
    _, unsmoothed, _ = process(raw, step_m=.1, smoothing_window_m=0)
    _, smoothed, _ = process(raw, step_m=.1, smoothing_window_m=1., max_deviation_m=.08)
    assert np.std(smoothed[:, 1]) < np.std(unsmoothed[:, 1]) * .5
    assert np.max(np.linalg.norm(smoothed - unsmoothed, axis=1)) <= .08 + 1e-9
    np.testing.assert_allclose(smoothed[[0, -1]], unsmoothed[[0, -1]])


def test_loop_workload_is_bounded_by_distance_and_point_count():
    """Repeated laps inside the circle cannot fill the view with the entire recording."""
    points = np.tile([[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0]], (20, 1))
    _, output, _ = process(points, radius_m=2, step_m=.001, smoothing_window_m=0)
    assert len(output) == 256
    length = np.linalg.norm(np.diff(output[:, :2], axis=0), axis=1).sum()
    assert length <= 4. + 1e-9


def test_duplicate_positions_and_quaternion_sign_changes_are_finite():
    """Stops and equivalent quaternion signs cannot create NaNs."""
    stamps, points, quaternions = prepare_path(
        [0, 1, 2], [[0, 0, 0], [0, 0, 0], [1, 0, 0]],
        [[0, 0, 0, 1], [0, 0, 0, -1], [0, 0, 0, -1]], radius_m=3., step_m=.1,
    )
    assert np.isfinite(points).all()
    np.testing.assert_allclose(np.linalg.norm(quaternions, axis=1), 1.)
    assert np.all(np.diff(stamps) >= 0)


@pytest.mark.parametrize('limit', [1, 4])
def test_configured_point_limit_is_preserved_by_resampling(limit):
    """Resampling must not exceed a caller's tighter output limit."""
    _, points, _ = process([[0, 0, 0], [10, 0, 0]], max_points=limit)
    assert len(points) == limit
    np.testing.assert_allclose(points[0], [0, 0, 0])


def moving_view(processor, start, *, correction=0., max_points=256):
    """Show successive windows of the same curved, timestamped recorded route."""
    x = np.r_[start, np.arange(.1, 20., .1)[np.arange(.1, 20., .1) > start]]
    xyz = np.column_stack((x, .2 * np.sin(x) + correction, .02 * x))
    return processor.process(
        np.rint((10. + x) * 1e9).astype(np.int64), xyz,
        np.tile([0., 0., 0., 1.], (len(x), 1)),
        radius_m=5., step_m=.2, max_points=max_points,
        smoothing_window_m=.8, max_deviation_m=.2,
    )


def test_landmarks_do_not_slide_when_the_current_pose_and_radius_boundary_move():
    """Overlapping map landmarks stay exact while new stations enter at the front."""
    processor = StablePathProcessor()
    previous = moving_view(processor, 0.)
    for start in (.07, .13, .41, .91, 1.23):
        current = moving_view(processor, start)
        old = dict(zip(previous[0][1:-1], previous[1][1:-1]))
        shared = [i for i, stamp in enumerate(current[0]) if stamp in old]
        assert len(shared) >= 15
        for i in shared:
            np.testing.assert_array_equal(current[1][i], old[current[0][i]])
        assert current[1][0, 0] == start
        assert np.all(np.linalg.norm(current[1][:, :2] - current[1][0, :2], axis=1) <= 5. + 1e-9)
        assert np.all(np.diff(current[0]) > 0)
        assert current[1][-1, 0] > previous[1][-1, 0]
        previous = current


def test_landmarks_follow_real_route_corrections_and_reset_on_rewind():
    """Holding landmarks cannot conceal map corrections beyond the smoothing bound."""
    processor = StablePathProcessor()
    moving_view(processor, 0.)
    corrected = moving_view(processor, .1, correction=1.)
    assert corrected[1][:, 1].min() > .7
    rewound = moving_view(processor, 0.)
    expected = moving_view(StablePathProcessor(), 0.)
    for actual, fresh in zip(rewound, expected):
        np.testing.assert_array_equal(actual, fresh)


def test_retained_landmarks_respect_the_output_budget():
    """Retaining points plus both moving boundaries must still fit the message cap."""
    processor = StablePathProcessor()
    for start in (0., .1, .3, 1., 2., 3.):
        times, xyz, _ = moving_view(processor, start, max_points=8)
        assert len(times) <= 8
        assert np.all(np.diff(times) > 0)
        assert xyz[-1, 0] - xyz[0, 0] > 4.


def test_retained_positions_do_not_freeze_updated_source_attitude():
    """A map landmark can stay fixed while the source corrects its orientation."""
    processor = StablePathProcessor()
    stamps = np.arange(10, dtype=np.int64) * 10 ** 9
    positions = np.column_stack((np.arange(10), np.zeros((10, 2))))
    first = processor.process(
        stamps, positions, np.tile([0., 0., 0., 1.], (10, 1)), radius_m=5., step_m=.2,
    )
    second = processor.process(
        stamps, positions, np.tile([0., 0., 1., 0.], (10, 1)), radius_m=5., step_m=.2,
    )
    np.testing.assert_array_equal(first[1], second[1])
    np.testing.assert_allclose(second[2], np.tile([0., 0., 1., 0.], (len(second[0]), 1)))


def test_short_input_gap_does_not_redistribute_retained_landmarks():
    """A transient empty path must not reset the landmarks behind a held display."""
    processor = StablePathProcessor()
    before = moving_view(processor, 0.)
    processor.process([], [], [], radius_m=5., step_m=.2)
    after = moving_view(processor, .07)
    old = dict(zip(before[0][1:-1], before[1][1:-1]))
    shared = [i for i, stamp in enumerate(after[0]) if stamp in old]
    assert len(shared) >= 15
    for i in shared:
        np.testing.assert_array_equal(after[1][i], old[after[0][i]])
