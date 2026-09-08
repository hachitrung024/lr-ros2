"""Tests for trajectory lookup, sampling, and reanchoring."""

import math

import numpy as np
import pytest

from lr_future_path.trajectory import PoseSeries, reanchor_pose


def _yaw_quaternion(yaw):
    return np.array([0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)])


def _series(stamps, positions):
    count = len(stamps)
    return PoseSeries(
        stamps_ns=np.asarray(stamps, dtype=np.int64),
        positions=np.asarray(positions, dtype=np.float64),
        orientations_xyzw=np.tile(
            np.array([0.0, 0.0, 0.0, 1.0]), (count, 1)
        ),
        frame_id="map",
    )


def test_nearest_index_honors_tolerance_and_skipped_frames():
    """Timestamp lookup handles skipped frames and deterministic ties."""
    series = _series(
        [1_000, 2_000, 4_000],
        [[0, 0, 0], [1, 0, 0], [2, 0, 0]],
    )

    assert series.nearest_index(3_900, 101) == 2
    assert series.nearest_index(3_000, 999) is None
    assert series.nearest_index(3_000, 1_000) == 1


def test_future_sampling_stops_at_along_path_distance_budget():
    """A route that returns toward its origin cannot exceed the budget."""
    series = _series(
        [0, 1, 2, 3, 4, 5],
        [
            [0.0, 0.0, 0.0],
            [0.1, 0.0, 5.0],
            [0.4, 0.0, 5.0],
            [1.0, 0.0, 5.0],
            [2.1, 0.0, 5.0],
            [1.5, 0.0, 5.0],
        ],
    )

    indices = series.future_indices(
        0,
        radius_m=2.0,
        step_m=0.3,
        max_gap_ns=10,
        max_points=100,
    )

    assert indices == [0, 2, 3]


def test_future_sampling_honors_time_horizon_and_ignores_small_jitter():
    series = _series(
        [0, 1, 2, 3, 20],
        [
            [0.0, 0.0, 0.0],
            [0.1, 0.0, 0.0],
            [-0.1, 0.0, 0.0],
            [0.1, 0.0, 0.0],
            [1.0, 0.0, 0.0],
        ],
    )

    indices = series.future_indices(
        0,
        radius_m=5.0,
        step_m=0.2,
        max_gap_ns=30,
        max_points=100,
        max_horizon_ns=10,
    )

    assert indices == [0]


def test_future_sampling_stops_at_timestamp_gap():
    """A tracking timestamp gap terminates the future segment."""
    series = _series(
        [0, 100, 200, 2_000],
        [[0, 0, 0], [0.2, 0, 0], [0.4, 0, 0], [0.6, 0, 0]],
    )

    indices = series.future_indices(
        0,
        radius_m=5.0,
        step_m=0.1,
        max_gap_ns=500,
        max_points=100,
    )

    assert indices == [0, 1, 2]


def test_future_sampling_enforces_point_limit():
    """Path sampling cannot exceed the configured message point limit."""
    series = _series(
        list(range(20)),
        [[float(index), 0, 0] for index in range(20)],
    )

    indices = series.future_indices(
        0,
        radius_m=50.0,
        step_m=0.1,
        max_gap_ns=2,
        max_points=4,
    )

    assert indices == [0, 1, 2, 3]


def test_reanchor_applies_cached_relative_transform_to_runtime_pose():
    """Cached motion is expressed relative to the runtime map pose."""
    output_position, output_orientation = reanchor_pose(
        cached_current_position=np.array([1.0, 2.0, 0.0]),
        cached_current_orientation=_yaw_quaternion(0.0),
        cached_future_position=np.array([2.0, 2.0, 0.0]),
        cached_future_orientation=_yaw_quaternion(math.pi / 2),
        runtime_position=np.array([10.0, 20.0, 1.0]),
        runtime_orientation=_yaw_quaternion(math.pi / 2),
    )

    assert output_position == pytest.approx([10.0, 21.0, 1.0])
    assert output_orientation == pytest.approx(
        _yaw_quaternion(math.pi), abs=1e-7
    )
