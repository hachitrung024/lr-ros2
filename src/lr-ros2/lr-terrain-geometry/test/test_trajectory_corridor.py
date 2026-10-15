"""Check conservative trajectory coverage and finite raster lookups."""

import numpy as np
import pytest

from lr_terrain_geometry.trajectory_corridor import TrajectoryCorridor


def test_bends_endpoints_and_sparse_segments():
    """Keep whole segments and rounded endpoints, excluding the inside of a turn."""
    grid = TrajectoryCorridor([[0, 0], [6, 0], [6, 6]], 0.5).grid(
        [3, 3], 10, 0.2, 0,
    )
    points = np.array([[3, 0], [6, 3], [6, 6.3], [-0.3, 0], [3, 3], [0, 6], [100, 0]])
    assert grid.point_mask(points).tolist() == [True, True, True, True, False, False, False]


def test_complete_cells_and_neighbor_halo_are_preserved():
    """Retain a selected cell's corners and the fitter's narrow neighboring strip."""
    grid = TrajectoryCorridor([[0, 0], [5, 0]], 0.5).grid([2, 0], 6, 1, 0.1)
    assert grid.allows_cell((2, 0))
    assert not grid.allows_cell((2, 1))
    points = np.array([[2.95, .95], [2.5, 1.05], [2.5, 1.2], [2.5, -1.05]])
    assert grid.point_mask(points).tolist() == [True, True, False, True]


def test_single_pose_duplicates_negative_world_keys_and_empty_cloud():
    """A stationary path remains useful without dividing by zero."""
    points = [[-3, -2], [-3, -2]]
    grid = TrajectoryCorridor(points, 0.4).grid([-3, -2], 3, .2, 0)
    single = TrajectoryCorridor(points[:1], .4).grid([-3, -2], 3, .2, 0)
    np.testing.assert_array_equal(grid.fit_mask, single.fit_mask)
    assert grid.point_mask(np.array([[-3, -2], [0, 0]])).tolist() == [True, False]
    assert grid.point_mask(np.empty((0, 2))).shape == (0,)


@pytest.mark.parametrize("points", [[], [[1]], [[np.nan, 0]], [[0, np.inf]], [[0, 0]] * 257])
def test_invalid_paths_are_rejected(points):
    """Reject malformed paths instead of silently truncating their coverage."""
    with pytest.raises(ValueError):
        TrajectoryCorridor(points, 1.)


@pytest.mark.parametrize("width", [0, -1, np.nan, np.inf])
def test_invalid_widths_are_rejected(width):
    """A corridor requires finite positive width."""
    with pytest.raises(ValueError):
        TrajectoryCorridor([[0, 0]], width)
