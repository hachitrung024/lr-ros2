"""Shared terrain sampler coverage migrated from the legacy prediction UI."""

import math

import numpy as np
import pytest
from grid_map_msgs.msg import GridMap
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

from lr_terrain_geometry.grid_map_sampling import GridMapSampler


def _layer(values):
    values = np.asarray(values, dtype=np.float32)
    rows, columns = values.shape
    message = Float32MultiArray()
    message.layout.dim = [
        MultiArrayDimension(
            label="column_index",
            size=columns,
            stride=rows * columns,
        ),
        MultiArrayDimension(
            label="row_index",
            size=rows,
            stride=rows,
        ),
    ]
    message.data = values.reshape(-1, order="F").tolist()
    return message


def _terrain_grid(slope=25.0):
    message = GridMap()
    message.header.frame_id = "map"
    message.info.resolution = 1.0
    message.info.length_x = 3.0
    message.info.length_y = 3.0
    message.info.pose.orientation.w = 1.0
    message.layers = [
        "elevation",
        "slope_deg",
        "normal_x",
        "normal_y",
        "normal_z",
        "state",
    ]
    center_only = np.full((3, 3), np.nan, dtype=np.float32)
    elevation = center_only.copy()
    elevation[1, 1] = 0.4
    slopes = center_only.copy()
    slopes[1, 1] = slope
    normal_x = center_only.copy()
    normal_x[1, 1] = 0.0
    normal_y = center_only.copy()
    normal_y[1, 1] = math.sin(math.radians(slope))
    normal_z = center_only.copy()
    normal_z[1, 1] = math.cos(math.radians(slope))
    state = center_only.copy()
    state[1, 1] = 1.0
    message.data = [
        _layer(values)
        for values in (
            elevation,
            slopes,
            normal_x,
            normal_y,
            normal_z,
            state,
        )
    ]
    return message


def test_grid_map_sampler_reads_slope_elevation_and_normal():
    sampler = GridMapSampler(_terrain_grid(slope=25.0))

    sample = sampler.sample(0.0, 0.0)

    assert sample.valid
    assert sample.elevation_m == np.float32(0.4)
    assert sample.slope_deg == 25.0
    np.testing.assert_allclose(
        sample.normal_xyz,
        [0.0, math.sin(math.radians(25.0)), math.cos(math.radians(25.0))],
        atol=1e-6,
    )
    assert not sampler.sample(5.0, 0.0).valid


def test_sampler_honors_invalid_state_and_rotated_circular_buffer():
    grid = _terrain_grid()
    grid.info.pose.orientation.z = math.sin(math.pi / 4)
    grid.info.pose.orientation.w = math.cos(math.pi / 4)
    grid.outer_start_index = 1
    # A +1 logical row offset addresses the one accepted stored cell.
    assert GridMapSampler(grid).sample(0.0, 1.0).valid
    grid.data[-1].data[4] = 0.0
    assert not GridMapSampler(grid).sample(0.0, 1.0).valid


def test_sampler_rejects_incomplete_layers_and_honors_data_offset():
    grid = _terrain_grid()
    for layer in grid.data:
        layer.layout.data_offset = 1
        layer.data.insert(0, -123.0)
    assert GridMapSampler(grid).sample(0.0, 0.0).valid
    grid.data[0].data.pop()
    with pytest.raises(ValueError, match='incomplete'):
        GridMapSampler(grid)
