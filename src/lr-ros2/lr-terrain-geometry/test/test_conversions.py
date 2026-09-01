import math

import numpy as np
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Transform
from grid_map_msgs.msg import GridMap
from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from visualization_msgs.msg import Marker

from lr_terrain_geometry.conversions import (
    GRID_MAP_LAYERS,
    grid_map_to_heatmap_image,
    pack_rgb_float,
    point_cloud_to_xyz,
    slope_to_rgb,
    terrain_result_to_grid_map,
    terrain_result_to_markers,
    transform_to_matrix,
    unpack_rgb_float,
)
from lr_terrain_geometry.estimator import (
    TerrainGeometryConfig,
    TerrainResult,
)


def make_plane(key=(0, -1), slope=0.0, rejected=None):
    x0, y0 = key
    center = np.asarray([x0 + 0.5, y0 + 0.5, 0.2], dtype=np.float32)
    footprint = np.asarray(
        [
            [x0, y0, 0.2],
            [x0 + 1.0, y0, 0.2],
            [x0 + 1.0, y0 + 1.0, 0.2],
            [x0, y0 + 1.0, 0.2],
        ],
        dtype=np.float32,
    )
    return {
        "center_world": center,
        "grid_center_world": center,
        "normal_world": np.asarray([0.0, 0.0, 1.0], dtype=np.float32),
        "footprint_world": footprint,
        "column_key": key,
        "slope_deg": float(slope),
        "rmse": 0.01,
        "inlier_ratio": 0.8,
        "inlier_count": 80,
        "sample_count": 100,
        "rejection_reason": rejected,
    }


def test_point_cloud_adapter_reads_xyz_and_keeps_nan_for_core_filtering():
    header = Header(frame_id="camera")
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    cloud = point_cloud2.create_cloud(
        header,
        fields,
        [(1.0, 2.0, 3.0, 0.0), (4.0, math.nan, 6.0, 0.0)],
    )

    points = point_cloud_to_xyz(cloud)

    assert points.shape == (2, 3)
    np.testing.assert_allclose(points[0], [1.0, 2.0, 3.0])
    assert math.isnan(points[1, 1])


def test_point_cloud_adapter_rejects_missing_xyz_field():
    header = Header(frame_id="camera")
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    ]
    cloud = point_cloud2.create_cloud(header, fields, [(1.0, 2.0)])

    try:
        point_cloud_to_xyz(cloud)
    except ValueError as error:
        assert "z" in str(error)
    else:
        raise AssertionError("missing z field was accepted")


def test_transform_to_matrix_normalizes_quaternion():
    transform = Transform()
    transform.translation.x = 1.0
    transform.translation.y = 2.0
    transform.translation.z = 3.0
    transform.rotation.z = math.sqrt(2.0)
    transform.rotation.w = math.sqrt(2.0)

    matrix = transform_to_matrix(transform)

    np.testing.assert_allclose(matrix[:3, 3], [1.0, 2.0, 3.0])
    np.testing.assert_allclose(
        matrix[:3, :3],
        [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]],
        atol=1e-7,
    )


def test_slope_color_anchors_and_grid_map_packing_round_trip():
    assert slope_to_rgb(0.0) == (0.0, 0.85, 0.1)
    assert slope_to_rgb(27.5) == (1.0, 0.9, 0.0)
    np.testing.assert_allclose(slope_to_rgb(55.0), (1.0, 0.05, 0.0))

    packed = pack_rgb_float(slope_to_rgb(27.5))
    assert unpack_rgb_float(packed) == (255, 230, 0)


def test_grid_map_layers_storage_and_accepted_state_precedence():
    config = TerrainGeometryConfig(
        radius_m=1.0,
        cell_size_m=1.0,
        min_forward_m=None,
        max_forward_m=None,
        min_lateral_m=None,
        max_lateral_m=None,
    )
    accepted = make_plane()
    rejected = make_plane(rejected="rmse")
    result = TerrainResult(
        accepted_planes=(accepted,),
        rejected_planes=(rejected,),
        debug_cells=({
            "column_key": (1, -1),
            "status": "unchecked",
            "footprint_world": accepted["footprint_world"] + [1.0, 0.0, 0.0],
        },),
        fit_cycle=True,
        display_changed=True,
        reset_reason=None,
        cell_count=2,
        point_count=100,
    )

    message = terrain_result_to_grid_map(
        result,
        np.asarray([0.1, -0.1, 0.0]),
        config,
        Time(sec=4),
        "map",
    )

    assert tuple(message.layers) == GRID_MAP_LAYERS
    assert message.basic_layers == ["elevation"]
    assert message.info.length_x == 3.0
    assert message.info.pose.position.x == 0.5
    assert message.info.pose.position.y == -0.5
    assert message.outer_start_index == 0
    assert message.inner_start_index == 0
    state_index = message.layers.index("state")
    state = np.asarray(message.data[state_index].data).reshape((3, 3), order="F")
    elevation_index = message.layers.index("elevation")
    elevation = np.asarray(message.data[elevation_index].data).reshape(
        (3, 3), order="F"
    )
    normal_z_index = message.layers.index("normal_z")
    normal_z = np.asarray(message.data[normal_z_index].data).reshape(
        (3, 3), order="F"
    )
    # center key is (0, -1); matrix indices run from positive to negative map
    # coordinates to match grid_map's Eigen storage convention.
    assert state[1, 1] == 1.0
    assert elevation[1, 1] == np.float32(0.2)
    assert normal_z[1, 1] == 1.0
    assert state[0, 1] == 0.0
    assert message.data[0].layout.dim[0].label == "column_index"
    assert message.data[0].layout.dim[1].label == "row_index"


def test_heatmap_image_uses_grid_orientation_state_and_slope_colors():
    config = TerrainGeometryConfig(
        radius_m=1.0,
        cell_size_m=1.0,
        min_forward_m=None,
        max_forward_m=None,
        min_lateral_m=None,
        max_lateral_m=None,
    )
    accepted = make_plane(slope=27.5)
    pending = {
        "column_key": (1, -1),
        "status": "unchecked",
        "footprint_world": accepted["footprint_world"] + [1.0, 0.0, 0.0],
    }
    result = TerrainResult(
        accepted_planes=(accepted,),
        rejected_planes=(),
        debug_cells=(pending,),
        fit_cycle=True,
        display_changed=True,
        reset_reason=None,
        cell_count=2,
        point_count=100,
    )
    grid_map = terrain_result_to_grid_map(
        result,
        np.asarray([0.1, -0.1, 0.0]),
        config,
        Time(sec=4),
        "map",
    )

    message = grid_map_to_heatmap_image(grid_map, pixels_per_cell=2)
    pixels = np.frombuffer(message.data, dtype=np.uint8).reshape(
        message.height,
        message.width,
        3,
    )

    assert message.header.frame_id == "map"
    assert message.encoding == "rgb8"
    assert message.height == 6
    assert message.width == 6
    assert message.step == 18
    # The accepted key (0, -1) is the central cell. Its lower-right pixel is
    # not a grid line and therefore contains the exact yellow slope anchor.
    np.testing.assert_array_equal(pixels[3, 3], [255, 230, 0])
    # Positive map X is at the top of the image, so key (1, -1) is one row up.
    np.testing.assert_array_equal(pixels[1, 3], [115, 115, 115])
    np.testing.assert_array_equal(pixels[5, 5], [28, 28, 28])


def test_heatmap_image_rejects_invalid_scale():
    try:
        grid_map_to_heatmap_image(GridMap(), pixels_per_cell=0)
    except ValueError as error:
        assert "pixels_per_cell" in str(error)
    else:
        raise AssertionError("invalid heatmap scale was accepted")


def test_marker_snapshot_has_delete_and_debug_namespaces():
    accepted = make_plane(slope=20.0)
    rejected = make_plane(key=(1, -1), rejected="coverage")
    debug = {
        "column_key": (2, -1),
        "status": "unchecked",
        "footprint_world": rejected["footprint_world"] + [1.0, 0.0, 0.0],
    }
    result = TerrainResult(
        accepted_planes=(accepted,),
        rejected_planes=(rejected,),
        debug_cells=(debug,),
        fit_cycle=True,
        display_changed=True,
        reset_reason=None,
        cell_count=3,
        point_count=300,
    )

    message = terrain_result_to_markers(
        result,
        np.zeros(3),
        TerrainGeometryConfig(),
        Time(sec=2),
        "map",
    )

    assert message.markers[0].action == Marker.DELETEALL
    namespaces = {marker.ns for marker in message.markers[1:]}
    assert "accepted_outlines" in namespaces
    assert "accepted_normals" in namespaces
    assert "rejected_fits" in namespaces
    assert "pending_cells" in namespaces
    assert "metric_labels" in namespaces
    assert "summary" in namespaces
