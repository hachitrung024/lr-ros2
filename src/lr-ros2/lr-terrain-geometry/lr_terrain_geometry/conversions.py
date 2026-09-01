"""ROS message conversions for the terrain geometry node."""

from __future__ import annotations

import math
import struct
from typing import Iterable

import numpy as np
from geometry_msgs.msg import Point
from grid_map_msgs.msg import GridMap
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import ColorRGBA, Float32MultiArray, MultiArrayDimension
from visualization_msgs.msg import Marker, MarkerArray

from .estimator import TerrainGeometryConfig, TerrainResult


GRID_MAP_LAYERS = (
    "elevation",
    "slope_deg",
    "normal_x",
    "normal_y",
    "normal_z",
    "rmse_m",
    "inlier_ratio",
    "state",
    "color",
)


def point_cloud_to_xyz(message: PointCloud2) -> np.ndarray:
    """Decode XYZ fields without importing the ZED SDK."""
    field_names = {field.name for field in message.fields}
    missing = {"x", "y", "z"} - field_names
    if missing:
        names = ", ".join(sorted(missing))
        raise ValueError(f"PointCloud2 is missing required field(s): {names}")

    points = point_cloud2.read_points_numpy(
        message,
        field_names=["x", "y", "z"],
        skip_nans=False,
        reshape_organized_cloud=False,
    )
    values = np.asarray(points, dtype=np.float64)
    if values.size == 0:
        return np.empty((0, 3), dtype=np.float64)
    return values.reshape(-1, 3)


def transform_to_matrix(transform) -> np.ndarray:
    """Convert geometry_msgs/Transform into a homogeneous matrix."""
    translation = transform.translation
    rotation = transform.rotation
    quaternion = np.asarray(
        [rotation.x, rotation.y, rotation.z, rotation.w],
        dtype=np.float64,
    )
    norm = float(np.linalg.norm(quaternion))
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError("TF quaternion must be finite and non-zero")
    x, y, z, w = quaternion / norm
    matrix = np.identity(4, dtype=np.float64)
    matrix[:3, :3] = np.asarray(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )
    matrix[:3, 3] = [translation.x, translation.y, translation.z]
    if not np.isfinite(matrix).all():
        raise ValueError("TF contains non-finite values")
    return matrix


def slope_to_rgb(
    slope_deg: float,
    max_slope_deg: float = 55.0,
) -> tuple[float, float, float]:
    """Match the green-yellow-red anchors used by the legacy debug UI."""
    normalized = float(np.clip(
        float(slope_deg) / max(float(max_slope_deg), 1e-6),
        0.0,
        1.0,
    ))
    green = np.asarray([0.0, 0.85, 0.10], dtype=np.float64)
    yellow = np.asarray([1.0, 0.90, 0.0], dtype=np.float64)
    red = np.asarray([1.0, 0.05, 0.0], dtype=np.float64)
    if normalized <= 0.5:
        color = green + (yellow - green) * (normalized * 2.0)
    else:
        color = yellow + (red - yellow) * ((normalized - 0.5) * 2.0)
    return tuple(float(value) for value in color)


def pack_rgb_float(rgb: Iterable[float]) -> float:
    """Pack RGB bytes into the float representation expected by grid_map."""
    channels = np.clip(
        np.rint(np.asarray(tuple(rgb), dtype=np.float64) * 255.0),
        0.0,
        255.0,
    ).astype(np.uint32)
    if channels.shape != (3,):
        raise ValueError("rgb must contain exactly three channels")
    packed = int((channels[0] << 16) | (channels[1] << 8) | channels[2])
    return struct.unpack("<f", struct.pack("<I", packed))[0]


def unpack_rgb_float(value: float) -> tuple[int, int, int]:
    """Inverse helper used by tests and downstream debugging."""
    packed = struct.unpack("<I", struct.pack("<f", float(value)))[0]
    return ((packed >> 16) & 0xFF, (packed >> 8) & 0xFF, packed & 0xFF)


def _multi_array(values: np.ndarray) -> Float32MultiArray:
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
    message.layout.data_offset = 0
    message.data = np.asarray(
        values,
        dtype=np.float32,
        order="F",
    ).reshape(-1, order="F").tolist()
    return message


def terrain_result_to_grid_map(
    result: TerrainResult,
    rover_position: np.ndarray,
    config: TerrainGeometryConfig,
    stamp,
    frame_id: str,
) -> GridMap:
    """Serialize a snap-aligned rolling map using grid_map storage order."""
    resolution = float(config.cell_size_m)
    rover = np.asarray(rover_position, dtype=np.float64)
    if rover.shape != (3,) or not np.isfinite(rover).all():
        raise ValueError("rover_position must be a finite XYZ vector")

    half_cells = int(math.ceil(config.radius_m / resolution))
    center_key = np.floor(rover[:2] / resolution).astype(np.int64)
    max_key = center_key + half_cells
    size = 2 * half_cells + 1
    layers = {
        name: np.full((size, size), np.nan, dtype=np.float32)
        for name in GRID_MAP_LAYERS
    }

    def indices(key) -> tuple[int, int] | None:
        key_array = np.asarray(key, dtype=np.int64)
        row = int(max_key[0] - key_array[0])
        column = int(max_key[1] - key_array[1])
        if 0 <= row < size and 0 <= column < size:
            return row, column
        return None

    for cell in result.debug_cells:
        location = indices(cell["column_key"])
        if location is not None:
            layers["state"][location] = (
                -1.0 if cell["status"] == "rejected" else 0.0
            )

    for plane in result.rejected_planes:
        location = indices(plane["column_key"])
        if location is None:
            continue
        layers["state"][location] = -1.0
        layers["slope_deg"][location] = float(plane["slope_deg"])
        layers["rmse_m"][location] = float(plane["rmse"])
        layers["inlier_ratio"][location] = float(plane["inlier_ratio"])

    # Accepted state deliberately wins over a failed candidate refit. The
    # cached plane remains authoritative until rejection_bad_fits is reached.
    for plane in result.accepted_planes:
        location = indices(plane["column_key"])
        if location is None:
            continue
        elevation = float(np.asarray(plane["grid_center_world"])[2])
        slope = float(plane["slope_deg"])
        layers["elevation"][location] = elevation
        layers["slope_deg"][location] = slope
        normal = np.asarray(plane["normal_world"], dtype=np.float32)
        layers["normal_x"][location] = float(normal[0])
        layers["normal_y"][location] = float(normal[1])
        layers["normal_z"][location] = float(normal[2])
        layers["rmse_m"][location] = float(plane["rmse"])
        layers["inlier_ratio"][location] = float(plane["inlier_ratio"])
        layers["state"][location] = 1.0
        layers["color"][location] = pack_rgb_float(
            slope_to_rgb(slope, config.max_plane_slope_deg)
        )

    message = GridMap()
    message.header.stamp = stamp
    message.header.frame_id = frame_id
    message.info.resolution = resolution
    message.info.length_x = size * resolution
    message.info.length_y = size * resolution
    message.info.pose.position.x = (center_key[0] + 0.5) * resolution
    message.info.pose.position.y = (center_key[1] + 0.5) * resolution
    message.info.pose.position.z = 0.0
    message.info.pose.orientation.w = 1.0
    message.layers = list(GRID_MAP_LAYERS)
    message.basic_layers = ["elevation"]
    message.data = [_multi_array(layers[name]) for name in message.layers]
    message.outer_start_index = 0
    message.inner_start_index = 0
    return message


def grid_map_to_heatmap_image(
    grid_map: GridMap,
    pixels_per_cell: int = 24,
) -> Image:
    """Render terrain states and slope colors as a top-down RGB image."""
    if (
        isinstance(pixels_per_cell, bool)
        or not isinstance(pixels_per_cell, int)
        or not 1 <= pixels_per_cell <= 64
    ):
        raise ValueError("pixels_per_cell must be an integer from 1 to 64")

    try:
        state_message = grid_map.data[grid_map.layers.index("state")]
        color_message = grid_map.data[grid_map.layers.index("color")]
    except (IndexError, ValueError) as error:
        raise ValueError("GridMap must contain state and color layers") from error

    if len(state_message.layout.dim) < 2:
        raise ValueError("GridMap layers must have two layout dimensions")
    columns = int(state_message.layout.dim[0].size)
    rows = int(state_message.layout.dim[1].size)
    if rows <= 0 or columns <= 0:
        raise ValueError("GridMap layer dimensions must be positive")
    expected_size = rows * columns
    if (
        len(state_message.data) != expected_size
        or len(color_message.data) != expected_size
    ):
        raise ValueError("GridMap state and color layer sizes are inconsistent")

    state = np.asarray(state_message.data, dtype=np.float32).reshape(
        (rows, columns),
        order="F",
    )
    packed_color = np.asarray(color_message.data, dtype=np.float32).reshape(
        (rows, columns),
        order="F",
    )
    cell_colors = np.full((rows, columns, 3), 28, dtype=np.uint8)
    cell_colors[state == 0.0] = (115, 115, 115)
    cell_colors[state == -1.0] = (230, 45, 20)
    for row, column in np.argwhere(
        (state == 1.0) & np.isfinite(packed_color)
    ):
        cell_colors[row, column] = unpack_rgb_float(
            packed_color[row, column]
        )

    image_data = np.repeat(
        np.repeat(cell_colors, pixels_per_cell, axis=0),
        pixels_per_cell,
        axis=1,
    )
    if pixels_per_cell > 1:
        image_data[::pixels_per_cell, :, :] = 50
        image_data[:, ::pixels_per_cell, :] = 50

    message = Image()
    message.header = grid_map.header
    message.height = rows * pixels_per_cell
    message.width = columns * pixels_per_cell
    message.encoding = "rgb8"
    message.is_bigendian = 0
    message.step = message.width * 3
    message.data = image_data.tobytes()
    return message


def _point(values) -> Point:
    point = Point()
    point.x, point.y, point.z = (float(value) for value in values)
    return point


def _line_marker(header, namespace: str, marker_id: int, width: float) -> Marker:
    marker = Marker()
    marker.header = header
    marker.ns = namespace
    marker.id = marker_id
    marker.type = Marker.LINE_LIST
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = width
    return marker


def _add_footprint(marker: Marker, footprint, rgb) -> None:
    corners = np.asarray(footprint, dtype=np.float64)
    if corners.shape != (4, 3) or not np.isfinite(corners).all():
        return
    for index in range(4):
        marker.points.append(_point(corners[index]))
        marker.points.append(_point(corners[(index + 1) % 4]))
        for _ in range(2):
            color = ColorRGBA()
            color.r, color.g, color.b = (float(value) for value in rgb)
            color.a = 1.0
            marker.colors.append(color)


def terrain_result_to_markers(
    result: TerrainResult,
    rover_position: np.ndarray,
    config: TerrainGeometryConfig,
    stamp,
    frame_id: str,
) -> MarkerArray:
    """Create a full, deterministic debug snapshot for RViz."""
    header_type = Marker().header
    header_type.stamp = stamp
    header_type.frame_id = frame_id

    delete_all = Marker()
    delete_all.header = header_type
    delete_all.action = Marker.DELETEALL
    markers = [delete_all]

    accepted = _line_marker(header_type, "accepted_outlines", 0, 0.035)
    normals = _line_marker(header_type, "accepted_normals", 0, 0.025)
    rejected = _line_marker(header_type, "rejected_fits", 0, 0.045)
    pending = _line_marker(header_type, "pending_cells", 0, 0.025)
    rejected_cells = _line_marker(header_type, "rejected_cells", 0, 0.035)

    for plane in sorted(result.accepted_planes, key=lambda item: item["column_key"]):
        color = slope_to_rgb(plane["slope_deg"], config.max_plane_slope_deg)
        _add_footprint(accepted, plane["footprint_world"], color)
        center = np.asarray(plane["grid_center_world"], dtype=np.float64)
        normal = np.asarray(plane["normal_world"], dtype=np.float64)
        normals.points.extend((_point(center), _point(center + normal * 0.45)))

    for plane in sorted(result.rejected_planes, key=lambda item: item["column_key"]):
        _add_footprint(rejected, plane["footprint_world"], (1.0, 0.1, 0.1))

    for cell in sorted(result.debug_cells, key=lambda item: item["column_key"]):
        if cell["status"] == "rejected":
            _add_footprint(rejected_cells, cell["footprint_world"], (1.0, 0.25, 0.0))
        else:
            _add_footprint(pending, cell["footprint_world"], (0.55, 0.55, 0.55))

    for marker in (accepted, normals, rejected, pending, rejected_cells):
        if marker.points:
            if not marker.colors:
                marker.color.r = 0.1
                marker.color.g = 0.9
                marker.color.b = 0.9
                marker.color.a = 1.0
            markers.append(marker)

    label_id = 0
    all_planes = list(result.accepted_planes) + list(result.rejected_planes)
    planes_for_labels = sorted(
        all_planes,
        key=lambda item: (
            item["column_key"],
            item["rejection_reason"] or "",
        ),
    )
    for plane in planes_for_labels:
        label = Marker()
        label.header = header_type
        label.ns = "metric_labels"
        label.id = label_id
        label_id += 1
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position = _point(
            np.asarray(plane["grid_center_world"], dtype=np.float64)
            + np.asarray(plane["normal_world"], dtype=np.float64) * 0.15
        )
        label.pose.orientation.w = 1.0
        label.scale.z = 0.16
        rejected_reason = plane["rejection_reason"]
        prefix = f"REJECTED {rejected_reason}\n" if rejected_reason else ""
        label.text = (
            f"{prefix}{tuple(plane['column_key'])} "
            f"slope={float(plane['slope_deg']):.1f}deg "
            f"inliers={int(plane['inlier_count'])}/{int(plane['sample_count'])} "
            f"rmse={float(plane['rmse']) * 100.0:.1f}cm"
        )
        if rejected_reason:
            label.color.r, label.color.g, label.color.b = 1.0, 0.2, 0.2
        else:
            label.color.r, label.color.g, label.color.b = 0.9, 0.9, 0.9
        label.color.a = 1.0
        markers.append(label)

    summary = Marker()
    summary.header = header_type
    summary.ns = "summary"
    summary.id = 0
    summary.type = Marker.TEXT_VIEW_FACING
    summary.action = Marker.ADD
    summary.pose.position = _point(
        np.asarray(rover_position, dtype=np.float64) + [0.0, 0.0, 1.5]
    )
    summary.pose.orientation.w = 1.0
    summary.scale.z = 0.22
    summary.color.r = summary.color.g = summary.color.b = 1.0
    summary.color.a = 1.0
    summary.text = (
        f"terrain cells={result.cell_count} points={result.point_count}\n"
        f"accepted={len(result.accepted_planes)} "
        f"rejected={len(result.rejected_planes)} "
        f"pending={len(result.debug_cells)}"
    )
    markers.append(summary)
    return MarkerArray(markers=markers)
