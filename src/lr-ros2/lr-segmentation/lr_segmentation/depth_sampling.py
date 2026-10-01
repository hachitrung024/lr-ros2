"""Decode registered depth and back-project labeled instance pixels."""

from __future__ import annotations

import numpy as np
from sensor_msgs.msg import CameraInfo, Image


def instance_mask_message_to_labels(message: Image) -> np.ndarray:
    """Decode a mono16 instance-label image while respecting row padding."""
    if message.encoding.lower() not in {"mono16", "16uc1"}:
        raise ValueError("instance mask encoding must be mono16 or 16UC1")
    height = int(message.height)
    width = int(message.width)
    if height <= 0 or width <= 0:
        raise ValueError("instance mask dimensions must be positive")
    packed_width = width * 2
    if int(message.step) < packed_width:
        raise ValueError("instance mask step is smaller than its packed row")
    required = int(message.step) * height
    if len(message.data) < required:
        raise ValueError("instance mask data is shorter than step * height")

    rows = np.frombuffer(
        message.data,
        dtype=np.uint8,
        count=required,
    ).reshape(height, int(message.step))
    packed = np.ascontiguousarray(rows[:, :packed_width])
    byte_order = ">u2" if bool(message.is_bigendian) else "<u2"
    return (
        packed.view(byte_order)
        .reshape(height, width)
        .astype(
            np.uint16,
            copy=False,
        )
    )


def depth_message_to_meters(message: Image) -> np.ndarray:
    """Decode registered float-metre or uint16-millimetre depth."""
    encoding = message.encoding.lower()
    if encoding == "32fc1":
        dtype = np.dtype(">f4" if message.is_bigendian else "<f4")
        scale = 1.0
    elif encoding in {"16uc1", "mono16"}:
        dtype = np.dtype(">u2" if message.is_bigendian else "<u2")
        scale = 0.001
    else:
        raise ValueError(f"unsupported depth encoding '{message.encoding}'")

    height = int(message.height)
    width = int(message.width)
    if height <= 0 or width <= 0:
        raise ValueError("depth dimensions must be positive")
    packed_width = width * dtype.itemsize
    if int(message.step) < packed_width:
        raise ValueError("depth step is smaller than its packed row")
    required = int(message.step) * height
    if len(message.data) < required:
        raise ValueError("depth data is shorter than step * height")

    rows = np.frombuffer(
        message.data,
        dtype=np.uint8,
        count=required,
    ).reshape(height, int(message.step))
    packed = np.ascontiguousarray(rows[:, :packed_width])
    depth = packed.view(dtype).reshape(height, width)
    return np.asarray(depth, dtype=np.float32) * scale


def instance_point_clouds(
    labels: np.ndarray,
    depth_m: np.ndarray,
    camera_info: CameraInfo,
    *,
    minimum_depth_m: float,
    maximum_depth_m: float,
    sampling_stride: int,
    maximum_points: int,
    depth_mad_scale: float,
    minimum_depth_band_m: float,
) -> dict[int, np.ndarray]:
    """Group valid pixels once, retaining empty entries for unobservable instances."""
    fx, fy, cx, cy = _camera_intrinsics(camera_info)
    clouds = {
        int(label): np.empty((0, 3), dtype=np.float64)
        for label in np.unique(labels) if label != 0
    }
    sampled_labels = labels[::sampling_stride, ::sampling_stride]
    sampled_depth = depth_m[::sampling_stride, ::sampling_stride]
    valid = (
        (sampled_labels != 0)
        & np.isfinite(sampled_depth)
        & (sampled_depth >= minimum_depth_m)
        & (sampled_depth <= maximum_depth_m)
    )
    rows, columns = np.nonzero(valid)
    if rows.size == 0:
        return clouds
    ids = sampled_labels[rows, columns]
    order = np.argsort(ids, kind="stable")
    rows, columns, ids = rows[order], columns[order], ids[order]
    boundaries = np.r_[0, np.flatnonzero(ids[1:] != ids[:-1]) + 1, len(ids)]
    for start, end in zip(boundaries[:-1], boundaries[1:]):
        group_rows, group_columns = rows[start:end], columns[start:end]
        depth = sampled_depth[group_rows, group_columns].astype(np.float64)
        keep = _depth_inlier_mask(
            depth, mad_scale=depth_mad_scale, minimum_band_m=minimum_depth_band_m
        )
        group_rows, group_columns, depth = group_rows[keep], group_columns[keep], depth[keep]
        if depth.size > maximum_points:
            selected = np.linspace(0, depth.size - 1, maximum_points, dtype=np.int64)
            group_rows, group_columns = group_rows[selected], group_columns[selected]
            depth = depth[selected]
        pixel_v = group_rows.astype(np.float64) * sampling_stride
        pixel_u = group_columns.astype(np.float64) * sampling_stride
        clouds[int(ids[start])] = np.column_stack(
            ((pixel_u - cx) * depth / fx, (pixel_v - cy) * depth / fy, depth)
        )
    return clouds


def _depth_inlier_mask(
    values: np.ndarray,
    *,
    mad_scale: float,
    minimum_band_m: float,
) -> np.ndarray:
    """Reject depth outliers without discarding legitimate object thickness."""
    if values.size < 4:
        return np.ones(values.size, dtype=bool)
    median = float(np.median(values))
    mad = float(np.median(np.abs(values - median)))
    robust_stddev = 1.4826 * mad
    band = max(minimum_band_m, mad_scale * robust_stddev)
    return np.abs(values - median) <= band


def _camera_intrinsics(camera_info: CameraInfo) -> tuple[float, float, float, float]:
    """Read usable pinhole intrinsics from a registered CameraInfo message."""
    projection = np.asarray(camera_info.p, dtype=np.float64).reshape(3, 4)
    if projection[0, 0] > 0.0 and projection[1, 1] > 0.0:
        values = (
            projection[0, 0],
            projection[1, 1],
            projection[0, 2],
            projection[1, 2],
        )
    else:
        intrinsic = np.asarray(camera_info.k, dtype=np.float64).reshape(3, 3)
        values = (
            intrinsic[0, 0],
            intrinsic[1, 1],
            intrinsic[0, 2],
            intrinsic[1, 2],
        )
    if not np.isfinite(values).all() or values[0] <= 0.0 or values[1] <= 0.0:
        raise ValueError("CameraInfo is uncalibrated or malformed")
    return tuple(float(value) for value in values)
