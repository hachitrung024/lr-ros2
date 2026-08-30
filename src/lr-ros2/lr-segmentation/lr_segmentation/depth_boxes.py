"""Estimate simple axis-aligned 3D boxes from 2D boxes and registered depth."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from geometry_msgs.msg import Point
from sensor_msgs.msg import CameraInfo, Image
from visualization_msgs.msg import Marker, MarkerArray

from .model import Box2D


@dataclass(frozen=True)
class Box3D:
    """Axis-aligned 3D box in the depth optical frame."""

    center: np.ndarray
    size: np.ndarray
    class_id: int
    class_name: str
    confidence: float


def depth_message_to_meters(message: Image) -> np.ndarray:
    """Decode a 32FC1 or millimeter uint16 depth image into meters."""
    encoding = message.encoding.lower()
    if encoding == "32fc1":
        dtype = np.dtype(">f4" if message.is_bigendian else "<f4")
        scale = 1.0
    elif encoding in {"16uc1", "mono16"}:
        dtype = np.dtype(">u2" if message.is_bigendian else "<u2")
        scale = 0.001
    else:
        raise ValueError(
            f"Unsupported depth encoding '{message.encoding}'; "
            "expected 32FC1, 16UC1, or mono16"
        )

    height = int(message.height)
    width = int(message.width)
    if height <= 0 or width <= 0:
        raise ValueError("Depth dimensions must be positive")
    packed_width = width * dtype.itemsize
    if int(message.step) < packed_width:
        raise ValueError("Depth step is smaller than its packed row size")
    required = int(message.step) * height
    if len(message.data) < required:
        raise ValueError("Depth data is shorter than step * height")

    byte_rows = np.frombuffer(
        message.data,
        dtype=np.uint8,
        count=required,
    ).reshape(height, int(message.step))
    packed = np.ascontiguousarray(byte_rows[:, :packed_width])
    depth = packed.view(dtype).reshape(height, width).astype(np.float32)
    return depth * scale


def _camera_intrinsics(
    camera_info: CameraInfo,
    depth_width: int,
    depth_height: int,
) -> tuple[float, float, float, float]:
    source_width = int(camera_info.width)
    source_height = int(camera_info.height)
    if source_width <= 0 or source_height <= 0:
        raise ValueError("CameraInfo dimensions must be positive")
    fx = float(camera_info.k[0])
    fy = float(camera_info.k[4])
    cx = float(camera_info.k[2])
    cy = float(camera_info.k[5])
    if fx <= 0.0 or fy <= 0.0:
        raise ValueError("CameraInfo focal lengths must be positive")
    scale_x = depth_width / source_width
    scale_y = depth_height / source_height
    return (
        fx * scale_x,
        fy * scale_y,
        cx * scale_x,
        cy * scale_y,
    )


def estimate_boxes_3d(
    detections: tuple[Box2D, ...],
    depth_m: np.ndarray,
    camera_info: CameraInfo,
    image_width: int,
    image_height: int,
    *,
    minimum_depth_m: float,
    maximum_depth_m: float,
    depth_tolerance_m: float,
    depth_tolerance_ratio: float,
    minimum_points: int,
) -> tuple[Box3D, ...]:
    """Back-project the robust center-depth span inside each 2D box."""
    depth = np.asarray(depth_m)
    if depth.ndim != 2:
        raise ValueError("depth_m must be two-dimensional")
    if image_width <= 0 or image_height <= 0:
        raise ValueError("Source image dimensions must be positive")

    depth_height, depth_width = depth.shape
    fx, fy, cx, cy = _camera_intrinsics(
        camera_info,
        depth_width,
        depth_height,
    )
    scale_x = depth_width / image_width
    scale_y = depth_height / image_height
    boxes = []

    for detection in detections:
        x1, y1, x2, y2 = detection.xyxy
        left = max(0, min(depth_width, int(np.floor(x1 * scale_x))))
        top = max(0, min(depth_height, int(np.floor(y1 * scale_y))))
        right = max(0, min(depth_width, int(np.ceil(x2 * scale_x))))
        bottom = max(0, min(depth_height, int(np.ceil(y2 * scale_y))))
        if right - left < 2 or bottom - top < 2:
            continue

        roi = depth[top:bottom, left:right]
        valid = (
            np.isfinite(roi)
            & (roi >= minimum_depth_m)
            & (roi <= maximum_depth_m)
        )
        center_left = (right - left) // 4
        center_top = (bottom - top) // 4
        center_right = max(center_left + 1, 3 * (right - left) // 4)
        center_bottom = max(center_top + 1, 3 * (bottom - top) // 4)
        center_depth = roi[center_top:center_bottom, center_left:center_right]
        center_valid = valid[
            center_top:center_bottom,
            center_left:center_right,
        ]
        seed_values = center_depth[center_valid]
        if seed_values.size == 0:
            seed_values = roi[valid]
        if seed_values.size == 0:
            continue

        seed_depth = float(np.median(seed_values))
        tolerance = max(
            depth_tolerance_m,
            seed_depth * depth_tolerance_ratio,
        )
        # A single band around the median only captures a thin slice of
        # objects that extend toward the camera (for example, a vehicle with
        # loading ramps). Use the robust center-region depth span instead,
        # then expand it slightly to recover the object's near and far edges.
        near_depth, far_depth = np.percentile(seed_values, (1.0, 99.0))
        selected = (
            valid
            & (roi >= near_depth - tolerance)
            & (roi <= far_depth + tolerance)
        )
        rows, columns = np.nonzero(selected)
        if rows.size < minimum_points:
            continue

        z = roi[rows, columns].astype(np.float64)
        u = columns.astype(np.float64) + left
        v = rows.astype(np.float64) + top
        points = np.column_stack((
            (u - cx) * z / fx,
            (v - cy) * z / fy,
            z,
        ))
        lower, upper = np.percentile(points, (5.0, 95.0), axis=0)
        if not np.isfinite(lower).all() or not np.isfinite(upper).all():
            continue
        size = np.maximum(upper - lower, 0.03)
        boxes.append(
            Box3D(
                center=(lower + upper) * 0.5,
                size=size,
                class_id=detection.class_id,
                class_name=detection.class_name,
                confidence=detection.confidence,
            )
        )

    return tuple(boxes)


def _marker_color(class_id: int) -> tuple[float, float, float]:
    palette = (
        (0.10, 0.90, 1.00),
        (1.00, 0.45, 0.10),
        (0.35, 1.00, 0.25),
        (1.00, 0.20, 0.75),
        (0.75, 0.35, 1.00),
        (1.00, 0.90, 0.10),
    )
    return palette[int(class_id) % len(palette)]


def boxes_to_markers(boxes: tuple[Box3D, ...], header) -> MarkerArray:
    """Create RViz wireframe markers for 3D boxes."""
    delete_all = Marker()
    delete_all.header = header
    delete_all.action = Marker.DELETEALL
    markers = [delete_all]
    edges = (
        (0, 1), (1, 2), (2, 3), (3, 0),
        (4, 5), (5, 6), (6, 7), (7, 4),
        (0, 4), (1, 5), (2, 6), (3, 7),
    )

    for marker_id, box in enumerate(boxes):
        half = box.size * 0.5
        minimum = box.center - half
        maximum = box.center + half
        corners = np.asarray((
            (minimum[0], minimum[1], minimum[2]),
            (maximum[0], minimum[1], minimum[2]),
            (maximum[0], maximum[1], minimum[2]),
            (minimum[0], maximum[1], minimum[2]),
            (minimum[0], minimum[1], maximum[2]),
            (maximum[0], minimum[1], maximum[2]),
            (maximum[0], maximum[1], maximum[2]),
            (minimum[0], maximum[1], maximum[2]),
        ))

        marker = Marker()
        marker.header = header
        marker.ns = "boxes_3d"
        marker.id = marker_id
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.035
        marker.color.r, marker.color.g, marker.color.b = _marker_color(
            box.class_id
        )
        marker.color.a = 1.0
        marker.lifetime.sec = 1
        marker.text = f"{box.class_name} {box.confidence:.0%}"
        for first, second in edges:
            marker.points.extend((
                Point(
                    x=float(corners[first, 0]),
                    y=float(corners[first, 1]),
                    z=float(corners[first, 2]),
                ),
                Point(
                    x=float(corners[second, 0]),
                    y=float(corners[second, 1]),
                    z=float(corners[second, 2]),
                ),
            ))
        markers.append(marker)

    return MarkerArray(markers=markers)
