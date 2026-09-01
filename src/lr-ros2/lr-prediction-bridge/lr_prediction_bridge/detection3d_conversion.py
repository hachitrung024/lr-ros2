"""Convert vision_msgs/Detection3D boxes into canonical tracked-object fields."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Sequence

Point2 = tuple[float, float]
Point3 = tuple[float, float, float]


@dataclass(frozen=True)
class ConvertedTrackedObject:
    """Intermediate representation before ROS message serialization."""

    track_id: int
    class_name: str
    confidence: float
    confidence_valid: bool
    footprint_polygon_xy: tuple[Point2, ...]
    velocity_valid: bool = False
    velocity_x: float = 0.0
    velocity_y: float = 0.0
    velocity_z: float = 0.0


@dataclass
class ConversionStats:
    received_detections: int = 0
    converted_objects: int = 0
    skipped_invalid: int = 0


def stamp_to_ns(sec: int, nanosec: int) -> int:
    return int(sec) * 1_000_000_000 + int(nanosec)


def frame_local_track_id(
    detection_id: str,
    detection_index: int,
    stamp_ns: int,
) -> int:
    """Return a NON-PERSISTENT track_id unique within one source message.

    P0 policy: segmentation/terrain do not provide temporal tracking. IDs are
    reproducible for a given Detection3DArray but must not be treated as stable
    across frames.
    """
    if detection_id:
        parts = str(detection_id).split(":", 1)
        if len(parts) == 2:
            try:
                left = int(parts[0])
                right = int(parts[1])
                # stamp_ns dominates; small index suffix keeps IDs unique per frame.
                return int((left % (1 << 58)) * 64 + (right & 0x3F))
            except ValueError:
                pass
        if str(detection_id).isdigit():
            return int(detection_id) % (1 << 64)
    return int((int(stamp_ns) % (1 << 58)) * 64 + (detection_index & 0x3F))


def normalize_quaternion(
    x: float,
    y: float,
    z: float,
    w: float,
) -> tuple[float, float, float, float] | None:
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm <= 1e-12:
        return None
    return x / norm, y / norm, z / norm, w / norm


def quaternion_to_rotation_matrix(
    x: float,
    y: float,
    z: float,
    w: float,
) -> tuple[tuple[float, float, float], ...]:
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z
    return (
        (1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)),
        (2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)),
        (2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)),
    )


def rotate_point(
    point: Point3,
    rotation: tuple[tuple[float, float, float], ...],
) -> Point3:
    x, y, z = point
    return (
        rotation[0][0] * x + rotation[0][1] * y + rotation[0][2] * z,
        rotation[1][0] * x + rotation[1][1] * y + rotation[1][2] * z,
        rotation[2][0] * x + rotation[2][1] * y + rotation[2][2] * z,
    )


def transform_point(
    point: Point3,
    rotation: tuple[tuple[float, float, float], ...],
    translation: Point3,
) -> Point3:
    rotated = rotate_point(point, rotation)
    return (
        rotated[0] + translation[0],
        rotated[1] + translation[1],
        rotated[2] + translation[2],
    )


def oriented_footprint_polygon_xy(
    center: Point3,
    orientation: tuple[float, float, float, float],
    size_x: float,
    size_y: float,
    *,
    transform_rotation: tuple[tuple[float, float, float], ...] | None = None,
    transform_translation: Point3 | None = None,
) -> tuple[Point2, ...] | None:
    """Build a 4-point oriented footprint in map XY from a 3D OBB base."""
    normalized = normalize_quaternion(*orientation)
    if normalized is None:
        return None
    rotation = quaternion_to_rotation_matrix(*normalized)
    local_corners: tuple[Point3, ...] = (
        (-0.5 * size_x, -0.5 * size_y, 0.0),
        (0.5 * size_x, -0.5 * size_y, 0.0),
        (0.5 * size_x, 0.5 * size_y, 0.0),
        (-0.5 * size_x, 0.5 * size_y, 0.0),
    )
    world_xy: list[Point2] = []
    for local in local_corners:
        rotated = rotate_point(local, rotation)
        world = (
            rotated[0] + center[0],
            rotated[1] + center[1],
            rotated[2] + center[2],
        )
        if transform_rotation is not None and transform_translation is not None:
            world = transform_point(world, transform_rotation, transform_translation)
        if not all(math.isfinite(value) for value in world):
            return None
        world_xy.append((world[0], world[1]))
    if len(set(world_xy)) < 3:
        return None
    return tuple(world_xy)


def _best_hypothesis(detection) -> tuple[str, float] | None:
    if not detection.results:
        return None
    result = max(
        detection.results,
        key=lambda item: float(item.hypothesis.score),
    )
    score = float(result.hypothesis.score)
    class_id = str(result.hypothesis.class_id)
    if not class_id.strip() or not math.isfinite(score):
        return None
    return class_id, score


def convert_detection3d(
    detection,
    detection_index: int,
    stamp_ns: int,
    *,
    transform_rotation: tuple[tuple[float, float, float], ...] | None = None,
    transform_translation: Point3 | None = None,
) -> ConvertedTrackedObject | None:
    hypothesis = _best_hypothesis(detection)
    if hypothesis is None:
        return None
    class_name, score = hypothesis

    center_x = float(detection.bbox.center.position.x)
    center_y = float(detection.bbox.center.position.y)
    center_z = float(detection.bbox.center.position.z)
    if not all(math.isfinite(v) for v in (center_x, center_y, center_z)):
        return None

    size_x = float(detection.bbox.size.x)
    size_y = float(detection.bbox.size.y)
    size_z = float(detection.bbox.size.z)
    if not all(math.isfinite(v) for v in (size_x, size_y, size_z)):
        return None
    if size_x <= 0.0 or size_y <= 0.0:
        return None

    orientation = detection.bbox.center.orientation
    footprint = oriented_footprint_polygon_xy(
        (center_x, center_y, center_z),
        (
            float(orientation.x),
            float(orientation.y),
            float(orientation.z),
            float(orientation.w),
        ),
        size_x,
        size_y,
        transform_rotation=transform_rotation,
        transform_translation=transform_translation,
    )
    if footprint is None:
        return None

    track_id = frame_local_track_id(
        str(getattr(detection, "id", "")),
        detection_index,
        stamp_ns,
    )
    return ConvertedTrackedObject(
        track_id=track_id,
        class_name=class_name,
        confidence=score,
        confidence_valid=True,
        footprint_polygon_xy=footprint,
        velocity_valid=False,
    )


def convert_detection3d_array(
    detections: Iterable,
    stamp_ns: int,
    *,
    transform_rotation: tuple[tuple[float, float, float], ...] | None = None,
    transform_translation: Point3 | None = None,
) -> tuple[tuple[ConvertedTrackedObject, ...], ConversionStats]:
    stats = ConversionStats()
    converted: list[ConvertedTrackedObject] = []
    used_ids: set[int] = set()
    for index, detection in enumerate(detections):
        stats.received_detections += 1
        item = convert_detection3d(
            detection,
            index,
            stamp_ns,
            transform_rotation=transform_rotation,
            transform_translation=transform_translation,
        )
        if item is None:
            stats.skipped_invalid += 1
            continue
        track_id = item.track_id
        if track_id in used_ids:
            track_id = frame_local_track_id(
                f"{stamp_ns}:{index}",
                index,
                stamp_ns,
            )
            item = ConvertedTrackedObject(
                track_id=track_id,
                class_name=item.class_name,
                confidence=item.confidence,
                confidence_valid=item.confidence_valid,
                footprint_polygon_xy=item.footprint_polygon_xy,
                velocity_valid=False,
            )
        used_ids.add(track_id)
        converted.append(item)
        stats.converted_objects += 1
    return tuple(converted), stats


def rotation_matrix_from_tf(transform) -> tuple[tuple[float, float, float], ...]:
    return quaternion_to_rotation_matrix(
        float(transform.rotation.x),
        float(transform.rotation.y),
        float(transform.rotation.z),
        float(transform.rotation.w),
    )


def translation_from_tf(transform) -> Point3:
    return (
        float(transform.translation.x),
        float(transform.translation.y),
        float(transform.translation.z),
    )
