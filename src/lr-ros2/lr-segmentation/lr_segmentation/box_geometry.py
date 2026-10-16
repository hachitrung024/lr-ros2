"""ROS-independent box fitting, footprint geometry and numeric validation."""

from __future__ import annotations

from dataclasses import dataclass
import math

import cv2
import numpy as np


@dataclass(frozen=True)
class BoxMeasurement:
    """An oriented 3D box fitted to one instance in the current frame."""

    center: np.ndarray
    size: np.ndarray
    orientation: np.ndarray
    footprint_xy: np.ndarray | None = None
    minimum_z: float | None = None
    maximum_z: float | None = None


@dataclass(frozen=True)
class TrackedBox:
    """A confirmed, temporally filtered 3D box with a persistent ID."""

    track_id: int
    center: np.ndarray
    size: np.ndarray
    orientation: np.ndarray
    footprint_xy: np.ndarray | None = None
    minimum_z: float | None = None
    maximum_z: float | None = None


def fit_oriented_box(
    points: np.ndarray,
    up_axis: np.ndarray,
    *,
    trim_fraction: float,
    minimum_size_m: float,
) -> BoxMeasurement:
    """Fit a robust OBB with its local z-axis constrained to ``up_axis``."""
    values = np.asarray(points, dtype=np.float64)
    if values.ndim != 2 or values.shape[1] != 3 or values.shape[0] < 3:
        raise ValueError("points must have shape (N, 3) with N >= 3")
    if not np.isfinite(values).all():
        raise ValueError("points must be finite")
    if not 0.0 <= trim_fraction < 0.5:
        raise ValueError("trim_fraction must be in [0, 0.5)")
    if not math.isfinite(minimum_size_m) or minimum_size_m <= 0.0:
        raise ValueError("minimum_size_m must be positive and finite")

    up = _unit_vector(up_axis, "up_axis")
    centroid = np.mean(values, axis=0)
    centered = values - centroid
    horizontal = centered - np.outer(centered @ up, up)
    covariance = horizontal.T @ horizontal / max(values.shape[0] - 1, 1)
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    axis_x = eigenvectors[:, int(np.argmax(eigenvalues))]
    axis_x = axis_x - np.dot(axis_x, up) * up
    if np.linalg.norm(axis_x) < 1e-8:
        axis_x = _perpendicular_unit(up)
    else:
        axis_x = _unit_vector(axis_x, "horizontal principal axis")
    axis_y = _unit_vector(np.cross(up, axis_x), "horizontal secondary axis")
    rotation = np.column_stack((axis_x, axis_y, up))

    projections = values @ rotation
    lower, upper = np.quantile(
        projections, [trim_fraction, 1.0 - trim_fraction], axis=0
    )
    center = rotation @ ((lower + upper) * 0.5)
    size = np.maximum(upper - lower, minimum_size_m)
    return BoxMeasurement(
        center=center.astype(np.float64),
        size=size.astype(np.float64),
        orientation=_quaternion_from_matrix(rotation),
    )


def fit_projected_footprint(
    points: np.ndarray,
    *,
    voxel_size_m: float,
    simplify_tolerance_m: float,
    minimum_area_m2: float,
    maximum_vertices: int,
    closing_radius_m: float = 0.06,
    minimum_thickness_m: float = 0.04,
) -> np.ndarray | None:
    """Fit a compact concave boundary to a map-frame XY occupancy grid."""
    values = np.asarray(points, dtype=np.float64)
    if values.ndim != 2 or values.shape[1] not in (2, 3):
        raise ValueError("points must have shape (N, 2) or (N, 3)")
    if not np.isfinite(values).all():
        raise ValueError("points must be finite")
    _require_positive("footprint.voxel_size_m", voxel_size_m)
    if not math.isfinite(simplify_tolerance_m) or simplify_tolerance_m < 0.0:
        raise ValueError("footprint.simplify_tolerance_m must be finite and non-negative")
    if not math.isfinite(minimum_area_m2) or minimum_area_m2 < 0.0:
        raise ValueError("footprint.minimum_area_m2 must be finite and non-negative")
    if not math.isfinite(closing_radius_m) or closing_radius_m < 0.0:
        raise ValueError("footprint.closing_radius_m must be finite and non-negative")
    if not math.isfinite(minimum_thickness_m) or minimum_thickness_m < 0.0:
        raise ValueError("footprint.minimum_thickness_m must be finite and non-negative")
    if isinstance(maximum_vertices, bool) or not isinstance(maximum_vertices, int):
        raise ValueError("footprint.maximum_vertices must be an integer")
    if maximum_vertices < 3:
        raise ValueError("footprint.maximum_vertices must be at least three")
    if values.shape[0] < 3:
        return None

    xy = values[:, :2]
    padding_cells = max(
        2,
        int(math.ceil(max(closing_radius_m, minimum_thickness_m) / voxel_size_m))
        + 2,
    )
    origin = np.min(xy, axis=0) - padding_cells * voxel_size_m
    indices = np.floor((xy - origin) / voxel_size_m).astype(np.int64)
    width, height = np.max(indices, axis=0) + padding_cells + 1
    if width * height > 4_000_000:
        return _simplified_convex_hull(
            xy,
            simplify_tolerance_m=simplify_tolerance_m,
            minimum_area_m2=minimum_area_m2,
            maximum_vertices=maximum_vertices,
        )

    occupied = np.zeros((int(height), int(width)), dtype=np.uint8)
    occupied[indices[:, 1], indices[:, 0]] = 255
    closing_cells = int(math.ceil(closing_radius_m / voxel_size_m))
    if closing_cells > 0:
        kernel = _ellipse_kernel(closing_cells)
        occupied = cv2.morphologyEx(occupied, cv2.MORPH_CLOSE, kernel)

    component_count, labels, stats, _ = cv2.connectedComponentsWithStats(
        occupied,
        connectivity=8,
    )
    if component_count <= 1:
        return None
    largest_component = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    occupied = np.where(labels == largest_component, 255, 0).astype(np.uint8)
    thickness_cells = int(math.ceil(minimum_thickness_m * 0.5 / voxel_size_m))
    if thickness_cells > 0:
        occupied = cv2.dilate(occupied, _ellipse_kernel(thickness_cells))

    contours, _ = cv2.findContours(
        occupied,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_NONE,
    )
    if not contours:
        return None
    contour = max(contours, key=cv2.contourArea)
    epsilon_cells = simplify_tolerance_m / voxel_size_m
    contour = _approximate_contour(contour, epsilon_cells, maximum_vertices)
    polygon = contour.astype(np.float64) * voxel_size_m + origin
    if polygon.shape[0] < 3:
        return None
    area = _signed_polygon_area(polygon)
    if abs(area) < minimum_area_m2:
        return None
    if area < 0.0:
        polygon = polygon[::-1]
    if not _polygon_is_simple(polygon):
        return _simplified_convex_hull(
            xy,
            simplify_tolerance_m=simplify_tolerance_m,
            minimum_area_m2=minimum_area_m2,
            maximum_vertices=maximum_vertices,
        )
    return polygon


def _robust_z_limits(
    values: np.ndarray,
    *,
    trim_fraction: float,
    minimum_height_m: float,
) -> tuple[float, float]:
    """Estimate prism bottom and top while trimming isolated Z samples."""
    minimum_z, maximum_z = map(
        float, np.quantile(values, [trim_fraction, 1.0 - trim_fraction])
    )
    if maximum_z - minimum_z < minimum_height_m:
        middle = (minimum_z + maximum_z) * 0.5
        minimum_z = middle - minimum_height_m * 0.5
        maximum_z = middle + minimum_height_m * 0.5
    return minimum_z, maximum_z


def _convex_hull_xy(points: np.ndarray) -> np.ndarray:
    """Return the convex hull without a repeated closing vertex."""
    unique = np.unique(np.asarray(points, dtype=np.float64), axis=0)
    if unique.shape[0] < 3:
        return unique
    ordered = unique[np.lexsort((unique[:, 1], unique[:, 0]))]

    def cross(origin, first, second):
        return (first[0] - origin[0]) * (second[1] - origin[1]) - (
            first[1] - origin[1]
        ) * (second[0] - origin[0])

    lower = []
    for point in ordered:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 0.0:
            lower.pop()
        lower.append(point)
    upper = []
    for point in reversed(ordered):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 0.0:
            upper.pop()
        upper.append(point)
    return np.asarray(lower[:-1] + upper[:-1], dtype=np.float64)


def _ellipse_kernel(radius_cells: int) -> np.ndarray:
    """Create a symmetric occupancy-grid morphology kernel."""
    size = radius_cells * 2 + 1
    return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size))


def _approximate_contour(
    contour: np.ndarray,
    minimum_epsilon: float,
    maximum_vertices: int,
) -> np.ndarray:
    """Simplify a closed contour without deleting vertices out of sequence."""
    approximation = cv2.approxPolyDP(contour, minimum_epsilon, True)[:, 0, :]
    if approximation.shape[0] <= maximum_vertices:
        return approximation

    low = minimum_epsilon
    high = float(cv2.arcLength(contour, True))
    best = None
    for _ in range(24):
        epsilon = (low + high) * 0.5
        candidate = cv2.approxPolyDP(contour, epsilon, True)[:, 0, :]
        if candidate.shape[0] > maximum_vertices:
            low = epsilon
        else:
            high = epsilon
            if candidate.shape[0] >= 3:
                best = candidate
    if best is not None:
        return best
    convex = cv2.convexHull(contour)[:, 0, :]
    return _limit_polygon_vertices(convex, maximum_vertices)


def _limit_polygon_vertices(polygon: np.ndarray, maximum_vertices: int) -> np.ndarray:
    """Remove the least significant vertices until a message-size bound is met."""
    limited = np.asarray(polygon, dtype=np.float64)
    while limited.shape[0] > maximum_vertices:
        index = int(np.argmin(_polygon_vertex_distances(limited)))
        limited = np.delete(limited, index, axis=0)
    return limited


def _polygon_is_simple(polygon: np.ndarray) -> bool:
    """Return whether non-adjacent edges of a polygon do not intersect."""
    count = polygon.shape[0]
    if count < 3 or np.unique(polygon, axis=0).shape[0] != count:
        return False
    for first in range(count):
        first_end = (first + 1) % count
        for second in range(first + 1, count):
            second_end = (second + 1) % count
            if first == second or first_end == second or second_end == first:
                continue
            if _segments_intersect(
                polygon[first],
                polygon[first_end],
                polygon[second],
                polygon[second_end],
            ):
                return False
    return True


def _segments_intersect(first, first_end, second, second_end) -> bool:
    """Test closed 2D segments, including collinear contact."""
    def orientation(origin, end, point):
        return (end[0] - origin[0]) * (point[1] - origin[1]) - (
            end[1] - origin[1]
        ) * (point[0] - origin[0])

    values = (
        orientation(first, first_end, second),
        orientation(first, first_end, second_end),
        orientation(second, second_end, first),
        orientation(second, second_end, first_end),
    )
    epsilon = 1e-10
    if values[0] * values[1] < -epsilon and values[2] * values[3] < -epsilon:
        return True

    def on_segment(origin, end, point):
        return (
            min(origin[0], end[0]) - epsilon <= point[0]
            <= max(origin[0], end[0]) + epsilon
            and min(origin[1], end[1]) - epsilon <= point[1]
            <= max(origin[1], end[1]) + epsilon
        )

    return any(
        abs(value) <= epsilon and on_segment(origin, end, point)
        for value, origin, end, point in (
            (values[0], first, first_end, second),
            (values[1], first, first_end, second_end),
            (values[2], second, second_end, first),
            (values[3], second, second_end, first_end),
        )
    )


def _simplified_convex_hull(
    points: np.ndarray,
    *,
    simplify_tolerance_m: float,
    minimum_area_m2: float,
    maximum_vertices: int,
) -> np.ndarray | None:
    """Bound memory with a valid hull when an occupancy grid would be huge."""
    polygon = _convex_hull_xy(points)
    while polygon.shape[0] > 3:
        distances = _polygon_vertex_distances(polygon)
        index = int(np.argmin(distances))
        if (
            polygon.shape[0] <= maximum_vertices
            and distances[index] > simplify_tolerance_m
        ):
            break
        polygon = np.delete(polygon, index, axis=0)
    if polygon.shape[0] < 3 or abs(_signed_polygon_area(polygon)) < minimum_area_m2:
        return None
    if _signed_polygon_area(polygon) < 0.0:
        polygon = polygon[::-1]
    return polygon


def _polygon_vertex_distances(polygon: np.ndarray) -> np.ndarray:
    """Measure each vertex's deviation from the line through its neighbours."""
    previous = np.roll(polygon, 1, axis=0)
    following = np.roll(polygon, -1, axis=0)
    spans = following - previous
    lengths = np.linalg.norm(spans, axis=1)
    cross = np.abs(
        spans[:, 0] * (polygon[:, 1] - previous[:, 1])
        - spans[:, 1] * (polygon[:, 0] - previous[:, 0])
    )
    return np.divide(cross, lengths, out=np.zeros_like(cross), where=lengths > 1e-12)


def _signed_polygon_area(polygon: np.ndarray) -> float:
    """Return signed XY polygon area; positive means counter-clockwise."""
    return 0.5 * float(
        np.dot(polygon[:, 0], np.roll(polygon[:, 1], -1))
        - np.dot(polygon[:, 1], np.roll(polygon[:, 0], -1))
    )


def _box_footprint_xy(box: BoxMeasurement) -> np.ndarray:
    """Return a conservative rectangle when the visible surface is degenerate."""
    half_x, half_y = float(box.size[0]) * 0.5, float(box.size[1]) * 0.5
    local = np.asarray(
        [
            [-half_x, -half_y, 0.0],
            [half_x, -half_y, 0.0],
            [half_x, half_y, 0.0],
            [-half_x, half_y, 0.0],
        ]
    )
    world = local @ _quaternion_to_matrix(box.orientation).T + box.center
    footprint = world[:, :2]
    if _signed_polygon_area(footprint) < 0.0:
        footprint = footprint[::-1]
    return footprint


def _unit_vector(values, name: str) -> np.ndarray:
    """Normalize a three-dimensional vector and reject degenerate inputs."""
    vector = np.asarray(values, dtype=np.float64).reshape(-1)
    if vector.shape != (3,) or not np.isfinite(vector).all():
        raise ValueError(f"{name} must contain three finite values")
    norm = float(np.linalg.norm(vector))
    if norm < 1e-8:
        raise ValueError(f"{name} must not be zero")
    return vector / norm


def _perpendicular_unit(vector: np.ndarray) -> np.ndarray:
    """Return a deterministic unit vector perpendicular to ``vector``."""
    basis = np.eye(3)[int(np.argmin(np.abs(vector)))]
    return _unit_vector(np.cross(vector, basis), "perpendicular axis")


def _quaternion_to_matrix(quaternion) -> np.ndarray:
    """Convert an xyzw quaternion message or vector to a rotation matrix."""
    if hasattr(quaternion, "x"):
        values = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    else:
        values = quaternion
    x, y, z, w = _normalized_quaternion(values)
    return np.asarray(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ]
    )


def _quaternion_from_matrix(matrix: np.ndarray) -> np.ndarray:
    """Convert a proper 3-by-3 rotation matrix to an xyzw quaternion."""
    values = np.asarray(matrix, dtype=np.float64)
    if values.shape != (3, 3):
        raise ValueError("rotation matrix must have shape (3, 3)")
    trace = float(np.trace(values))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion = np.asarray(
            [
                (values[2, 1] - values[1, 2]) / scale,
                (values[0, 2] - values[2, 0]) / scale,
                (values[1, 0] - values[0, 1]) / scale,
                0.25 * scale,
            ]
        )
    else:
        index = int(np.argmax(np.diag(values)))
        if index == 0:
            scale = math.sqrt(1.0 + values[0, 0] - values[1, 1] - values[2, 2]) * 2.0
            quaternion = np.asarray(
                [
                    0.25 * scale,
                    (values[0, 1] + values[1, 0]) / scale,
                    (values[0, 2] + values[2, 0]) / scale,
                    (values[2, 1] - values[1, 2]) / scale,
                ]
            )
        elif index == 1:
            scale = math.sqrt(1.0 + values[1, 1] - values[0, 0] - values[2, 2]) * 2.0
            quaternion = np.asarray(
                [
                    (values[0, 1] + values[1, 0]) / scale,
                    0.25 * scale,
                    (values[1, 2] + values[2, 1]) / scale,
                    (values[0, 2] - values[2, 0]) / scale,
                ]
            )
        else:
            scale = math.sqrt(1.0 + values[2, 2] - values[0, 0] - values[1, 1]) * 2.0
            quaternion = np.asarray(
                [
                    (values[0, 2] + values[2, 0]) / scale,
                    (values[1, 2] + values[2, 1]) / scale,
                    0.25 * scale,
                    (values[1, 0] - values[0, 1]) / scale,
                ]
            )
    return _normalized_quaternion(quaternion)


def _nlerp_quaternion(
    first: np.ndarray,
    second: np.ndarray,
    fraction: float,
) -> np.ndarray:
    """Smoothly blend equivalent-orientation quaternions without angle wrap."""
    initial = _normalized_quaternion(first)
    target = _normalized_quaternion(second)
    if float(np.dot(initial, target)) < 0.0:
        target = -target
    return _normalized_quaternion((1.0 - fraction) * initial + fraction * target)


def _normalized_quaternion(values) -> np.ndarray:
    """Normalize an xyzw quaternion."""
    quaternion = np.asarray(values, dtype=np.float64).reshape(-1)
    if quaternion.shape != (4,) or not np.isfinite(quaternion).all():
        raise ValueError("quaternion must contain four finite values")
    norm = float(np.linalg.norm(quaternion))
    if norm < 1e-8:
        raise ValueError("quaternion must not be zero")
    return quaternion / norm


def _require_positive(name: str, value: float) -> None:
    """Raise a clear error unless ``value`` is finite and positive."""
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f"{name} must be positive and finite")


def _require_unit_interval(name: str, value: float) -> None:
    """Raise a clear error unless ``value`` lies in the closed unit interval."""
    if not math.isfinite(value) or not 0.0 <= value <= 1.0:
        raise ValueError(f"{name} must be in [0, 1]")
