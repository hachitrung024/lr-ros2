"""Project 3D points into 2D detections and remove object volumes."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class ObjectFilterConfig:
    """Tuning values for conservative 2D-to-3D object filtering."""

    minimum_confidence: float = 0.25
    classes: tuple[str, ...] = ()
    minimum_cluster_points: int = 20
    depth_bin_size_m: float = 0.15
    depth_tolerance_m: float = 0.20
    relative_depth_tolerance: float = 0.03
    box_padding_m: float = 0.05
    lower_percentile: float = 2.0
    upper_percentile: float = 98.0
    minimum_orientation_ratio: float = 1.15

    def validate(self) -> None:
        """Reject settings that could produce unbounded or empty boxes."""
        numeric = (
            self.minimum_confidence,
            self.depth_bin_size_m,
            self.depth_tolerance_m,
            self.relative_depth_tolerance,
            self.box_padding_m,
            self.lower_percentile,
            self.upper_percentile,
            self.minimum_orientation_ratio,
        )
        if not all(math.isfinite(value) for value in numeric):
            raise ValueError("object filter parameters must be finite")
        if not 0.0 <= self.minimum_confidence <= 1.0:
            raise ValueError("minimum_confidence must be in [0, 1]")
        if (
            isinstance(self.minimum_cluster_points, bool)
            or not isinstance(self.minimum_cluster_points, int)
            or self.minimum_cluster_points < 3
        ):
            raise ValueError("minimum_cluster_points must be at least 3")
        if self.depth_bin_size_m <= 0.0:
            raise ValueError("depth_bin_size_m must be greater than zero")
        if self.depth_tolerance_m <= 0.0:
            raise ValueError("depth_tolerance_m must be greater than zero")
        if self.relative_depth_tolerance < 0.0:
            raise ValueError(
                "relative_depth_tolerance must be non-negative"
            )
        if self.box_padding_m < 0.0:
            raise ValueError("box_padding_m must be non-negative")
        if not 0.0 <= self.lower_percentile < self.upper_percentile <= 100.0:
            raise ValueError(
                "box percentiles must satisfy 0 <= lower < upper <= 100"
            )
        if self.minimum_orientation_ratio < 1.0:
            raise ValueError("minimum_orientation_ratio must be at least 1")


@dataclass(frozen=True)
class ObjectBox3D:
    """Yaw-oriented object box expressed in the input cloud frame."""

    detection_id: str
    class_id: str
    confidence: float
    center: np.ndarray
    size: np.ndarray
    yaw: float

    @property
    def corners(self) -> np.ndarray:
        """Return eight cloud-frame box corners."""
        half_size = np.asarray(self.size, dtype=np.float64) * 0.5
        signs = np.asarray([
            [-1.0, -1.0, -1.0],
            [1.0, -1.0, -1.0],
            [1.0, 1.0, -1.0],
            [-1.0, 1.0, -1.0],
            [-1.0, -1.0, 1.0],
            [1.0, -1.0, 1.0],
            [1.0, 1.0, 1.0],
            [-1.0, 1.0, 1.0],
        ])
        cosine = math.cos(self.yaw)
        sine = math.sin(self.yaw)
        rotation = np.asarray([
            [cosine, -sine, 0.0],
            [sine, cosine, 0.0],
            [0.0, 0.0, 1.0],
        ])
        return signs * half_size @ rotation.T + self.center

    @property
    def minimum(self) -> np.ndarray:
        """Return the minimum corner of the enclosing cloud-frame AABB."""
        return np.min(self.corners, axis=0)

    @property
    def maximum(self) -> np.ndarray:
        """Return the maximum corner of the enclosing cloud-frame AABB."""
        return np.max(self.corners, axis=0)

    def contains(self, points) -> np.ndarray:
        """Return a mask for points inside the oriented volume."""
        values = np.asarray(points, dtype=np.float64)
        cosine = math.cos(self.yaw)
        sine = math.sin(self.yaw)
        rotation = np.asarray([
            [cosine, -sine, 0.0],
            [sine, cosine, 0.0],
            [0.0, 0.0, 1.0],
        ])
        local = (values - self.center) @ rotation
        return np.logical_and(
            local >= -0.5 * self.size,
            local <= 0.5 * self.size,
        ).all(axis=1)


def detection_hypothesis(detection) -> tuple[str, float] | None:
    """Return the highest-scoring class hypothesis for one detection."""
    if not detection.results:
        return None
    result = max(
        detection.results,
        key=lambda item: float(item.hypothesis.score),
    )
    score = float(result.hypothesis.score)
    if not math.isfinite(score):
        return None
    return str(result.hypothesis.class_id), score


def eligible_detections(detections, config: ObjectFilterConfig) -> tuple:
    """Select detections that match the configured class and score gates."""
    allowed_classes = set(config.classes)
    selected = []
    for detection in detections:
        hypothesis = detection_hypothesis(detection)
        if hypothesis is None:
            continue
        class_id, score = hypothesis
        if score < config.minimum_confidence:
            continue
        if allowed_classes and class_id not in allowed_classes:
            continue
        selected.append(detection)
    return tuple(selected)


def _validated_matrix(value, shape, name: str) -> np.ndarray:
    matrix = np.asarray(value, dtype=np.float64)
    if matrix.shape != shape or not np.isfinite(matrix).all():
        raise ValueError(
            f"{name} must be a finite {shape[0]}x{shape[1]} matrix"
        )
    return matrix


def _bbox(detection) -> tuple[float, float, float, float] | None:
    center_x = float(detection.bbox.center.position.x)
    center_y = float(detection.bbox.center.position.y)
    size_x = float(detection.bbox.size_x)
    size_y = float(detection.bbox.size_y)
    values = (center_x, center_y, size_x, size_y)
    if not all(math.isfinite(value) for value in values):
        return None
    if size_x <= 0.0 or size_y <= 0.0:
        return None
    return (
        center_x - 0.5 * size_x,
        center_y - 0.5 * size_y,
        center_x + 0.5 * size_x,
        center_y + 0.5 * size_y,
    )


def _depth_cluster_indices(
    indices: np.ndarray,
    image_x: np.ndarray,
    image_y: np.ndarray,
    camera_depth: np.ndarray,
    bounds: tuple[float, float, float, float],
    config: ObjectFilterConfig,
) -> np.ndarray:
    """Choose the nearest well-supported depth mode near the bbox center."""
    if indices.size < config.minimum_cluster_points:
        return np.empty(0, dtype=np.int64)

    depths = camera_depth[indices]
    bin_ids = np.floor(depths / config.depth_bin_size_m).astype(np.int64)
    unique_bins, inverse, counts = np.unique(
        bin_ids,
        return_inverse=True,
        return_counts=True,
    )

    x0, y0, x1, y1 = bounds
    center_x = 0.5 * (x0 + x1)
    center_y = 0.5 * (y0 + y1)
    half_width = max(0.5 * (x1 - x0), 1e-6)
    half_height = max(0.5 * (y1 - y0), 1e-6)
    normalized_radius_sq = (
        np.square((image_x[indices] - center_x) / half_width)
        + np.square((image_y[indices] - center_y) / half_height)
    )
    center_weights = np.maximum(
        0.05,
        np.exp(-2.0 * normalized_radius_sq),
    )
    weighted_counts = np.bincount(
        inverse,
        weights=center_weights,
        minlength=unique_bins.size,
    )
    supported = (
        (counts >= config.minimum_cluster_points)
        & (weighted_counts >= 0.10 * config.minimum_cluster_points)
    )
    supported_indices = np.flatnonzero(supported)
    if supported_indices.size == 0:
        return np.empty(0, dtype=np.int64)

    # Occluding object points should form the nearest supported depth mode.
    selected_bin_index = supported_indices[
        np.argmin(unique_bins[supported_indices])
    ]
    seed_depth = float(np.median(depths[inverse == selected_bin_index]))
    tolerance = max(
        config.depth_tolerance_m,
        config.relative_depth_tolerance * seed_depth,
    )
    selected = indices[np.abs(depths - seed_depth) <= tolerance]
    if selected.size < config.minimum_cluster_points:
        return np.empty(0, dtype=np.int64)

    # One robust recentering step reduces sensitivity to histogram edges.
    refined_depth = float(np.median(camera_depth[selected]))
    selected = indices[np.abs(depths - refined_depth) <= tolerance]
    if selected.size < config.minimum_cluster_points:
        return np.empty(0, dtype=np.int64)
    return selected


def _fit_oriented_box(
    cluster_points: np.ndarray,
    detection,
    config: ObjectFilterConfig,
) -> ObjectBox3D | None:
    """Fit a robust yaw-only OBB to cloud-frame instance points."""
    points = np.asarray(cluster_points, dtype=np.float64)
    if (
        points.ndim != 2
        or points.shape[1] != 3
        or points.shape[0] < config.minimum_cluster_points
        or not np.isfinite(points).all()
    ):
        return None

    xy = points[:, :2]
    xy_center = np.median(xy, axis=0)
    radii = np.linalg.norm(xy - xy_center, axis=1)
    robust_radius = np.percentile(radii, config.upper_percentile)
    orientation_points = xy[radii <= robust_radius]
    yaw = 0.0
    if orientation_points.shape[0] >= 3:
        covariance = np.cov(orientation_points, rowvar=False)
        eigenvalues, eigenvectors = np.linalg.eigh(covariance)
        largest = float(eigenvalues[-1])
        smallest = max(float(eigenvalues[0]), 1e-12)
        if (
            math.isfinite(largest)
            and largest > 1e-12
            and largest / smallest >= config.minimum_orientation_ratio
        ):
            principal = eigenvectors[:, -1]
            yaw = math.atan2(float(principal[1]), float(principal[0]))
            while yaw >= 0.5 * math.pi:
                yaw -= math.pi
            while yaw < -0.5 * math.pi:
                yaw += math.pi

    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    rotation = np.asarray([
        [cosine, -sine, 0.0],
        [sine, cosine, 0.0],
        [0.0, 0.0, 1.0],
    ])
    local_points = points @ rotation
    minimum = np.percentile(
        local_points,
        config.lower_percentile,
        axis=0,
    ) - config.box_padding_m
    maximum = np.percentile(
        local_points,
        config.upper_percentile,
        axis=0,
    ) + config.box_padding_m
    size = maximum - minimum
    if not np.isfinite(size).all() or np.any(size <= 0.0):
        return None
    center_local = 0.5 * (minimum + maximum)
    center = center_local @ rotation.T

    hypothesis = detection_hypothesis(detection)
    if hypothesis is None:
        return None
    class_id, score = hypothesis
    return ObjectBox3D(
        detection_id=str(detection.id),
        class_id=class_id,
        confidence=score,
        center=center,
        size=size,
        yaw=yaw,
    )


def filter_points_in_instance_masks(
    points_cloud,
    detections,
    instance_labels,
    cloud_to_image,
    projection_matrix,
    config: ObjectFilterConfig,
) -> tuple[np.ndarray, tuple[ObjectBox3D, ...], int]:
    """Infer oriented boxes from instance masks and remove their volumes."""
    points = np.asarray(points_cloud, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points_cloud must have shape (N, 3)")
    labels = np.asarray(instance_labels)
    if labels.ndim != 2 or labels.shape[0] <= 0 or labels.shape[1] <= 0:
        raise ValueError("instance_labels must be a non-empty 2D array")
    if not np.issubdtype(labels.dtype, np.integer):
        raise ValueError("instance_labels must use an integer dtype")
    transform = _validated_matrix(cloud_to_image, (4, 4), "cloud_to_image")
    projection = _validated_matrix(
        projection_matrix,
        (3, 4),
        "projection_matrix",
    )
    if points.shape[0] == 0:
        return points.copy(), (), 0

    selected_ids = {
        id(detection)
        for detection in eligible_detections(detections, config)
    }
    if not selected_ids:
        return points.copy(), (), 0

    finite = np.isfinite(points).all(axis=1)
    image_points = np.full_like(points, np.nan)
    image_points[finite] = (
        points[finite] @ transform[:3, :3].T + transform[:3, 3]
    )
    homogeneous = np.column_stack((
        image_points,
        np.ones(points.shape[0], dtype=np.float64),
    ))
    projected = homogeneous @ projection.T
    projectable = (
        finite
        & np.isfinite(projected).all(axis=1)
        & (image_points[:, 2] > 1e-6)
        & (projected[:, 2] > 1e-9)
    )
    image_x = np.full(points.shape[0], np.nan, dtype=np.float64)
    image_y = np.full(points.shape[0], np.nan, dtype=np.float64)
    image_x[projectable] = (
        projected[projectable, 0] / projected[projectable, 2]
    )
    image_y[projectable] = (
        projected[projectable, 1] / projected[projectable, 2]
    )

    sampled_labels = np.zeros(points.shape[0], dtype=np.uint32)
    projectable_indices = np.flatnonzero(projectable)
    pixel_x = np.floor(image_x[projectable_indices] + 0.5).astype(np.int64)
    pixel_y = np.floor(image_y[projectable_indices] + 0.5).astype(np.int64)
    in_image = (
        (pixel_x >= 0)
        & (pixel_x < labels.shape[1])
        & (pixel_y >= 0)
        & (pixel_y < labels.shape[0])
    )
    sampled_indices = projectable_indices[in_image]
    sampled_labels[sampled_indices] = labels[
        pixel_y[in_image],
        pixel_x[in_image],
    ].astype(np.uint32, copy=False)

    removed = np.zeros(points.shape[0], dtype=bool)
    boxes = []
    for detection_index, detection in enumerate(detections):
        if id(detection) not in selected_ids:
            continue
        bounds = _bbox(detection)
        if bounds is None:
            continue
        mask_indices = np.flatnonzero(
            sampled_labels == detection_index + 1
        )
        cluster_indices = _depth_cluster_indices(
            mask_indices,
            image_x,
            image_y,
            image_points[:, 2],
            bounds,
            config,
        )
        if cluster_indices.size == 0:
            continue
        box = _fit_oriented_box(
            points[cluster_indices],
            detection,
            config,
        )
        if box is None:
            continue
        boxes.append(box)
        removed[cluster_indices] = True
        inside_box = np.zeros(points.shape[0], dtype=bool)
        inside_box[finite] = box.contains(points[finite])
        removed |= inside_box

    return points[~removed], tuple(boxes), int(np.count_nonzero(removed))


def filter_points_in_detection_boxes(
    points_cloud,
    detections,
    cloud_to_image,
    projection_matrix,
    config: ObjectFilterConfig,
) -> tuple[np.ndarray, tuple[ObjectBox3D, ...], int]:
    """Infer 3D boxes from 2D detections and remove points inside them."""
    points = np.asarray(points_cloud, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points_cloud must have shape (N, 3)")
    transform = _validated_matrix(cloud_to_image, (4, 4), "cloud_to_image")
    projection = _validated_matrix(
        projection_matrix,
        (3, 4),
        "projection_matrix",
    )
    if points.shape[0] == 0:
        return points.copy(), (), 0

    selected_detections = eligible_detections(detections, config)
    if not selected_detections:
        return points.copy(), (), 0

    finite = np.isfinite(points).all(axis=1)
    image_points = np.full_like(points, np.nan)
    image_points[finite] = (
        points[finite] @ transform[:3, :3].T + transform[:3, 3]
    )
    homogeneous = np.column_stack((
        image_points,
        np.ones(points.shape[0], dtype=np.float64),
    ))
    projected = homogeneous @ projection.T
    projectable = (
        finite
        & np.isfinite(projected).all(axis=1)
        & (image_points[:, 2] > 1e-6)
        & (projected[:, 2] > 1e-9)
    )
    image_x = np.full(points.shape[0], np.nan, dtype=np.float64)
    image_y = np.full(points.shape[0], np.nan, dtype=np.float64)
    image_x[projectable] = (
        projected[projectable, 0] / projected[projectable, 2]
    )
    image_y[projectable] = (
        projected[projectable, 1] / projected[projectable, 2]
    )

    removed = np.zeros(points.shape[0], dtype=bool)
    boxes = []
    for detection in selected_detections:
        bounds = _bbox(detection)
        if bounds is None:
            continue
        x0, y0, x1, y1 = bounds
        roi = (
            projectable
            & (image_x >= x0)
            & (image_x <= x1)
            & (image_y >= y0)
            & (image_y <= y1)
        )
        roi_indices = np.flatnonzero(roi)
        cluster_indices = _depth_cluster_indices(
            roi_indices,
            image_x,
            image_y,
            image_points[:, 2],
            bounds,
            config,
        )
        if cluster_indices.size == 0:
            continue

        cluster_points = points[cluster_indices]
        minimum = np.percentile(
            cluster_points,
            config.lower_percentile,
            axis=0,
        ) - config.box_padding_m
        maximum = np.percentile(
            cluster_points,
            config.upper_percentile,
            axis=0,
        ) + config.box_padding_m
        if not np.isfinite(minimum).all() or not np.isfinite(maximum).all():
            continue
        if np.any(maximum <= minimum):
            continue

        hypothesis = detection_hypothesis(detection)
        if hypothesis is None:  # Guard for non-standard mutable messages.
            continue
        class_id, score = hypothesis
        box = ObjectBox3D(
            detection_id=str(detection.id),
            class_id=class_id,
            confidence=score,
            center=0.5 * (minimum + maximum),
            size=maximum - minimum,
            yaw=0.0,
        )
        boxes.append(box)
        removed |= finite & np.logical_and(
            points >= minimum,
            points <= maximum,
        ).all(axis=1)

    return points[~removed], tuple(boxes), int(np.count_nonzero(removed))
