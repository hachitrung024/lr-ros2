"""Estimate light-weight tracked 3D boxes from instance masks and depth."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Pose
from rclpy.clock import JumpThreshold
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose


@dataclass(frozen=True)
class BoxMeasurement:
    """An oriented 3D box fitted to one instance in the current frame."""

    center: np.ndarray
    size: np.ndarray
    orientation: np.ndarray


@dataclass(frozen=True)
class TrackedBox:
    """A confirmed, temporally filtered 3D box with a persistent ID."""

    track_id: int
    center: np.ndarray
    size: np.ndarray
    orientation: np.ndarray


@dataclass
class _Track:
    """Mutable state for one constant-velocity 3D box track."""

    track_id: int
    state: np.ndarray
    covariance: np.ndarray
    size: np.ndarray
    orientation: np.ndarray
    last_stamp_sec: float
    hits: int = 1
    missed: int = 0


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
    return packed.view(byte_order).reshape(height, width).astype(
        np.uint16,
        copy=False,
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
    lower = np.quantile(projections, trim_fraction, axis=0)
    upper = np.quantile(projections, 1.0 - trim_fraction, axis=0)
    center = rotation @ ((lower + upper) * 0.5)
    size = np.maximum(upper - lower, minimum_size_m)
    return BoxMeasurement(
        center=center.astype(np.float64),
        size=size.astype(np.float64),
        orientation=_quaternion_from_matrix(rotation),
    )


class BoxTracker:
    """Associate fitted boxes and filter their centers with a 3D Kalman filter."""

    def __init__(
        self,
        *,
        association_distance_m: float,
        max_missed_frames: int,
        min_confirmations: int,
        measurement_stddev_m: float,
        acceleration_stddev_mps2: float,
        size_smoothing: float,
        orientation_smoothing: float,
    ) -> None:
        _require_positive("association_distance_m", association_distance_m)
        _require_positive("measurement_stddev_m", measurement_stddev_m)
        _require_positive("acceleration_stddev_mps2", acceleration_stddev_mps2)
        if max_missed_frames < 0:
            raise ValueError("max_missed_frames must not be negative")
        if min_confirmations < 1:
            raise ValueError("min_confirmations must be at least one")
        _require_unit_interval("size_smoothing", size_smoothing)
        _require_unit_interval("orientation_smoothing", orientation_smoothing)

        self._association_distance_m = float(association_distance_m)
        self._max_missed_frames = int(max_missed_frames)
        self._min_confirmations = int(min_confirmations)
        self._measurement_variance = float(measurement_stddev_m) ** 2
        self._acceleration_variance = float(acceleration_stddev_mps2) ** 2
        self._size_smoothing = float(size_smoothing)
        self._orientation_smoothing = float(orientation_smoothing)
        self._tracks: list[_Track] = []
        self._next_track_id = 1

    def reset(self) -> None:
        """Discard state from a previous, now-invalid timeline."""
        self._tracks.clear()
        self._next_track_id = 1

    def update(
        self,
        measurements: list[BoxMeasurement],
        stamp_sec: float,
    ) -> list[TrackedBox]:
        """Update tracks from current-frame boxes and return confirmed boxes."""
        if not math.isfinite(stamp_sec):
            raise ValueError("stamp_sec must be finite")
        for track in self._tracks:
            _predict_track(track, stamp_sec, self._acceleration_variance)

        matches = _associate_tracks(
            self._tracks,
            measurements,
            self._association_distance_m,
        )
        matched_track_indices = set()
        matched_measurement_indices = set()
        updated_tracks: list[_Track] = []
        for track_index, measurement_index in matches:
            track = self._tracks[track_index]
            measurement = measurements[measurement_index]
            _update_track(
                track,
                measurement,
                self._measurement_variance,
                self._size_smoothing,
                self._orientation_smoothing,
            )
            matched_track_indices.add(track_index)
            matched_measurement_indices.add(measurement_index)
            updated_tracks.append(track)

        for index, track in enumerate(self._tracks):
            if index not in matched_track_indices:
                track.missed += 1

        for index, measurement in enumerate(measurements):
            if index in matched_measurement_indices:
                continue
            track = _new_track(self._next_track_id, measurement, stamp_sec)
            self._next_track_id += 1
            self._tracks.append(track)
            updated_tracks.append(track)

        self._tracks = [
            track
            for track in self._tracks
            if track.missed <= self._max_missed_frames
        ]
        return [
            TrackedBox(
                track_id=track.track_id,
                center=track.state[:3].copy(),
                size=track.size.copy(),
                orientation=track.orientation.copy(),
            )
            for track in updated_tracks
            if track.hits >= self._min_confirmations
        ]


class BoxEstimator3DNode(Node):
    """Fit and track 3D boxes from synchronized instance masks and depth."""

    def __init__(
        self,
        *,
        parameter_overrides=None,
        node_name="box_estimator_3d",
        namespace="",
    ) -> None:
        super().__init__(
            node_name,
            namespace=namespace,
            parameter_overrides=parameter_overrides,
        )
        self._mask_topic = self.declare_parameter(
            "input.mask_topic",
            "/segmentation/instance_mask",
        ).value
        self._depth_topic = self.declare_parameter(
            "input.depth_topic",
            "/zed/zed_node/depth/depth_registered",
        ).value
        self._camera_info_topic = self.declare_parameter(
            "input.camera_info_topic",
            "/zed/zed_node/rgb/color/rect/camera_info",
        ).value
        self._box_topic = self.declare_parameter(
            "output.box_topic",
            "/segmentation/boxes_3d",
        ).value
        self._marker_topic = self.declare_parameter(
            "output.marker_topic",
            "/segmentation/box_markers",
        ).value
        self._output_frame = str(
            self.declare_parameter("output.frame_id", "").value
        ).strip()
        self._up_axis = np.asarray(
            self.declare_parameter(
                "geometry.up_axis",
                [0.0, -1.0, 0.0],
            ).value,
            dtype=np.float64,
        )
        self._sync_tolerance_ns = int(
            float(self.declare_parameter("sync_tolerance_sec", 0.05).value)
            * 1_000_000_000
        )
        self._minimum_depth_m = float(
            self.declare_parameter("minimum_depth_m", 0.2).value
        )
        self._maximum_depth_m = float(
            self.declare_parameter("maximum_depth_m", 20.0).value
        )
        self._sampling_stride = self.declare_parameter(
            "sampling_stride",
            2,
        ).value
        self._maximum_points_per_instance = self.declare_parameter(
            "maximum_points_per_instance",
            15000,
        ).value
        self._minimum_points_per_instance = self.declare_parameter(
            "minimum_points_per_instance",
            64,
        ).value
        self._depth_mad_scale = float(
            self.declare_parameter("depth_mad_scale", 8.0).value
        )
        self._minimum_depth_band_m = float(
            self.declare_parameter("minimum_depth_band_m", 0.15).value
        )
        self._trim_fraction = float(
            self.declare_parameter("box.trim_fraction", 0.02).value
        )
        self._minimum_box_size_m = float(
            self.declare_parameter("box.minimum_size_m", 0.05).value
        )
        self._box_marker_line_width_m = float(
            self.declare_parameter(
                "visualization.box_line_width_m", 0.07
            ).value
        )
        self._box_marker_fill_alpha = float(
            self.declare_parameter(
                "visualization.box_fill_alpha", 0.12
            ).value
        )
        self._box_marker_label_scale_m = float(
            self.declare_parameter(
                "visualization.label_scale_m", 0.28
            ).value
        )
        self._validate_parameters()
        self._tracker = BoxTracker(
            association_distance_m=float(
                self.declare_parameter(
                    "tracking.association_distance_m",
                    1.0,
                ).value
            ),
            max_missed_frames=self.declare_parameter(
                "tracking.max_missed_frames",
                5,
            ).value,
            min_confirmations=self.declare_parameter(
                "tracking.min_confirmations",
                2,
            ).value,
            measurement_stddev_m=float(
                self.declare_parameter(
                    "tracking.measurement_stddev_m",
                    0.08,
                ).value
            ),
            acceleration_stddev_mps2=float(
                self.declare_parameter(
                    "tracking.acceleration_stddev_mps2",
                    1.0,
                ).value
            ),
            size_smoothing=float(
                self.declare_parameter(
                    "tracking.size_smoothing",
                    0.35,
                ).value
            ),
            orientation_smoothing=float(
                self.declare_parameter(
                    "tracking.orientation_smoothing",
                    0.35,
                ).value
            ),
        )

        sensor_qos = QoSProfile(depth=1)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        sensor_qos.durability = DurabilityPolicy.VOLATILE
        self._box_publisher = self.create_publisher(
            Detection3DArray,
            str(self._box_topic),
            sensor_qos,
        )
        marker_qos = QoSProfile(depth=1)
        marker_qos.reliability = ReliabilityPolicy.RELIABLE
        marker_qos.durability = DurabilityPolicy.VOLATILE
        self._marker_publisher = self.create_publisher(
            MarkerArray,
            str(self._marker_topic),
            marker_qos,
        )
        self._mask_subscription = self.create_subscription(
            Image,
            str(self._mask_topic),
            self._on_mask,
            sensor_qos,
        )
        self._depth_subscription = self.create_subscription(
            Image,
            str(self._depth_topic),
            self._on_depth,
            sensor_qos,
        )
        self._camera_info_subscription = self.create_subscription(
            CameraInfo,
            str(self._camera_info_topic),
            self._on_camera_info,
            sensor_qos,
        )
        self._tf_buffer = Buffer() if self._output_frame else None
        self._tf_listener = (
            TransformListener(self._tf_buffer, self, spin_thread=False)
            if self._tf_buffer is not None
            else None
        )
        self._pending_mask = None
        self._pending_depth = None
        self._camera_info = None
        # Humble's Python TF buffer does not clear itself when /clock jumps
        # backwards. SVO seek creates exactly that jump, so reset every state
        # that depends on the old timeline ourselves.
        self._time_jump_handle = self.get_clock().create_jump_callback(
            JumpThreshold(
                min_forward=None,
                min_backward=Duration(nanoseconds=-1),
                on_clock_change=True,
            ),
            post_callback=self._on_time_jump,
        )
        self.get_logger().info(
            f"mask={self._mask_topic}; depth={self._depth_topic}; "
            f"boxes={self._box_topic}; markers={self._marker_topic}; "
            f"frame={self._output_frame or 'source'}"
        )

    def _validate_parameters(self) -> None:
        for name, value in (
            ("input.mask_topic", self._mask_topic),
            ("input.depth_topic", self._depth_topic),
            ("input.camera_info_topic", self._camera_info_topic),
            ("output.box_topic", self._box_topic),
            ("output.marker_topic", self._marker_topic),
        ):
            if not str(value).strip():
                raise ValueError(f"{name} must not be empty")
        if self._sync_tolerance_ns < 0:
            raise ValueError("sync_tolerance_sec cannot be negative")
        if (
            not math.isfinite(self._minimum_depth_m)
            or self._minimum_depth_m < 0.0
            or not math.isfinite(self._maximum_depth_m)
            or self._maximum_depth_m <= self._minimum_depth_m
        ):
            raise ValueError("depth range is invalid")
        for name, value in (
            ("sampling_stride", self._sampling_stride),
            ("maximum_points_per_instance", self._maximum_points_per_instance),
            ("minimum_points_per_instance", self._minimum_points_per_instance),
        ):
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                raise ValueError(f"{name} must be a positive integer")
        _require_positive("depth_mad_scale", self._depth_mad_scale)
        _require_positive("minimum_depth_band_m", self._minimum_depth_band_m)
        if not 0.0 <= self._trim_fraction < 0.5:
            raise ValueError("box.trim_fraction must be in [0, 0.5)")
        _require_positive("box.minimum_size_m", self._minimum_box_size_m)
        _require_positive(
            "visualization.box_line_width_m",
            self._box_marker_line_width_m,
        )
        if not 0.0 <= self._box_marker_fill_alpha <= 1.0:
            raise ValueError(
                "visualization.box_fill_alpha must be in [0, 1]"
            )
        _require_positive(
            "visualization.label_scale_m",
            self._box_marker_label_scale_m,
        )
        self._up_axis = _unit_vector(self._up_axis, "geometry.up_axis")

    def _on_mask(self, message: Image) -> None:
        self._pending_mask = message
        self._try_estimate()

    def _on_depth(self, message: Image) -> None:
        self._pending_depth = message
        self._try_estimate()

    def _on_camera_info(self, message: CameraInfo) -> None:
        self._camera_info = message
        self._try_estimate()

    def _on_time_jump(self, _time_jump) -> None:
        """Reset state after SVO seek or a ROS time-source change."""
        self._pending_mask = None
        self._pending_depth = None
        self._tracker.reset()
        if self._tf_buffer is not None:
            self._tf_buffer.clear()

        empty_detections = Detection3DArray()
        empty_detections.header.frame_id = self._output_frame
        empty_detections.header.stamp = self.get_clock().now().to_msg()
        self._box_publisher.publish(empty_detections)

        markers = MarkerArray()
        clear = Marker()
        clear.header = empty_detections.header
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        self._marker_publisher.publish(markers)
        self.get_logger().info(
            "ROS time changed; cleared 3D-box TF cache, input queue, and tracker."
        )

    def _try_estimate(self) -> None:
        if (
            self._pending_mask is None
            or self._pending_depth is None
            or self._camera_info is None
        ):
            return
        difference = _stamp_nanoseconds(self._pending_mask) - _stamp_nanoseconds(
            self._pending_depth
        )
        if abs(difference) > self._sync_tolerance_ns:
            if difference < 0:
                self._pending_mask = None
            else:
                self._pending_depth = None
            return

        mask_message = self._pending_mask
        depth_message = self._pending_depth
        self._pending_mask = None
        self._pending_depth = None
        try:
            labels = instance_mask_message_to_labels(mask_message)
            depth_m = depth_message_to_meters(depth_message)
            measurements, header = self._measure_boxes(
                labels,
                depth_m,
                depth_message.header,
            )
        except (TypeError, ValueError) as error:
            self.get_logger().error(f"3D box estimation failed: {error}")
            return
        except TransformException as error:
            self.get_logger().warning(
                f"3D box transform unavailable: {error}",
                throttle_duration_sec=2.0,
            )
            return

        tracked_boxes = self._tracker.update(
            measurements,
            _stamp_seconds(header),
        )
        self._box_publisher.publish(_detection_array(header, tracked_boxes))
        self._marker_publisher.publish(_marker_array(
            header,
            tracked_boxes,
            line_width_m=self._box_marker_line_width_m,
            fill_alpha=self._box_marker_fill_alpha,
            label_scale_m=self._box_marker_label_scale_m,
        ))

    def _measure_boxes(
        self,
        labels: np.ndarray,
        depth_m: np.ndarray,
        header: Header,
    ) -> tuple[list[BoxMeasurement], Header]:
        if labels.shape != depth_m.shape:
            raise ValueError("instance mask and registered depth sizes differ")
        if (
            self._camera_info.width not in {0, labels.shape[1]}
            or self._camera_info.height not in {0, labels.shape[0]}
        ):
            raise ValueError("CameraInfo dimensions differ from mask and depth")
        output_header, rotation, translation = self._output_transform(header)
        measurements = []
        for instance_id in np.unique(labels):
            if instance_id == 0:
                continue
            points = _instance_points(
                labels,
                depth_m,
                int(instance_id),
                self._camera_info,
                minimum_depth_m=self._minimum_depth_m,
                maximum_depth_m=self._maximum_depth_m,
                sampling_stride=int(self._sampling_stride),
                maximum_points=int(self._maximum_points_per_instance),
                depth_mad_scale=self._depth_mad_scale,
                minimum_depth_band_m=self._minimum_depth_band_m,
            )
            if rotation is not None:
                points = points @ rotation.T + translation
            if points.shape[0] < int(self._minimum_points_per_instance):
                continue
            measurements.append(
                fit_oriented_box(
                    points,
                    self._up_axis,
                    trim_fraction=self._trim_fraction,
                    minimum_size_m=self._minimum_box_size_m,
                )
            )
        return measurements, output_header

    def _output_transform(
        self,
        header: Header,
    ) -> tuple[Header, np.ndarray | None, np.ndarray | None]:
        source_frame = str(header.frame_id).strip()
        if not source_frame:
            raise ValueError("depth header frame_id is empty")
        if not self._output_frame or self._output_frame == source_frame:
            return _header_with_frame(header, source_frame), None, None
        transform = self._tf_buffer.lookup_transform(
            self._output_frame,
            source_frame,
            Time.from_msg(header.stamp),
            timeout=Duration(seconds=0.05),
        )
        translation = transform.transform.translation
        return (
            _header_with_frame(header, self._output_frame),
            _quaternion_to_matrix(transform.transform.rotation),
            np.asarray([translation.x, translation.y, translation.z]),
        )


def _instance_points(
    labels: np.ndarray,
    depth_m: np.ndarray,
    instance_id: int,
    camera_info: CameraInfo,
    *,
    minimum_depth_m: float,
    maximum_depth_m: float,
    sampling_stride: int,
    maximum_points: int,
    depth_mad_scale: float,
    minimum_depth_band_m: float,
) -> np.ndarray:
    """Back-project robust depth samples belonging to one instance label."""
    fx, fy, cx, cy = _camera_intrinsics(camera_info)
    sampled_labels = labels[::sampling_stride, ::sampling_stride]
    sampled_depth = depth_m[::sampling_stride, ::sampling_stride]
    valid = (
        (sampled_labels == instance_id)
        & np.isfinite(sampled_depth)
        & (sampled_depth >= minimum_depth_m)
        & (sampled_depth <= maximum_depth_m)
    )
    rows, columns = np.nonzero(valid)
    if rows.size == 0:
        return np.empty((0, 3), dtype=np.float64)
    depth = sampled_depth[rows, columns].astype(np.float64, copy=False)
    keep = _depth_inlier_mask(
        depth,
        mad_scale=depth_mad_scale,
        minimum_band_m=minimum_depth_band_m,
    )
    rows = rows[keep]
    columns = columns[keep]
    depth = depth[keep]
    if depth.size > maximum_points:
        selected = np.linspace(
            0,
            depth.size - 1,
            maximum_points,
            dtype=np.int64,
        )
        rows = rows[selected]
        columns = columns[selected]
        depth = depth[selected]
    pixel_v = rows.astype(np.float64) * sampling_stride
    pixel_u = columns.astype(np.float64) * sampling_stride
    return np.column_stack((
        (pixel_u - cx) * depth / fx,
        (pixel_v - cy) * depth / fy,
        depth,
    ))


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


def _associate_tracks(
    tracks: list[_Track],
    measurements: list[BoxMeasurement],
    association_distance_m: float,
) -> list[tuple[int, int]]:
    """Greedily associate nearby predicted tracks with current measurements."""
    candidates = []
    for track_index, track in enumerate(tracks):
        for measurement_index, measurement in enumerate(measurements):
            distance = float(np.linalg.norm(track.state[:3] - measurement.center))
            adaptive_gate = max(
                association_distance_m,
                0.5 * float(np.linalg.norm(track.size + measurement.size)),
            )
            if distance <= adaptive_gate:
                candidates.append((distance, track_index, measurement_index))
    candidates.sort()
    matched_tracks = set()
    matched_measurements = set()
    matches = []
    for _, track_index, measurement_index in candidates:
        if (
            track_index in matched_tracks
            or measurement_index in matched_measurements
        ):
            continue
        matched_tracks.add(track_index)
        matched_measurements.add(measurement_index)
        matches.append((track_index, measurement_index))
    return matches


def _new_track(
    track_id: int,
    measurement: BoxMeasurement,
    stamp_sec: float,
) -> _Track:
    """Create a track with a zero initial velocity."""
    return _Track(
        track_id=track_id,
        state=np.concatenate((measurement.center, np.zeros(3))),
        covariance=np.diag([0.1, 0.1, 0.1, 1.0, 1.0, 1.0]),
        size=measurement.size.copy(),
        orientation=measurement.orientation.copy(),
        last_stamp_sec=stamp_sec,
    )


def _predict_track(
    track: _Track,
    stamp_sec: float,
    acceleration_variance: float,
) -> None:
    """Propagate a constant-velocity track to the supplied timestamp."""
    dt = min(max(stamp_sec - track.last_stamp_sec, 0.0), 1.0)
    transition = np.eye(6)
    transition[:3, 3:] = np.eye(3) * dt
    process = np.zeros((6, 6))
    process[:3, :3] = np.eye(3) * 0.25 * dt**4 * acceleration_variance
    process[:3, 3:] = np.eye(3) * 0.5 * dt**3 * acceleration_variance
    process[3:, :3] = process[:3, 3:]
    process[3:, 3:] = np.eye(3) * dt**2 * acceleration_variance
    track.state = transition @ track.state
    track.covariance = transition @ track.covariance @ transition.T + process
    track.last_stamp_sec = stamp_sec


def _update_track(
    track: _Track,
    measurement: BoxMeasurement,
    measurement_variance: float,
    size_smoothing: float,
    orientation_smoothing: float,
) -> None:
    """Correct one track from a current OBB measurement."""
    innovation = measurement.center - track.state[:3]
    residual_covariance = track.covariance[:3, :3] + (
        np.eye(3) * measurement_variance
    )
    gain = track.covariance[:, :3] @ np.linalg.inv(residual_covariance)
    track.state = track.state + gain @ innovation
    observation = np.zeros((3, 6))
    observation[:, :3] = np.eye(3)
    track.covariance = (
        np.eye(6) - gain @ observation
    ) @ track.covariance
    track.size = (
        (1.0 - size_smoothing) * track.size
        + size_smoothing * measurement.size
    )
    track.orientation = _nlerp_quaternion(
        track.orientation,
        measurement.orientation,
        orientation_smoothing,
    )
    track.hits += 1
    track.missed = 0


def _detection_array(
    header: Header,
    boxes: list[TrackedBox],
) -> Detection3DArray:
    """Convert tracked OBBs into the standard ROS vision_msgs output."""
    array = Detection3DArray()
    array.header = header
    for box in boxes:
        detection = Detection3D()
        detection.header = header
        detection.id = str(box.track_id)
        detection.bbox.center = _pose(box.center, box.orientation)
        detection.bbox.size.x = float(box.size[0])
        detection.bbox.size.y = float(box.size[1])
        detection.bbox.size.z = float(box.size[2])
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = "unknown"
        hypothesis.hypothesis.score = 1.0
        hypothesis.pose.pose = detection.bbox.center
        detection.results.append(hypothesis)
        array.detections.append(detection)
    return array


def _marker_array(
    header: Header,
    boxes: list[TrackedBox],
    *,
    line_width_m: float,
    fill_alpha: float,
    label_scale_m: float,
) -> MarkerArray:
    """Render tracked boxes with bold wireframes and persistent-ID labels."""
    array = MarkerArray()
    clear = Marker()
    clear.header = header
    clear.action = Marker.DELETEALL
    array.markers.append(clear)
    for box in boxes:
        red, green, blue = _track_color(box.track_id)
        cube = Marker()
        cube.header = header
        cube.ns = "segmentation_box_fills"
        cube.id = box.track_id * 3
        cube.type = Marker.CUBE
        cube.action = Marker.ADD
        cube.pose = _pose(box.center, box.orientation)
        cube.scale.x = float(box.size[0])
        cube.scale.y = float(box.size[1])
        cube.scale.z = float(box.size[2])
        cube.color.r = red
        cube.color.g = green
        cube.color.b = blue
        cube.color.a = fill_alpha
        array.markers.append(cube)

        frame = Marker()
        frame.header = header
        frame.ns = "segmentation_box_frames"
        frame.id = box.track_id * 3 + 1
        frame.type = Marker.LINE_LIST
        frame.action = Marker.ADD
        frame.pose = _pose(box.center, box.orientation)
        frame.scale.x = line_width_m
        frame.color.r = red
        frame.color.g = green
        frame.color.b = blue
        frame.color.a = 1.0
        frame.points = _box_wireframe_points(box.size)
        array.markers.append(frame)

        rotation = _quaternion_to_matrix(box.orientation)
        label = Marker()
        label.header = header
        label.ns = "segmentation_box_ids"
        label.id = box.track_id * 3 + 2
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x = float(
            box.center[0] + rotation[0, 2] * (box.size[2] * 0.5 + 0.15)
        )
        label.pose.position.y = float(
            box.center[1] + rotation[1, 2] * (box.size[2] * 0.5 + 0.15)
        )
        label.pose.position.z = float(
            box.center[2] + rotation[2, 2] * (box.size[2] * 0.5 + 0.15)
        )
        label.pose.orientation.w = 1.0
        label.scale.z = label_scale_m
        label.color.r = red
        label.color.g = green
        label.color.b = blue
        label.color.a = 1.0
        label.text = f"ID {box.track_id}"
        array.markers.append(label)
    return array


def _box_wireframe_points(size: np.ndarray) -> list[Point]:
    """Return the twelve local-space edges of an oriented box."""
    half_x, half_y, half_z = (
        float(value) * 0.5 for value in size
    )
    corners = (
        (-half_x, -half_y, -half_z),
        (half_x, -half_y, -half_z),
        (half_x, half_y, -half_z),
        (-half_x, half_y, -half_z),
        (-half_x, -half_y, half_z),
        (half_x, -half_y, half_z),
        (half_x, half_y, half_z),
        (-half_x, half_y, half_z),
    )
    edge_indices = (
        (0, 1), (1, 2), (2, 3), (3, 0),
        (4, 5), (5, 6), (6, 7), (7, 4),
        (0, 4), (1, 5), (2, 6), (3, 7),
    )
    return [
        Point(x=corners[index][0], y=corners[index][1], z=corners[index][2])
        for edge in edge_indices
        for index in edge
    ]


def _track_color(track_id: int) -> tuple[float, float, float]:
    """Generate a stable, high-contrast RGB color for a tracker ID."""
    return (
        0.35 + 0.65 * ((track_id * 53) % 255) / 255.0,
        0.35 + 0.65 * ((track_id * 97) % 255) / 255.0,
        0.35 + 0.65 * ((track_id * 193) % 255) / 255.0,
    )


def _pose(center: np.ndarray, orientation: np.ndarray) -> Pose:
    """Create a geometry pose from numeric center and quaternion values."""
    pose = Pose()
    pose.position.x = float(center[0])
    pose.position.y = float(center[1])
    pose.position.z = float(center[2])
    pose.orientation.x = float(orientation[0])
    pose.orientation.y = float(orientation[1])
    pose.orientation.z = float(orientation[2])
    pose.orientation.w = float(orientation[3])
    return pose


def _header_with_frame(header: Header, frame_id: str) -> Header:
    """Copy a source header while setting the requested coordinate frame."""
    copied = Header()
    copied.stamp = header.stamp
    copied.frame_id = frame_id
    return copied


def _stamp_nanoseconds(header_or_message) -> int:
    """Convert a header or message header timestamp to integer nanoseconds."""
    header = getattr(header_or_message, "header", header_or_message)
    return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)


def _stamp_seconds(header: Header) -> float:
    """Convert a ROS timestamp to seconds as a floating-point value."""
    return float(header.stamp.sec) + float(header.stamp.nanosec) * 1e-9


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
    return np.asarray([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ])


def _quaternion_from_matrix(matrix: np.ndarray) -> np.ndarray:
    """Convert a proper 3-by-3 rotation matrix to an xyzw quaternion."""
    values = np.asarray(matrix, dtype=np.float64)
    if values.shape != (3, 3):
        raise ValueError("rotation matrix must have shape (3, 3)")
    trace = float(np.trace(values))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion = np.asarray([
            (values[2, 1] - values[1, 2]) / scale,
            (values[0, 2] - values[2, 0]) / scale,
            (values[1, 0] - values[0, 1]) / scale,
            0.25 * scale,
        ])
    else:
        index = int(np.argmax(np.diag(values)))
        if index == 0:
            scale = math.sqrt(1.0 + values[0, 0] - values[1, 1] - values[2, 2]) * 2.0
            quaternion = np.asarray([
                0.25 * scale,
                (values[0, 1] + values[1, 0]) / scale,
                (values[0, 2] + values[2, 0]) / scale,
                (values[2, 1] - values[1, 2]) / scale,
            ])
        elif index == 1:
            scale = math.sqrt(1.0 + values[1, 1] - values[0, 0] - values[2, 2]) * 2.0
            quaternion = np.asarray([
                (values[0, 1] + values[1, 0]) / scale,
                0.25 * scale,
                (values[1, 2] + values[2, 1]) / scale,
                (values[0, 2] - values[2, 0]) / scale,
            ])
        else:
            scale = math.sqrt(1.0 + values[2, 2] - values[0, 0] - values[1, 1]) * 2.0
            quaternion = np.asarray([
                (values[0, 2] + values[2, 0]) / scale,
                (values[1, 2] + values[2, 1]) / scale,
                0.25 * scale,
                (values[1, 0] - values[0, 1]) / scale,
            ])
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


def main(args=None) -> None:
    """Run the 3D box estimator node."""
    rclpy.init(args=args)
    node = None
    try:
        node = BoxEstimator3DNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        # A subscription may be torn down while the executor is taking its
        # final message after SIGINT. Do not turn a clean launch shutdown into
        # a traceback, but preserve genuine runtime failures.
        if rclpy.ok():
            raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
