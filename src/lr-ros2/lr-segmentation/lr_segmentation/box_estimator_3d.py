"""ROS orchestration for synchronized depth, box estimation and tracking."""

from __future__ import annotations

import math
import time
from contextlib import nullcontext
from threading import RLock

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.clock import Clock, ClockType, JumpThreshold
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from safety_perception_msgs.msg import TrackedObjectArray
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray
from vision_msgs.msg import Detection3DArray

from .box_geometry import (
    BoxMeasurement, _box_footprint_xy, _quaternion_to_matrix, _require_positive,
    _robust_z_limits, _unit_vector, fit_oriented_box, fit_projected_footprint,
)
from .box_outputs import _detection_array, _tracked_object_array
from .box_geometry import TrackedBox as TrackedBox  # Historical public import.
from .box_tracking import BoxTracker
from .box_display import BoxDisplayBuffer
from .depth_sampling import (
    depth_message_to_meters, instance_mask_message_to_labels, instance_point_clouds,
)
from .image_pair_buffer import ImagePairBuffer

__all__ = [
    "BoxEstimator3DNode", "BoxMeasurement", "BoxTracker", "TrackedBox",
    "depth_message_to_meters", "instance_mask_message_to_labels",
    "fit_oriented_box", "fit_projected_footprint", "main",
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
        self._mask_reliability = self.declare_parameter(
            "input.mask_reliability", "reliable",
        ).value
        self._depth_reliability = self.declare_parameter(
            "input.depth_reliability", "reliable",
        ).value
        self._input_queue_depth = self.declare_parameter("input.queue_depth", 10).value
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
        self._footprint_topic = self.declare_parameter(
            "output.footprint_topic",
            "/segmentation/tracked_footprints",
        ).value
        self._output_frame = str(self.declare_parameter("output.frame_id", "").value).strip()
        self._up_axis = np.asarray(
            self.declare_parameter(
                "geometry.up_axis",
                [0.0, -1.0, 0.0],
            ).value,
            dtype=np.float64,
        )
        self._sync_tolerance_ns = int(
            float(self.declare_parameter("sync_tolerance_sec", 0.05).value) * 1_000_000_000
        )
        self._tf_timeout_sec = float(self.declare_parameter("tf_timeout_sec", 0.2).value)
        self._stale_after_sec = float(
            self.declare_parameter("diagnostics.stale_after_sec", 2.0).value
        )
        self._minimum_depth_m = float(self.declare_parameter("minimum_depth_m", 0.2).value)
        self._maximum_depth_m = float(self.declare_parameter("maximum_depth_m", 20.0).value)
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
        self._depth_mad_scale = float(self.declare_parameter("depth_mad_scale", 8.0).value)
        self._minimum_depth_band_m = float(
            self.declare_parameter("minimum_depth_band_m", 0.15).value
        )
        self._trim_fraction = float(self.declare_parameter("box.trim_fraction", 0.02).value)
        self._minimum_box_size_m = float(self.declare_parameter("box.minimum_size_m", 0.05).value)
        self._footprint_voxel_size_m = float(
            self.declare_parameter("footprint.voxel_size_m", 0.02).value
        )
        self._footprint_simplify_tolerance_m = float(
            self.declare_parameter("footprint.simplify_tolerance_m", 0.02).value
        )
        self._footprint_minimum_area_m2 = float(
            self.declare_parameter("footprint.minimum_area_m2", 0.01).value
        )
        self._footprint_maximum_vertices = self.declare_parameter(
            "footprint.maximum_vertices",
            32,
        ).value
        self._footprint_closing_radius_m = float(
            self.declare_parameter("footprint.closing_radius_m", 0.06).value
        )
        self._footprint_minimum_thickness_m = float(
            self.declare_parameter("footprint.minimum_thickness_m", 0.04).value
        )
        self._footprint_minimum_height_m = float(
            self.declare_parameter("footprint.minimum_height_m", 0.10).value
        )
        self._show_box_markers = self.declare_parameter(
            "visualization.show_boxes",
            False,
        ).value
        self._box_marker_line_width_m = float(
            self.declare_parameter("visualization.box_line_width_m", 0.07).value
        )
        self._box_marker_fill_alpha = float(
            self.declare_parameter("visualization.box_fill_alpha", 0.12).value
        )
        self._box_marker_label_scale_m = float(
            self.declare_parameter("visualization.label_scale_m", 0.28).value
        )
        self._footprint_marker_z_offset_m = float(
            self.declare_parameter("visualization.footprint_z_offset_m", 0.03).value
        )
        self._footprint_prism_fill_alpha = float(
            self.declare_parameter(
                "visualization.footprint_prism_fill_alpha",
                0.18,
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
        self._footprint_publisher = self.create_publisher(
            TrackedObjectArray,
            str(self._footprint_topic),
            sensor_qos,
        )
        marker_qos = QoSProfile(depth=1)
        marker_qos.reliability = ReliabilityPolicy.RELIABLE
        marker_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._marker_publisher = self.create_publisher(
            MarkerArray,
            str(self._marker_topic),
            marker_qos,
        )
        self._box_display = BoxDisplayBuffer(float(self.declare_parameter(
            "visualization.hold_sec", 0.5,
        ).value))
        self._display_timer = self.create_timer(0.1, self._publish_box_display)
        self._mask_subscription = self.create_subscription(
            Image,
            str(self._mask_topic),
            self._on_mask,
            self._input_image_qos(self._mask_reliability),
        )
        self._depth_subscription = self.create_subscription(
            Image,
            str(self._depth_topic),
            self._on_depth,
            self._input_image_qos(self._depth_reliability),
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
        self._pairs = ImagePairBuffer(
            self._sync_tolerance_ns,
            max_samples=int(self.declare_parameter("sync_buffer.max_samples", 30).value),
            max_age_ns=round(
                float(self.declare_parameter("sync_buffer.max_age_sec", 1.0).value) * 1e9
            ),
        )
        self._camera_info = None
        self._processing_ms = 0.0
        self._published_boxes = 0
        self._tf_failures = 0
        self._invalid_pairs = 0
        self._unobservable_instances = []
        self._last_success_ns = None
        self._last_error = "Waiting for a valid mask/depth observation"
        self._pending_pair = None
        self._retry_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._retry_timer = self.create_timer(
            0.01, self._try_estimate, clock=self._retry_clock
        )
        self._diagnostics = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self._diagnostic_timer = self.create_timer(1.0, self._publish_diagnostics)
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
        self._source_guard = None
        self._live_state_lock = RLock()
        if self.declare_parameter('source_guard_enabled', False).value:
            from lr_mavros_bridge.source_guard import SourceGuard
            self._source_guard = SourceGuard(
                self, self._on_source_reset, self._on_source_resume)
        self.get_logger().info(
            f"mask={self._mask_topic}; depth={self._depth_topic}; "
            f"boxes={self._box_topic}; footprints={self._footprint_topic}; "
            f"markers={self._marker_topic}; "
            f"frame={self._output_frame or 'source'}"
        )

    def _input_image_qos(self, reliability: str) -> QoSProfile:
        """Retain image fragments and short callback bursts for delayed masks."""
        return QoSProfile(
            depth=self._input_queue_depth,
            reliability=(ReliabilityPolicy.RELIABLE if reliability == "reliable"
                         else ReliabilityPolicy.BEST_EFFORT),
            durability=DurabilityPolicy.VOLATILE,
        )

    def _validate_parameters(self) -> None:
        for name, value in (
            ("input.mask_reliability", self._mask_reliability),
            ("input.depth_reliability", self._depth_reliability),
        ):
            if value not in {"reliable", "best_effort"}:
                raise ValueError(f"{name} must be reliable or best_effort")
        if not math.isfinite(self._tf_timeout_sec) or self._tf_timeout_sec < 0.0:
            raise ValueError("tf_timeout_sec must be finite and non-negative")
        _require_positive("diagnostics.stale_after_sec", self._stale_after_sec)
        for name, value in (
            ("input.mask_topic", self._mask_topic),
            ("input.depth_topic", self._depth_topic),
            ("input.camera_info_topic", self._camera_info_topic),
            ("output.box_topic", self._box_topic),
            ("output.footprint_topic", self._footprint_topic),
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
            ("input.queue_depth", self._input_queue_depth),
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
        _require_positive("footprint.voxel_size_m", self._footprint_voxel_size_m)
        if (
            not math.isfinite(self._footprint_simplify_tolerance_m)
            or self._footprint_simplify_tolerance_m < 0.0
        ):
            raise ValueError("footprint.simplify_tolerance_m must be finite and non-negative")
        if (
            not math.isfinite(self._footprint_minimum_area_m2)
            or self._footprint_minimum_area_m2 < 0.0
        ):
            raise ValueError("footprint.minimum_area_m2 must be finite and non-negative")
        if (
            isinstance(self._footprint_maximum_vertices, bool)
            or not isinstance(self._footprint_maximum_vertices, int)
            or self._footprint_maximum_vertices < 3
        ):
            raise ValueError("footprint.maximum_vertices must be an integer of at least three")
        for name, value in (
            ("footprint.closing_radius_m", self._footprint_closing_radius_m),
            ("footprint.minimum_thickness_m", self._footprint_minimum_thickness_m),
        ):
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and non-negative")
        _require_positive(
            "footprint.minimum_height_m",
            self._footprint_minimum_height_m,
        )
        if not isinstance(self._show_box_markers, bool):
            raise ValueError("visualization.show_boxes must be boolean")
        _require_positive(
            "visualization.box_line_width_m",
            self._box_marker_line_width_m,
        )
        if not 0.0 <= self._box_marker_fill_alpha <= 1.0:
            raise ValueError("visualization.box_fill_alpha must be in [0, 1]")
        _require_positive(
            "visualization.label_scale_m",
            self._box_marker_label_scale_m,
        )
        if not math.isfinite(self._footprint_marker_z_offset_m):
            raise ValueError("visualization.footprint_z_offset_m must be finite")
        if not 0.0 <= self._footprint_prism_fill_alpha <= 1.0:
            raise ValueError(
                "visualization.footprint_prism_fill_alpha must be in [0, 1]"
            )
        self._up_axis = _unit_vector(self._up_axis, "geometry.up_axis")

    def _on_mask(self, message: Image) -> None:
        guard = getattr(self, '_source_guard', None)
        if guard and not guard.valid(message.header.stamp, guard.token()):
            return
        self._pairs.add("mask", message, now_ns=self.get_clock().now().nanoseconds)
        self._try_estimate()

    def _on_depth(self, message: Image) -> None:
        guard = getattr(self, '_source_guard', None)
        if guard and not guard.valid(message.header.stamp, guard.token()):
            return
        self._pairs.add("depth", message, now_ns=self.get_clock().now().nanoseconds)
        self._try_estimate()

    def _on_camera_info(self, message: CameraInfo) -> None:
        self._camera_info = message
        self._try_estimate()

    def _on_time_jump(self, _time_jump) -> None:
        """Reset state after SVO seek or a ROS time-source change."""
        self._pairs.reset()
        self._tracker.reset()
        self._box_display.reset()
        self._published_boxes = 0
        self._tf_failures = 0
        self._processing_ms = 0.0
        self._invalid_pairs = 0
        self._unobservable_instances = []
        self._pending_pair = None
        self._last_success_ns = None
        self._last_error = "Waiting for an observation after ROS time change"
        if self._tf_buffer is not None:
            self._tf_buffer.clear()

        # A reset is not an observation of free space. Only clear presentation;
        # the prediction runtime waits for a measured batch on the new timeline.
        markers = MarkerArray()
        clear = Marker()
        clear.header.frame_id = self._output_frame
        clear.header.stamp = self.get_clock().now().to_msg()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        self._marker_publisher.publish(markers)
        self.get_logger().info(
            "ROS time changed; cleared 3D-box TF cache, input queue, and tracker."
        )

    def _try_estimate(self) -> None:
        with getattr(self, '_live_state_lock', nullcontext()):
            self._try_estimate_locked()

    def _on_source_reset(self, _event):
        with self._live_state_lock:
            self._on_time_jump(None)

    def _on_source_resume(self):
        with self._live_state_lock:
            self._camera_info = None
            self.destroy_subscription(self._depth_subscription)
            self.destroy_subscription(self._camera_info_subscription)
            self._depth_subscription = self.create_subscription(
                Image, str(self._depth_topic), self._on_depth,
                self._input_image_qos(self._depth_reliability))
            self._camera_info_subscription = self.create_subscription(
                CameraInfo, str(self._camera_info_topic), self._on_camera_info,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))

    def _try_estimate_locked(self) -> None:
        if self._camera_info is None:
            return
        while True:
            if self._pending_pair is None:
                pair = self._pairs.pop_pair()
                if pair is None:
                    return
                self._pending_pair = (*pair, time.monotonic() + self._tf_timeout_sec)
            mask, depth, deadline = self._pending_pair
            try:
                output_transform = self._output_transform(depth.header)
            except TransformException as error:
                self._last_error = f"Waiting for TF: {error}"
                if time.monotonic() < deadline:
                    return
                self._tf_failures += 1
                self._pending_pair = None
                self.get_logger().warning(self._last_error, throttle_duration_sec=2.0)
                continue
            except ValueError as error:
                self._record_invalid_pair(error)
                self._pending_pair = None
                continue
            self._pending_pair = None
            started = time.monotonic()
            self._estimate_pair(mask, depth, output_transform=output_transform)
            self._processing_ms = (time.monotonic() - started) * 1000.0

    def _record_invalid_pair(self, error):
        self._invalid_pairs += 1
        self._last_error = str(error)
        self.get_logger().warning(self._last_error, throttle_duration_sec=2.0)

    def _estimate_pair(self, mask_message, depth_message, *, output_transform=None) -> None:
        guard = getattr(self, '_source_guard', None)
        token = guard.token() if guard else None
        if guard and not guard.valid(depth_message.header.stamp, token):
            return
        try:
            labels = instance_mask_message_to_labels(mask_message)
            depth_m = depth_message_to_meters(depth_message)
            measurements, header = self._measure_boxes(
                labels,
                depth_m,
                depth_message.header,
                output_transform=output_transform,
            )
        except (TypeError, ValueError) as error:
            self._record_invalid_pair(error)
            return
        except TransformException as error:
            self._tf_failures += 1
            self._last_error = f"TF unavailable: {error}"
            self.get_logger().warning(
                f"3D box transform unavailable: {error}",
                throttle_duration_sec=2.0,
            )
            return

        if not measurements and self._unobservable_instances:
            self._record_invalid_pair("All instances have insufficient valid depth")
            return
        tracked_boxes = self._tracker.update(
            measurements,
            _stamp_seconds(header),
        )
        if measurements and not tracked_boxes:
            self._last_error = "Waiting for object confirmation"
            return
        if guard and not guard.valid(header.stamp, token):
            return
        self._box_publisher.publish(_detection_array(header, tracked_boxes))
        self._footprint_publisher.publish(_tracked_object_array(header, tracked_boxes))
        self._published_boxes += 1
        self._last_success_ns = self.get_clock().now().nanoseconds
        self._last_error = ""
        if self._unobservable_instances:
            self._last_error = (
                f"Partial observation: {len(self._unobservable_instances)} instances "
                "have insufficient valid depth"
            )
        elif len(tracked_boxes) != len(measurements):
            self._last_error = "Partial observation: some objects await confirmation"
        now_ns = self.get_clock().now().nanoseconds
        self._box_display.update(tracked_boxes, header, now_ns)
        self._publish_box_display(now_ns)

    def _publish_box_display(self, now_ns=None):
        markers = self._box_display.markers(
            self.get_clock().now().nanoseconds if now_ns is None else now_ns,
            line_width_m=self._box_marker_line_width_m,
            fill_alpha=self._box_marker_fill_alpha,
            label_scale_m=self._box_marker_label_scale_m,
            footprint_z_offset_m=self._footprint_marker_z_offset_m,
            prism_fill_alpha=self._footprint_prism_fill_alpha,
            show_boxes=self._show_box_markers,
        )
        if markers is not None:
            self._marker_publisher.publish(markers)

    def _publish_diagnostics(self) -> None:
        now = self.get_clock().now()
        age = (
            math.inf if self._last_success_ns is None
            else max(0.0, (now.nanoseconds - self._last_success_ns) * 1e-9)
        )
        reason = self._last_error
        if not reason and age > self._stale_after_sec:
            reason = "Mask/depth output is stale"
        message = DiagnosticArray()
        message.header.stamp = now.to_msg()
        message.status = [
            DiagnosticStatus(
                name=self.get_name(),
                level=DiagnosticStatus.WARN if reason else DiagnosticStatus.OK,
                message=reason or "mask-depth synchronization",
                values=[
                    KeyValue(key=key, value=str(value))
                    for key, value in {
                        "masks_received": self._pairs.mask_count,
                        "matched_pairs": self._pairs.pair_count,
                        "dropped_masks": self._pairs.dropped_masks,
                        "future_inputs_dropped": self._pairs.future_inputs,
                        "boxes_published": self._published_boxes,
                        "tf_failures": self._tf_failures,
                        "invalid_pairs": self._invalid_pairs,
                        "unobservable_instances": len(self._unobservable_instances),
                        "output_age_sec": age,
                        "callback_ms": self._processing_ms,
                    }.items()
                ],
            )
        ]
        self._diagnostics.publish(message)

    def _measure_boxes(
        self,
        labels: np.ndarray,
        depth_m: np.ndarray,
        header: Header,
        *,
        output_transform=None,
    ) -> tuple[list[BoxMeasurement], Header]:
        if labels.shape != depth_m.shape:
            raise ValueError("instance mask and registered depth sizes differ")
        if self._camera_info.width not in {0, labels.shape[1]} or self._camera_info.height not in {
            0,
            labels.shape[0],
        }:
            raise ValueError("CameraInfo dimensions differ from mask and depth")
        output_header, rotation, translation = (
            self._output_transform(header) if output_transform is None else output_transform
        )
        measurements = []
        self._unobservable_instances = []
        clouds = instance_point_clouds(
            labels, depth_m, self._camera_info,
            minimum_depth_m=self._minimum_depth_m,
            maximum_depth_m=self._maximum_depth_m,
            sampling_stride=int(self._sampling_stride),
            maximum_points=int(self._maximum_points_per_instance),
            depth_mad_scale=self._depth_mad_scale,
            minimum_depth_band_m=self._minimum_depth_band_m,
        )
        for instance_id, points in clouds.items():
            if rotation is not None:
                points = points @ rotation.T + translation
            if points.shape[0] < int(self._minimum_points_per_instance):
                self._unobservable_instances.append(instance_id)
                continue
            box = fit_oriented_box(
                points,
                self._up_axis,
                trim_fraction=self._trim_fraction,
                minimum_size_m=self._minimum_box_size_m,
            )
            footprint = fit_projected_footprint(
                points,
                voxel_size_m=self._footprint_voxel_size_m,
                simplify_tolerance_m=self._footprint_simplify_tolerance_m,
                minimum_area_m2=self._footprint_minimum_area_m2,
                maximum_vertices=int(self._footprint_maximum_vertices),
                closing_radius_m=self._footprint_closing_radius_m,
                minimum_thickness_m=self._footprint_minimum_thickness_m,
            )
            if footprint is None:
                footprint = _box_footprint_xy(box)
            minimum_z, maximum_z = _robust_z_limits(
                points[:, 2],
                trim_fraction=self._trim_fraction,
                minimum_height_m=self._footprint_minimum_height_m,
            )
            measurements.append(
                BoxMeasurement(
                    center=box.center,
                    size=box.size,
                    orientation=box.orientation,
                    footprint_xy=footprint,
                    minimum_z=minimum_z,
                    maximum_z=maximum_z,
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
        )
        translation = transform.transform.translation
        return (
            _header_with_frame(header, self._output_frame),
            _quaternion_to_matrix(transform.transform.rotation),
            np.asarray([translation.x, translation.y, translation.z]),
        )


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


def main(args=None) -> None:
    """Run the 3D box estimator node."""
    rclpy.init(args=args)
    node = None
    try:
        node = BoxEstimator3DNode()
        if node._source_guard is None:
            rclpy.spin(node)
        else:
            from rclpy.executors import MultiThreadedExecutor
            executor = MultiThreadedExecutor(num_threads=2)
            executor.add_node(node)
            try:
                executor.spin()
            finally:
                executor.shutdown()
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
