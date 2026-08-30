"""ROS 2 node wrapping the NumPy terrain geometry estimator."""

from __future__ import annotations

import math
import time
from dataclasses import fields

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from grid_map_msgs.msg import GridMap
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import MarkerArray
from vision_msgs.msg import Detection2DArray, Detection3DArray

from .conversions import (
    grid_map_to_heatmap_image,
    instance_mask_image_to_labels,
    object_boxes_to_detection_array,
    object_boxes_to_markers,
    point_cloud_to_xyz,
    terrain_result_to_grid_map,
    terrain_result_to_markers,
    transform_to_matrix,
)
from .estimator import (
    TerrainGeometryConfig,
    TerrainGeometryEstimator,
    TerrainResult,
)
from .object_filter import (
    ObjectFilterConfig,
    eligible_detections,
    filter_points_in_instance_masks,
    transform_object_boxes_to_map,
)


def _stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class TerrainGeometryNode(Node):
    """Estimate local terrain planes from a stamped ROS PointCloud2."""

    def __init__(self, *, parameter_overrides=None) -> None:
        super().__init__(
            "terrain_geometry",
            parameter_overrides=parameter_overrides,
        )
        read_only = ParameterDescriptor(read_only=True)

        self._point_cloud_topic = self.declare_parameter(
            "input.point_cloud_topic",
            "/zed/zed_node/point_cloud/cloud_registered",
            read_only,
        ).value
        self._map_frame = self.declare_parameter(
            "frames.map_frame",
            "map",
            read_only,
        ).value
        self._sensor_frame = self.declare_parameter(
            "frames.sensor_frame",
            "zed_camera_link",
            read_only,
        ).value
        self._tf_timeout_sec = float(self.declare_parameter(
            "tf.lookup_timeout_sec",
            0.1,
            read_only,
        ).value)
        self._diagnostic_period_sec = float(self.declare_parameter(
            "diagnostics.period_sec",
            1.0,
            read_only,
        ).value)
        self._stale_after_sec = float(self.declare_parameter(
            "diagnostics.stale_after_sec",
            2.0,
            read_only,
        ).value)
        self._heatmap_pixels_per_cell = self.declare_parameter(
            "output.heatmap_pixels_per_cell",
            24,
            read_only,
        ).value

        self._object_filter_enabled = bool(self.declare_parameter(
            "object_filter.enabled",
            False,
            read_only,
        ).value)
        self._detections_topic = self.declare_parameter(
            "object_filter.detections_topic",
            "/segmentation/detections_2d",
            read_only,
        ).value
        self._instance_masks_topic = self.declare_parameter(
            "object_filter.instance_masks_topic",
            "/segmentation/instance_masks",
            read_only,
        ).value
        self._camera_info_topic = self.declare_parameter(
            "object_filter.camera_info_topic",
            "/zed/zed_node/rgb/color/rect/camera_info",
            read_only,
        ).value
        self._sync_tolerance_sec = float(self.declare_parameter(
            "object_filter.sync_tolerance_sec",
            0.05,
            read_only,
        ).value)
        self._sync_cache_sec = float(self.declare_parameter(
            "object_filter.sync_cache_sec",
            3.0,
            read_only,
        ).value)
        self._sync_queue_size = self.declare_parameter(
            "object_filter.sync_queue_size",
            50,
            read_only,
        ).value

        filter_classes = self.declare_parameter(
            "object_filter.classes",
            Parameter.Type.STRING_ARRAY,
            read_only,
        ).value
        self._object_filter_config = ObjectFilterConfig(
            minimum_confidence=float(self.declare_parameter(
                "object_filter.minimum_confidence",
                0.25,
                read_only,
            ).value),
            classes=tuple(str(value) for value in (filter_classes or ())),
            minimum_cluster_points=self.declare_parameter(
                "object_filter.minimum_cluster_points",
                20,
                read_only,
            ).value,
            depth_bin_size_m=float(self.declare_parameter(
                "object_filter.depth_bin_size_m",
                0.15,
                read_only,
            ).value),
            depth_tolerance_m=float(self.declare_parameter(
                "object_filter.depth_tolerance_m",
                0.20,
                read_only,
            ).value),
            relative_depth_tolerance=float(self.declare_parameter(
                "object_filter.relative_depth_tolerance",
                0.03,
                read_only,
            ).value),
            box_padding_m=float(self.declare_parameter(
                "object_filter.box_padding_m",
                0.05,
                read_only,
            ).value),
            lower_percentile=float(self.declare_parameter(
                "object_filter.lower_percentile",
                2.0,
                read_only,
            ).value),
            upper_percentile=float(self.declare_parameter(
                "object_filter.upper_percentile",
                98.0,
                read_only,
            ).value),
            minimum_orientation_ratio=float(self.declare_parameter(
                "object_filter.minimum_orientation_ratio",
                1.15,
                read_only,
            ).value),
        )
        self._validate_node_parameters()
        self._object_filter_config.validate()

        defaults = TerrainGeometryConfig()
        config_values = {}
        for config_field in fields(TerrainGeometryConfig):
            default = getattr(defaults, config_field.name)
            parameter = self.declare_parameter(
                f"terrain.{config_field.name}",
                default,
                read_only,
            )
            config_values[config_field.name] = parameter.value
        self._config = TerrainGeometryConfig(**config_values)
        self._estimator = TerrainGeometryEstimator(self._config)

        transient_qos = QoSProfile(depth=1)
        transient_qos.reliability = ReliabilityPolicy.RELIABLE
        transient_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._grid_map_publisher = self.create_publisher(
            GridMap,
            "~/grid_map",
            transient_qos,
        )
        self._marker_publisher = self.create_publisher(
            MarkerArray,
            "~/markers",
            transient_qos,
        )
        self._heatmap_publisher = self.create_publisher(
            Image,
            "~/heatmap",
            transient_qos,
        )
        self._diagnostic_publisher = self.create_publisher(
            DiagnosticArray,
            "/diagnostics",
            10,
        )

        sensor_qos = QoSProfile(depth=1)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        sensor_qos.durability = DurabilityPolicy.VOLATILE
        object_output_qos = QoSProfile(depth=1)
        object_output_qos.reliability = ReliabilityPolicy.RELIABLE
        object_output_qos.durability = DurabilityPolicy.VOLATILE
        self._object_boxes_publisher = self.create_publisher(
            Detection3DArray,
            "~/object_boxes_3d",
            object_output_qos,
        )
        self._object_box_markers_publisher = self.create_publisher(
            MarkerArray,
            "~/object_box_markers",
            object_output_qos,
        )
        self._cloud_subscription = self.create_subscription(
            PointCloud2,
            self._point_cloud_topic,
            self._on_point_cloud,
            sensor_qos,
        )
        self._detection_subscription = None
        self._instance_mask_subscription = None
        self._camera_info_subscription = None
        if self._object_filter_enabled:
            self._detection_subscription = self.create_subscription(
                Detection2DArray,
                self._detections_topic,
                self._on_detections,
                sensor_qos,
            )
            self._instance_mask_subscription = self.create_subscription(
                Image,
                self._instance_masks_topic,
                self._on_instance_masks,
                sensor_qos,
            )
            self._camera_info_subscription = self.create_subscription(
                CameraInfo,
                self._camera_info_topic,
                self._on_camera_info,
                sensor_qos,
            )
        self._reset_service = self.create_service(
            Trigger,
            "~/reset",
            self._on_reset,
        )

        self._tf_buffer = Buffer(
            cache_time=Duration(seconds=max(10.0, self._config.ttl_seconds)),
            node=self,
        )
        self._tf_listener = TransformListener(
            self._tf_buffer,
            self,
            spin_thread=False,
        )
        self._diagnostic_timer = self.create_timer(
            self._diagnostic_period_sec,
            self._publish_diagnostics,
        )

        self._started_monotonic = time.monotonic()
        self._last_receive_monotonic = None
        self._last_detection_receive_monotonic = None
        self._last_instance_mask_receive_monotonic = None
        self._last_success_monotonic = None
        self._last_warn_monotonic = 0.0
        self._last_processing_ms = 0.0
        self._processing_ms_ema = 0.0
        self._input_count = 0
        self._processed_count = 0
        self._tf_failure_count = 0
        self._malformed_cloud_count = 0
        self._malformed_detection_count = 0
        self._malformed_instance_mask_count = 0
        self._filter_failure_count = 0
        self._detection_count = 0
        self._instance_mask_count = 0
        self._synchronized_pair_count = 0
        self._unmatched_cloud_count = 0
        self._unmatched_detection_count = 0
        self._unmatched_instance_mask_count = 0
        self._last_box_count = 0
        self._last_removed_point_count = 0
        self._total_removed_point_count = 0
        self._last_error = ""
        self._last_input_frame = ""
        self._latest_camera_info = None
        self._pending_clouds = []
        self._pending_detections = []
        self._pending_instance_masks = []
        self._last_cloud_timestamp = None
        self._last_detection_timestamp = None
        self._last_instance_mask_timestamp = None
        self._last_rover_position = np.zeros(3, dtype=np.float64)
        self._last_result = TerrainResult((), (), (), False, True, None, 0, 0)

        filter_description = (
            f"; object filtering from {self._detections_topic} and "
            f"{self._instance_masks_topic}"
            if self._object_filter_enabled
            else ""
        )
        self.get_logger().info(
            f"Terrain geometry listening on {self._point_cloud_topic}; "
            f"map frame is {self._map_frame}{filter_description}"
        )

    def _validate_node_parameters(self) -> None:
        if not self._point_cloud_topic:
            raise ValueError("input.point_cloud_topic must not be empty")
        if not self._map_frame:
            raise ValueError("frames.map_frame must not be empty")
        if self._object_filter_enabled and not self._detections_topic:
            raise ValueError(
                "object_filter.detections_topic must not be empty"
            )
        if self._object_filter_enabled and not self._instance_masks_topic:
            raise ValueError(
                "object_filter.instance_masks_topic must not be empty"
            )
        if self._object_filter_enabled and not self._camera_info_topic:
            raise ValueError(
                "object_filter.camera_info_topic must not be empty"
            )
        for name, value in (
            ("tf.lookup_timeout_sec", self._tf_timeout_sec),
            ("diagnostics.period_sec", self._diagnostic_period_sec),
            ("diagnostics.stale_after_sec", self._stale_after_sec),
            ("object_filter.sync_tolerance_sec", self._sync_tolerance_sec),
            ("object_filter.sync_cache_sec", self._sync_cache_sec),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(
                    f"{name} must be finite and greater than zero"
                )
        if (
            isinstance(self._sync_queue_size, bool)
            or not isinstance(self._sync_queue_size, int)
            or self._sync_queue_size < 2
        ):
            raise ValueError(
                "object_filter.sync_queue_size must be an integer >= 2"
            )
        if (
            isinstance(self._heatmap_pixels_per_cell, bool)
            or not isinstance(self._heatmap_pixels_per_cell, int)
            or not 1 <= self._heatmap_pixels_per_cell <= 64
        ):
            raise ValueError(
                "output.heatmap_pixels_per_cell must be an integer from 1 to 64"
            )

    def _on_point_cloud(self, message: PointCloud2) -> None:
        callback_started = time.monotonic()
        self._last_receive_monotonic = callback_started
        self._input_count += 1
        self._last_input_frame = message.header.frame_id
        timestamp = _stamp_seconds(message.header.stamp)
        if not math.isfinite(timestamp):
            self._record_malformed("PointCloud2 timestamp is not finite")
            return
        if not message.header.frame_id:
            self._record_malformed("PointCloud2 frame_id is empty")
            return

        if self._object_filter_enabled:
            self._handle_timestamp_order("cloud", timestamp)
            self._pending_clouds.append((timestamp, message))
            self._try_synchronize()
            self._prune_sync_queues()
            return

        self._process_point_cloud(
            message,
            timestamp,
            callback_started,
            detections=None,
            instance_masks=None,
        )

    def _on_detections(self, message: Detection2DArray) -> None:
        callback_started = time.monotonic()
        self._last_detection_receive_monotonic = callback_started
        self._detection_count += 1
        timestamp = _stamp_seconds(message.header.stamp)
        if not math.isfinite(timestamp):
            self._malformed_detection_count += 1
            self._last_error = "Detection2DArray timestamp is not finite"
            self._warn_throttled(self._last_error)
            return
        if not message.header.frame_id:
            self._malformed_detection_count += 1
            self._last_error = "Detection2DArray frame_id is empty"
            self._warn_throttled(self._last_error)
            return

        self._handle_timestamp_order("detection", timestamp)
        self._pending_detections.append((timestamp, message))
        self._try_synchronize()
        self._prune_sync_queues()

    def _on_instance_masks(self, message: Image) -> None:
        callback_started = time.monotonic()
        self._last_instance_mask_receive_monotonic = callback_started
        self._instance_mask_count += 1
        timestamp = _stamp_seconds(message.header.stamp)
        if not math.isfinite(timestamp):
            self._malformed_instance_mask_count += 1
            self._last_error = "Instance mask timestamp is not finite"
            self._warn_throttled(self._last_error)
            return
        if not message.header.frame_id:
            self._malformed_instance_mask_count += 1
            self._last_error = "Instance mask frame_id is empty"
            self._warn_throttled(self._last_error)
            return
        if (
            message.encoding.lower() not in {"mono16", "16uc1"}
            or message.width <= 0
            or message.height <= 0
        ):
            self._malformed_instance_mask_count += 1
            self._last_error = "Instance mask is malformed"
            self._warn_throttled(self._last_error)
            return

        self._handle_timestamp_order("instance_mask", timestamp)
        self._pending_instance_masks.append((timestamp, message))
        self._try_synchronize()
        self._prune_sync_queues()

    def _on_camera_info(self, message: CameraInfo) -> None:
        projection = np.asarray(message.p, dtype=np.float64).reshape(3, 4)
        if (
            not message.header.frame_id
            or message.width <= 0
            or message.height <= 0
            or not np.isfinite(projection).all()
            or projection[0, 0] <= 0.0
            or projection[1, 1] <= 0.0
        ):
            self._filter_failure_count += 1
            self._last_error = "CameraInfo is uncalibrated or malformed"
            self._warn_throttled(self._last_error)
            return
        self._latest_camera_info = message
        self._try_synchronize()
        self._prune_sync_queues()

    def _handle_timestamp_order(self, stream: str, timestamp: float) -> None:
        attributes = {
            "cloud": "_last_cloud_timestamp",
            "detection": "_last_detection_timestamp",
            "instance_mask": "_last_instance_mask_timestamp",
        }
        attribute = attributes[stream]
        previous = getattr(self, attribute)
        if (
            previous is not None
            and timestamp + self._sync_tolerance_sec < previous
        ):
            self._unmatched_cloud_count += len(self._pending_clouds)
            self._unmatched_detection_count += len(
                self._pending_detections
            )
            self._unmatched_instance_mask_count += len(
                self._pending_instance_masks
            )
            self._pending_clouds.clear()
            self._pending_detections.clear()
            self._pending_instance_masks.clear()
            self._last_cloud_timestamp = None
            self._last_detection_timestamp = None
            self._last_instance_mask_timestamp = None
        setattr(self, attribute, timestamp)

    def _try_synchronize(self) -> None:
        while (
            self._pending_clouds
            and self._pending_detections
            and self._pending_instance_masks
        ):
            best_triplet = None
            for cloud_index, (cloud_stamp, _cloud) in enumerate(
                self._pending_clouds
            ):
                for detection_index, (
                    detection_stamp,
                    detection_message,
                ) in enumerate(self._pending_detections):
                    cloud_delta = abs(cloud_stamp - detection_stamp)
                    if cloud_delta > self._sync_tolerance_sec:
                        continue
                    needs_camera_info = bool(eligible_detections(
                        detection_message.detections,
                        self._object_filter_config,
                    ))
                    if needs_camera_info and self._latest_camera_info is None:
                        continue
                    for mask_index, (mask_stamp, _mask) in enumerate(
                        self._pending_instance_masks
                    ):
                        mask_delta = abs(detection_stamp - mask_stamp)
                        if mask_delta > self._sync_tolerance_sec:
                            continue
                        score = max(cloud_delta, mask_delta)
                        if best_triplet is None or score < best_triplet[0]:
                            best_triplet = (
                                score,
                                cloud_index,
                                detection_index,
                                mask_index,
                            )
            if best_triplet is None:
                return

            _, cloud_index, detection_index, mask_index = best_triplet
            cloud_stamp, cloud = self._pending_clouds.pop(cloud_index)
            _, detections = self._pending_detections.pop(detection_index)
            _, instance_masks = self._pending_instance_masks.pop(mask_index)
            self._synchronized_pair_count += 1
            self._process_point_cloud(
                cloud,
                cloud_stamp,
                time.monotonic(),
                detections=detections,
                instance_masks=instance_masks,
            )

    def _prune_sync_queues(self) -> None:
        timestamps = [
            value
            for value in (
                self._last_cloud_timestamp,
                self._last_detection_timestamp,
                self._last_instance_mask_timestamp,
            )
            if value is not None
        ]
        if timestamps:
            cutoff = max(timestamps) - self._sync_cache_sec
            while (
                self._pending_clouds
                and self._pending_clouds[0][0] < cutoff
            ):
                self._pending_clouds.pop(0)
                self._unmatched_cloud_count += 1
            while (
                self._pending_detections
                and self._pending_detections[0][0] < cutoff
            ):
                self._pending_detections.pop(0)
                self._unmatched_detection_count += 1
            while (
                self._pending_instance_masks
                and self._pending_instance_masks[0][0] < cutoff
            ):
                self._pending_instance_masks.pop(0)
                self._unmatched_instance_mask_count += 1

        while len(self._pending_clouds) > self._sync_queue_size:
            self._pending_clouds.pop(0)
            self._unmatched_cloud_count += 1
        while len(self._pending_detections) > self._sync_queue_size:
            self._pending_detections.pop(0)
            self._unmatched_detection_count += 1
        while len(self._pending_instance_masks) > self._sync_queue_size:
            self._pending_instance_masks.pop(0)
            self._unmatched_instance_mask_count += 1

    def _process_point_cloud(
        self,
        message: PointCloud2,
        timestamp: float,
        callback_started: float,
        detections: Detection2DArray | None,
        instance_masks: Image | None,
    ) -> None:

        try:
            if message.header.frame_id == self._map_frame:
                # Pre-transformed map clouds (e.g. /lr/point_cloud/cloud_in_map)
                # carry absolute map coordinates; recover sensor-frame points and
                # the matching map pose from TF for the estimator.
                transform = self._tf_buffer.lookup_transform(
                    self._map_frame,
                    self._sensor_frame,
                    Time.from_msg(message.header.stamp),
                    timeout=Duration(seconds=self._tf_timeout_sec),
                )
                sensor_to_map = transform_to_matrix(transform.transform)
                points_map = point_cloud_to_xyz(message)
                rotation = sensor_to_map[:3, :3]
                translation = sensor_to_map[:3, 3]
                points = (points_map - translation) @ rotation
            else:
                transform = self._tf_buffer.lookup_transform(
                    self._map_frame,
                    message.header.frame_id,
                    Time.from_msg(message.header.stamp),
                    timeout=Duration(seconds=self._tf_timeout_sec),
                )
                sensor_to_map = transform_to_matrix(transform.transform)
                points = point_cloud_to_xyz(message)
        except (TransformException, ValueError) as error:
            self._tf_failure_count += 1
            self._last_error = f"TF unavailable: {error}"
            result = self._estimator.update(
                np.empty((0, 3), dtype=np.float64),
                None,
                timestamp,
            )
            self._last_result = result
            if result.display_changed or result.reset_reason:
                self._publish_snapshot(
                    result,
                    message.header.stamp,
                    self._last_rover_position,
                )
            self._warn_throttled(self._last_error)
            self._finish_timing(callback_started)
            return

        if detections is not None:
            try:
                points = self._remove_detected_objects(
                    points,
                    message,
                    detections,
                    instance_masks,
                    sensor_to_map=(
                        sensor_to_map
                        if message.header.frame_id == self._map_frame
                        else None
                    ),
                )
            except (TransformException, TypeError, ValueError) as error:
                self._filter_failure_count += 1
                self._last_error = f"Object filtering failed: {error}"
                self._publish_object_boxes((), message.header)
                self._warn_throttled(self._last_error)
                self._finish_timing(callback_started)
                return

        try:
            result = self._estimator.update(points, sensor_to_map, timestamp)
        except (TypeError, ValueError) as error:
            self._record_malformed(str(error))
            self._finish_timing(callback_started)
            return

        self._last_rover_position = sensor_to_map[:3, 3].copy()
        self._last_result = result
        self._last_success_monotonic = time.monotonic()
        self._last_error = ""
        self._processed_count += 1
        self._publish_snapshot(
            result,
            message.header.stamp,
            self._last_rover_position,
        )
        if result.reset_reason:
            self.get_logger().warning(
                f"Terrain state reset: {result.reset_reason}"
            )
        self._finish_timing(callback_started)

    def _remove_detected_objects(
        self,
        points: np.ndarray,
        cloud: PointCloud2,
        detections: Detection2DArray,
        instance_masks: Image | None,
        sensor_to_map: np.ndarray | None = None,
    ) -> np.ndarray:
        selected = eligible_detections(
            detections.detections,
            self._object_filter_config,
        )
        if not selected:
            self._last_box_count = 0
            self._last_removed_point_count = 0
            self._publish_object_boxes((), cloud.header)
            return points

        camera_info = self._latest_camera_info
        if camera_info is None:
            raise ValueError("waiting for calibrated CameraInfo")
        if instance_masks is None:
            raise ValueError("waiting for synchronized instance mask")
        if detections.header.frame_id != instance_masks.header.frame_id:
            raise ValueError("detection and instance mask frames differ")
        if detections.header.frame_id != camera_info.header.frame_id:
            raise ValueError("detection and CameraInfo frames differ")
        labels = instance_mask_image_to_labels(instance_masks)
        if labels.shape != (camera_info.height, camera_info.width):
            raise ValueError(
                "instance mask dimensions differ from CameraInfo"
            )
        if labels.size and int(np.max(labels)) > len(detections.detections):
            raise ValueError("instance mask contains an unknown detection ID")
        points_frame = cloud.header.frame_id
        if cloud.header.frame_id == self._map_frame:
            # Map clouds are converted back to sensor-frame XYZ for terrain
            # fitting; object projection must use the same coordinate frame.
            points_frame = self._sensor_frame
        cloud_to_image_transform = self._tf_buffer.lookup_transform(
            camera_info.header.frame_id,
            points_frame,
            Time.from_msg(cloud.header.stamp),
            timeout=Duration(seconds=self._tf_timeout_sec),
        )
        cloud_to_image = transform_to_matrix(
            cloud_to_image_transform.transform
        )
        projection = np.asarray(
            camera_info.p,
            dtype=np.float64,
        ).reshape(3, 4)
        filtered, boxes, removed_count = filter_points_in_instance_masks(
            points,
            detections.detections,
            labels,
            cloud_to_image,
            projection,
            self._object_filter_config,
        )
        if (
            cloud.header.frame_id == self._map_frame
            and sensor_to_map is not None
            and boxes
        ):
            boxes = transform_object_boxes_to_map(boxes, sensor_to_map)
        self._last_box_count = len(boxes)
        self._last_removed_point_count = removed_count
        self._total_removed_point_count += removed_count
        self._publish_object_boxes(boxes, cloud.header)
        return filtered

    def _publish_object_boxes(self, boxes, header) -> None:
        self._object_boxes_publisher.publish(
            object_boxes_to_detection_array(boxes, header)
        )
        self._object_box_markers_publisher.publish(
            object_boxes_to_markers(boxes, header)
        )

    def _record_malformed(self, reason: str) -> None:
        self._malformed_cloud_count += 1
        self._last_error = f"Malformed PointCloud2: {reason}"
        self._warn_throttled(self._last_error)

    def _warn_throttled(self, message: str) -> None:
        now = time.monotonic()
        if now - self._last_warn_monotonic >= 5.0:
            self.get_logger().warning(message)
            self._last_warn_monotonic = now

    def _finish_timing(self, callback_started: float) -> None:
        self._last_processing_ms = (
            time.monotonic() - callback_started
        ) * 1000.0
        if self._processing_ms_ema <= 0.0:
            self._processing_ms_ema = self._last_processing_ms
        else:
            self._processing_ms_ema = (
                0.9 * self._processing_ms_ema
                + 0.1 * self._last_processing_ms
            )

    def _publish_snapshot(self, result, stamp, rover_position) -> None:
        grid_map = terrain_result_to_grid_map(
            result,
            rover_position,
            self._config,
            stamp,
            self._map_frame,
        )
        markers = terrain_result_to_markers(
            result,
            rover_position,
            self._config,
            stamp,
            self._map_frame,
        )
        heatmap = grid_map_to_heatmap_image(
            grid_map,
            self._heatmap_pixels_per_cell,
        )
        self._grid_map_publisher.publish(grid_map)
        self._marker_publisher.publish(markers)
        self._heatmap_publisher.publish(heatmap)

    def _on_reset(self, request, response):
        del request
        self._estimator.reset("manual reset")
        self._unmatched_cloud_count += len(self._pending_clouds)
        self._unmatched_detection_count += len(self._pending_detections)
        self._unmatched_instance_mask_count += len(
            self._pending_instance_masks
        )
        self._pending_clouds.clear()
        self._pending_detections.clear()
        self._pending_instance_masks.clear()
        self._last_cloud_timestamp = None
        self._last_detection_timestamp = None
        self._last_instance_mask_timestamp = None
        self._last_box_count = 0
        self._last_removed_point_count = 0
        self._last_result = TerrainResult(
            (), (), (), False, True, "manual reset", 0, 0
        )
        self._publish_snapshot(
            self._last_result,
            self.get_clock().now().to_msg(),
            self._last_rover_position,
        )
        boxes_header = Header()
        boxes_header.stamp = self.get_clock().now().to_msg()
        boxes_header.frame_id = self._last_input_frame
        self._publish_object_boxes((), boxes_header)
        response.success = True
        response.message = "Terrain state reset"
        return response

    def _publish_diagnostics(self) -> None:
        now_monotonic = time.monotonic()
        status = DiagnosticStatus()
        status.name = "lr_terrain_geometry/terrain_geometry"
        status.hardware_id = self._point_cloud_topic

        if self._last_receive_monotonic is None:
            status.level = DiagnosticStatus.WARN
            status.message = "Waiting for point cloud"
            receive_age = math.inf
        else:
            receive_age = now_monotonic - self._last_receive_monotonic
            if receive_age > self._stale_after_sec:
                status.level = DiagnosticStatus.WARN
                status.message = "Point cloud is stale"
            elif (
                self._object_filter_enabled
                and self._last_detection_receive_monotonic is None
            ):
                status.level = DiagnosticStatus.WARN
                status.message = "Waiting for Detection2DArray"
            elif (
                self._object_filter_enabled
                and now_monotonic - self._last_detection_receive_monotonic
                > self._stale_after_sec
            ):
                status.level = DiagnosticStatus.WARN
                status.message = "Detection2DArray is stale"
            elif (
                self._object_filter_enabled
                and self._last_instance_mask_receive_monotonic is None
            ):
                status.level = DiagnosticStatus.WARN
                status.message = "Waiting for instance masks"
            elif (
                self._object_filter_enabled
                and now_monotonic
                - self._last_instance_mask_receive_monotonic
                > self._stale_after_sec
            ):
                status.level = DiagnosticStatus.WARN
                status.message = "Instance masks are stale"
            elif self._last_error:
                status.level = DiagnosticStatus.WARN
                status.message = self._last_error
            else:
                status.level = DiagnosticStatus.OK
                status.message = "Terrain estimator running"

        elapsed = max(now_monotonic - self._started_monotonic, 1e-6)
        result = self._last_result
        values = {
            "input_topic": self._point_cloud_topic,
            "input_frame": self._last_input_frame,
            "map_frame": self._map_frame,
            "input_count": self._input_count,
            "processed_count": self._processed_count,
            "average_input_hz": f"{self._input_count / elapsed:.2f}",
            "cloud_age_sec": (
                "inf"
                if not math.isfinite(receive_age)
                else f"{receive_age:.3f}"
            ),
            "processing_ms": f"{self._last_processing_ms:.2f}",
            "processing_ms_ema": f"{self._processing_ms_ema:.2f}",
            "cells": result.cell_count,
            "points": result.point_count,
            "accepted": len(result.accepted_planes),
            "rejected": len(result.rejected_planes),
            "pending": len(result.debug_cells),
            "tf_failures": self._tf_failure_count,
            "malformed_clouds": self._malformed_cloud_count,
            "object_filter_enabled": self._object_filter_enabled,
            "detections_topic": self._detections_topic,
            "instance_masks_topic": self._instance_masks_topic,
            "camera_info_topic": self._camera_info_topic,
            "detection_count": self._detection_count,
            "instance_mask_count": self._instance_mask_count,
            "synchronized_pairs": self._synchronized_pair_count,
            "pending_clouds": len(self._pending_clouds),
            "pending_detections": len(self._pending_detections),
            "pending_instance_masks": len(self._pending_instance_masks),
            "unmatched_clouds": self._unmatched_cloud_count,
            "unmatched_detections": self._unmatched_detection_count,
            "unmatched_instance_masks": (
                self._unmatched_instance_mask_count
            ),
            "malformed_detections": self._malformed_detection_count,
            "malformed_instance_masks": (
                self._malformed_instance_mask_count
            ),
            "filter_failures": self._filter_failure_count,
            "last_object_boxes": self._last_box_count,
            "last_removed_points": self._last_removed_point_count,
            "total_removed_points": self._total_removed_point_count,
            "last_reset_reason": self._estimator.last_reset_reason or "",
        }
        status.values = [
            KeyValue(key=str(key), value=str(value))
            for key, value in values.items()
        ]
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [status]
        self._diagnostic_publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    executor = None
    try:
        node = TerrainGeometryNode()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            try:
                node.destroy_node()
            except KeyboardInterrupt:
                pass
        if rclpy.ok():
            rclpy.shutdown()
