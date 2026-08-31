"""ROS 2 node wrapping the NumPy terrain geometry estimator."""

from __future__ import annotations

import math
import time
from dataclasses import fields
from threading import RLock

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from grid_map_msgs.msg import GridMap
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import JumpThreshold
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Image, PointCloud2
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import MarkerArray

from .conversions import (
    grid_map_to_heatmap_image,
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


def _stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class TerrainGeometryNode(Node):
    """Estimate local terrain planes from a stamped ROS PointCloud2."""

    def __init__(
        self,
        *,
        parameter_overrides=None,
        node_name="terrain_geometry",
        namespace="",
    ) -> None:
        super().__init__(
            node_name,
            namespace=namespace,
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

        self._validate_node_parameters()

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
        self._cloud_subscription = self.create_subscription(
            PointCloud2,
            self._point_cloud_topic,
            self._on_point_cloud,
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
        self._last_success_monotonic = None
        self._last_warn_monotonic = 0.0
        self._last_processing_ms = 0.0
        self._processing_ms_ema = 0.0
        self._input_count = 0
        self._processed_count = 0
        self._tf_failure_count = 0
        self._malformed_cloud_count = 0
        self._last_error = ""
        self._last_input_frame = ""
        self._last_rover_position = np.zeros(3, dtype=np.float64)
        self._last_result = TerrainResult((), (), (), False, True, None, 0, 0)
        self._state_lock = RLock()
        # tf2_ros.Buffer in Humble Python does not reset itself after an SVO
        # seek. Terrain updates run in a multi-threaded executor, so protect
        # the estimator and TF cache while replacing the old timeline.
        self._time_jump_handle = self.get_clock().create_jump_callback(
            JumpThreshold(
                min_forward=None,
                min_backward=Duration(nanoseconds=-1),
                on_clock_change=True,
            ),
            post_callback=self._on_time_jump,
        )

        self.get_logger().info(
            f"Terrain geometry listening on {self._point_cloud_topic}; "
            f"map frame is {self._map_frame}"
        )

    def _validate_node_parameters(self) -> None:
        if not self._point_cloud_topic:
            raise ValueError("input.point_cloud_topic must not be empty")
        if not self._map_frame:
            raise ValueError("frames.map_frame must not be empty")
        for name, value in (
            ("tf.lookup_timeout_sec", self._tf_timeout_sec),
            ("diagnostics.period_sec", self._diagnostic_period_sec),
            ("diagnostics.stale_after_sec", self._stale_after_sec),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(
                    f"{name} must be finite and greater than zero"
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

        with self._state_lock:
            self._process_point_cloud(
                message,
                timestamp,
                callback_started,
            )

    def _on_time_jump(self, _time_jump) -> None:
        """Clear terrain state when SVO playback moves to a new timeline."""
        with self._state_lock:
            self._tf_buffer.clear()
            self._estimator.reset("ROS time changed")
            self._last_result = TerrainResult(
                (), (), (), False, True, "ROS time changed", 0, 0
            )
            self._last_error = ""
            self._last_rover_position.fill(0.0)
            self._publish_snapshot(
                self._last_result,
                self.get_clock().now().to_msg(),
                self._last_rover_position,
            )
        self.get_logger().info(
            "ROS time changed; cleared terrain TF cache and estimator state."
        )

    def _process_point_cloud(
        self,
        message: PointCloud2,
        timestamp: float,
        callback_started: float,
    ) -> None:

        try:
            transform = self._tf_buffer.lookup_transform(
                self._map_frame,
                message.header.frame_id,
                Time.from_msg(message.header.stamp),
                timeout=Duration(seconds=self._tf_timeout_sec),
            )
            sensor_to_map = transform_to_matrix(transform.transform)
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

        try:
            points = point_cloud_to_xyz(message)
        except (TypeError, ValueError) as error:
            self._record_malformed(str(error))
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
        with self._state_lock:
            self._estimator.reset("manual reset")
            self._last_result = TerrainResult(
                (), (), (), False, True, "manual reset", 0, 0
            )
            self._publish_snapshot(
                self._last_result,
                self.get_clock().now().to_msg(),
                self._last_rover_position,
            )
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
