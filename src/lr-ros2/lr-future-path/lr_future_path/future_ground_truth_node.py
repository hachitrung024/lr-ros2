"""Publish a cached SVO look-ahead trajectory as nav_msgs/Path."""

from __future__ import annotations

from pathlib import Path
import math
import time

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as PathMessage
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)

from .bag_io import (
    load_pose_series,
    nanoseconds_to_stamp,
    stamp_to_nanoseconds,
)
from .cache import cache_is_valid, load_manifest
from .trajectory import normalize_quaternion, reanchor_pose


class FutureGroundTruthNode(Node):
    """Align cached relative future poses to each current ZED map pose."""

    def __init__(self, *, pose_series=None, **node_kwargs) -> None:
        """Load a cache and configure the runtime pose-to-path pipeline."""
        super().__init__("future_ground_truth", **node_kwargs)
        cache_path_value = str(
            self.declare_parameter("cache_path", "").value
        )
        cache_path = Path(cache_path_value).expanduser()
        input_topic = str(
            self.declare_parameter(
                "input_pose_topic", "/zed/zed_node/pose"
            ).value
        )
        output_topic = str(
            self.declare_parameter(
                "output_topic", "/lr/future_path/ground_truth"
            ).value
        )
        self._radius_m = float(
            self.declare_parameter("radius_m", 15.0).value
        )
        self._step_m = float(
            self.declare_parameter("step_m", 0.2).value
        )
        horizon_s = float(
            self.declare_parameter("horizon_s", 20.0).value
        )
        max_gap_s = float(
            self.declare_parameter("max_gap_s", 1.0).value
        )
        alignment_tolerance_s = float(
            self.declare_parameter(
                "alignment_tolerance_s", 0.1
            ).value
        )
        self._max_points = int(
            self.declare_parameter("max_points", 1000).value
        )
        if not all(
            math.isfinite(value)
            for value in (self._radius_m, self._step_m, horizon_s)
        ) or any(value <= 0.0 for value in (self._radius_m, self._step_m, horizon_s)):
            raise ValueError("radius_m, step_m, and horizon_s must be positive")
        if max_gap_s <= 0.0 or alignment_tolerance_s <= 0.0:
            raise ValueError(
                "max_gap_s and alignment_tolerance_s must be positive"
            )
        if self._max_points < 1:
            raise ValueError("max_points must be at least one")

        if pose_series is None:
            if not cache_path_value:
                raise ValueError("cache_path is required")
            if not cache_is_valid(cache_path):
                raise ValueError(f"future-path cache is invalid: {cache_path}")
            manifest = load_manifest(cache_path)
            self._series = load_pose_series(cache_path)
            expected_count = int(manifest["pose_count"])
            if len(self._series.stamps_ns) != expected_count:
                raise ValueError(
                    "cache pose count does not match its manifest"
                )
        else:
            self._series = pose_series
        self._max_gap_ns = int(round(max_gap_s * 1_000_000_000))
        self._horizon_ns = int(round(horizon_s * 1_000_000_000))
        self._alignment_tolerance_ns = int(
            round(alignment_tolerance_s * 1_000_000_000)
        )
        self._last_warning_time = 0.0

        output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._publisher = self.create_publisher(
            PathMessage, output_topic, output_qos
        )
        self._subscription = self.create_subscription(
            PoseStamped,
            input_topic,
            self._pose_callback,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            "Future ground truth: input=%s output=%s cache_poses=%d "
            "distance=%.1f m horizon=%.1f s step=%.2f m"
            % (
                input_topic,
                output_topic,
                len(self._series.stamps_ns),
                self._radius_m,
                horizon_s,
                self._step_m,
            )
        )

    def _warn_alignment(self, timestamp: int) -> None:
        now = time.monotonic()
        if now - self._last_warning_time < 5.0:
            return
        self._last_warning_time = now
        self.get_logger().warning(
            "No cached pose within %.3f s of runtime timestamp %.9f"
            % (
                self._alignment_tolerance_ns / 1e9,
                timestamp / 1e9,
            )
        )

    def _pose_callback(self, current: PoseStamped) -> None:
        timestamp = stamp_to_nanoseconds(current.header.stamp)
        current_index = self._series.nearest_index(
            timestamp, self._alignment_tolerance_ns
        )
        if current_index is None:
            self._warn_alignment(timestamp)
            return
        try:
            runtime_position = np.array(
                [
                    current.pose.position.x,
                    current.pose.position.y,
                    current.pose.position.z,
                ],
                dtype=np.float64,
            )
            runtime_orientation = normalize_quaternion(
                np.array(
                    [
                        current.pose.orientation.x,
                        current.pose.orientation.y,
                        current.pose.orientation.z,
                        current.pose.orientation.w,
                    ],
                    dtype=np.float64,
                )
            )
            indices = self._series.future_indices(
                current_index,
                radius_m=self._radius_m,
                step_m=self._step_m,
                max_gap_ns=self._max_gap_ns,
                max_points=self._max_points,
                max_horizon_ns=self._horizon_ns,
            )
            message = self._build_path(
                current,
                current_index,
                indices,
                runtime_position,
                runtime_orientation,
            )
        except (IndexError, ValueError) as exception:
            self.get_logger().error(f"Cannot build future path: {exception}")
            return
        self._publisher.publish(message)

    def _build_path(
        self,
        current: PoseStamped,
        current_index: int,
        indices: list[int],
        runtime_position: np.ndarray,
        runtime_orientation: np.ndarray,
    ) -> PathMessage:
        message = PathMessage()
        message.header = current.header
        if not message.header.frame_id:
            message.header.frame_id = self._series.frame_id
        cached_position = self._series.positions[current_index]
        cached_orientation = self._series.orientations_xyzw[current_index]
        for offset, index in enumerate(indices):
            position, orientation = reanchor_pose(
                cached_position,
                cached_orientation,
                self._series.positions[index],
                self._series.orientations_xyzw[index],
                runtime_position,
                runtime_orientation,
            )
            pose = PoseStamped()
            pose.header.frame_id = message.header.frame_id
            if offset == 0:
                pose.header.stamp = current.header.stamp
            else:
                nanoseconds_to_stamp(
                    int(self._series.stamps_ns[index]),
                    pose.header.stamp,
                )
            pose.pose.position.x = float(position[0])
            pose.pose.position.y = float(position[1])
            pose.pose.position.z = float(position[2])
            pose.pose.orientation.x = float(orientation[0])
            pose.pose.orientation.y = float(orientation[1])
            pose.pose.orientation.z = float(orientation[2])
            pose.pose.orientation.w = float(orientation[3])
            message.poses.append(pose)
        return message


def main(args=None) -> None:
    """Run the future ground-truth publisher."""
    rclpy.init(args=args)
    node = None
    try:
        node = FutureGroundTruthNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
