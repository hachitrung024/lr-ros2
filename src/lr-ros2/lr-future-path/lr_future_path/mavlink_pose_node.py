"""Publish offline MAVLink pose, TF, and optional future ground truth."""

from __future__ import annotations

from pathlib import Path
import sys
import time
from typing import Callable

from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path as PathMessage
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rosgraph_msgs.msg import Clock
from tf2_ros import TransformBroadcaster

from .bag_io import nanoseconds_to_stamp
from .mavlink import (
    MavlinkDataError,
    MavlinkTrajectory,
    PoseSample,
    SessionSummary,
    apply_body_to_camera,
    load_trajectory,
    select_session,
)


class MavlinkPoseNode(Node):
    """Drive the ZED camera frame from a timestamped MAVLink SQLite log."""

    def __init__(
        self,
        *,
        trajectory: MavlinkTrajectory | None = None,
        session_selector: Callable[..., SessionSummary] = select_session,
        trajectory_loader: Callable[[Path], MavlinkTrajectory] = (
            load_trajectory
        ),
        **node_kwargs,
    ) -> None:
        """Configure session matching and ROS outputs."""
        super().__init__("mavlink_pose", **node_kwargs)
        self._mavlink_dir = str(
            self.declare_parameter("mavlink_dir", "mavlink").value
        )
        self._database_path = str(
            self.declare_parameter("mavlink_db_path", "").value
        )
        pose_topic = str(
            self.declare_parameter(
                "pose_topic", "/lr/mavlink/pose"
            ).value
        )
        self._future_enabled = bool(
            self.declare_parameter("future_path_enabled", False).value
        )
        future_topic = str(
            self.declare_parameter(
                "future_path_topic", "/lr/future_path/ground_truth"
            ).value
        )
        self._map_frame = str(
            self.declare_parameter("map_frame", "map").value
        ).strip()
        self._child_frame = str(
            self.declare_parameter(
                "child_frame", "zed_camera_link"
            ).value
        ).strip()
        match_tolerance_s = float(
            self.declare_parameter("match_tolerance_s", 60.0).value
        )
        max_gps_gap_s = float(
            self.declare_parameter("max_gps_gap_s", 1.5).value
        )
        self._radius_m = float(
            self.declare_parameter("radius_m", 15.0).value
        )
        self._step_m = float(
            self.declare_parameter("step_m", 0.2).value
        )
        self._max_points = int(
            self.declare_parameter("max_points", 1000).value
        )
        self._body_to_camera = np.asarray(
            self.declare_parameter(
                "body_to_camera", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            ).value,
            dtype=np.float64,
        )
        if not self._map_frame or not self._child_frame:
            raise ValueError("map_frame and child_frame must not be empty")
        if match_tolerance_s < 0.0 or max_gps_gap_s <= 0.0:
            raise ValueError(
                "match_tolerance_s must be non-negative and "
                "max_gps_gap_s must be positive"
            )
        if self._radius_m <= 0.0 or self._step_m <= 0.0:
            raise ValueError("radius_m and step_m must be positive")
        if self._max_points < 1:
            raise ValueError("max_points must be at least one")
        if (
            self._body_to_camera.shape != (6,)
            or not np.all(np.isfinite(self._body_to_camera))
        ):
            raise ValueError(
                "body_to_camera must contain six finite XYZ/RPY values"
            )

        self._match_tolerance_ns = int(
            round(match_tolerance_s * 1_000_000_000)
        )
        self._max_gps_gap_ns = int(
            round(max_gps_gap_s * 1_000_000_000)
        )
        self._edge_tolerance_ns = 1_000_000_000
        self._session_selector = session_selector
        self._trajectory_loader = trajectory_loader
        self._trajectory = trajectory
        self._fatal_error: str | None = None
        self._last_gap_warning_time = 0.0

        output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        clock_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self._pose_publisher = self.create_publisher(
            PoseStamped, pose_topic, output_qos
        )
        self._future_publisher = (
            self.create_publisher(PathMessage, future_topic, output_qos)
            if self._future_enabled
            else None
        )
        self._tf_broadcaster = TransformBroadcaster(self)
        self._clock_subscription = self.create_subscription(
            Clock, "/clock", self._clock_callback, clock_qos
        )
        if self._trajectory is not None:
            self._log_selected_trajectory(self._trajectory)
        else:
            source = self._database_path or self._mavlink_dir
            self.get_logger().info(
                "Waiting for the first SVO /clock to match MAVLink in "
                f"{source}"
            )
        self.get_logger().info(
            "MAVLink pose output: frame=%s child=%s pose=%s future=%s"
            % (
                self._map_frame,
                self._child_frame,
                pose_topic,
                "enabled" if self._future_enabled else "disabled",
            )
        )

    @property
    def fatal_error(self) -> str | None:
        """Return a fatal asynchronous error for the process exit code."""
        return self._fatal_error

    def _load_matching_trajectory(self, stamp_ns: int) -> bool:
        try:
            summary = self._session_selector(
                stamp_ns,
                directory=self._mavlink_dir,
                explicit_path=self._database_path,
                tolerance_ns=self._match_tolerance_ns,
            )
            self._trajectory = self._trajectory_loader(summary.path)
        except (MavlinkDataError, OSError, ValueError) as exception:
            self._fail(str(exception))
            return False
        self._log_selected_trajectory(self._trajectory)
        return True

    def _log_selected_trajectory(
        self, trajectory: MavlinkTrajectory
    ) -> None:
        self.get_logger().info(
            "Selected MAVLink session: %s (%d GPS, %d attitude samples)"
            % (
                trajectory.path,
                len(trajectory.gps_stamps_ns),
                len(trajectory.attitude_stamps_ns),
            )
        )

    def _fail(self, message: str) -> None:
        if self._fatal_error is not None:
            return
        self._fatal_error = message
        self.get_logger().fatal(f"MAVLink pose node failed: {message}")

    def _warn_gap(self, stamp_ns: int) -> None:
        now = time.monotonic()
        if now - self._last_gap_warning_time < 5.0:
            return
        self._last_gap_warning_time = now
        self.get_logger().warning(
            "No valid MAVLink GPS interpolation at %.9f; suppressing "
            "pose, TF, and future path"
            % (stamp_ns / 1e9)
        )

    def _clock_callback(self, message: Clock) -> None:
        stamp_ns = (
            int(message.clock.sec) * 1_000_000_000
            + int(message.clock.nanosec)
        )
        if stamp_ns <= 0 or self._fatal_error is not None:
            return
        if self._trajectory is None and not self._load_matching_trajectory(
            stamp_ns
        ):
            return
        body_pose = self._trajectory.pose_at(
            stamp_ns,
            max_gps_gap_ns=self._max_gps_gap_ns,
            edge_tolerance_ns=self._edge_tolerance_ns,
        )
        if body_pose is None:
            self._warn_gap(stamp_ns)
            return
        camera_pose = apply_body_to_camera(
            body_pose, self._body_to_camera
        )
        self._pose_publisher.publish(self._pose_message(camera_pose))
        self._tf_broadcaster.sendTransform(self._transform(camera_pose))
        if self._future_publisher is not None:
            samples = self._trajectory.future_samples(
                stamp_ns,
                radius_m=self._radius_m,
                step_m=self._step_m,
                max_gps_gap_ns=self._max_gps_gap_ns,
                max_points=self._max_points,
                edge_tolerance_ns=self._edge_tolerance_ns,
            )
            if samples:
                self._future_publisher.publish(
                    self._path_message(samples)
                )

    def _pose_message(self, sample: PoseSample) -> PoseStamped:
        message = PoseStamped()
        message.header.frame_id = self._map_frame
        nanoseconds_to_stamp(sample.stamp_ns, message.header.stamp)
        self._copy_pose(sample, message)
        return message

    def _transform(self, sample: PoseSample) -> TransformStamped:
        message = TransformStamped()
        message.header.frame_id = self._map_frame
        message.child_frame_id = self._child_frame
        nanoseconds_to_stamp(sample.stamp_ns, message.header.stamp)
        message.transform.translation.x = float(sample.position[0])
        message.transform.translation.y = float(sample.position[1])
        message.transform.translation.z = float(sample.position[2])
        message.transform.rotation.x = float(sample.orientation_xyzw[0])
        message.transform.rotation.y = float(sample.orientation_xyzw[1])
        message.transform.rotation.z = float(sample.orientation_xyzw[2])
        message.transform.rotation.w = float(sample.orientation_xyzw[3])
        return message

    def _path_message(self, samples: list[PoseSample]) -> PathMessage:
        message = PathMessage()
        message.header.frame_id = self._map_frame
        nanoseconds_to_stamp(samples[0].stamp_ns, message.header.stamp)
        for body_sample in samples:
            camera_sample = apply_body_to_camera(
                body_sample, self._body_to_camera
            )
            pose = PoseStamped()
            pose.header.frame_id = self._map_frame
            nanoseconds_to_stamp(camera_sample.stamp_ns, pose.header.stamp)
            self._copy_pose(camera_sample, pose)
            message.poses.append(pose)
        return message

    @staticmethod
    def _copy_pose(sample: PoseSample, message: PoseStamped) -> None:
        message.pose.position.x = float(sample.position[0])
        message.pose.position.y = float(sample.position[1])
        message.pose.position.z = float(sample.position[2])
        message.pose.orientation.x = float(sample.orientation_xyzw[0])
        message.pose.orientation.y = float(sample.orientation_xyzw[1])
        message.pose.orientation.z = float(sample.orientation_xyzw[2])
        message.pose.orientation.w = float(sample.orientation_xyzw[3])


def main(args=None) -> None:
    """Run the MAVLink pose and TF publisher."""
    rclpy.init(args=args)
    node = None
    exit_code = 0
    try:
        node = MavlinkPoseNode()
        while rclpy.ok() and node.fatal_error is None:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.fatal_error is not None:
            exit_code = 1
    except (KeyboardInterrupt, ExternalShutdownException):
        if node is not None and node.fatal_error is not None:
            exit_code = 1
    except Exception as exception:  # Ensure launch sees initialization errors.
        print(f"MAVLink pose node failed: {exception}", file=sys.stderr)
        exit_code = 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if exit_code:
        raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
