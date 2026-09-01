#!/usr/bin/env python3
"""Convert the LR future Path into the canonical prediction trajectory."""

from __future__ import annotations

import math

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from safety_perception_msgs.msg import Trajectory, TrajectoryStep

from lr_prediction_bridge.helpers import subsample_indices, yaw_from_quaternion


class TrajectoryAdapterNode(Node):
    def __init__(self) -> None:
        super().__init__("trajectory_adapter_node")
        self.declare_parameter("input_topic", "/lr/future_path/ground_truth")
        self.declare_parameter("output_topic", "/trajectory")
        self.declare_parameter("expected_frame_id", "map")
        self.declare_parameter("horizon_steps", 20)
        self.declare_parameter("path_stride", 1)
        # If Path stamps allow, prefer ~output_dt_sec spacing; 0 disables.
        self.declare_parameter("output_dt_sec", 0.25)
        self.declare_parameter("force_frame_id_map", True)
        # Skip poses closer than this to the path origin so steps land in the
        # terrain forward coverage zone (matches terrain.min_forward_m default).
        self.declare_parameter("min_distance_from_start_m", 1.0)
        # PredictionCore consumes a trajectory as one evaluation cycle.  Do
        # not create cycles at the much higher Path visualization rate.
        self.declare_parameter("minimum_cycle_period_sec", 0.25)

        self._horizon_steps = int(self.get_parameter("horizon_steps").value)
        self._path_stride = int(self.get_parameter("path_stride").value)
        self._output_dt_sec = float(self.get_parameter("output_dt_sec").value)
        self._expected_frame = str(self.get_parameter("expected_frame_id").value)
        self._force_map = bool(self.get_parameter("force_frame_id_map").value)
        self._min_distance_from_start_m = float(
            self.get_parameter("min_distance_from_start_m").value
        )
        self._minimum_cycle_period_sec = float(
            self.get_parameter("minimum_cycle_period_sec").value
        )
        if self._minimum_cycle_period_sec < 0.0:
            raise ValueError("minimum_cycle_period_sec must be non-negative")
        self._trajectory_id = 0
        self._last_cycle_stamp_sec: float | None = None

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )
        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self._pub = self.create_publisher(Trajectory, output_topic, qos)
        self.create_subscription(Path, input_topic, self._on_path, qos)
        self.get_logger().info(
            f"trajectory adapter: {input_topic} → {output_topic} "
            f"(horizon_steps={self._horizon_steps}, output_dt_sec={self._output_dt_sec})"
        )

    def _on_path(self, msg: Path) -> None:
        if not msg.poses:
            self.get_logger().warn("ignoring empty Path")
            return

        stamp_sec = (
            float(msg.header.stamp.sec)
            + float(msg.header.stamp.nanosec) * 1e-9
        )
        if stamp_sec <= 0.0:
            stamp_sec = self.get_clock().now().nanoseconds * 1e-9
        if self._last_cycle_stamp_sec is not None:
            elapsed = stamp_sec - self._last_cycle_stamp_sec
            if 0.0 <= elapsed < self._minimum_cycle_period_sec:
                return
            if elapsed < 0.0:
                self.get_logger().info(
                    "ROS time moved backward; resetting trajectory throttle"
                )

        frame_id = msg.header.frame_id.strip() or self._expected_frame
        if self._force_map:
            frame_id = "map"
        if frame_id != self._expected_frame:
            self.get_logger().warn(
                f"Path frame_id={msg.header.frame_id!r} ≠ expected {self._expected_frame!r}"
            )

        start_idx = self._first_index_beyond_min_distance(msg.poses)
        if start_idx is None:
            self.get_logger().warn(
                "no path pose beyond min_distance_from_start_m="
                f"{self._min_distance_from_start_m:.2f}; skipping trajectory"
            )
            return

        indices = self._select_indices(msg.poses, start_idx=start_idx)
        steps: list[TrajectoryStep] = []
        for step_id, pose_idx in enumerate(indices):
            pose: PoseStamped = msg.poses[pose_idx]
            q = pose.pose.orientation
            yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
            if not all(
                math.isfinite(v)
                for v in (pose.pose.position.x, pose.pose.position.y, yaw)
            ):
                self.get_logger().warn(f"skipping non-finite pose index={pose_idx}")
                continue
            steps.append(
                TrajectoryStep(
                    step_id=step_id,
                    x=float(pose.pose.position.x),
                    y=float(pose.pose.position.y),
                    yaw=float(yaw),
                )
            )
        if not steps:
            self.get_logger().warn("no valid steps after conversion")
            return

        self._trajectory_id += 1
        out = Trajectory()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = frame_id
        out.trajectory_id = self._trajectory_id
        out.steps = steps
        self._pub.publish(out)
        self._last_cycle_stamp_sec = stamp_sec

    def _first_index_beyond_min_distance(
        self, poses: list[PoseStamped]
    ) -> int | None:
        min_d = self._min_distance_from_start_m
        if min_d <= 0.0 or len(poses) <= 1:
            return 0
        origin = poses[0].pose.position
        for index, pose in enumerate(poses):
            if index == 0:
                continue
            dx = float(pose.pose.position.x - origin.x)
            dy = float(pose.pose.position.y - origin.y)
            if math.hypot(dx, dy) >= min_d:
                return index
        return None

    def _select_indices(
        self, poses: list[PoseStamped], *, start_idx: int = 0
    ) -> list[int]:
        subset = poses[start_idx:]
        n = len(subset)
        if n <= 0:
            return []
        # Prefer time-based stride when stamps are monotonic and output_dt_sec > 0.
        if self._output_dt_sec > 0.0 and n >= 2:
            t0 = subset[0].header.stamp.sec + subset[0].header.stamp.nanosec * 1e-9
            timed: list[int] = [0]
            next_t = t0 + self._output_dt_sec
            for i in range(1, n):
                ti = subset[i].header.stamp.sec + subset[i].header.stamp.nanosec * 1e-9
                if ti + 1e-9 >= next_t:
                    timed.append(i)
                    next_t = ti + self._output_dt_sec
                if len(timed) >= self._horizon_steps:
                    break
            if timed[-1] != n - 1 and len(timed) < self._horizon_steps:
                timed.append(n - 1)
            if len(timed) >= 2:
                # Detect collapsed stamps (all equal) → fall back to index stride.
                t_last = (
                    subset[timed[-1]].header.stamp.sec
                    + subset[timed[-1]].header.stamp.nanosec * 1e-9
                )
                if abs(t_last - t0) > 1e-6:
                    return [start_idx + idx for idx in timed[: self._horizon_steps]]
        local = subsample_indices(
            n, horizon_steps=self._horizon_steps, stride=self._path_stride
        )
        return [start_idx + idx for idx in local]


def main(argv: list[str] | None = None) -> None:
    import rclpy

    rclpy.init(args=argv)
    node = TrajectoryAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
