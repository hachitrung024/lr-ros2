#!/usr/bin/env python3
"""Build canonical rover state from the LR MAVLink PoseStamped stream."""

from __future__ import annotations

import math
from collections import deque

from geometry_msgs.msg import Accel, Pose, PoseStamped, Twist
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from safety_perception_msgs.msg import RoverState

from lr_prediction_bridge.helpers import finite_difference


class RoverStateAdapterNode(Node):
    """Estimate map-frame velocity and kinematic acceleration from poses."""

    def __init__(self) -> None:
        """Connect the MAVLink pose input to the canonical state output."""
        super().__init__("rover_state_adapter_node")
        self.declare_parameter("pose_topic", "/lr/mavlink/pose")
        self.declare_parameter("output_topic", "/rover/state")
        self.declare_parameter("expected_frame_id", "map")
        self.declare_parameter("force_frame_id_map", True)
        self.declare_parameter("max_dt_sec", 1.0)
        self.declare_parameter("min_dt_sec", 1e-3)

        self._expected_frame = str(self.get_parameter("expected_frame_id").value)
        self._force_map = bool(self.get_parameter("force_frame_id_map").value)
        self._max_dt = float(self.get_parameter("max_dt_sec").value)
        self._min_dt = float(self.get_parameter("min_dt_sec").value)
        # (time, x, y, z, qx, qy, qz, qw)
        self._history: deque[tuple[float, ...]] = deque(maxlen=3)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )
        self._pub = self.create_publisher(
            RoverState, self.get_parameter("output_topic").value, qos
        )
        self.create_subscription(
            PoseStamped,
            self.get_parameter("pose_topic").value,
            self._on_pose,
            qos,
        )
        self.get_logger().info(
            "rover state adapter: /lr/mavlink/pose → /rover/state "
            "(finite-difference velocity/acceleration, gravity excluded)"
        )

    def _on_pose(self, message: PoseStamped) -> None:
        stamp = message.header.stamp
        t = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        position = message.pose.position
        orientation = message.pose.orientation
        sample = (
            t,
            float(position.x),
            float(position.y),
            float(position.z),
            float(orientation.x),
            float(orientation.y),
            float(orientation.z),
            float(orientation.w),
        )
        if not all(math.isfinite(value) for value in sample):
            self.get_logger().warn("dropping non-finite pose sample")
            return
        if self._history and t <= self._history[-1][0]:
            self._history.clear()
        self._history.append(sample)

        state = RoverState()
        state.header.stamp = stamp
        state.header.frame_id = (
            "map"
            if self._force_map
            else (message.header.frame_id.strip() or self._expected_frame)
        )
        state.pose = Pose()
        state.pose.position = position
        state.pose.orientation = orientation
        state.pose_valid = True
        state.twist = Twist()
        state.twist_valid = False
        state.acceleration = Accel()
        state.acceleration_valid = False

        latest_velocity = None
        if len(self._history) >= 2:
            previous = self._history[-2]
            latest = self._history[-1]
            dt = latest[0] - previous[0]
            if self._min_dt <= dt <= self._max_dt:
                latest_velocity = finite_difference(
                    previous[1:4], latest[1:4], dt
                )
                state.twist.linear.x = latest_velocity[0]
                state.twist.linear.y = latest_velocity[1]
                state.twist.linear.z = latest_velocity[2]
                state.twist_valid = True

        if len(self._history) >= 3 and latest_velocity is not None:
            first, middle, latest = self._history
            dt_first = middle[0] - first[0]
            dt_latest = latest[0] - middle[0]
            if (
                self._min_dt <= dt_first <= self._max_dt
                and self._min_dt <= dt_latest <= self._max_dt
            ):
                previous_velocity = finite_difference(
                    first[1:4], middle[1:4], dt_first
                )
                center_dt = 0.5 * (dt_first + dt_latest)
                acceleration = finite_difference(
                    previous_velocity, latest_velocity, center_dt
                )
                state.acceleration.linear.x = acceleration[0]
                state.acceleration.linear.y = acceleration[1]
                state.acceleration.linear.z = acceleration[2]
                state.acceleration_valid = True

        self._pub.publish(state)


def main(argv: list[str] | None = None) -> None:
    """Run the rover-state adapter."""
    import rclpy

    rclpy.init(args=argv)
    node = RoverStateAdapterNode()
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
