#!/usr/bin/env python3
"""Explicit empty /tracked_objects bridge for pipeline smoke tests ONLY.

This is NOT a real object detector. Enable only with:
  enable_empty_objects:=true

Publishes TrackedObjectArray with objects=[] on each /trajectory so Prediction
treats objects as available-and-empty (valid static/dynamic readiness).
"""

from __future__ import annotations

from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from safety_perception_msgs.msg import TrackedObjectArray, Trajectory


class EmptyObjectsBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("empty_objects_bridge_node")
        self.declare_parameter("enable_empty_objects", False)
        self.declare_parameter("trajectory_topic", "/trajectory")
        self.declare_parameter("output_topic", "/tracked_objects")
        self.declare_parameter("force_frame_id_map", True)

        enabled = bool(self.get_parameter("enable_empty_objects").value)
        if not enabled:
            self.get_logger().error(
                "empty_objects_bridge_node started with enable_empty_objects:=false; "
                "refusing to publish. This node is smoke-test only."
            )
            raise SystemExit(1)

        self._force_map = bool(self.get_parameter("force_frame_id_map").value)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._pub = self.create_publisher(
            TrackedObjectArray, self.get_parameter("output_topic").value, qos
        )
        self.create_subscription(
            Trajectory,
            self.get_parameter("trajectory_topic").value,
            self._on_trajectory,
            qos,
        )
        self.get_logger().warn(
            "SMOKE ONLY: publishing empty TrackedObjectArray on each /trajectory. "
            "Replace with a real tracked-object adapter before production."
        )

    def _on_trajectory(self, traj: Trajectory) -> None:
        msg = TrackedObjectArray()
        msg.header.stamp = traj.header.stamp
        frame = traj.header.frame_id.strip() or "map"
        msg.header.frame_id = "map" if self._force_map else frame
        msg.objects = []
        self._pub.publish(msg)


def main(argv: list[str] | None = None) -> None:
    import rclpy

    rclpy.init(args=argv)
    node = EmptyObjectsBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
