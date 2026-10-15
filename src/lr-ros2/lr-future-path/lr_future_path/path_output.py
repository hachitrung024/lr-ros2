"""Publish a smoothed, latched path with bounded retention across input gaps."""

import math

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.clock import JumpThreshold
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from .bag_io import nanoseconds_to_stamp, stamp_to_nanoseconds
from .path_processing import StablePathProcessor


class PathOutput:
    """Share source-path processing without changing source pose or TF outputs."""

    def __init__(self, node, topic, *, radius_m, step_m, max_points=256):
        """Create a durable output and use ROS time for display retention."""
        self._node = node
        self._radius = radius_m
        self._step = step_m
        self._max_points = min(max_points, 256)
        self._window = float(node.declare_parameter("smoothing_window_m", 0.8).value)
        self._deviation = float(node.declare_parameter("smoothing_max_deviation_m", 0.2).value)
        self._hold = float(node.declare_parameter("display_hold_s", 1.5).value)
        if not all(math.isfinite(v) and v >= 0 for v in
                   (self._window, self._deviation, self._hold)):
            raise ValueError("path smoothing and display hold must be finite and non-negative")
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._publisher = node.create_publisher(Path, topic, qos)
        self._last_receive_ns = None
        self._processor = StablePathProcessor()
        self._frame = "map"
        self._timer = node.create_timer(0.25, self._expire)
        self._jump = node.get_clock().create_jump_callback(
            JumpThreshold(min_forward=None, min_backward=Duration(nanoseconds=-1),
                          on_clock_change=True),
            post_callback=self._reset,
        )

    def publish(self, path):
        """Process one recorded path, retaining its source timestamps and frame."""
        if path.header.frame_id != self._frame:
            self._processor.reset()
        stamps, xyz, quaternions = self._processor.process(
            [stamp_to_nanoseconds(p.header.stamp) for p in path.poses],
            [[p.pose.position.x, p.pose.position.y, p.pose.position.z] for p in path.poses],
            [[p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z,
              p.pose.orientation.w] for p in path.poses],
            radius_m=self._radius, step_m=self._step,
            max_points=self._max_points,
            smoothing_window_m=self._window, max_deviation_m=self._deviation,
        )
        if len(stamps) < 2 and self._last_receive_ns is not None:
            # A transient one-point look-ahead cannot draw a line. Keep the last
            # usable path for the bounded hold interval, with its original stamp.
            self._expire()
            return
        output = Path(header=path.header)
        for stamp, position, quaternion in zip(stamps, xyz, quaternions):
            pose = PoseStamped()
            pose.header.frame_id = path.header.frame_id
            nanoseconds_to_stamp(int(stamp), pose.header.stamp)
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, position)
            (pose.pose.orientation.x, pose.pose.orientation.y,
             pose.pose.orientation.z, pose.pose.orientation.w) = map(float, quaternion)
            output.poses.append(pose)
        self._frame = path.header.frame_id
        self._last_receive_ns = (
            self._node.get_clock().now().nanoseconds if len(stamps) > 1 else None
        )
        self._publisher.publish(output)

    def _expire(self):
        if self._last_receive_ns is not None and (
            self._node.get_clock().now().nanoseconds - self._last_receive_ns > self._hold * 1e9
        ):
            self._reset(None)

    def _reset(self, _jump):
        self._processor.reset()
        if self._last_receive_ns is None:
            return
        self._last_receive_ns = None
        message = Path()
        message.header.frame_id = self._frame
        message.header.stamp = self._node.get_clock().now().to_msg()
        self._publisher.publish(message)
