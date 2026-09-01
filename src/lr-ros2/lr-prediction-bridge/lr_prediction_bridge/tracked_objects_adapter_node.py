#!/usr/bin/env python3
"""Adapt terrain Detection3DArray boxes to canonical /tracked_objects."""

from __future__ import annotations

from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from safety_perception_msgs.msg import Point2D, TrackedObject, TrackedObjectArray
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection3DArray

from lr_prediction_bridge.detection3d_conversion import (
    convert_detection3d_array,
    rotation_matrix_from_tf,
    stamp_to_ns,
    translation_from_tf,
)


class TrackedObjectsAdapterNode(Node):
    def __init__(self) -> None:
        super().__init__("tracked_objects_adapter_node")
        self.declare_parameter(
            "input_topic",
            "/segmentation/boxes_3d",
        )
        self.declare_parameter("output_topic", "/tracked_objects")
        self.declare_parameter("expected_frame_id", "map")
        self.declare_parameter("tf_timeout_sec", 0.2)

        self._expected_frame = str(self.get_parameter("expected_frame_id").value)
        self._tf_timeout_sec = float(self.get_parameter("tf_timeout_sec").value)
        self._tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._received_messages = 0
        self._empty_messages = 0
        self._non_empty_messages = 0
        self._total_objects = 0
        self._skipped_invalid = 0
        self._tf_failures = 0

        output_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        # The segmentation box estimator is a sensor-data publisher.
        input_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._pub = self.create_publisher(
            TrackedObjectArray,
            self.get_parameter("output_topic").value,
            output_qos,
        )
        self.create_subscription(
            Detection3DArray,
            self.get_parameter("input_topic").value,
            self._on_detections,
            input_qos,
        )
        self.get_logger().info(
            "tracked objects adapter: "
            f"{self.get_parameter('input_topic').value} → "
            f"{self.get_parameter('output_topic').value} "
            f"(frame={self._expected_frame}, NON-PERSISTENT frame-local track_id)"
        )

    def _on_detections(self, message: Detection3DArray) -> None:
        self._received_messages += 1
        stamp_ns = stamp_to_ns(
            message.header.stamp.sec,
            message.header.stamp.nanosec,
        )
        source_frame = message.header.frame_id.strip() or self._expected_frame

        transform_rotation = None
        transform_translation = None
        if source_frame != self._expected_frame:
            try:
                transform = self._tf_buffer.lookup_transform(
                    self._expected_frame,
                    source_frame,
                    Time.from_msg(message.header.stamp),
                    timeout=Duration(seconds=self._tf_timeout_sec),
                )
                transform_rotation = rotation_matrix_from_tf(transform.transform)
                transform_translation = translation_from_tf(transform.transform)
            except TransformException as error:
                self._tf_failures += 1
                self.get_logger().warning(
                    f"TF {self._expected_frame} <- {source_frame} failed: {error}"
                )
                return

        converted, stats = convert_detection3d_array(
            message.detections,
            stamp_ns,
            transform_rotation=transform_rotation,
            transform_translation=transform_translation,
        )
        self._skipped_invalid += stats.skipped_invalid

        output = TrackedObjectArray()
        output.header.stamp = message.header.stamp
        output.header.frame_id = self._expected_frame
        output.objects = [self._to_ros_object(item) for item in converted]
        self._pub.publish(output)

        if output.objects:
            self._non_empty_messages += 1
            self._total_objects += len(output.objects)
        else:
            self._empty_messages += 1

        if stats.skipped_invalid:
            self.get_logger().debug(
                "skipped invalid detections: "
                f"{stats.skipped_invalid}/{stats.received_detections}"
            )

    @staticmethod
    def _to_ros_object(item) -> TrackedObject:
        tracked = TrackedObject()
        tracked.track_id = int(item.track_id)
        tracked.class_name = item.class_name
        tracked.confidence = float(item.confidence)
        tracked.confidence_valid = bool(item.confidence_valid)
        tracked.velocity_valid = False
        tracked.velocity.x = 0.0
        tracked.velocity.y = 0.0
        tracked.velocity.z = 0.0
        tracked.footprint_polygon_xy = [
            Point2D(x=float(x), y=float(y)) for x, y in item.footprint_polygon_xy
        ]
        return tracked


def main(argv: list[str] | None = None) -> None:
    import rclpy

    rclpy.init(args=argv)
    node = TrackedObjectsAdapterNode()
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
