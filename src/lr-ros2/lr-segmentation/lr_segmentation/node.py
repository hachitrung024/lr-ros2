"""ROS 2 node for RGB segmentation overlays and simple depth-based 3D boxes."""

from __future__ import annotations

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from visualization_msgs.msg import MarkerArray

from .conversions import bgr_to_image_message, image_message_to_bgr
from .depth_boxes import (
    boxes_to_markers,
    depth_message_to_meters,
    estimate_boxes_3d,
)
from .model import create_segmentation_model


def _stamp_nanoseconds(message) -> int:
    return (
        int(message.header.stamp.sec) * 1_000_000_000
        + int(message.header.stamp.nanosec)
    )


class SegmentationNode(Node):
    """Synchronize RGB and depth, then publish overlay and 3D box markers."""

    def __init__(
        self,
        *,
        parameter_overrides=None,
        model_factory=create_segmentation_model,
    ) -> None:
        super().__init__("segmentation", parameter_overrides=parameter_overrides)

        image_topic = self.declare_parameter(
            "input.image_topic",
            "/zed/zed_node/rgb/color/rect/image",
        ).value
        depth_topic = self.declare_parameter(
            "input.depth_topic",
            "/zed/zed_node/depth/depth_registered",
        ).value
        camera_info_topic = self.declare_parameter(
            "input.camera_info_topic",
            "/zed/zed_node/rgb/color/rect/camera_info",
        ).value
        model_path = self.declare_parameter("model.path", "").value
        device = self.declare_parameter("model.device", "0").value
        confidence = self.declare_parameter("model.confidence", 0.25).value
        image_size = self.declare_parameter("model.image_size", 960).value
        iou = self.declare_parameter("model.iou", 0.60).value
        self._sync_tolerance_ns = int(
            float(self.declare_parameter("box3d.sync_tolerance_sec", 0.05).value)
            * 1_000_000_000
        )
        self._box_parameters = {
            "minimum_depth_m": float(
                self.declare_parameter("box3d.minimum_depth_m", 0.2).value
            ),
            "maximum_depth_m": float(
                self.declare_parameter("box3d.maximum_depth_m", 20.0).value
            ),
            "depth_tolerance_m": float(
                self.declare_parameter("box3d.depth_tolerance_m", 0.25).value
            ),
            "depth_tolerance_ratio": float(
                self.declare_parameter("box3d.depth_tolerance_ratio", 0.08).value
            ),
            "minimum_points": int(
                self.declare_parameter("box3d.minimum_points", 20).value
            ),
        }
        self._validate_parameters(image_topic, depth_topic, camera_info_topic)

        self._model = model_factory(
            model_path=str(model_path),
            device=str(device),
            confidence=float(confidence),
            image_size=int(image_size),
            iou=float(iou),
        )

        image_qos = QoSProfile(depth=1)
        image_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        image_qos.durability = DurabilityPolicy.VOLATILE
        self._overlay_publisher = self.create_publisher(
            Image,
            "~/overlay",
            image_qos,
        )
        self._boxes_publisher = self.create_publisher(
            MarkerArray,
            "~/boxes_3d",
            image_qos,
        )
        self._image_subscription = self.create_subscription(
            Image,
            str(image_topic),
            self._on_image,
            image_qos,
        )
        self._depth_subscription = self.create_subscription(
            Image,
            str(depth_topic),
            self._on_depth,
            image_qos,
        )
        self._camera_info_subscription = self.create_subscription(
            CameraInfo,
            str(camera_info_topic),
            self._on_camera_info,
            10,
        )
        self._pending_image = None
        self._pending_depth = None
        self._camera_info = None

        self.get_logger().info(
            f"RGB={image_topic}; depth={depth_topic}; "
            f"3D boxes=~/boxes_3d"
        )

    def _validate_parameters(self, image_topic, depth_topic, info_topic) -> None:
        for name, value in (
            ("input.image_topic", image_topic),
            ("input.depth_topic", depth_topic),
            ("input.camera_info_topic", info_topic),
        ):
            if not str(value).strip():
                raise ValueError(f"{name} must not be empty")
        if self._sync_tolerance_ns < 0:
            raise ValueError("box3d.sync_tolerance_sec cannot be negative")
        values = self._box_parameters
        if (
            not math.isfinite(values["minimum_depth_m"])
            or values["minimum_depth_m"] < 0.0
            or not math.isfinite(values["maximum_depth_m"])
            or values["maximum_depth_m"] <= values["minimum_depth_m"]
        ):
            raise ValueError("box3d depth range is invalid")
        if (
            not math.isfinite(values["depth_tolerance_m"])
            or values["depth_tolerance_m"] < 0.0
        ):
            raise ValueError("box3d.depth_tolerance_m cannot be negative")
        if (
            not math.isfinite(values["depth_tolerance_ratio"])
            or values["depth_tolerance_ratio"] < 0.0
        ):
            raise ValueError("box3d.depth_tolerance_ratio cannot be negative")
        if values["minimum_points"] <= 0:
            raise ValueError("box3d.minimum_points must be positive")

    def _on_image(self, message: Image) -> None:
        self._pending_image = message
        self._try_process_pair()

    def _on_depth(self, message: Image) -> None:
        self._pending_depth = message
        self._try_process_pair()

    def _on_camera_info(self, message: CameraInfo) -> None:
        self._camera_info = message
        self._try_process_pair()

    def _try_process_pair(self) -> None:
        if (
            self._pending_image is None
            or self._pending_depth is None
            or self._camera_info is None
        ):
            return
        image_stamp = _stamp_nanoseconds(self._pending_image)
        depth_stamp = _stamp_nanoseconds(self._pending_depth)
        difference = image_stamp - depth_stamp
        if abs(difference) > self._sync_tolerance_ns:
            if difference < 0:
                self._pending_image = None
            else:
                self._pending_depth = None
            return

        image_message = self._pending_image
        depth_message = self._pending_depth
        camera_info = self._camera_info
        self._pending_image = None
        self._pending_depth = None
        self._process_pair(image_message, depth_message, camera_info)

    def _process_pair(
        self,
        image_message: Image,
        depth_message: Image,
        camera_info: CameraInfo,
    ) -> None:
        try:
            image_bgr = image_message_to_bgr(image_message)
            prediction = self._model.predict(image_bgr)
            overlay = bgr_to_image_message(
                prediction.overlay_bgr,
                image_message.header,
            )
        except Exception as error:  # Keep processing later camera frames.
            self.get_logger().error(f"Segmentation failed: {error}")
            return

        self._overlay_publisher.publish(overlay)
        try:
            depth_m = depth_message_to_meters(depth_message)
            boxes = estimate_boxes_3d(
                prediction.boxes,
                depth_m,
                camera_info,
                int(image_message.width),
                int(image_message.height),
                **self._box_parameters,
            )
            markers = boxes_to_markers(boxes, depth_message.header)
        except Exception as error:
            self.get_logger().error(f"3D box estimation failed: {error}")
            markers = boxes_to_markers((), depth_message.header)
        self._boxes_publisher.publish(markers)


def main(args=None) -> None:
    """Run the segmentation node."""
    rclpy.init(args=args)
    node = None
    try:
        node = SegmentationNode()
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
