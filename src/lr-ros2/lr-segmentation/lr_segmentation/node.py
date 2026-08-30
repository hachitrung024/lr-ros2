"""ROS 2 node for RGB segmentation overlays."""

from __future__ import annotations

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from .conversions import bgr_to_image_message, image_message_to_bgr
from .model import create_segmentation_model


class SegmentationNode(Node):
    """Run instance segmentation on RGB images and publish an overlay."""

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
        model_path = self.declare_parameter("model.path", "").value
        device = self.declare_parameter("model.device", "0").value
        confidence = self.declare_parameter("model.confidence", 0.25).value
        image_size = self.declare_parameter("model.image_size", 960).value
        iou = self.declare_parameter("model.iou", 0.60).value
        if not str(image_topic).strip():
            raise ValueError("input.image_topic must not be empty")

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
        self._image_subscription = self.create_subscription(
            Image,
            str(image_topic),
            self._on_image,
            image_qos,
        )
        self.get_logger().info(f"RGB={image_topic}; overlay=~/overlay")

    def _on_image(self, message: Image) -> None:
        try:
            image_bgr = image_message_to_bgr(message)
            prediction = self._model.predict(image_bgr)
            overlay = bgr_to_image_message(
                prediction.overlay_bgr,
                message.header,
            )
        except Exception as error:  # Keep processing later camera frames.
            self.get_logger().error(f"Segmentation failed: {error}")
            return

        self._overlay_publisher.publish(overlay)


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
