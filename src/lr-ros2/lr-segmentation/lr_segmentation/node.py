"""ROS 2 node for RGB segmentation overlays."""

from __future__ import annotations

import rclpy
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from .conversions import (
    bgr_to_image_message,
    image_message_to_bgr,
    labels_to_image_message,
)
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
        self._mask_publisher = self.create_publisher(
            Image,
            "~/instance_mask",
            image_qos,
        )
        self._image_subscription = self.create_subscription(
            Image,
            str(image_topic),
            self._on_image,
            image_qos,
        )
        self._processing_ms = 0.0
        self._diagnostics = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self._diagnostic_timer = self.create_timer(1.0, self._publish_diagnostics)
        self.get_logger().info(f"RGB={image_topic}; overlay=~/overlay; mask=~/instance_mask")

    def _on_image(self, message: Image) -> None:
        started = time.monotonic()
        try:
            image_bgr = image_message_to_bgr(message)
            render_overlay = self._overlay_publisher.get_subscription_count() > 0
            prediction = self._model.predict(image_bgr, render_overlay=render_overlay)
            mask = labels_to_image_message(
                prediction.instance_labels,
                message.header,
            )
        except Exception as error:  # Keep processing later camera frames.
            self.get_logger().error(f"Segmentation failed: {error}")
            return

        self._mask_publisher.publish(mask)
        if render_overlay and prediction.overlay_bgr is not None:
            self._overlay_publisher.publish(
                bgr_to_image_message(prediction.overlay_bgr, message.header)
            )
        self._processing_ms = (time.monotonic() - started) * 1000.0

    def _publish_diagnostics(self):
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [
            DiagnosticStatus(
                name=self.get_name(),
                level=DiagnosticStatus.OK,
                message="segmentation",
                values=[KeyValue(key="callback_ms", value=str(self._processing_ms))],
            )
        ]
        self._diagnostics.publish(message)


def main(args=None) -> None:
    """Run the segmentation node."""
    rclpy.init(args=args)
    node = None
    try:
        node = SegmentationNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        # See the matching handler in box_estimator_3d: rclpy can raise while
        # taking a subscription message during process teardown.
        if rclpy.ok():
            raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
