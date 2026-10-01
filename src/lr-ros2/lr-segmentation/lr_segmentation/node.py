"""ROS 2 node for RGB segmentation overlays."""

from __future__ import annotations

import math
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.clock import JumpThreshold
from rclpy.duration import Duration
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
        self._stale_after_sec = float(
            self.declare_parameter("diagnostics.stale_after_sec", 2.0).value
        )
        if not math.isfinite(self._stale_after_sec) or self._stale_after_sec <= 0:
            raise ValueError("diagnostics.stale_after_sec must be finite and positive")
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
        self._failure_count = 0
        self._last_success_ns = None
        self._last_error = "Waiting for an image"
        self._diagnostics = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self._diagnostic_timer = self.create_timer(1.0, self._publish_diagnostics)
        self._jump_handle = self.get_clock().create_jump_callback(
            JumpThreshold(
                min_forward=None, min_backward=Duration(nanoseconds=-1), on_clock_change=True
            ),
            post_callback=self._on_time_jump,
        )
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
            self._mask_publisher.publish(mask)
            if render_overlay and prediction.overlay_bgr is not None:
                self._overlay_publisher.publish(
                    bgr_to_image_message(prediction.overlay_bgr, message.header)
                )
            self._last_success_ns = self.get_clock().now().nanoseconds
            self._last_error = ""
        except Exception as error:  # Keep processing later camera frames.
            self._failure_count += 1
            self._last_error = f"Segmentation failed: {error}"
            self.get_logger().error(self._last_error, throttle_duration_sec=2.0)
        finally:
            self._processing_ms = (time.monotonic() - started) * 1000.0

    def _on_time_jump(self, _jump):
        self._last_success_ns = None
        self._last_error = "Waiting for an image after ROS time change"

    def _publish_diagnostics(self):
        now = self.get_clock().now()
        age = (
            math.inf if self._last_success_ns is None
            else max(0.0, (now.nanoseconds - self._last_success_ns) * 1e-9)
        )
        reason = self._last_error
        if not reason and age > self._stale_after_sec:
            reason = "Segmentation output is stale"
        message = DiagnosticArray()
        message.header.stamp = now.to_msg()
        message.status = [
            DiagnosticStatus(
                name=self.get_name(),
                level=DiagnosticStatus.WARN if reason else DiagnosticStatus.OK,
                message=reason or "segmentation",
                values=[
                    KeyValue(key="callback_ms", value=str(self._processing_ms)),
                    KeyValue(key="failure_count", value=str(self._failure_count)),
                    KeyValue(key="output_age_sec", value=str(age)),
                ],
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
