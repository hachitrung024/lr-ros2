"""ROS 2 node for bounded-latency image-only instance segmentation."""

from __future__ import annotations

import math
import threading
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from vision_msgs.msg import Detection2DArray

from .config import CoreConfig, SegmentationConfig
from .conversions import (
    bgr_to_image_message,
    empty_detection_array,
    image_message_to_bgr,
    result_to_detection_array,
    result_to_instance_mask_image,
)
from .processor import SegmentationFrameResult, SegmentationProcessor
from .visualization import draw_instances


def _stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class SegmentationNode(Node):
    """Run inference on only the freshest pending ZED image."""

    def __init__(
        self,
        *,
        parameter_overrides=None,
        processor_factory=SegmentationProcessor,
    ) -> None:
        super().__init__("segmentation", parameter_overrides=parameter_overrides)
        read_only = ParameterDescriptor(read_only=True)
        self._image_topic = self.declare_parameter(
            "input.image_topic",
            "/zed/zed_node/rgb/color/rect/image",
            read_only,
        ).value
        model_path = self.declare_parameter("model.path", "", read_only).value
        model_device = self.declare_parameter("model.device", "0", read_only).value
        confidence = self.declare_parameter(
            "model.confidence", 0.25, read_only
        ).value
        image_size = self.declare_parameter(
            "model.image_size", 960, read_only
        ).value
        iou = self.declare_parameter("model.iou", 0.60, read_only).value
        classes = self.declare_parameter(
            "model.classes",
            Parameter.Type.INTEGER_ARRAY,
            read_only,
        ).value
        quantize = self.declare_parameter(
            "model.quantize", False, read_only
        ).value
        self._diagnostic_period_sec = float(
            self.declare_parameter(
                "diagnostics.period_sec", 1.0, read_only
            ).value
        )
        self._stale_after_sec = float(
            self.declare_parameter(
                "diagnostics.stale_after_sec", 2.0, read_only
            ).value
        )
        self._validate_node_parameters()

        self._model_config = SegmentationConfig(
            model_path=str(model_path),
            device=str(model_device),
            confidence=float(confidence),
            image_size=int(image_size),
            iou=float(iou),
            classes=tuple(int(value) for value in (classes or ())) or None,
            quantize=bool(quantize),
        )
        self._core_config = CoreConfig(
            merge_enabled=bool(self._parameter("merge.enabled", True)),
            merge_excluded_classes=tuple(
                str(value)
                for value in self._parameter("merge.excluded_classes", ["person"])
            ),
            mask_dilation_px=int(
                self._parameter("merge.mask_dilation_px", 20)
            ),
            minimum_mask_iou=float(
                self._parameter("merge.minimum_mask_iou", 0.02)
            ),
            maximum_mask_gap_px=float(
                self._parameter("merge.maximum_mask_gap_px", 35.0)
            ),
        )
        self._model_config.validate()
        self._core_config.validate()
        self._processor = processor_factory(self._model_config, self._core_config)

        stream_qos = QoSProfile(depth=1)
        stream_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        stream_qos.durability = DurabilityPolicy.VOLATILE
        self._overlay_publisher = self.create_publisher(
            Image, "~/overlay", stream_qos
        )
        self._detections_2d_publisher = self.create_publisher(
            Detection2DArray, "~/detections_2d", stream_qos
        )
        self._instance_masks_publisher = self.create_publisher(
            Image, "~/instance_masks", stream_qos
        )
        self._diagnostic_publisher = self.create_publisher(
            DiagnosticArray, "/diagnostics", 10
        )
        self._reset_service = self.create_service(
            Trigger, "~/reset", self._on_reset
        )

        self._condition = threading.Condition()
        self._pending_image = None
        self._generation = 0
        self._stopping = False
        self._started_monotonic = time.monotonic()
        self._last_receive_monotonic = None
        self._last_processed_monotonic = None
        self._last_timestamp = None
        self._last_image_header = Header()
        self._last_image_shape = (1, 1)
        self._image_count = 0
        self._processed_count = 0
        self._dropped_busy_count = 0
        self._malformed_count = 0
        self._inference_failure_count = 0
        self._reset_count = 0
        self._processing_ms = 0.0
        self._processing_ms_ema = 0.0
        self._raw_detection_count = 0
        self._merged_detection_count = 0
        self._last_error = ""
        self._last_reset_reason = "startup"

        self._image_subscription = self.create_subscription(
            Image,
            self._image_topic,
            self._on_image,
            stream_qos,
        )
        self._diagnostic_timer = self.create_timer(
            self._diagnostic_period_sec,
            self._publish_diagnostics,
        )
        self._worker = threading.Thread(
            target=self._worker_loop,
            name="segmentation-inference",
            daemon=True,
        )
        self._worker.start()
        self.get_logger().info(
            f"Segmentation listening on {self._image_topic}; "
            f"model={self._model_config.model_path}; "
            f"device={self._model_config.device}"
        )

    def _parameter(self, name, default):
        return self.declare_parameter(
            name,
            default,
            ParameterDescriptor(read_only=True),
        ).value

    def _validate_node_parameters(self) -> None:
        if not str(self._image_topic).strip():
            raise ValueError("input.image_topic must not be empty")
        for name, value in (
            ("diagnostics.period_sec", self._diagnostic_period_sec),
            ("diagnostics.stale_after_sec", self._stale_after_sec),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and greater than zero")

    def _on_image(self, image: Image) -> None:
        timestamp = _stamp_seconds(image.header.stamp)
        now = time.monotonic()
        with self._condition:
            self._image_count += 1
            self._last_receive_monotonic = now
            self._last_image_header = image.header
            self._last_image_shape = (int(image.height), int(image.width))
            if not math.isfinite(timestamp):
                self._malformed_count += 1
                self._last_error = "Image timestamp is not finite"
                return
            if not image.header.frame_id:
                self._malformed_count += 1
                self._last_error = "Image frame_id must not be empty"
                return
            if self._last_timestamp is not None and timestamp < self._last_timestamp:
                self._reset_locked("timestamp rollback")
                self._publish_empty(
                    image.header,
                    int(image.height),
                    int(image.width),
                )
            self._last_timestamp = timestamp
            if self._pending_image is not None:
                self._dropped_busy_count += 1
            self._pending_image = (self._generation, image)
            self._condition.notify()

    def _worker_loop(self) -> None:
        while True:
            with self._condition:
                self._condition.wait_for(
                    lambda: self._pending_image is not None or self._stopping
                )
                if self._stopping:
                    return
                generation, image_message = self._pending_image
                self._pending_image = None
            started = time.monotonic()
            image_bgr = None
            stage = "input adapter"
            try:
                image_bgr = image_message_to_bgr(image_message)
                stage = "inference"
                result = self._processor.process(image_bgr)
                stage = "serialization"
                overlay = bgr_to_image_message(
                    draw_instances(image_bgr, result.detections),
                    image_message.header,
                )
                detections_2d = result_to_detection_array(
                    result,
                    image_message.header,
                )
                instance_masks = result_to_instance_mask_image(
                    result,
                    image_message.header,
                    image_bgr.shape[0],
                    image_bgr.shape[1],
                )
            except Exception as error:  # Keep the camera pipeline alive.
                self._record_processing_error(stage, error, started)
                self._publish_error_snapshot(
                    generation,
                    image_bgr,
                    image_message.header,
                    int(image_message.height),
                    int(image_message.width),
                )
                continue
            elapsed_ms = (time.monotonic() - started) * 1000.0
            with self._condition:
                if generation != self._generation or self._stopping:
                    continue
                self._processed_count += 1
                self._processing_ms = elapsed_ms
                if self._processed_count == 1:
                    self._processing_ms_ema = elapsed_ms
                else:
                    self._processing_ms_ema = (
                        0.2 * elapsed_ms + 0.8 * self._processing_ms_ema
                    )
                self._last_processed_monotonic = time.monotonic()
                self._last_error = ""
                self._raw_detection_count = result.raw_detection_count
                self._merged_detection_count = result.merged_detection_count
            self._overlay_publisher.publish(overlay)
            self._detections_2d_publisher.publish(detections_2d)
            self._instance_masks_publisher.publish(instance_masks)

    def _record_processing_error(self, stage, error, started) -> None:
        with self._condition:
            if stage == "input adapter":
                self._malformed_count += 1
            else:
                self._inference_failure_count += 1
            self._processing_ms = (time.monotonic() - started) * 1000.0
            self._last_error = f"{stage} failed: {error}"
        self.get_logger().error(self._last_error)

    def _publish_error_snapshot(
        self,
        generation,
        image_bgr,
        image_header,
        image_height,
        image_width,
    ) -> None:
        with self._condition:
            if generation != self._generation or self._stopping:
                return
        if image_bgr is not None:
            self._overlay_publisher.publish(
                bgr_to_image_message(image_bgr, image_header)
            )
        self._publish_empty(image_header, image_height, image_width)

    def _publish_empty(self, image_header, image_height, image_width) -> None:
        self._detections_2d_publisher.publish(
            empty_detection_array(image_header)
        )
        self._instance_masks_publisher.publish(
            result_to_instance_mask_image(
                SegmentationFrameResult.empty(),
                image_header,
                image_height,
                image_width,
            )
        )

    def _reset_locked(self, reason: str) -> None:
        self._generation += 1
        self._pending_image = None
        self._reset_count += 1
        self._last_reset_reason = reason
        self._last_timestamp = None
        self._raw_detection_count = 0
        self._merged_detection_count = 0
        self._last_error = ""

    def _on_reset(self, _request, response):
        with self._condition:
            self._reset_locked("reset service")
            image_header = self._last_image_header
            image_height, image_width = self._last_image_shape
        self._publish_empty(image_header, image_height, image_width)
        response.success = True
        response.message = "Segmentation detections and pending image cleared"
        return response

    def _publish_diagnostics(self) -> None:
        now = time.monotonic()
        with self._condition:
            snapshot = {
                "image_count": self._image_count,
                "processed_count": self._processed_count,
                "dropped_busy_count": self._dropped_busy_count,
                "malformed_count": self._malformed_count,
                "inference_failure_count": self._inference_failure_count,
                "reset_count": self._reset_count,
                "processing_ms": self._processing_ms,
                "processing_ms_ema": self._processing_ms_ema,
                "raw_detection_count": self._raw_detection_count,
                "merged_detection_count": self._merged_detection_count,
                "last_error": self._last_error,
                "last_reset_reason": self._last_reset_reason,
                "last_receive": self._last_receive_monotonic,
                "last_processed": self._last_processed_monotonic,
            }
        elapsed = max(now - self._started_monotonic, 1e-6)
        status = DiagnosticStatus()
        status.name = f"{self.get_fully_qualified_name()}: segmentation"
        status.hardware_id = f"ultralytics:{self._model_config.device}"
        if snapshot["last_error"]:
            status.level = DiagnosticStatus.ERROR
            status.message = snapshot["last_error"]
        elif (
            snapshot["last_receive"] is None
            or now - snapshot["last_receive"] > self._stale_after_sec
        ):
            status.level = DiagnosticStatus.WARN
            status.message = "Waiting for ZED image"
        else:
            status.level = DiagnosticStatus.OK
            status.message = "Segmentation running"
        values = {
            "model_path": self._model_config.model_path,
            "device": self._model_config.device,
            "image_rate_hz": snapshot["image_count"] / elapsed,
            "processed_rate_hz": snapshot["processed_count"] / elapsed,
            **{
                key: value
                for key, value in snapshot.items()
                if key not in {"last_receive", "last_processed"}
            },
        }
        status.values = [
            KeyValue(key=str(key), value=str(value))
            for key, value in values.items()
        ]
        diagnostics = DiagnosticArray()
        diagnostics.header.stamp = self.get_clock().now().to_msg()
        diagnostics.status = [status]
        self._diagnostic_publisher.publish(diagnostics)

    def destroy_node(self):
        """Stop the inference worker before destroying ROS publishers."""
        if hasattr(self, "_condition"):
            with self._condition:
                self._stopping = True
                self._pending_image = None
                self._generation += 1
                self._condition.notify_all()
        if hasattr(self, "_worker") and self._worker.is_alive():
            self._worker.join(timeout=5.0)
        return super().destroy_node()


def main(args=None) -> None:
    """Run the segmentation node in a multi-threaded executor."""
    rclpy.init(args=args)
    node = None
    executor = MultiThreadedExecutor(num_threads=2)
    try:
        node = SegmentationNode()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            executor.shutdown()
        except KeyboardInterrupt:
            pass
        if node is not None:
            try:
                node.destroy_node()
            except KeyboardInterrupt:
                pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass


if __name__ == "__main__":
    main()
