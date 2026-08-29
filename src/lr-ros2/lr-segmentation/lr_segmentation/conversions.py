"""ROS image adapters and 2D detection serialization."""

from __future__ import annotations

from array import array

import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)

from .processor import SegmentationFrameResult
from .visualization import resize_binary_mask


_IMAGE_ENCODINGS = {
    "bgr8": (3, (0, 1, 2)),
    "rgb8": (3, (2, 1, 0)),
    "bgra8": (4, (0, 1, 2)),
    "rgba8": (4, (2, 1, 0)),
}


def image_message_to_bgr(message: Image) -> np.ndarray:
    """Decode common 8-bit color encodings into contiguous BGR."""
    encoding = message.encoding.lower()
    if encoding not in _IMAGE_ENCODINGS:
        supported = ", ".join(sorted(_IMAGE_ENCODINGS))
        raise ValueError(
            f"Unsupported image encoding '{message.encoding}'; expected {supported}"
        )
    channels, order = _IMAGE_ENCODINGS[encoding]
    if message.height <= 0 or message.width <= 0:
        raise ValueError("Image dimensions must be positive")
    packed_width = int(message.width) * channels
    if message.step < packed_width:
        raise ValueError("Image step is smaller than its packed row size")
    required = int(message.step) * int(message.height)
    if len(message.data) < required:
        raise ValueError("Image data is shorter than step * height")
    rows = np.frombuffer(message.data, dtype=np.uint8, count=required).reshape(
        int(message.height), int(message.step)
    )
    pixels = rows[:, :packed_width].reshape(
        int(message.height), int(message.width), channels
    )
    return np.ascontiguousarray(pixels[..., list(order)])


def bgr_to_image_message(image_bgr: np.ndarray, header: Header) -> Image:
    """Encode a BGR array as an RViz-friendly RGB8 image."""
    image = np.asarray(image_bgr)
    if image.ndim != 3 or image.shape[2] != 3 or image.dtype != np.uint8:
        raise ValueError("image_bgr must be uint8 with shape (H, W, 3)")
    rgb = np.ascontiguousarray(image[..., ::-1])
    message = Image()
    message.header = header
    message.height = rgb.shape[0]
    message.width = rgb.shape[1]
    message.encoding = "rgb8"
    message.is_bigendian = 0
    message.step = rgb.shape[1] * 3
    # ROS 2's generated uint8[] setter validates bytes element-by-element.
    # Supplying the native array type avoids that O(image size) Python loop.
    message.data = array("B", rgb.tobytes())
    return message


def result_to_instance_mask_image(
    result: SegmentationFrameResult,
    image_header: Header,
    image_height: int,
    image_width: int,
) -> Image:
    """Encode instance IDs aligned with the serialized detection order."""
    height = int(image_height)
    width = int(image_width)
    if height <= 0 or width <= 0:
        raise ValueError("instance mask dimensions must be positive")
    if len(result.detections) > np.iinfo(np.uint16).max:
        raise ValueError("too many detections for a mono16 instance mask")

    labels = np.zeros((height, width), dtype=np.uint16)
    # Let the most confident instance own pixels where model masks overlap,
    # while preserving i + 1 as the label-to-detection mapping.
    confidence_order = sorted(
        range(len(result.detections)),
        key=lambda index: float(result.detections[index].confidence),
    )
    for index in confidence_order:
        mask = resize_binary_mask(
            result.detections[index].mask,
            width,
            height,
        )
        labels[mask] = index + 1

    message = Image()
    message.header = image_header
    message.height = height
    message.width = width
    message.encoding = "mono16"
    message.is_bigendian = 0
    message.step = width * np.dtype(np.uint16).itemsize
    little_endian = labels.astype("<u2", copy=False)
    message.data = array("B", little_endian.tobytes())
    return message


def _hypothesis(detection) -> ObjectHypothesisWithPose:
    result = ObjectHypothesisWithPose()
    result.hypothesis.class_id = detection.class_name
    result.hypothesis.score = float(detection.confidence)
    return result


def result_to_detection_array(
    result: SegmentationFrameResult,
    image_header: Header,
) -> Detection2DArray:
    """Serialize merged image-space detections."""
    detections = Detection2DArray()
    detections.header = image_header
    stamp_ns = int(image_header.stamp.sec) * 1_000_000_000 + int(
        image_header.stamp.nanosec
    )
    for index, detection in enumerate(result.detections):
        instance_id = f"{stamp_ns}:{index}"
        box = np.asarray(detection.bbox_2d, dtype=np.float64)
        message = Detection2D()
        message.header = image_header
        message.id = instance_id
        message.results = [_hypothesis(detection)]
        message.bbox.center.position.x = float((box[0] + box[2]) * 0.5)
        message.bbox.center.position.y = float((box[1] + box[3]) * 0.5)
        message.bbox.center.theta = 0.0
        message.bbox.size_x = float(box[2] - box[0])
        message.bbox.size_y = float(box[3] - box[1])
        detections.detections.append(message)
    return detections


def empty_detection_array(image_header: Header) -> Detection2DArray:
    """Create a stamped empty 2D detection array."""
    detections = Detection2DArray()
    detections.header = image_header
    return detections
