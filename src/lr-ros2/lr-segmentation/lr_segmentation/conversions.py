"""Conversions between ROS color images and NumPy arrays."""

from __future__ import annotations

from array import array

import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Header


_IMAGE_ENCODINGS = {
    "bgr8": (3, (0, 1, 2)),
    "rgb8": (3, (2, 1, 0)),
    "bgra8": (4, (0, 1, 2)),
    "rgba8": (4, (2, 1, 0)),
}


def image_message_to_bgr(message: Image) -> np.ndarray:
    """Decode a common 8-bit color ROS image into contiguous BGR."""
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
    """Encode a BGR array as an RGB8 ROS image."""
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
    message.data = array("B", rgb.tobytes())
    return message
