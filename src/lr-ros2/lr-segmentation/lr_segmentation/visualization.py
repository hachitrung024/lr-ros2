"""Image-space instance-segmentation visualization."""

from __future__ import annotations

import cv2
import numpy as np

from .types import InstanceDetection


def resize_binary_mask(mask: np.ndarray, width: int, height: int) -> np.ndarray:
    """Resize a binary mask with nearest-neighbor sampling."""
    array = np.asarray(mask)
    if array.ndim != 2:
        raise ValueError("mask must be two-dimensional")
    if width <= 0 or height <= 0:
        raise ValueError("mask target dimensions must be positive")
    if array.shape == (height, width):
        return array.astype(bool, copy=False)
    return cv2.resize(
        array.astype(np.uint8),
        (width, height),
        interpolation=cv2.INTER_NEAREST,
    ).astype(bool)


def instance_color_bgr(class_id: int) -> np.ndarray:
    """Return the deterministic class palette used by the legacy UI."""
    return np.array(
        [
            (53 * int(class_id) + 40) % 255,
            (97 * int(class_id) + 80) % 255,
            (29 * int(class_id) + 160) % 255,
        ],
        dtype=np.uint8,
    )


def draw_instances(
    image_bgr: np.ndarray,
    detections: tuple[InstanceDetection, ...] | list[InstanceDetection],
) -> np.ndarray:
    """Draw translucent masks, contours, classes, and confidence."""
    image = np.asarray(image_bgr)
    if image.ndim != 3 or image.shape[2] != 3 or image.dtype != np.uint8:
        raise ValueError("image_bgr must be uint8 with shape (H, W, 3)")
    output = image.copy()
    overlay = output.copy()
    for detection in detections:
        mask = resize_binary_mask(
            detection.mask,
            image.shape[1],
            image.shape[0],
        )
        color = instance_color_bgr(detection.class_id)
        overlay[mask] = color
        contours, _ = cv2.findContours(
            mask.astype(np.uint8),
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )
        cv2.drawContours(output, contours, -1, color.tolist(), 2)
        x1, y1, _, _ = detection.bbox_2d.astype(int)
        cv2.putText(
            output,
            f"{detection.class_name} {detection.confidence:.2f}",
            (x1, max(18, y1 - 5)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            color.tolist(),
            1,
            cv2.LINE_AA,
        )
    return cv2.addWeighted(output, 0.65, overlay, 0.35, 0.0)
