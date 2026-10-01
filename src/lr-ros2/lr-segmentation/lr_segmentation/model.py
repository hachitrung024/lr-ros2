"""Ultralytics adapter for rendered overlay images."""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class SegmentationPrediction:
    """Rendered overlay and source-resolution instance labels."""

    overlay_bgr: np.ndarray | None
    instance_labels: np.ndarray


def _as_numpy(value) -> np.ndarray:
    if hasattr(value, "detach"):
        value = value.detach()
    if hasattr(value, "cpu"):
        value = value.cpu()
    if hasattr(value, "numpy"):
        value = value.numpy()
    return np.asarray(value)


def _restore_labels(
    labels: np.ndarray,
    height: int,
    width: int,
) -> np.ndarray:
    """Remove centered letterbox padding before mapping labels to source pixels."""
    if labels.shape == (height, width):
        return labels
    source_height, source_width = labels.shape
    gain = min(source_height / height, source_width / width)
    scaled_height, scaled_width = round(height * gain), round(width * gain)
    top = round((source_height - scaled_height) / 2 - 0.1)
    left = round((source_width - scaled_width) / 2 - 0.1)
    rows = np.minimum(
        (np.arange(height) * scaled_height // height),
        scaled_height - 1,
    )
    columns = np.minimum(
        (np.arange(width) * scaled_width // width),
        scaled_width - 1,
    )
    return labels[(rows + top)[:, None], (columns + left)[None, :]]


def _instance_labels(result, height: int, width: int) -> np.ndarray:
    result_masks = getattr(result, "masks", None)
    mask_data = getattr(result_masks, "data", None)
    if mask_data is None:
        return np.zeros((height, width), dtype=np.uint16)

    masks = _as_numpy(mask_data)
    if masks.ndim != 3 or min(masks.shape[1:]) <= 0:
        raise ValueError("segmentation masks must have shape (N, H, W)")
    # Resolve overlapping instances before resizing. Nearest-neighbor sampling
    # preserves the result while allocating only one source-resolution image.
    labels = np.zeros(masks.shape[1:], dtype=np.uint16)
    instance_count = min(masks.shape[0], np.iinfo(np.uint16).max)

    boxes = getattr(result, "boxes", None)
    confidences = getattr(boxes, "conf", None)
    if confidences is not None:
        scores = _as_numpy(confidences).reshape(-1)[:instance_count]
        if scores.size != instance_count:
            raise ValueError("mask and confidence counts differ")
        order = np.argsort(-scores, kind="stable")
    else:
        order = np.arange(instance_count)

    for index in order:
        pixels = (masks[index] > 0.5) & (labels == 0)
        labels[pixels] = int(index) + 1
    return _restore_labels(labels, height, width)


class UltralyticsSegmentationModel:
    """Load YOLO once and render its result for each image."""

    def __init__(
        self,
        *,
        model_path: str,
        device: str,
        confidence: float,
        image_size: int,
        iou: float,
    ) -> None:
        path = Path(model_path).expanduser()
        if not path.is_file():
            raise FileNotFoundError(f"Segmentation checkpoint not found: {path}")
        if not device.strip():
            raise ValueError("model.device must not be empty")
        if not math.isfinite(confidence) or not 0.0 <= confidence <= 1.0:
            raise ValueError("model.confidence must be in [0, 1]")
        if not math.isfinite(iou) or not 0.0 <= iou <= 1.0:
            raise ValueError("model.iou must be in [0, 1]")
        if image_size <= 0:
            raise ValueError("model.image_size must be greater than zero")

        try:
            from ultralytics import YOLO
        except ImportError as error:
            raise RuntimeError("Ultralytics is not installed; install requirements.txt") from error

        self._model = YOLO(str(path.resolve()))
        self._arguments = {
            "device": device,
            "conf": confidence,
            "imgsz": image_size,
            "iou": iou,
            "verbose": False,
        }

    def predict(
        self, image_bgr: np.ndarray, *, render_overlay: bool = True
    ) -> SegmentationPrediction:
        """Run inference and return its overlay and instance-label image."""
        result = self._model.predict(source=image_bgr, **self._arguments)[0]
        height, width = image_bgr.shape[:2]
        return SegmentationPrediction(
            np.asarray(result.plot(), dtype=np.uint8) if render_overlay else None,
            _instance_labels(result, height, width),
        )


def create_segmentation_model(**kwargs) -> UltralyticsSegmentationModel:
    """Create the supported segmentation model."""
    return UltralyticsSegmentationModel(**kwargs)
