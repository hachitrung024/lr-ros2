"""Ultralytics adapter for rendered overlay images."""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class SegmentationPrediction:
    """Rendered overlay and source-resolution instance labels."""

    overlay_bgr: np.ndarray
    instance_labels: np.ndarray


def _as_numpy(value) -> np.ndarray:
    if hasattr(value, "detach"):
        value = value.detach()
    if hasattr(value, "cpu"):
        value = value.cpu()
    if hasattr(value, "numpy"):
        value = value.numpy()
    return np.asarray(value)


def _resize_masks_nearest(
    masks: np.ndarray,
    height: int,
    width: int,
) -> np.ndarray:
    if masks.shape[1:] == (height, width):
        return masks
    source_height, source_width = masks.shape[1:]
    rows = np.minimum(
        (np.arange(height) * source_height // height),
        source_height - 1,
    )
    columns = np.minimum(
        (np.arange(width) * source_width // width),
        source_width - 1,
    )
    return masks[:, rows][:, :, columns]


def _instance_labels(result, height: int, width: int) -> np.ndarray:
    labels = np.zeros((height, width), dtype=np.uint16)
    result_masks = getattr(result, "masks", None)
    mask_data = getattr(result_masks, "data", None)
    if mask_data is None:
        return labels

    masks = _as_numpy(mask_data)
    if masks.ndim != 3 or masks.shape[0] == 0:
        return labels
    masks = _resize_masks_nearest(masks, height, width)
    instance_count = min(masks.shape[0], np.iinfo(np.uint16).max)

    boxes = getattr(result, "boxes", None)
    confidences = getattr(boxes, "conf", None)
    if confidences is not None:
        scores = _as_numpy(confidences).reshape(-1)[:instance_count]
        order = np.argsort(-scores, kind="stable")
    else:
        order = np.arange(instance_count)

    for index in order:
        pixels = (masks[index] > 0.5) & (labels == 0)
        labels[pixels] = int(index) + 1
    return labels


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
            raise RuntimeError(
                "Ultralytics is not installed; install requirements.txt"
            ) from error

        self._model = YOLO(str(path.resolve()))
        self._arguments = {
            "device": device,
            "conf": confidence,
            "imgsz": image_size,
            "iou": iou,
            "verbose": False,
        }

    def predict(self, image_bgr: np.ndarray) -> SegmentationPrediction:
        """Run inference and return its overlay and instance-label image."""
        result = self._model.predict(source=image_bgr, **self._arguments)[0]
        height, width = image_bgr.shape[:2]
        return SegmentationPrediction(
            np.asarray(result.plot(), dtype=np.uint8),
            _instance_labels(result, height, width),
        )


def create_segmentation_model(**kwargs) -> UltralyticsSegmentationModel:
    """Create the supported segmentation model."""
    return UltralyticsSegmentationModel(**kwargs)
