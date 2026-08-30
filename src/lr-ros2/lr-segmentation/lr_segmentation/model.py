"""Ultralytics adapter for overlay images and 2D boxes."""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class Box2D:
    """One model detection in source-image pixel coordinates."""

    xyxy: tuple[float, float, float, float]
    class_id: int
    class_name: str
    confidence: float


@dataclass(frozen=True)
class SegmentationPrediction:
    """Rendered overlay and boxes produced by one inference."""

    overlay_bgr: np.ndarray
    boxes: tuple[Box2D, ...]


def _as_numpy(value) -> np.ndarray:
    if hasattr(value, "detach"):
        value = value.detach()
    if hasattr(value, "cpu"):
        value = value.cpu()
    if hasattr(value, "numpy"):
        value = value.numpy()
    return np.asarray(value)


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
        """Run inference and return its rendered overlay and 2D boxes."""
        result = self._model.predict(source=image_bgr, **self._arguments)[0]
        detections = []
        if result.boxes is not None:
            coordinates = _as_numpy(result.boxes.xyxy)
            class_ids = _as_numpy(result.boxes.cls).astype(np.int32)
            confidences = _as_numpy(result.boxes.conf)
            names = result.names
            for xyxy, class_id, confidence in zip(
                coordinates,
                class_ids,
                confidences,
            ):
                numeric_id = int(class_id)
                class_name = (
                    str(names.get(numeric_id, numeric_id))
                    if isinstance(names, dict)
                    else str(names[numeric_id])
                )
                detections.append(
                    Box2D(
                        tuple(float(value) for value in xyxy),
                        numeric_id,
                        class_name,
                        float(confidence),
                    )
                )

        return SegmentationPrediction(
            np.asarray(result.plot(), dtype=np.uint8),
            tuple(detections),
        )


def create_segmentation_model(**kwargs) -> UltralyticsSegmentationModel:
    """Create the supported segmentation model."""
    return UltralyticsSegmentationModel(**kwargs)
