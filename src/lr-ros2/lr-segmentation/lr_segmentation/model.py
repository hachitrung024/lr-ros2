"""Ultralytics adapter that returns a rendered segmentation overlay."""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np


class UltralyticsOverlayModel:
    """Load YOLO once and render its segmentation result for each image."""

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

    def predict_overlay(self, image_bgr: np.ndarray) -> np.ndarray:
        """Run inference and return the rendered BGR overlay."""
        result = self._model.predict(source=image_bgr, **self._arguments)[0]
        return np.asarray(result.plot(), dtype=np.uint8)


def create_overlay_model(**kwargs) -> UltralyticsOverlayModel:
    """Create the supported overlay model."""
    return UltralyticsOverlayModel(**kwargs)
