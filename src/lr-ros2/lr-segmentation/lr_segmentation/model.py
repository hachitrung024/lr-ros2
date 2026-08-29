"""Ultralytics instance-segmentation adapter."""

from __future__ import annotations

from abc import ABC, abstractmethod
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from .types import InstanceDetection


class InstanceSegmenter(ABC):
    """Model-independent inference interface."""

    @abstractmethod
    def predict(self, image: np.ndarray) -> list[InstanceDetection]:
        """Return independent object masks and metadata."""


def mask_to_bbox(mask: np.ndarray, width: int, height: int) -> np.ndarray | None:
    """Derive an XYXY box only from a binary segmentation mask."""
    array = np.asarray(mask)
    if array.ndim != 2:
        raise ValueError("mask must be two-dimensional")
    resized = (
        cv2.resize(
            array.astype(np.uint8),
            (width, height),
            interpolation=cv2.INTER_NEAREST,
        ).astype(bool)
        if array.shape != (height, width)
        else array.astype(bool, copy=False)
    )
    rows, columns = np.nonzero(resized)
    if len(rows) == 0:
        return None
    return np.array(
        [columns.min(), rows.min(), columns.max() + 1, rows.max() + 1],
        dtype=np.float32,
    )


def _normalize_names(names: Any) -> dict[int, str]:
    if isinstance(names, dict):
        return {int(key): str(value) for key, value in names.items()}
    if isinstance(names, (list, tuple)):
        return {index: str(value) for index, value in enumerate(names)}
    return {}


class UltralyticsInstanceSegmenter(InstanceSegmenter):
    """Load YOLO once and return thresholded binary masks per frame."""

    def __init__(
        self,
        model_path: str | Path,
        device: str,
        image_size: int,
        confidence: float,
        iou: float,
        classes: tuple[int, ...] | None,
        quantize: bool,
    ) -> None:
        try:
            from ultralytics import YOLO
        except ImportError as error:
            raise RuntimeError(
                "Ultralytics is not installed; install requirements.txt"
            ) from error
        self.model = YOLO(str(model_path))
        self.names = _normalize_names(self.model.names)
        self.device = device
        self.image_size = image_size
        self.confidence = confidence
        self.iou = iou
        self.classes = list(classes) if classes is not None else None
        self.quantize = quantize

    def predict(self, image: np.ndarray) -> list[InstanceDetection]:
        """Run one Ultralytics prediction and normalize its result."""
        arguments = {
            "source": image,
            "imgsz": self.image_size,
            "conf": self.confidence,
            "iou": self.iou,
            "classes": self.classes,
            "device": self.device,
            "retina_masks": False,
            "verbose": False,
        }
        if self.quantize:
            arguments["quantize"] = True
        result = self.model.predict(**arguments)[0]
        if result.masks is None or result.boxes is None:
            return []
        masks = result.masks.data.detach().float().cpu().numpy()
        classes = result.boxes.cls.detach().cpu().numpy().astype(np.int32)
        scores = result.boxes.conf.detach().cpu().numpy()
        height, width = image.shape[:2]
        detections = []
        for mask, class_id, score in zip(masks, classes, scores):
            binary_mask = mask >= 0.5
            box = mask_to_bbox(binary_mask, width, height)
            if box is None:
                continue
            detections.append(
                InstanceDetection(
                    mask=binary_mask,
                    class_id=int(class_id),
                    class_name=self.names.get(int(class_id), str(class_id)),
                    confidence=float(score),
                    bbox_2d=box,
                )
            )
        return detections


def create_instance_segmenter(config) -> InstanceSegmenter:
    """Create the supported Ultralytics backend."""
    return UltralyticsInstanceSegmenter(
        Path(config.model_path).expanduser().resolve(),
        config.device,
        config.image_size,
        config.confidence,
        config.iou,
        config.classes,
        config.quantize,
    )
