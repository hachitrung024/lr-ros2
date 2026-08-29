"""Per-frame orchestration for image-only instance segmentation."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

import numpy as np

from .config import CoreConfig, SegmentationConfig
from .merge import merge_instance_detections
from .model import create_instance_segmenter
from .types import InstanceDetection


@dataclass(frozen=True)
class SegmentationFrameResult:
    """Image-space detections for one ZED frame."""

    detections: tuple[InstanceDetection, ...]
    raw_detection_count: int

    @classmethod
    def empty(cls) -> "SegmentationFrameResult":
        """Return an empty, publishable result."""
        return cls((), 0)

    @property
    def merged_detection_count(self) -> int:
        """Return the number of detections after fragment merging."""
        return len(self.detections)


class SegmentationProcessor:
    """Run model inference and image-space fragment merging."""

    def __init__(
        self,
        config: SegmentationConfig,
        core_config: CoreConfig | None = None,
        *,
        segmenter_factory: Callable = create_instance_segmenter,
    ) -> None:
        config.validate()
        self.config = config
        self.core_config = core_config or CoreConfig()
        self.core_config.validate()
        self.segmenter = segmenter_factory(config)

    def process(self, image_bgr: np.ndarray) -> SegmentationFrameResult:
        """Process one color image."""
        image = np.asarray(image_bgr)
        if image.ndim != 3 or image.shape[2] != 3 or image.dtype != np.uint8:
            raise ValueError("image_bgr must be uint8 with shape (H, W, 3)")
        raw_detections = list(self.segmenter.predict(image))
        detections = merge_instance_detections(
            raw_detections,
            image.shape[:2],
            self.core_config.merge_settings,
        )
        return SegmentationFrameResult(
            tuple(detections),
            len(raw_detections),
        )
