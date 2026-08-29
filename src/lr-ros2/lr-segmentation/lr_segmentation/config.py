"""Validated configuration for the segmentation core."""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class SegmentationConfig:
    """Runtime model and algorithm configuration."""

    model_path: str
    device: str = "0"
    confidence: float = 0.25
    image_size: int = 960
    iou: float = 0.60
    classes: tuple[int, ...] | None = None
    quantize: bool = False

    def validate(self, *, require_model: bool = True) -> None:
        """Reject invalid inference values and missing model artifacts."""
        if not math.isfinite(self.confidence) or not 0.0 <= self.confidence <= 1.0:
            raise ValueError("model.confidence must be in [0, 1]")
        if not math.isfinite(self.iou) or not 0.0 <= self.iou <= 1.0:
            raise ValueError("model.iou must be in [0, 1]")
        if self.image_size <= 0:
            raise ValueError("model.image_size must be greater than zero")
        if not str(self.device).strip():
            raise ValueError("model.device must not be empty")
        if self.classes is not None and any(value < 0 for value in self.classes):
            raise ValueError("model.classes values cannot be negative")
        if not require_model:
            return
        if not str(self.model_path).strip():
            raise ValueError("model.path must be provided")
        model_path = Path(self.model_path).expanduser()
        if not model_path.is_file():
            raise FileNotFoundError(f"Segmentation checkpoint not found: {model_path}")


@dataclass(frozen=True)
class CoreConfig:
    """Validated image-space fragment-merge settings."""

    merge_enabled: bool = True
    merge_excluded_classes: tuple[str, ...] = ("person",)
    mask_dilation_px: int = 20
    minimum_mask_iou: float = 0.02
    maximum_mask_gap_px: float = 35.0

    def validate(self) -> None:
        """Validate all core settings before the first frame."""
        if self.mask_dilation_px < 0:
            raise ValueError("merge.mask_dilation_px cannot be negative")
        if not 0.0 <= self.minimum_mask_iou <= 1.0:
            raise ValueError("merge.minimum_mask_iou must be in [0, 1]")
        if (
            not math.isfinite(self.maximum_mask_gap_px)
            or self.maximum_mask_gap_px < 0.0
        ):
            raise ValueError(
                "merge.maximum_mask_gap_px must be finite and nonnegative"
            )

    @property
    def merge_settings(self) -> dict[str, Any]:
        """Return the settings consumed by fragment merging."""
        return {
            "enabled": self.merge_enabled,
            "excluded_classes": list(self.merge_excluded_classes),
            "mask_dilation_px": self.mask_dilation_px,
            "minimum_mask_iou": self.minimum_mask_iou,
            "maximum_mask_gap_px": self.maximum_mask_gap_px,
        }
