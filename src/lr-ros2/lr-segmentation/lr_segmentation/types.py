"""Model-independent segmentation data types."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(slots=True)
class InstanceDetection:
    """One instance mask and its image-space metadata."""

    mask: np.ndarray
    class_id: int
    class_name: str
    confidence: float
    bbox_2d: np.ndarray
