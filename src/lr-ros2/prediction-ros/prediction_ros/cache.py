"""Re-export ROS-independent input cache from prediction_core."""

from prediction_core.cache import (
    ExternalWrenchData,
    PredictionInputCache,
    PredictionSnapshot,
)

__all__ = [
    "ExternalWrenchData",
    "PredictionInputCache",
    "PredictionSnapshot",
]
