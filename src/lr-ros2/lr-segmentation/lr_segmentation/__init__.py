"""Image-only instance segmentation for ROS 2."""

from .config import CoreConfig, SegmentationConfig
from .merge import merge_instance_detections
from .model import InstanceSegmenter, mask_to_bbox
from .processor import SegmentationFrameResult, SegmentationProcessor
from .types import InstanceDetection
from .visualization import draw_instances, instance_color_bgr, resize_binary_mask

__all__ = [
    "CoreConfig",
    "InstanceDetection",
    "InstanceSegmenter",
    "SegmentationConfig",
    "SegmentationFrameResult",
    "SegmentationProcessor",
    "draw_instances",
    "instance_color_bgr",
    "mask_to_bbox",
    "merge_instance_detections",
    "resize_binary_mask",
]
