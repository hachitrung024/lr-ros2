"""Class-aware merging for fragmented instance masks."""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from .model import mask_to_bbox
from .types import InstanceDetection


@dataclass(slots=True)
class _MaskFeatures:
    detection: InstanceDetection
    mask: np.ndarray
    bbox: np.ndarray
    area_px: int
    dilated_mask: np.ndarray
    dilated_bbox: np.ndarray


def _mask_iou(first: _MaskFeatures, second: _MaskFeatures) -> float:
    left = max(int(first.bbox[0]), int(second.bbox[0]))
    top = max(int(first.bbox[1]), int(second.bbox[1]))
    right = min(int(first.bbox[2]), int(second.bbox[2]))
    bottom = min(int(first.bbox[3]), int(second.bbox[3]))
    if right <= left or bottom <= top:
        return 0.0
    intersection = int(
        np.count_nonzero(
            first.mask[top:bottom, left:right]
            & second.mask[top:bottom, left:right]
        )
    )
    if intersection == 0:
        return 0.0
    union = first.area_px + second.area_px - intersection
    return intersection / max(union, 1)


def _bbox_gap(first: np.ndarray, second: np.ndarray) -> float:
    horizontal = max(
        float(first[0] - second[2]),
        float(second[0] - first[2]),
        0.0,
    )
    vertical = max(
        float(first[1] - second[3]),
        float(second[1] - first[3]),
        0.0,
    )
    return float(np.hypot(horizontal, vertical))


def _masks_are_close(
    first: _MaskFeatures,
    second: _MaskFeatures,
    minimum_iou: float,
    maximum_gap_px: float,
) -> bool:
    if _mask_iou(first, second) >= minimum_iou:
        return True
    if _bbox_gap(first.bbox, second.bbox) > maximum_gap_px:
        return False
    left = max(int(first.dilated_bbox[0]), int(second.dilated_bbox[0]))
    top = max(int(first.dilated_bbox[1]), int(second.dilated_bbox[1]))
    right = min(int(first.dilated_bbox[2]), int(second.dilated_bbox[2]))
    bottom = min(int(first.dilated_bbox[3]), int(second.dilated_bbox[3]))
    if right <= left or bottom <= top:
        return False
    first_left, first_top = first.dilated_bbox[:2].astype(int)
    second_left, second_top = second.dilated_bbox[:2].astype(int)
    first_roi = first.dilated_mask[
        top - first_top:bottom - first_top,
        left - first_left:right - first_left,
    ]
    second_roi = second.dilated_mask[
        top - second_top:bottom - second_top,
        left - second_left:right - second_left,
    ]
    return bool(np.any(first_roi & second_roi))


def merge_instance_detections(
    detections: list[InstanceDetection],
    image_shape: tuple[int, int],
    config: dict,
) -> list[InstanceDetection]:
    """Union nearby same-class masks using image-space evidence only."""
    if not bool(config.get("enabled", True)) or len(detections) < 2:
        return detections
    height, width = image_shape
    excluded = {
        str(name).strip().lower() for name in config.get("excluded_classes", [])
    }
    dilation_px = max(int(config.get("mask_dilation_px", 20)), 0)
    kernel = (
        cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (2 * dilation_px + 1, 2 * dilation_px + 1),
        )
        if dilation_px > 0
        else None
    )
    features = []
    for detection in detections:
        source_mask = np.asarray(detection.mask)
        if source_mask.ndim != 2:
            continue
        mask = (
            cv2.resize(
                source_mask.astype(np.uint8),
                (width, height),
                interpolation=cv2.INTER_NEAREST,
            ).astype(bool)
            if source_mask.shape != (height, width)
            else source_mask.astype(bool, copy=False)
        )
        bbox = mask_to_bbox(mask, width, height)
        if bbox is None:
            continue
        left = max(0, int(np.floor(bbox[0])) - dilation_px)
        top = max(0, int(np.floor(bbox[1])) - dilation_px)
        right = min(width, int(np.ceil(bbox[2])) + dilation_px)
        bottom = min(height, int(np.ceil(bbox[3])) + dilation_px)
        mask_crop = mask[top:bottom, left:right].astype(np.uint8)
        dilated = (
            cv2.dilate(mask_crop, kernel).astype(bool)
            if kernel is not None
            else mask_crop.astype(bool)
        )
        features.append(
            _MaskFeatures(
                detection,
                mask,
                bbox,
                int(np.count_nonzero(mask)),
                dilated,
                np.array([left, top, right, bottom], dtype=np.int32),
            )
        )

    parents = list(range(len(features)))

    def find(index: int) -> int:
        while parents[index] != index:
            parents[index] = parents[parents[index]]
            index = parents[index]
        return index

    def union(first_index: int, second_index: int) -> None:
        first_root = find(first_index)
        second_root = find(second_index)
        if first_root != second_root:
            parents[second_root] = first_root

    for first_index, first in enumerate(features):
        if first.detection.class_name.lower() in excluded:
            continue
        for second_index in range(first_index + 1, len(features)):
            second = features[second_index]
            if first.detection.class_id != second.detection.class_id:
                continue
            if not _masks_are_close(
                first,
                second,
                float(config.get("minimum_mask_iou", 0.02)),
                float(config.get("maximum_mask_gap_px", 35.0)),
            ):
                continue
            union(first_index, second_index)

    groups: dict[int, list[_MaskFeatures]] = {}
    for index, feature in enumerate(features):
        groups.setdefault(find(index), []).append(feature)
    merged = []
    for group in groups.values():
        if len(group) == 1:
            merged.append(group[0].detection)
            continue
        merged_mask = np.logical_or.reduce([feature.mask for feature in group])
        bbox = mask_to_bbox(merged_mask, width, height)
        if bbox is None:
            continue
        best = max(group, key=lambda item: item.detection.confidence).detection
        merged.append(
            InstanceDetection(
                mask=merged_mask,
                class_id=best.class_id,
                class_name=best.class_name,
                confidence=max(item.detection.confidence for item in group),
                bbox_2d=bbox,
            )
        )
    return merged
