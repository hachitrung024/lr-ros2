import tempfile
from pathlib import Path

import numpy as np
import pytest

from lr_segmentation.config import CoreConfig, SegmentationConfig
from lr_segmentation.merge import merge_instance_detections
from lr_segmentation.model import mask_to_bbox
from lr_segmentation.processor import SegmentationProcessor
from lr_segmentation.types import InstanceDetection
from lr_segmentation.visualization import draw_instances, resize_binary_mask


def detection(mask, class_id=2, class_name="pipe", confidence=0.8):
    box = mask_to_bbox(mask, mask.shape[1], mask.shape[0])
    return InstanceDetection(
        np.asarray(mask, dtype=bool),
        class_id,
        class_name,
        confidence,
        box,
    )


def test_segmentation_config_rejects_missing_model_and_invalid_values():
    with pytest.raises(ValueError, match="model.path"):
        SegmentationConfig("").validate()
    with pytest.raises(FileNotFoundError):
        SegmentationConfig("missing.pt").validate()
    for values in (
        {"confidence": -0.1},
        {"confidence": 1.1},
        {"iou": -0.1},
        {"iou": 1.1},
        {"image_size": 0},
        {"classes": (-1,)},
    ):
        with pytest.raises(ValueError):
            SegmentationConfig("unused.pt", **values).validate(
                require_model=False
            )


def test_core_config_rejects_invalid_image_merge_values():
    for config in (
        CoreConfig(mask_dilation_px=-1),
        CoreConfig(minimum_mask_iou=-0.1),
        CoreConfig(minimum_mask_iou=1.1),
        CoreConfig(maximum_mask_gap_px=-0.1),
        CoreConfig(maximum_mask_gap_px=float("nan")),
    ):
        with pytest.raises(ValueError):
            config.validate()


def test_mask_to_bbox_and_resize_are_image_only():
    mask = np.zeros((4, 5), dtype=bool)
    mask[1:3, 2:4] = True
    box = mask_to_bbox(mask, 10, 8)
    resized = resize_binary_mask(mask, 10, 8)
    assert np.allclose(box, [4, 2, 8, 6])
    assert resized.shape == (8, 10)
    assert resized.dtype == bool
    assert mask_to_bbox(np.zeros((2, 2), dtype=bool), 2, 2) is None
    with pytest.raises(ValueError, match="two-dimensional"):
        mask_to_bbox(np.zeros((2, 2, 1)), 2, 2)


def test_class_aware_merge_uses_pixel_proximity_only():
    first = np.zeros((20, 20), dtype=bool)
    second = np.zeros((20, 20), dtype=bool)
    first[4:16, 3:8] = True
    second[4:16, 9:14] = True
    merged = merge_instance_detections(
        [detection(first, confidence=0.6), detection(second, confidence=0.9)],
        (20, 20),
        CoreConfig().merge_settings,
    )
    assert len(merged) == 1
    assert merged[0].confidence == pytest.approx(0.9)
    assert np.count_nonzero(merged[0].mask) == (
        np.count_nonzero(first) + np.count_nonzero(second)
    )


def test_merge_respects_class_exclusions_and_distance():
    first = np.zeros((40, 80), dtype=bool)
    nearby = np.zeros((40, 80), dtype=bool)
    far = np.zeros((40, 80), dtype=bool)
    first[5:20, 3:10] = True
    nearby[5:20, 11:18] = True
    far[5:20, 60:67] = True
    config = CoreConfig(maximum_mask_gap_px=5.0).merge_settings

    people = [
        detection(first, 0, "person"),
        detection(nearby, 0, "person"),
    ]
    assert len(merge_instance_detections(people, (40, 80), config)) == 2

    different_classes = [
        detection(first, 2, "pipe"),
        detection(nearby, 3, "box"),
    ]
    assert len(
        merge_instance_detections(different_classes, (40, 80), config)
    ) == 2

    separated = [detection(first), detection(far)]
    assert len(merge_instance_detections(separated, (40, 80), config)) == 2


def test_processor_accepts_only_an_image_and_merges_detections():
    first = np.zeros((20, 20), dtype=bool)
    second = np.zeros((20, 20), dtype=bool)
    first[3:17, 3:8] = True
    second[3:17, 9:14] = True

    class FakeSegmenter:
        def predict(self, _image):
            return [
                detection(first, confidence=0.7),
                detection(second, confidence=0.9),
            ]

    with tempfile.TemporaryDirectory() as temporary:
        model = Path(temporary) / "model.pt"
        model.touch()
        processor = SegmentationProcessor(
            SegmentationConfig(str(model), device="cpu"),
            CoreConfig(),
            segmenter_factory=lambda _config: FakeSegmenter(),
        )
        result = processor.process(np.zeros((20, 20, 3), dtype=np.uint8))
        assert result.raw_detection_count == 2
        assert result.merged_detection_count == 1
        assert len(result.detections) == 1
        with pytest.raises(ValueError, match="image_bgr"):
            processor.process(np.zeros((20, 20), dtype=np.uint8))


def test_overlay_changes_mask_pixels_but_preserves_shape():
    mask = np.zeros((10, 12), dtype=bool)
    mask[2:8, 3:9] = True
    image = np.zeros((10, 12, 3), dtype=np.uint8)
    overlay = draw_instances(image, [detection(mask)])
    assert overlay.shape == image.shape
    assert overlay.dtype == np.uint8
    assert np.any(overlay[mask] != 0)
