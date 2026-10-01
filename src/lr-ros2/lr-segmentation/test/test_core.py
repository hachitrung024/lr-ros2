import sys
from types import SimpleNamespace

import numpy as np
import pytest
from lr_segmentation.model import UltralyticsSegmentationModel, _instance_labels


@pytest.mark.parametrize(
    "overrides,match",
    [
        ({"device": ""}, "device"),
        ({"confidence": -0.1}, "confidence"),
        ({"confidence": 1.1}, "confidence"),
        ({"iou": -0.1}, "iou"),
        ({"iou": 1.1}, "iou"),
        ({"image_size": 0}, "image_size"),
    ],
)
def test_model_rejects_invalid_settings(tmp_path, overrides, match):
    values = {
        "model_path": str(tmp_path / "model.pt"),
        "device": "cpu",
        "confidence": 0.25,
        "image_size": 640,
        "iou": 0.6,
    }
    (tmp_path / "model.pt").touch()
    values.update(overrides)
    with pytest.raises(ValueError, match=match):
        UltralyticsSegmentationModel(**values)


def test_model_returns_rendered_overlay(tmp_path, monkeypatch):
    checkpoint = tmp_path / "model.pt"
    checkpoint.touch()
    calls = {}

    class FakeResult:
        masks = SimpleNamespace(data=np.asarray([
            [
                [0, 0, 0, 0, 0, 0],
                [0, 1, 1, 1, 0, 0],
                [0, 1, 1, 1, 0, 0],
                [0, 1, 1, 1, 0, 0],
                [0, 0, 0, 0, 0, 0],
            ],
            [
                [0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0],
                [0, 0, 1, 1, 1, 0],
                [0, 0, 1, 1, 1, 0],
                [0, 0, 1, 1, 1, 0],
            ],
        ], dtype=np.float32))
        boxes = SimpleNamespace(conf=np.asarray([0.9, 0.8]))

        def plot(self):
            calls['plot'] = calls.get('plot', 0) + 1
            return np.full((10, 12, 3), 17, dtype=np.uint8)

    class FakeYolo:
        def __init__(self, model_path):
            calls["model_path"] = model_path

        def predict(self, **kwargs):
            calls["predict"] = kwargs
            return [FakeResult()]

    monkeypatch.setitem(
        sys.modules,
        "ultralytics",
        SimpleNamespace(YOLO=FakeYolo),
    )
    model = UltralyticsSegmentationModel(
        model_path=str(checkpoint),
        device="cpu",
        confidence=0.4,
        image_size=640,
        iou=0.5,
    )
    source = np.zeros((10, 12, 3), dtype=np.uint8)
    prediction = model.predict(source)

    assert calls["model_path"] == str(checkpoint.resolve())
    assert calls["predict"]["source"] is source
    assert prediction.overlay_bgr.shape == source.shape
    assert prediction.instance_labels.shape == source.shape[:2]
    assert prediction.instance_labels.dtype == np.uint16
    assert set(np.unique(prediction.instance_labels)) == {0, 1, 2}
    assert prediction.instance_labels[5, 5] == 1

    assert calls['plot'] == 1
    no_overlay = model.predict(source, render_overlay=False)
    assert no_overlay.overlay_bgr is None
    assert calls['plot'] == 1
    np.testing.assert_array_equal(no_overlay.instance_labels, prediction.instance_labels)


@pytest.mark.parametrize('portrait', [False, True])
def test_masks_remove_stride_padding_before_alignment_with_depth(portrait):
    # 1280x720 at imgsz=960 becomes 960x540 plus two rows of padding on each side.
    mask = np.zeros((544, 960), dtype=np.float32)
    mask[2:92] = 1
    if portrait:
        mask = mask.T
    height, width = (1280, 720) if portrait else (720, 1280)
    result = SimpleNamespace(masks=SimpleNamespace(data=mask[None]), boxes=None)
    labels = _instance_labels(result, height, width)
    if portrait:
        labels = labels.T
    expected = np.zeros((720, 1280), dtype=np.uint16)
    expected[:120] = 1
    np.testing.assert_array_equal(labels, expected)


def test_odd_letterbox_padding_and_overlapping_instances():
    masks = np.zeros((2, 9, 8), dtype=np.float32)
    masks[0, 2:6, :5] = 1
    masks[1, 2:6, 3:] = 1
    result = SimpleNamespace(
        masks=SimpleNamespace(data=masks), boxes=SimpleNamespace(conf=np.array([0.4, 0.9]))
    )
    labels = _instance_labels(result, 4, 8)
    np.testing.assert_array_equal(labels, np.tile([1, 1, 1, 2, 2, 2, 2, 2], (4, 1)))
