import sys
from types import SimpleNamespace

import numpy as np
import pytest

from lr_segmentation.model import UltralyticsOverlayModel


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
        UltralyticsOverlayModel(**values)


def test_model_returns_ultralytics_plot(tmp_path, monkeypatch):
    checkpoint = tmp_path / "model.pt"
    checkpoint.touch()
    calls = {}

    class FakeResult:
        def plot(self):
            return np.full((3, 4, 3), 17, dtype=np.uint8)

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
    model = UltralyticsOverlayModel(
        model_path=str(checkpoint),
        device="cpu",
        confidence=0.4,
        image_size=640,
        iou=0.5,
    )
    source = np.zeros((3, 4, 3), dtype=np.uint8)
    overlay = model.predict_overlay(source)

    assert calls["model_path"] == str(checkpoint.resolve())
    assert calls["predict"]["source"] is source
    assert calls["predict"]["device"] == "cpu"
    assert calls["predict"]["conf"] == 0.4
    assert overlay.shape == source.shape
    assert np.all(overlay == 17)
