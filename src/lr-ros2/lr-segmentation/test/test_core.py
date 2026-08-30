import sys
from types import SimpleNamespace

import numpy as np
import pytest
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Header
from visualization_msgs.msg import Marker

from lr_segmentation.depth_boxes import boxes_to_markers, estimate_boxes_3d
from lr_segmentation.model import Box2D, UltralyticsSegmentationModel


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


def test_model_returns_plot_and_2d_boxes(tmp_path, monkeypatch):
    checkpoint = tmp_path / "model.pt"
    checkpoint.touch()
    calls = {}

    class FakeBoxes:
        xyxy = np.asarray([[1.0, 2.0, 8.0, 9.0]])
        cls = np.asarray([3])
        conf = np.asarray([0.75])

    class FakeResult:
        boxes = FakeBoxes()
        names = {3: "box"}

        def plot(self):
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
    assert prediction.boxes == (
        Box2D((1.0, 2.0, 8.0, 9.0), 3, "box", 0.75),
    )


def test_2d_box_and_depth_create_3d_wireframe():
    depth = np.full((100, 100), 5.0, dtype=np.float32)
    depth[20:80, 20:80] = 2.0
    camera_info = CameraInfo()
    camera_info.width = 100
    camera_info.height = 100
    camera_info.k = [100.0, 0.0, 50.0, 0.0, 100.0, 50.0, 0.0, 0.0, 1.0]
    detection = Box2D((20.0, 20.0, 80.0, 80.0), 2, "pipe", 0.8)

    boxes = estimate_boxes_3d(
        (detection,),
        depth,
        camera_info,
        100,
        100,
        minimum_depth_m=0.2,
        maximum_depth_m=20.0,
        depth_tolerance_m=0.25,
        depth_tolerance_ratio=0.08,
        minimum_points=20,
    )

    assert len(boxes) == 1
    assert boxes[0].center[2] == pytest.approx(2.0)
    assert boxes[0].size[0] > 1.0
    assert boxes[0].size[1] > 1.0
    assert boxes[0].size[2] == pytest.approx(0.03)

    header = Header(frame_id="camera_optical")
    markers = boxes_to_markers(boxes, header).markers
    assert markers[0].action == Marker.DELETEALL
    assert markers[1].type == Marker.LINE_LIST
    assert len(markers[1].points) == 24


def test_3d_box_keeps_objects_with_a_large_center_depth_span():
    depth = np.full((80, 100), 12.0, dtype=np.float32)
    depth_levels = np.asarray([1.0, 2.3, 3.7, 5.0], dtype=np.float32)
    depth[10:70, 20:80] = np.resize(depth_levels, (60, 60))
    camera_info = CameraInfo()
    camera_info.width = 100
    camera_info.height = 80
    camera_info.k = [100.0, 0.0, 50.0, 0.0, 100.0, 40.0, 0.0, 0.0, 1.0]
    detection = Box2D((20.0, 10.0, 80.0, 70.0), 3, "vehicle", 0.98)

    boxes = estimate_boxes_3d(
        (detection,),
        depth,
        camera_info,
        100,
        80,
        minimum_depth_m=0.2,
        maximum_depth_m=20.0,
        depth_tolerance_m=0.25,
        depth_tolerance_ratio=0.08,
        minimum_points=20,
    )

    assert len(boxes) == 1
    assert boxes[0].size[2] > 3.5
