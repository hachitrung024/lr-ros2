import numpy as np
import pytest
from builtin_interfaces.msg import Time
from sensor_msgs.msg import Image
from std_msgs.msg import Header

from lr_segmentation.conversions import (
    bgr_to_image_message,
    empty_detection_array,
    image_message_to_bgr,
    result_to_detection_array,
    result_to_instance_mask_image,
)
from lr_segmentation.model import mask_to_bbox
from lr_segmentation.processor import SegmentationFrameResult
from lr_segmentation.types import InstanceDetection


def header(frame_id="camera", sec=4, nanosec=25):
    return Header(stamp=Time(sec=sec, nanosec=nanosec), frame_id=frame_id)


def image_message(encoding, values, padding=0):
    array = np.asarray(values, dtype=np.uint8)
    height, width, channels = array.shape
    step = width * channels + padding
    rows = np.zeros((height, step), dtype=np.uint8)
    rows[:, : width * channels] = array.reshape(height, width * channels)
    message = Image()
    message.header = header("optical")
    message.height = height
    message.width = width
    message.encoding = encoding
    message.step = step
    message.data = rows.tobytes()
    return message


@pytest.mark.parametrize(
    "encoding,pixel,expected",
    [
        ("bgr8", [1, 2, 3], [1, 2, 3]),
        ("rgb8", [3, 2, 1], [1, 2, 3]),
        ("bgra8", [1, 2, 3, 255], [1, 2, 3]),
        ("rgba8", [3, 2, 1, 255], [1, 2, 3]),
    ],
)
def test_image_adapter_supports_color_encodings_and_padding(
    encoding, pixel, expected
):
    values = np.tile(np.asarray(pixel, np.uint8), (2, 3, 1))
    decoded = image_message_to_bgr(image_message(encoding, values, padding=5))
    assert decoded.shape == (2, 3, 3)
    assert decoded.flags.c_contiguous
    assert decoded[0, 0].tolist() == expected
    encoded = bgr_to_image_message(decoded, header("optical"))
    assert encoded.encoding == "rgb8"
    assert list(encoded.data[:3]) == [3, 2, 1]


def test_image_adapter_rejects_unsupported_and_short_messages():
    message = Image(height=2, width=2, encoding="mono8", step=2, data=b"1234")
    with pytest.raises(ValueError, match="Unsupported"):
        image_message_to_bgr(message)
    message.encoding = "bgr8"
    message.step = 6
    with pytest.raises(ValueError, match="shorter"):
        image_message_to_bgr(message)


def sample_result():
    first_mask = np.ones((4, 6), dtype=bool)
    second_mask = np.zeros((4, 6), dtype=bool)
    second_mask[1, 1] = True
    detections = (
        InstanceDetection(
            first_mask,
            2,
            "pipe",
            0.8,
            mask_to_bbox(first_mask, 6, 4),
        ),
        InstanceDetection(
            second_mask,
            4,
            "box",
            0.6,
            mask_to_bbox(second_mask, 6, 4),
        ),
    )
    return SegmentationFrameResult(detections, 2)


def test_detection_array_preserves_ids_headers_boxes_and_hypotheses():
    image_header = header("optical", sec=7, nanosec=9)
    detections = result_to_detection_array(sample_result(), image_header)
    assert detections.header.frame_id == "optical"
    assert len(detections.detections) == 2
    first = detections.detections[0]
    assert first.id == "7000000009:0"
    assert first.results[0].hypothesis.class_id == "pipe"
    assert first.results[0].hypothesis.score == pytest.approx(0.8)
    assert first.bbox.center.position.x == pytest.approx(3.0)
    assert first.bbox.center.position.y == pytest.approx(2.0)
    assert first.bbox.size_x == pytest.approx(6.0)
    assert first.bbox.size_y == pytest.approx(4.0)


def test_instance_mask_image_maps_labels_to_detection_order():
    image_header = header("optical", sec=7, nanosec=9)
    message = result_to_instance_mask_image(
        sample_result(),
        image_header,
        4,
        6,
    )
    labels = np.frombuffer(message.data, dtype="<u2").reshape(4, 6)

    assert message.header == image_header
    assert message.encoding == "mono16"
    assert message.step == 12
    # Detection 0 has the higher confidence and owns the overlapping pixel.
    assert np.all(labels == 1)


def test_empty_instance_mask_image_is_publishable():
    message = result_to_instance_mask_image(
        SegmentationFrameResult.empty(),
        header("optical"),
        3,
        5,
    )
    labels = np.frombuffer(message.data, dtype="<u2").reshape(3, 5)
    assert not np.any(labels)


def test_instance_mask_image_resizes_model_resolution():
    low_resolution_mask = np.asarray([[1, 0], [0, 0]], dtype=bool)
    detection = InstanceDetection(
        low_resolution_mask,
        2,
        "pipe",
        0.8,
        mask_to_bbox(low_resolution_mask, 4, 4),
    )
    result = SegmentationFrameResult((detection,), 1)

    message = result_to_instance_mask_image(
        result,
        header("optical"),
        4,
        4,
    )
    labels = np.frombuffer(message.data, dtype="<u2").reshape(4, 4)

    assert np.all(labels[:2, :2] == 1)
    assert np.count_nonzero(labels) == 4


def test_empty_detection_array_retains_image_header():
    image_header = header("optical", sec=8, nanosec=10)
    detections = empty_detection_array(image_header)
    assert detections.header == image_header
    assert detections.detections == []
