import numpy as np
import pytest
from builtin_interfaces.msg import Time
from sensor_msgs.msg import Image
from std_msgs.msg import Header

from lr_segmentation.conversions import (
    bgr_to_image_message,
    image_message_to_bgr,
)


def header():
    return Header(stamp=Time(sec=4, nanosec=25), frame_id="optical")


def image_message(encoding, values, padding=0):
    pixels = np.asarray(values, dtype=np.uint8)
    height, width, channels = pixels.shape
    step = width * channels + padding
    rows = np.zeros((height, step), dtype=np.uint8)
    rows[:, : width * channels] = pixels.reshape(height, width * channels)
    message = Image()
    message.header = header()
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
    encoding,
    pixel,
    expected,
):
    values = np.tile(np.asarray(pixel, np.uint8), (2, 3, 1))
    decoded = image_message_to_bgr(image_message(encoding, values, padding=5))
    assert decoded.shape == (2, 3, 3)
    assert decoded.flags.c_contiguous
    assert decoded[0, 0].tolist() == expected

    encoded = bgr_to_image_message(decoded, header())
    assert encoded.encoding == "rgb8"
    assert list(encoded.data[:3]) == [3, 2, 1]


def test_image_adapter_rejects_invalid_messages():
    message = Image(height=2, width=2, encoding="mono8", step=2, data=b"1234")
    with pytest.raises(ValueError, match="Unsupported"):
        image_message_to_bgr(message)

    message.encoding = "bgr8"
    message.step = 6
    with pytest.raises(ValueError, match="shorter"):
        image_message_to_bgr(message)

    with pytest.raises(ValueError, match="image_bgr"):
        bgr_to_image_message(np.zeros((2, 2), dtype=np.uint8), header())
