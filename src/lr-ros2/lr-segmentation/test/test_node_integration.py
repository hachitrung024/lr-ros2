import time

import numpy as np
import pytest
import rclpy
from builtin_interfaces.msg import Time
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Header

from lr_segmentation.model import SegmentationPrediction
from lr_segmentation.node import SegmentationNode


def spin_until(executor, predicate, timeout=4.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.03)
        if predicate():
            return True
    return False


def make_header(frame_id="camera_optical"):
    return Header(stamp=Time(sec=10), frame_id=frame_id)


def make_color_image(value=0):
    pixels = np.full((8, 10, 3), value, dtype=np.uint8)
    message = Image()
    message.header = make_header()
    message.height = 8
    message.width = 10
    message.encoding = "bgr8"
    message.step = 30
    message.data = pixels.tobytes()
    return message


class FakeSegmentationModel:
    def predict(self, image_bgr):
        overlay = image_bgr.copy()
        overlay[1:7, 2:8] = [10, 20, 30]
        labels = np.zeros(image_bgr.shape[:2], dtype=np.uint16)
        labels[1:7, 2:8] = 4
        return SegmentationPrediction(overlay, labels)


@pytest.fixture
def ros_context():
    rclpy.init()
    try:
        yield
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def test_node_publishes_rgb_overlay(ros_context):
    node = SegmentationNode(
        parameter_overrides=[
            Parameter("model.path", value="unused.pt"),
            Parameter("model.device", value="cpu"),
            Parameter("input.image_topic", value="/test/image"),
        ],
        model_factory=lambda **_kwargs: FakeSegmentationModel(),
    )
    driver = rclpy.create_node("segmentation_test_driver")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(driver)

    image_qos = QoSProfile(depth=1)
    image_qos.reliability = ReliabilityPolicy.BEST_EFFORT
    image_publisher = driver.create_publisher(Image, "/test/image", image_qos)
    overlays = []
    masks = []
    driver.create_subscription(
        Image,
        "/segmentation/overlay",
        overlays.append,
        image_qos,
    )
    driver.create_subscription(
        Image,
        "/segmentation/instance_mask",
        masks.append,
        image_qos,
    )

    try:
        assert spin_until(
            executor,
            lambda: image_publisher.get_subscription_count() > 0,
        )
        publishers = dict(
            driver.get_publisher_names_and_types_by_node(
                "segmentation",
                "/",
            )
        )
        segmentation_outputs = {
            name: types
            for name, types in publishers.items()
            if name.startswith("/segmentation/")
        }
        assert segmentation_outputs == {
            "/segmentation/instance_mask": ["sensor_msgs/msg/Image"],
            "/segmentation/overlay": ["sensor_msgs/msg/Image"],
        }

        image_publisher.publish(make_color_image(value=5))
        assert spin_until(executor, lambda: bool(overlays) and bool(masks))

        overlay = overlays[-1]
        assert overlay.encoding == "rgb8"
        pixels = np.frombuffer(overlay.data, dtype=np.uint8).reshape(8, 10, 3)
        assert pixels[2, 3].tolist() == [30, 20, 10]

        mask = masks[-1]
        assert mask.encoding == "mono16"
        labels = np.frombuffer(mask.data, dtype="<u2").reshape(8, 10)
        assert labels[2, 3] == 4
        assert labels[0, 0] == 0

    finally:
        executor.remove_node(node)
        executor.remove_node(driver)
        node.destroy_node()
        driver.destroy_node()
        executor.shutdown()
