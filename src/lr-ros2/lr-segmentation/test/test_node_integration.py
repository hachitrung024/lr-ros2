import time

import numpy as np
import pytest
import rclpy
from builtin_interfaces.msg import Time
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

from lr_segmentation.model import Box2D, SegmentationPrediction
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


def make_depth_image():
    depth = np.full((8, 10), 5.0, dtype=np.float32)
    depth[1:7, 2:8] = 2.0
    message = Image()
    message.header = make_header()
    message.height = 8
    message.width = 10
    message.encoding = "32FC1"
    message.step = 40
    message.data = depth.tobytes()
    return message


def make_camera_info():
    message = CameraInfo()
    message.header = make_header()
    message.height = 8
    message.width = 10
    message.k = [10.0, 0.0, 5.0, 0.0, 10.0, 4.0, 0.0, 0.0, 1.0]
    return message


class FakeSegmentationModel:
    def predict(self, image_bgr):
        overlay = image_bgr.copy()
        overlay[1:7, 2:8] = [10, 20, 30]
        detection = Box2D((2.0, 1.0, 8.0, 7.0), 2, "pipe", 0.85)
        return SegmentationPrediction(overlay, (detection,))


@pytest.fixture
def ros_context():
    rclpy.init()
    try:
        yield
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def test_node_publishes_rgb_overlay_and_depth_based_3d_boxes(ros_context):
    node = SegmentationNode(
        parameter_overrides=[
            Parameter("model.path", value="unused.pt"),
            Parameter("model.device", value="cpu"),
            Parameter("input.image_topic", value="/test/image"),
            Parameter("input.depth_topic", value="/test/depth"),
            Parameter("input.camera_info_topic", value="/test/camera_info"),
            Parameter("box3d.minimum_points", value=5),
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
    depth_publisher = driver.create_publisher(Image, "/test/depth", image_qos)
    info_publisher = driver.create_publisher(CameraInfo, "/test/camera_info", 10)
    overlays = []
    marker_arrays = []
    driver.create_subscription(
        Image,
        "/segmentation/overlay",
        overlays.append,
        image_qos,
    )
    driver.create_subscription(
        MarkerArray,
        "/segmentation/boxes_3d",
        marker_arrays.append,
        image_qos,
    )

    try:
        assert spin_until(
            executor,
            lambda: (
                image_publisher.get_subscription_count() > 0
                and depth_publisher.get_subscription_count() > 0
                and info_publisher.get_subscription_count() > 0
            ),
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
            "/segmentation/boxes_3d": [
                "visualization_msgs/msg/MarkerArray"
            ],
            "/segmentation/overlay": ["sensor_msgs/msg/Image"],
        }

        info_publisher.publish(make_camera_info())
        image_publisher.publish(make_color_image(value=5))
        depth_publisher.publish(make_depth_image())
        assert spin_until(
            executor,
            lambda: bool(overlays) and bool(marker_arrays),
        )

        overlay = overlays[-1]
        assert overlay.encoding == "rgb8"
        pixels = np.frombuffer(overlay.data, dtype=np.uint8).reshape(8, 10, 3)
        assert pixels[2, 3].tolist() == [30, 20, 10]

        markers = marker_arrays[-1].markers
        assert markers[0].action == Marker.DELETEALL
        assert len(markers) == 2
        box = markers[1]
        assert box.header.frame_id == "camera_optical"
        assert box.type == Marker.LINE_LIST
        assert len(box.points) == 24
        assert min(point.z for point in box.points) == pytest.approx(1.985)
        assert max(point.z for point in box.points) == pytest.approx(2.015)
    finally:
        executor.remove_node(node)
        executor.remove_node(driver)
        node.destroy_node()
        driver.destroy_node()
        executor.shutdown()
