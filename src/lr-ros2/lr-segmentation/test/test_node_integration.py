import threading
import time

import numpy as np
import pytest
import rclpy
from builtin_interfaces.msg import Time
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from vision_msgs.msg import Detection2DArray

from lr_segmentation.model import mask_to_bbox
from lr_segmentation.node import SegmentationNode
from lr_segmentation.processor import SegmentationFrameResult
from lr_segmentation.types import InstanceDetection


def spin_until(executor, predicate, timeout=4.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.03)
        if predicate():
            return True
    return False


def make_header(sec, nanosec=0, frame_id="camera_optical"):
    return Header(
        stamp=Time(sec=int(sec), nanosec=int(nanosec)),
        frame_id=frame_id,
    )


def make_image(sec, value=0, nanosec=0):
    pixels = np.full((8, 10, 3), value, dtype=np.uint8)
    message = Image()
    message.header = make_header(sec, nanosec)
    message.height = 8
    message.width = 10
    message.encoding = "bgr8"
    message.step = 30
    message.data = pixels.tobytes()
    return message


class FakeProcessor:
    def __init__(self, _model_config, _core_config):
        self.values = []

    def process(self, image):
        self.values.append(int(image[0, 0, 0]))
        mask = np.zeros(image.shape[:2], dtype=bool)
        mask[1:7, 2:8] = True
        detection = InstanceDetection(
            mask,
            2,
            "pipe",
            0.85,
            mask_to_bbox(mask, image.shape[1], image.shape[0]),
        )
        return SegmentationFrameResult((detection,), 1)


class SlowProcessor:
    def __init__(self, _model_config, _core_config):
        self.started = threading.Event()
        self.release = threading.Event()
        self.values = []

    def process(self, image):
        value = int(image[0, 0, 0])
        self.values.append(value)
        if len(self.values) == 1:
            self.started.set()
            assert self.release.wait(timeout=3.0)
        return SegmentationFrameResult.empty()


@pytest.fixture
def ros_context():
    rclpy.init()
    try:
        yield
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def test_image_only_node_publishes_resets_and_has_no_3d_api(
    ros_context, tmp_path
):
    model = tmp_path / "model.pt"
    model.touch()
    overrides = [
        Parameter("model.path", value=str(model)),
        Parameter("model.device", value="cpu"),
        Parameter("input.image_topic", value="/test/image"),
        Parameter("diagnostics.period_sec", value=0.05),
    ]
    node = SegmentationNode(
        parameter_overrides=overrides,
        processor_factory=FakeProcessor,
    )
    driver = rclpy.create_node("segmentation_test_driver")
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor.add_node(driver)

    input_qos = QoSProfile(depth=1)
    input_qos.reliability = ReliabilityPolicy.BEST_EFFORT
    image_publisher = driver.create_publisher(Image, "/test/image", input_qos)
    overlays = []
    detections_2d = []
    instance_masks = []
    diagnostics = []
    driver.create_subscription(
        Image,
        "/segmentation/overlay",
        overlays.append,
        input_qos,
    )
    driver.create_subscription(
        Detection2DArray,
        "/segmentation/detections_2d",
        detections_2d.append,
        input_qos,
    )
    driver.create_subscription(
        Image,
        "/segmentation/instance_masks",
        instance_masks.append,
        input_qos,
    )
    driver.create_subscription(
        DiagnosticArray,
        "/diagnostics",
        diagnostics.append,
        10,
    )

    try:
        assert spin_until(
            executor,
            lambda: image_publisher.get_subscription_count() > 0,
        )
        subscriptions = dict(
            driver.get_subscriber_names_and_types_by_node(
                "segmentation", "/"
            )
        )
        publishers = dict(
            driver.get_publisher_names_and_types_by_node(
                "segmentation", "/"
            )
        )
        assert subscriptions["/test/image"] == ["sensor_msgs/msg/Image"]
        assert all(
            "PointCloud2" not in message_type
            for types in subscriptions.values()
            for message_type in types
        )
        assert "/segmentation/detections_3d" not in publishers
        assert "/segmentation/markers" not in publishers

        image_publisher.publish(make_image(10, value=20))
        assert spin_until(
            executor,
            lambda: overlays and detections_2d and instance_masks,
        )
        assert overlays[-1].encoding == "rgb8"
        assert len(detections_2d[-1].detections) == 1
        assert instance_masks[-1].encoding == "mono16"
        labels = np.frombuffer(instance_masks[-1].data, dtype="<u2").reshape(
            8, 10
        )
        assert np.all(labels[1:7, 2:8] == 1)
        assert np.count_nonzero(labels) == 36
        assert spin_until(
            executor,
            lambda: diagnostics
            and diagnostics[-1].status[0].message == "Segmentation running"
            and any(
                value.key == "processed_count" and int(value.value) >= 1
                for value in diagnostics[-1].status[0].values
            ),
        )
        status = diagnostics[-1].status[0]
        values = {value.key: value.value for value in status.values}
        assert float(values["processed_rate_hz"]) > 0.0
        assert int(values["processed_count"]) >= 1
        assert int(values["merged_detection_count"]) == 1
        assert "cloud_rate_hz" not in values
        assert "valid_geometry_count" not in values
        assert float(values["processing_ms"]) >= 0.0

        processed = node._processed_count
        image_publisher.publish(make_image(5, value=40))
        assert spin_until(executor, lambda: node._reset_count >= 1)
        assert spin_until(executor, lambda: node._processed_count > processed)

        reset_client = driver.create_client(Trigger, "/segmentation/reset")
        assert reset_client.wait_for_service(timeout_sec=2.0)
        future = reset_client.call_async(Trigger.Request())
        assert spin_until(executor, future.done)
        assert future.result().success
        assert spin_until(
            executor,
            lambda: detections_2d
            and len(detections_2d[-1].detections) == 0
            and instance_masks
            and not np.any(np.frombuffer(
                instance_masks[-1].data,
                dtype="<u2",
            )),
        )
        assert spin_until(
            executor,
            lambda: diagnostics
            and any(
                value.key == "last_reset_reason"
                and value.value == "reset service"
                for value in diagnostics[-1].status[0].values
            ),
        )
    finally:
        executor.remove_node(node)
        executor.remove_node(driver)
        node.destroy_node()
        driver.destroy_node()
        executor.shutdown()


def test_latest_image_slot_replaces_pending_work(ros_context, tmp_path):
    model = tmp_path / "model.pt"
    model.touch()
    processor = None

    def factory(model_config, core_config):
        nonlocal processor
        processor = SlowProcessor(model_config, core_config)
        return processor

    node = SegmentationNode(
        parameter_overrides=[Parameter("model.path", value=str(model))],
        processor_factory=factory,
    )
    try:
        node._on_image(make_image(1, value=1))
        assert processor.started.wait(timeout=2.0)
        node._on_image(make_image(2, value=2))
        node._on_image(make_image(3, value=3))
        processor.release.set()
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and node._processed_count < 2:
            time.sleep(0.02)
        assert node._processed_count == 2
        assert processor.values == [1, 3]
        assert node._dropped_busy_count == 1
    finally:
        if processor is not None:
            processor.release.set()
        node.destroy_node()
