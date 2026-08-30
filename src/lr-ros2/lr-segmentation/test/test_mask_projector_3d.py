import time

import numpy as np
import pytest
import rclpy
from builtin_interfaces.msg import Time
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import Header

from lr_segmentation.conversions import labels_to_image_message
from lr_segmentation.mask_projector_3d import (
    MaskProjector3DNode,
    depth_message_to_meters,
    instance_mask_message_to_labels,
    project_mask_to_cloud,
)


CLOUD_DTYPE = np.dtype([
    ("x", "<f4"),
    ("y", "<f4"),
    ("z", "<f4"),
    ("rgb", "<f4"),
    ("instance_id", "<u4"),
])


def spin_until(executor, predicate, timeout=4.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.03)
        if predicate():
            return True
    return False


def camera_info(width=4, height=3):
    message = CameraInfo()
    message.header.frame_id = "camera_optical"
    message.width = width
    message.height = height
    message.p = [
        2.0, 0.0, 1.0, 0.0,
        0.0, 2.0, 1.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
    ]
    return message


def depth_message(values, header):
    depth = np.asarray(values, dtype="<f4")
    message = Image()
    message.header = header
    message.height, message.width = depth.shape
    message.encoding = "32FC1"
    message.step = message.width * 4
    message.data = depth.tobytes()
    return message


def test_image_decoders_and_projection_create_labeled_cloud():
    header = Header(
        stamp=Time(sec=12, nanosec=50),
        frame_id="camera_optical",
    )
    labels = np.asarray([
        [0, 1, 0, 0],
        [0, 1, 2, 0],
        [0, 0, 2, 0],
    ], dtype=np.uint16)
    depth = np.full(labels.shape, 2.0, dtype=np.float32)
    mask_message = labels_to_image_message(labels, header)
    registered_depth = depth_message(depth, header)

    decoded_labels = instance_mask_message_to_labels(mask_message)
    decoded_depth = depth_message_to_meters(registered_depth)
    cloud = project_mask_to_cloud(
        decoded_labels,
        decoded_depth,
        camera_info(),
        header,
        minimum_depth_m=0.2,
        maximum_depth_m=10.0,
        sampling_stride=1,
        maximum_points=100,
    )

    assert cloud.header == header
    assert cloud.width == 4
    assert cloud.point_step == CLOUD_DTYPE.itemsize
    assert [field.name for field in cloud.fields] == [
        "x", "y", "z", "rgb", "instance_id"
    ]
    points = np.frombuffer(cloud.data, dtype=CLOUD_DTYPE)
    assert points["instance_id"].tolist() == [1, 1, 2, 2]
    assert points["z"].tolist() == pytest.approx([2.0] * 4)
    assert points["x"].tolist() == pytest.approx([0.0, 0.0, 1.0, 1.0])


@pytest.fixture
def ros_context():
    rclpy.init()
    try:
        yield
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def test_projector_node_synchronizes_inputs_and_publishes_cloud(ros_context):
    node = MaskProjector3DNode(
        parameter_overrides=[
            Parameter("input.mask_topic", value="/projector_test/mask"),
            Parameter("input.depth_topic", value="/projector_test/depth"),
            Parameter(
                "input.camera_info_topic",
                value="/projector_test/camera_info",
            ),
            Parameter(
                "output.cloud_topic",
                value="/projector_test/mask_cloud",
            ),
        ],
        node_name="mask_projector_3d_test",
    )
    driver = rclpy.create_node("mask_projector_3d_test_driver")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(driver)

    sensor_qos = QoSProfile(depth=1)
    sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
    sensor_qos.durability = DurabilityPolicy.VOLATILE
    mask_publisher = driver.create_publisher(
        Image,
        "/projector_test/mask",
        sensor_qos,
    )
    depth_publisher = driver.create_publisher(
        Image,
        "/projector_test/depth",
        sensor_qos,
    )
    info_publisher = driver.create_publisher(
        CameraInfo,
        "/projector_test/camera_info",
        sensor_qos,
    )
    clouds = []
    driver.create_subscription(
        PointCloud2,
        "/projector_test/mask_cloud",
        clouds.append,
        sensor_qos,
    )

    try:
        assert spin_until(
            executor,
            lambda: (
                mask_publisher.get_subscription_count() > 0
                and depth_publisher.get_subscription_count() > 0
                and info_publisher.get_subscription_count() > 0
            ),
        )
        header = Header(
            stamp=Time(sec=20, nanosec=100),
            frame_id="camera_optical",
        )
        labels = np.zeros((3, 4), dtype=np.uint16)
        labels[1, 1:3] = [1, 2]
        info_publisher.publish(camera_info())
        mask_publisher.publish(labels_to_image_message(labels, header))
        depth_publisher.publish(
            depth_message(np.full((3, 4), 3.0, np.float32), header)
        )

        assert spin_until(executor, lambda: bool(clouds))
        assert clouds[-1].width == 2
        points = np.frombuffer(clouds[-1].data, dtype=CLOUD_DTYPE)
        assert points["instance_id"].tolist() == [1, 2]
    finally:
        executor.remove_node(node)
        executor.remove_node(driver)
        node.destroy_node()
        driver.destroy_node()
        executor.shutdown()
