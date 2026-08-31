import time

import numpy as np
import pytest
import rclpy
from builtin_interfaces.msg import Time
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from vision_msgs.msg import Detection3DArray

from lr_segmentation.box_estimator_3d import (
    BoxEstimator3DNode,
    BoxMeasurement,
    BoxTracker,
    depth_message_to_meters,
    fit_oriented_box,
    instance_mask_message_to_labels,
)
from lr_segmentation.conversions import labels_to_image_message


def spin_until(executor, predicate, timeout=4.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.03)
        if predicate():
            return True
    return False


def camera_info(width=12, height=10):
    message = CameraInfo()
    message.header.frame_id = "camera_optical"
    message.width = width
    message.height = height
    message.p = [
        20.0, 0.0, 5.0, 0.0,
        0.0, 20.0, 4.0, 0.0,
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


def measurement(center):
    return BoxMeasurement(
        center=np.asarray(center, dtype=np.float64),
        size=np.asarray([1.0, 0.8, 0.6], dtype=np.float64),
        orientation=np.asarray([0.0, 0.0, 0.0, 1.0], dtype=np.float64),
    )


def test_fit_oriented_box_recovers_size_with_a_vertical_constraint():
    x_values = np.linspace(-2.0, 2.0, 9)
    y_values = np.linspace(-1.0, 1.0, 5)
    z_values = np.linspace(0.5, 1.5, 3)
    grid = np.meshgrid(x_values, y_values, z_values, indexing="ij")
    points = np.column_stack([value.reshape(-1) for value in grid])

    box = fit_oriented_box(
        points,
        np.asarray([0.0, 0.0, 1.0]),
        trim_fraction=0.0,
        minimum_size_m=0.01,
    )

    assert box.center == pytest.approx([0.0, 0.0, 1.0])
    assert box.size == pytest.approx([4.0, 2.0, 1.0])


def test_tracker_preserves_the_id_and_filters_position_noise():
    tracker = BoxTracker(
        association_distance_m=1.0,
        max_missed_frames=2,
        min_confirmations=1,
        measurement_stddev_m=0.08,
        acceleration_stddev_mps2=1.0,
        size_smoothing=0.5,
        orientation_smoothing=0.5,
    )

    first = tracker.update([measurement([0.0, 0.0, 2.0])], 1.0)
    second = tracker.update([measurement([0.2, 0.0, 2.0])], 1.1)

    assert first[0].track_id == second[0].track_id
    assert 0.0 < second[0].center[0] < 0.2


@pytest.fixture
def ros_context():
    rclpy.init()
    try:
        yield
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def test_box_node_publishes_standard_detection3d_array(ros_context):
    node = BoxEstimator3DNode(
        parameter_overrides=[
            Parameter("input.mask_topic", value="/box_test/mask"),
            Parameter("input.depth_topic", value="/box_test/depth"),
            Parameter("input.camera_info_topic", value="/box_test/camera_info"),
            Parameter("output.box_topic", value="/box_test/boxes_3d"),
            Parameter("output.marker_topic", value="/box_test/box_markers"),
            Parameter("sampling_stride", value=1),
            Parameter("minimum_points_per_instance", value=3),
            Parameter("box.trim_fraction", value=0.0),
            Parameter("tracking.min_confirmations", value=1),
        ],
        node_name="box_estimator_3d_test",
    )
    driver = rclpy.create_node("box_estimator_3d_test_driver")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(driver)

    sensor_qos = QoSProfile(depth=1)
    sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
    sensor_qos.durability = DurabilityPolicy.VOLATILE
    mask_publisher = driver.create_publisher(Image, "/box_test/mask", sensor_qos)
    depth_publisher = driver.create_publisher(Image, "/box_test/depth", sensor_qos)
    info_publisher = driver.create_publisher(
        CameraInfo,
        "/box_test/camera_info",
        sensor_qos,
    )
    detections = []
    driver.create_subscription(
        Detection3DArray,
        "/box_test/boxes_3d",
        detections.append,
        sensor_qos,
    )
    markers = []
    driver.create_subscription(
        MarkerArray,
        "/box_test/box_markers",
        markers.append,
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
        labels = np.zeros((10, 12), dtype=np.uint16)
        labels[2:8, 3:9] = 7
        info_publisher.publish(camera_info())
        mask_publisher.publish(labels_to_image_message(labels, header))
        depth_publisher.publish(
            depth_message(np.full(labels.shape, 2.0, np.float32), header)
        )

        assert spin_until(executor, lambda: bool(detections) and bool(markers))
        output = detections[-1]
        assert output.header.frame_id == "camera_optical"
        assert len(output.detections) == 1
        detection = output.detections[0]
        assert detection.id == "1"
        assert detection.bbox.center.position.z == pytest.approx(2.0)
        assert detection.bbox.size.x > 0.0
        assert detection.bbox.size.y > 0.0
        assert detection.bbox.size.z > 0.0
        assert [marker.type for marker in markers[-1].markers] == [
            Marker.ARROW,
            Marker.CUBE,
            Marker.TEXT_VIEW_FACING,
        ]
        assert markers[-1].markers[0].action == Marker.DELETEALL
    finally:
        executor.remove_node(node)
        executor.remove_node(driver)
        node.destroy_node()
        driver.destroy_node()
        executor.shutdown()


def test_image_decoders_preserve_mask_and_depth_values():
    header = Header(stamp=Time(sec=12), frame_id="camera_optical")
    labels = np.asarray([[0, 3], [4, 0]], dtype=np.uint16)
    mask = labels_to_image_message(labels, header)
    depth = depth_message(np.asarray([[1.0, 2.0], [3.0, 4.0]]), header)

    assert instance_mask_message_to_labels(mask).tolist() == [[0, 3], [4, 0]]
    assert np.allclose(
        depth_message_to_meters(depth),
        [[1.0, 2.0], [3.0, 4.0]],
    )
