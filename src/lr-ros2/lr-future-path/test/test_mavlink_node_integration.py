"""ROS integration checks for reference-only DB output in the NET architecture."""

from pathlib import Path
import time

from geographic_msgs.msg import GeoPointStamped
from nav_msgs.msg import Path as PathMessage
import numpy as np
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from rosgraph_msgs.msg import Clock

from lr_future_path.mavlink import MavlinkTrajectory
from lr_future_path.mavlink_ground_truth_node import MavlinkGroundTruthNode


SECOND = 1_000_000_000


def spin_until(executor, predicate, timeout=4.0):
    """Wait for DDS discovery and delivery without changing recorded time."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.03)
        if predicate():
            return True
    return False


@pytest.fixture
def ros_context():
    """Provide an isolated ROS context."""
    rclpy.init()
    try:
        yield
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def trajectory():
    """Create reference motion with a fixed ENU origin, independent of the estimator."""
    stamps = np.array([10, 11, 12], dtype=np.int64) * SECOND
    return MavlinkTrajectory(
        path=Path('injected.db'), gps_stamps_ns=stamps,
        gps_positions=np.array([[0., 0., 0.], [1., 0., 0.], [2., 0., 0.]]),
        attitude_stamps_ns=stamps,
        attitude_orientations_xyzw=np.tile([0., 0., 0., 1.], (3, 1)),
        origin_lla=np.array([34.2, -118.3, 506.45]),
    )


def clock(seconds, nanoseconds=0):
    """Build a recorded clock frame."""
    message = Clock()
    message.clock.sec = seconds
    message.clock.nanosec = nanoseconds
    return message


def test_ground_truth_publishes_latched_origin_path_and_handles_rewind_without_pose(ros_context):
    """Only MAVROS may own camera pose/TF; the DB remains an offline reference."""
    node = MavlinkGroundTruthNode(trajectory=trajectory(), parameter_overrides=[
        Parameter('use_sim_time', value=True), Parameter('future_path_enabled', value=True),
        Parameter('future_path_topic', value='/test/future_path'),
        Parameter('map_frame', value='test_map'), Parameter('max_gps_gap_s', value=2.0),
    ])
    driver = rclpy.create_node('reference_test_driver')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(driver)
    publisher = driver.create_publisher(Clock, '/clock', 10)
    paths, origins = [], []
    driver.create_subscription(PathMessage, '/test/future_path', paths.append,
                               QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    driver.create_subscription(GeoPointStamped, '/lr/mavlink/origin', origins.append,
                               QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    try:
        assert spin_until(executor, lambda: bool(origins)
                          and publisher.get_subscription_count() > 0)
        assert origins[0].header.frame_id == 'test_map'
        assert origins[0].position.altitude == pytest.approx(506.45)
        publisher.publish(clock(10, 500_000_000))
        assert spin_until(executor, lambda: bool(paths) and bool(paths[-1].poses))
        assert paths[-1].poses[0].pose.position.x == pytest.approx(.5)
        assert paths[-1].poses[-1].pose.position.x == pytest.approx(2.)
        publisher.publish(clock(10, 250_000_000))
        assert spin_until(executor, lambda: bool(paths[-1].poses)
                          and paths[-1].header.stamp.nanosec == 250_000_000)
        assert paths[-1].poses[0].pose.position.x == pytest.approx(.25)
        topics = {name for name, _ in driver.get_publisher_names_and_types_by_node(
            node.get_name(), node.get_namespace())}
        assert '/lr/mavlink/pose' not in topics
        assert '/tf' not in topics
        assert '/tf_static' not in topics
    finally:
        executor.remove_node(node)
        executor.remove_node(driver)
        node.destroy_node()
        driver.destroy_node()
        executor.shutdown()


def test_origin_is_available_when_future_path_is_disabled(ros_context):
    """Pose initialization still has its fixed origin without enabling a reference path."""
    node = MavlinkGroundTruthNode(trajectory=trajectory())
    try:
        assert node._future_publisher is None
        node._clock_callback(clock(10, 500_000_000))
        assert not hasattr(node, '_pose_publisher')
        assert not hasattr(node, '_tf_broadcaster')
    finally:
        node.destroy_node()
