"""ROS integration test for the future ground-truth publisher."""

import time

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import numpy as np
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter

from lr_future_path.future_ground_truth_node import FutureGroundTruthNode
from lr_future_path.trajectory import PoseSeries


def _spin_until(executor, predicate, timeout=4.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.03)
        if predicate():
            return True
    return False


@pytest.fixture
def ros_context():
    """Provide a clean rclpy context for each integration test."""
    rclpy.init()
    try:
        yield
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def test_node_publishes_reanchored_future_path(ros_context):
    """A current pose triggers a correctly reanchored nav_msgs Path."""
    series = PoseSeries(
        stamps_ns=np.asarray(
            [10_000_000_000, 11_000_000_000, 12_000_000_000],
            dtype=np.int64,
        ),
        positions=np.asarray(
            [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]]
        ),
        orientations_xyzw=np.tile(
            np.array([0.0, 0.0, 0.0, 1.0]), (3, 1)
        ),
        frame_id="map",
    )
    node = FutureGroundTruthNode(
        pose_series=series,
        parameter_overrides=[
            Parameter("input_pose_topic", value="/test/current_pose"),
            Parameter("output_topic", value="/test/future_path"),
            Parameter("radius_m", value=15.0),
            Parameter("step_m", value=0.2),
        ],
    )
    driver = rclpy.create_node("future_path_test_driver")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(driver)
    publisher = driver.create_publisher(
        PoseStamped, "/test/current_pose", 10
    )
    received = []
    driver.create_subscription(Path, "/test/future_path", received.append, 10)

    try:
        assert _spin_until(
            executor, lambda: publisher.get_subscription_count() > 0
        )
        current = PoseStamped()
        current.header.frame_id = "map"
        current.header.stamp.sec = 10
        current.pose.position.x = 5.0
        current.pose.position.y = 3.0
        current.pose.orientation.w = 1.0
        publisher.publish(current)
        assert _spin_until(executor, lambda: bool(received))

        path = received[-1]
        assert path.header.frame_id == "map"
        assert [pose.pose.position.x for pose in path.poses] == pytest.approx(
            [5.0, 6.0, 7.0]
        )
        assert [pose.pose.position.y for pose in path.poses] == pytest.approx(
            [3.0, 3.0, 3.0]
        )
        assert path.poses[0].header.stamp.sec == 10
        assert path.poses[-1].header.stamp.sec == 12
    finally:
        executor.remove_node(node)
        executor.remove_node(driver)
        node.destroy_node()
        driver.destroy_node()
        executor.shutdown()
