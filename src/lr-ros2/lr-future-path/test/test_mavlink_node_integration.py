"""ROS integration tests for the MAVLink pose/TF/path publisher."""

from pathlib import Path
import time

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as PathMessage
import numpy as np
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage

from lr_future_path.mavlink import MavlinkTrajectory
from lr_future_path.mavlink_pose_node import MavlinkPoseNode


SECOND = 1_000_000_000


def _spin_until(executor, predicate, timeout=4.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.03)
        if predicate():
            return True
    return False


@pytest.fixture
def ros_context():
    """Provide a clean rclpy context for the integration test."""
    rclpy.init()
    try:
        yield
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def _trajectory():
    stamps = np.asarray(
        [10 * SECOND, 11 * SECOND, 12 * SECOND], dtype=np.int64
    )
    return MavlinkTrajectory(
        path=Path("injected.db"),
        gps_stamps_ns=stamps,
        gps_positions=np.asarray(
            [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]]
        ),
        attitude_stamps_ns=stamps,
        attitude_orientations_xyzw=np.tile(
            np.array([0.0, 0.0, 0.0, 1.0]), (3, 1)
        ),
    )


def _clock(seconds: int, nanoseconds: int = 0) -> Clock:
    message = Clock()
    message.clock.sec = seconds
    message.clock.nanosec = nanoseconds
    return message


def test_clock_publishes_pose_tf_path_and_supports_backward_seek(ros_context):
    """One SVO clock drives all outputs and a backward seek is stateless."""
    node = MavlinkPoseNode(
        trajectory=_trajectory(),
        parameter_overrides=[
            Parameter("pose_topic", value="/test/mavlink_pose"),
            Parameter("future_path_enabled", value=True),
            Parameter("future_path_topic", value="/test/future_path"),
            Parameter("map_frame", value="test_map"),
            Parameter("child_frame", value="test_camera_link"),
            Parameter("max_gps_gap_s", value=2.0),
            Parameter("step_m", value=0.2),
        ],
    )
    driver = rclpy.create_node("mavlink_pose_test_driver")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(driver)
    clock_publisher = driver.create_publisher(Clock, "/clock", 10)
    poses = []
    paths = []
    transforms = []
    driver.create_subscription(
        PoseStamped, "/test/mavlink_pose", poses.append, 10
    )
    driver.create_subscription(
        PathMessage, "/test/future_path", paths.append, 10
    )
    driver.create_subscription(
        TFMessage, "/tf", transforms.append, qos_profile_sensor_data
    )

    try:
        assert _spin_until(
            executor, lambda: clock_publisher.get_subscription_count() > 0
        )
        clock_publisher.publish(_clock(10, 500_000_000))
        assert _spin_until(
            executor,
            lambda: bool(poses) and bool(paths) and bool(transforms),
        )

        assert poses[-1].header.frame_id == "test_map"
        assert poses[-1].pose.position.x == pytest.approx(0.5)
        assert paths[-1].poses[0].pose.position.x == pytest.approx(0.5)
        assert paths[-1].poses[-1].pose.position.x == pytest.approx(2.0)
        transform = transforms[-1].transforms[-1]
        assert transform.header.frame_id == "test_map"
        assert transform.child_frame_id == "test_camera_link"
        assert transform.transform.translation.x == pytest.approx(0.5)

        pose_count = len(poses)
        clock_publisher.publish(_clock(20))
        for _ in range(5):
            executor.spin_once(timeout_sec=0.03)
        assert len(poses) == pose_count

        clock_publisher.publish(_clock(10, 250_000_000))
        assert _spin_until(executor, lambda: len(poses) > pose_count)
        assert poses[-1].pose.position.x == pytest.approx(0.25)
    finally:
        executor.remove_node(node)
        executor.remove_node(driver)
        node.destroy_node()
        driver.destroy_node()
        executor.shutdown()
