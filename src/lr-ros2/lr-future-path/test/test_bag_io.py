"""Tests for rosbag2 pose serialization."""

from geometry_msgs.msg import PoseStamped
from rclpy.serialization import serialize_message
import rosbag2_py

from lr_future_path.bag_io import (
    load_pose_series,
    POSE_TYPE,
)
from lr_future_path.cache import CACHE_TOPIC


def test_pose_bag_round_trip(tmp_path):
    """Cached poses round-trip through the rosbag2 format."""
    bag_path = tmp_path / "pose_bag"
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(
            uri=str(bag_path), storage_id="sqlite3"
        ),
        rosbag2_py.ConverterOptions("", ""),
    )
    writer.create_topic(
        rosbag2_py.TopicMetadata(
            name=CACHE_TOPIC,
            type=POSE_TYPE,
            serialization_format="cdr",
        )
    )
    for index in range(3):
        message = PoseStamped()
        message.header.frame_id = "map"
        message.header.stamp.sec = 10 + index
        message.pose.position.x = float(index)
        message.pose.orientation.w = 1.0
        writer.write(
            CACHE_TOPIC,
            serialize_message(message),
            (10 + index) * 1_000_000_000,
        )
    writer.close()

    series = load_pose_series(bag_path)

    assert series.frame_id == "map"
    assert series.stamps_ns.tolist() == [
        10_000_000_000,
        11_000_000_000,
        12_000_000_000,
    ]
    assert series.positions[:, 0].tolist() == [0.0, 1.0, 2.0]
