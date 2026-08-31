"""Rosbag2 serialization helpers for cached poses."""

from __future__ import annotations

from pathlib import Path

from geometry_msgs.msg import PoseStamped
import numpy as np
from rclpy.serialization import deserialize_message
import rosbag2_py

from .cache import CACHE_TOPIC
from .trajectory import normalize_quaternion, PoseSeries


POSE_TYPE = "geometry_msgs/msg/PoseStamped"


def stamp_to_nanoseconds(stamp) -> int:
    """Convert a builtin_interfaces Time message to integer nanoseconds."""
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def nanoseconds_to_stamp(value: int, stamp) -> None:
    """Populate a builtin_interfaces Time message from nanoseconds."""
    stamp.sec = int(value) // 1_000_000_000
    stamp.nanosec = int(value) % 1_000_000_000


def load_pose_series(cache_path: str | Path) -> PoseSeries:
    """Load the internal PoseStamped topic from a rosbag cache."""
    path = Path(cache_path).expanduser().resolve()
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    if topic_types.get(CACHE_TOPIC) != POSE_TYPE:
        raise ValueError(
            f"cache topic {CACHE_TOPIC} is missing or has the wrong type"
        )

    stamps = []
    positions = []
    orientations = []
    frame_id = ""
    while reader.has_next():
        topic, serialized, _bag_timestamp = reader.read_next()
        if topic != CACHE_TOPIC:
            continue
        message = deserialize_message(serialized, PoseStamped)
        timestamp = stamp_to_nanoseconds(message.header.stamp)
        if stamps and timestamp <= stamps[-1]:
            raise ValueError("cache pose timestamps are not strictly increasing")
        if frame_id and message.header.frame_id != frame_id:
            raise ValueError("cache pose frame_id changes within the bag")
        frame_id = message.header.frame_id
        stamps.append(timestamp)
        positions.append(
            [
                message.pose.position.x,
                message.pose.position.y,
                message.pose.position.z,
            ]
        )
        orientations.append(
            normalize_quaternion(
                np.array(
                    [
                        message.pose.orientation.x,
                        message.pose.orientation.y,
                        message.pose.orientation.z,
                        message.pose.orientation.w,
                    ],
                    dtype=np.float64,
                )
            )
        )
    return PoseSeries(
        stamps_ns=np.asarray(stamps, dtype=np.int64),
        positions=np.asarray(positions, dtype=np.float64),
        orientations_xyzw=np.asarray(orientations, dtype=np.float64),
        frame_id=frame_id,
    )
