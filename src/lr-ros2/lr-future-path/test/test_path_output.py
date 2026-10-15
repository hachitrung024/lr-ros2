"""ROS checks for path retention, late subscribers, pauses and clock rewinds."""

import time

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time

from lr_future_path.path_output import PathOutput


def test_latched_path_retention_uses_ros_time_and_resets_on_rewind():
    """Keep a paused path for late RViz readers; clear real gaps and old timelines."""
    rclpy.init()
    source = rclpy.create_node(
        'path_output_test', parameter_overrides=[Parameter('use_sim_time', value=True)],
    )
    driver = rclpy.create_node('path_output_reader')
    executor = SingleThreadedExecutor()
    executor.add_node(source)
    executor.add_node(driver)
    received = []
    try:
        output = PathOutput(source, '/test/retained_path', radius_m=5., step_m=.2)
        clock = source.get_clock()
        clock.set_ros_time_override(Time(seconds=10))
        message = Path()
        message.header.frame_id = 'map'
        message.header.stamp.sec = 10
        for x in (0., 2.):
            pose = PoseStamped(header=message.header)
            pose.pose.position.x = x
            pose.pose.orientation.w = 1.
            message.poses.append(pose)
        output.publish(message)
        # The reader appears after publication while simulation is paused.
        driver.create_subscription(
            Path, '/test/retained_path', received.append,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )

        def spin_until(predicate):
            deadline = time.monotonic() + 3.
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=.03)
                if predicate():
                    return True
            return False

        assert spin_until(lambda: received and len(received[-1].poses) > 1)
        count = len(received)
        for _ in range(3):
            output._expire()
            executor.spin_once(timeout_sec=.03)
        assert len(received) == count
        clock.set_ros_time_override(Time(seconds=11))
        output.publish(Path(header=message.header, poses=message.poses[:1]))
        output._expire()
        executor.spin_once(timeout_sec=.03)
        assert len(received) == count  # A one-point update does not erase the visible line.
        assert received[-1].header.stamp.sec == 10  # Retention never makes old input fresh.
        clock.set_ros_time_override(Time(seconds=12))
        output._expire()
        assert spin_until(lambda: not received[-1].poses)
        output.publish(message)
        assert spin_until(lambda: bool(received[-1].poses))
        clock.set_ros_time_override(Time(seconds=5))
        assert spin_until(lambda: not received[-1].poses)
    finally:
        executor.shutdown()
        source.destroy_node()
        driver.destroy_node()
        rclpy.shutdown()
