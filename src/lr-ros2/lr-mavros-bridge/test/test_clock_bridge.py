"""Check real UDP-to-ROS publication and fail on a second clock source."""

import json
import socket
import time
import uuid

import pytest
import rclpy
from rclpy.parameter import Parameter
from rosgraph_msgs.msg import Clock

from lr_mavros_bridge.clock_bridge_node import ReplayClockBridge


def test_udp_ros_clock_freezes_and_foreign_publisher_stops_bridge():
    rclpy.init(domain_id=157)
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    probe.bind(('127.0.0.1', 0))
    port = probe.getsockname()[1]
    probe.close()
    bridge = ReplayClockBridge(parameter_overrides=[
        Parameter('bind_address', value='127.0.0.1'), Parameter('bind_port', value=port),
    ])
    observer = rclpy.create_node('clock_observer')
    received = []
    observer.create_subscription(Clock, '/clock', lambda m: received.append(
        m.clock.sec * 1_000_000_000 + m.clock.nanosec), 10)
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        deadline = time.monotonic() + 3
        while not bridge.publisher.get_subscription_count() and time.monotonic() < deadline:
            rclpy.spin_once(observer, timeout_sec=.01)
        stamp = 1783700660123456789
        sender.sendto(json.dumps(dict(version=1, session_id=str(uuid.uuid4()),
                                     sequence=10, stamp_ns=stamp)).encode(), ('127.0.0.1', port))
        deadline = time.monotonic() + 3
        while not received and time.monotonic() < deadline:
            rclpy.spin_once(bridge, timeout_sec=.01)
            rclpy.spin_once(observer, timeout_sec=.01)
        assert received == [stamp]
        for _ in range(10):
            rclpy.spin_once(bridge, timeout_sec=.01)
            rclpy.spin_once(observer, timeout_sec=.01)
        assert received == [stamp]
        observer.create_publisher(Clock, '/clock', 10)
        deadline = time.monotonic() + 3
        with pytest.raises(RuntimeError, match='Another /clock publisher'):
            while time.monotonic() < deadline:
                rclpy.spin_once(bridge, timeout_sec=.01)
    finally:
        sender.close()
        observer.destroy_node()
        bridge.destroy_node()
        rclpy.shutdown()
