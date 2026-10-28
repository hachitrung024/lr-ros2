"""A live pose is causal and needs neither a database nor /clock."""

import json
import sqlite3
import time

import numpy as np
import rclpy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

from lr_mavros_bridge.node import MavrosPose


def test_live_origin_is_first_fix_and_survives_generation_without_db(monkeypatch):
    monkeypatch.setattr(sqlite3, 'connect', lambda *_a, **_k: (_ for _ in ()).throw(
        AssertionError('live backend opened a database')))
    rclpy.init(args=['--ros-args', '-p', 'time_mode:=live'])
    node = MavrosPose()
    try:
        start = time.time_ns()
        def generation(number, boundary):
            node.on_generation(String(data=json.dumps({
                'generation': number, 'start_ns': boundary, 'ready': True})))
        generation(1, start)
        gps = NavSatFix()
        gps.header.stamp.sec, gps.header.stamp.nanosec = divmod(start, 1_000_000_000)
        gps.status.status = 0
        gps.latitude, gps.longitude = 10., 106.
        gps.altitude = 100. + node.geoid_height(10., 106.)
        node.on_gps(gps)
        assert np.allclose(node.origin, [10., 106., 100.])
        assert len(node.estimator.gps) == 1
        origin = node.origin.copy()
        generation(2, start + 1_000_000)
        node.on_gps(gps)
        assert not node.estimator.gps
        np.testing.assert_array_equal(node.origin, origin)
        assert not node.accepted_stamp(start)
        assert node.accepted_stamp(start + 2_000_000)
        assert all(sub.topic_name != '/clock' for sub in node.subscriptions)
        # A respawned adapter restores the origin held by the supervisor.
        node.origin = None
        node.on_generation(String(data=json.dumps({
            'generation': 3, 'ready': True, 'start_ns': start,
            'origin': origin.tolist()})))
        np.testing.assert_array_equal(node.origin, origin)
    finally:
        node.destroy_node()
        rclpy.shutdown()
