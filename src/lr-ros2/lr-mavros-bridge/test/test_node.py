"""Exercise ROS message construction, readiness and rewind reset."""

import rclpy
from geographic_msgs.msg import GeoPointStamped
from rosgraph_msgs.msg import Clock
from std_msgs.msg import UInt64
from lr_mavros_bridge.node import MavrosPose


def test_diagnostics_and_reset():
    rclpy.init(domain_id=159)
    node = MavrosPose()
    try:
        node.diagnostics()
        node.on_epoch(UInt64(data=1783700660000000))
        m = GeoPointStamped()
        m.header.frame_id = "map"
        m.position.latitude, m.position.longitude, m.position.altitude = 34.0, -118.0, 500.0
        node.on_origin(m)
        c = Clock()
        c.clock.sec = 1783700661
        node.on_clock(c)
        c.clock.sec -= 1
        node.on_clock(c)
        assert len(node.pending) == 1
        assert node.pending[0][0] == 1783700660000000000
        assert not node.estimator.gps
        node.on_epoch(UInt64(data=0))
        assert not node.accepted_stamp(1783700660000000000)
        node.diagnostics()
    finally:
        node.destroy_node()
        rclpy.shutdown()
