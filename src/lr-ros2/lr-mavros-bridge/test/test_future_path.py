"""Check ENU, integer time, session isolation and stale path clearing without DB."""

import json
import uuid

from geographic_msgs.msg import GeoPointStamped
import pytest
import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import String, UInt8MultiArray

from lr_mavros_bridge.future_path_node import MavlinkFuturePath
from lr_mavros_bridge.path_protocol import PathPoint, Snapshot, fragments


def state(node, session, generation=1, start=None):
    now = node.get_clock().now().nanoseconds
    node.guard.on_state(String(data=json.dumps({
        'runtime_id': 'test-runtime', 'generation': generation, 'ready': True,
        'start_ns': start or now - 1_000_000_000, 'session_id': session})))
    node.on_time(String(data=json.dumps({
        'version': 1, 'offset_ns': now - 5_000_000_123, 'sync_ready': True,
        'session_id': session})))
    return now


def path(session, sequence=0, boot=5_000_000_123):
    return Snapshot(session, sequence, boot, tuple(
        PathPoint(boot + i * 1_000_000_001, 340000000, -1180000000 + i * 10, 500000,
                  (0., 0., 0., 1.)) for i in range(8)))


def origin(node, latitude=34.):
    message = GeoPointStamped()
    message.header.frame_id = 'map'
    message.position.latitude = latitude
    message.position.longitude = -118.
    message.position.altitude = 500.
    node.on_origin(message)


@pytest.fixture
def receiver(monkeypatch):
    # Runtime must never consult a recording on the receiving machine.
    import sqlite3
    monkeypatch.setattr(sqlite3, 'connect', lambda *_a, **_k: pytest.fail('Jetson opened DB'))
    rclpy.init()
    node = MavlinkFuturePath(parameter_overrides=[Parameter('step_m', value=.1)])
    output = []
    node.output = type('Sink', (), {'publish': lambda _self, msg: output.append(msg)})()
    try:
        yield node, output
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_received_path_uses_fixed_origin_and_exact_integer_mapping(receiver):
    node, output = receiver
    session = uuid.uuid4().hex
    now = state(node, session)
    origin(node)
    source = path(session)
    for packet in reversed(fragments(source)):
        node.on_fragment(UInt8MultiArray(data=packet))
    assert len(output) == 1
    result = output[-1]
    assert len(result.poses) >= 2
    assert result.header.stamp.sec * 1_000_000_000 + result.header.stamp.nanosec == now
    first = result.poses[0]
    assert [first.pose.position.x, first.pose.position.y, first.pose.position.z] == pytest.approx(
        [0., 0., 0.], abs=1e-7)
    assert result.poses[-1].pose.position.x > 0
    assert all(p.header.frame_id == 'map' for p in result.poses)
    assert result.poses[-1].header.stamp.sec >= result.header.stamp.sec
    origin(node, latitude=35.)
    assert node.origin == pytest.approx([34., -118., 500.])
    # Complete duplicates and malformed fragments produce no extra path.
    for packet in fragments(source):
        node.on_fragment(UInt8MultiArray(data=packet))
    node.on_fragment(UInt8MultiArray(data=b'broken'))
    assert len(output) == 1 and node.rejected == 1
    node.last_seen -= 1.1
    node.expire()
    assert not output[-1].poses


def test_restart_rejects_old_session_and_clears_incomplete_work(receiver):
    node, output = receiver
    old, new = uuid.uuid4().hex, uuid.uuid4().hex
    state(node, old)
    origin(node)
    for packet in fragments(path(old)):
        node.on_fragment(UInt8MultiArray(data=packet))
    assert output[-1].poses
    state(node, new, generation=2)
    node.expire()
    assert not output[-1].poses
    before = len(output)
    for packet in fragments(path(old, sequence=9999)):
        node.on_fragment(UInt8MultiArray(data=packet))
    assert len(output) == before
    for packet in fragments(path(new))[:-1]:
        node.on_fragment(UInt8MultiArray(data=packet))
    assert len(output) == before
    node.on_fragment(UInt8MultiArray(data=fragments(path(new))[-1]))
    assert output[-1].poses
    assert node.origin == pytest.approx([34., -118., 500.])


def test_early_new_metadata_cannot_cross_generation_boundary(receiver):
    node, output = receiver
    old, new = uuid.uuid4().hex, uuid.uuid4().hex
    state(node, old)
    origin(node)
    node.state['session_id'] = new  # Runtime has not approved the new session yet.
    for packet in fragments(path(new)):
        node.on_fragment(UInt8MultiArray(data=packet))
    assert not output


def test_malicious_time_does_not_overflow_ros_message(receiver):
    node, output = receiver
    session = uuid.uuid4().hex
    state(node, session)
    origin(node)
    for packet in fragments(Snapshot(session, 0, 2**64 - 1, ())):
        node.on_fragment(UInt8MultiArray(data=packet))
    assert not output and node.rejected == 1


def test_protocol_copy_is_identical_when_workspace_contains_host_tool():
    from pathlib import Path
    import lr_mavros_bridge.path_protocol as protocol
    source = Path(__file__).resolve().parents[4] / 'tools/host_replay/host_replay/path_protocol.py'
    if not source.is_file():
        pytest.skip('Host-only tool is not deployed to Jetson')
    assert Path(protocol.__file__).read_bytes() == source.read_bytes()
