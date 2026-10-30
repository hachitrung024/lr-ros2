"""Mission geometry and transport freshness must never depend on replay data."""

import json
import time
from types import SimpleNamespace

import numpy as np
import pytest
import rclpy
from mavros_msgs.msg import Waypoint, WaypointList, HomePosition
from std_msgs.msg import String

from lr_future_path.mavlink import PoseSample
from lr_mavros_bridge.mission import mission_path
from lr_mavros_bridge.mission_path_node import MissionPath
from lr_future_path.trajectory import rotate_vector


def waypoint(lat=10.0001, lon=106., alt=100., frame=0, command=16):
    return Waypoint(frame=frame, command=command, x_lat=lat, y_long=lon, z_alt=alt)


def route(items, home=None, mount=None, seq=0):
    return mission_path(items, seq, [10., 106., 100.], home,
                        PoseSample(123, np.zeros(3), np.array([0., 0., 0., 1.])),
                        np.asarray(mount or [0.] * 6))


def test_route_clipped_spatially_and_has_no_invented_eta():
    stamps, positions, quaternions = route([waypoint(lat=10.001)])
    assert len(stamps) > 20 and len(stamps) <= 128
    assert np.all(stamps == 123)
    assert np.linalg.norm(positions[-1, :2]) == pytest.approx(15., abs=1e-6)
    assert np.max(np.linalg.norm(np.diff(positions[:, :2], axis=0), axis=1)) <= .200001
    np.testing.assert_allclose(np.linalg.norm(quaternions, axis=1), 1.)
    # Current pose faces east, but this mission segment runs north. Even the
    # first local sample must use route tangent rather than a slow attitude lerp.
    tangents = np.diff(positions[:, :2], axis=0)
    tangents /= np.linalg.norm(tangents, axis=1)[:, None]
    headings = np.array([rotate_vector(q, np.array([1., 0., 0.]))[:2]
                         for q in quaternions[:-1]])
    np.testing.assert_allclose(headings, tangents, atol=1e-6)


def test_relative_altitude_and_extrinsics_use_same_pose_frame():
    absolute = route([waypoint()])[1]
    relative = route([waypoint(alt=5., frame=6)], home=95.)[1]
    np.testing.assert_allclose(absolute, relative)
    shifted = route([waypoint()], mount=[1., 0., .5, 0., 0., 0.])[1]
    assert np.linalg.norm(shifted[-1] - absolute[-1]) > .5
    np.testing.assert_allclose(shifted[0], np.zeros(3))


@pytest.mark.parametrize('items,seq,error', [
    ([], 0, 'empty_or_finished'), ([waypoint()], 1, 'empty_or_finished'),
    ([waypoint(frame=6)], 0, 'waiting_home'),
    ([waypoint(frame=11)], 0, 'unsupported_mission_frame'),
    ([waypoint(command=177)], 0, 'unsupported_mission_command'),
    ([waypoint(lat=float('nan'))], 0, 'invalid_waypoint'),
])
def test_unsupported_or_incomplete_missions_block_route(items, seq, error):
    with pytest.raises(ValueError, match=error):
        route(items, seq=seq)


def test_download_freshness_is_not_renewed_by_progress_and_reconnect_clears():
    rclpy.init()
    node = MissionPath()
    try:
        state = {'version': 1, 'connected': True, 'telemetry_epoch': 1}
        node.on_state(String(data=json.dumps(state)))
        now = time.monotonic()
        node.pull_started = now - .1
        node.pull_future = SimpleNamespace(done=lambda: True, result=lambda: SimpleNamespace(
            success=True, wp_received=1))
        node.on_list(WaypointList(waypoints=[waypoint()], current_seq=0))
        node.download(now)
        assert len(node.snapshot) == 1 and node.snapshot_seen == now
        node.on_list(WaypointList(waypoints=[waypoint()], current_seq=0))
        assert node.snapshot_seen == now
        node.snapshot_seen -= 31.
        node.tick()
        assert node.status == 'waiting_complete_mission' and not node.published
        node.on_state(String(data=json.dumps({**state, 'telemetry_epoch': 2})))
        assert node.snapshot is None and node.home is None
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_home_mavros_ellipsoid_is_converted_to_msl():
    rclpy.init()
    node = MissionPath()
    try:
        node.on_state(String(data=json.dumps({
            'version': 1, 'connected': True, 'telemetry_epoch': 1})))
        home = HomePosition()
        home.geo.latitude, home.geo.longitude = 10., 106.
        home.geo.altitude = 100. + node.geoid(10., 106.)
        node.on_home(home)
        assert node.home == pytest.approx(100.)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_changed_mission_id_discards_old_download_immediately():
    rclpy.init()
    node = MissionPath()
    try:
        state = {'version': 1, 'connected': True, 'telemetry_epoch': 1, 'mission_id': 10}
        node.on_state(String(data=json.dumps(state)))
        node.snapshot = [waypoint()]
        node.snapshot_seen = time.monotonic()
        node.published = True
        node.on_state(String(data=json.dumps({**state, 'mission_id': 11})))
        assert node.snapshot is None and not node.published and node.next_pull == 0.
    finally:
        node.destroy_node()
        rclpy.shutdown()
