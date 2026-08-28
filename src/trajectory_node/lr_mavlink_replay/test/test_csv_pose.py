# Copyright 2026 Landfill Rover Maintainers
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Tests for MAVLink CSV parsing and exact-time pose interpolation."""

from pathlib import Path

import numpy as np

from lr_mavlink_replay.csv_pose import load_attitude
from lr_mavlink_replay.csv_pose import load_pose_log
from lr_mavlink_replay.csv_pose import slerp_xyzw


def _write(path: Path, text: str) -> None:
    path.write_text(text, encoding='utf-8')


def test_load_and_interpolate_wall_epoch_csv(tmp_path):
    gps = tmp_path / 'gps.csv'
    attitude = tmp_path / 'attitude.csv'
    _write(
        gps,
        't_wall_epoch_us,lat,lon,alt,fix_type\n'
        '1000000,34.0,-118.0,100.0,4\n'
        '2000000,34.00001,-118.0,101.0,4\n'
        '3000000,34.00002,-118.0,102.0,4\n',
    )
    _write(
        attitude,
        't_wall_epoch_us,roll,pitch,yaw\n'
        '1000000,0,0,0\n'
        '2000000,0,0,0.5\n'
        '3000000,0,0,1.0\n',
    )
    log = load_pose_log(
        str(gps), str(attitude), origin_samples=1,
        maximum_attitude_gap_s=2.0,
    )
    sample = log.sample_at(1.5)
    assert sample is not None
    assert sample.position_enu_m.shape == (3,)
    assert np.isfinite(sample.position_enu_m).all()
    np.testing.assert_allclose(
        sample.position_enu_m,
        (log.gps_position[0] + log.gps_position[1]) / 2.0,
    )
    assert np.isclose(np.linalg.norm(sample.quaternion_enu_flu_xyzw), 1.0)


def test_timestamp_unix_s_and_invalid_fix_filtering(tmp_path):
    gps = tmp_path / 'gps.csv'
    attitude = tmp_path / 'attitude.csv'
    _write(
        gps,
        'timestamp_unix_s,lat,lon,alt,fix_type\n'
        '1,34,-118,100,2\n'
        '2,34,-118,100,4\n'
        '3,34,-118,100,4\n',
    )
    _write(
        attitude,
        'timestamp_unix_s,roll,pitch,yaw\n'
        '1,0,0,0\n'
        '2,0,0,0\n'
        '3,0,0,0\n',
    )
    log = load_pose_log(
        str(gps), str(attitude), origin_samples=1,
        maximum_attitude_gap_s=2.0,
    )
    assert log.sample_at(1.5) is None
    assert log.unavailable_reason(1.5) == 'outside_log'
    assert log.sample_at(2.5) is not None


def test_gap_reason_and_no_fallback(tmp_path):
    gps = tmp_path / 'gps.csv'
    attitude = tmp_path / 'attitude.csv'
    _write(
        gps,
        'timestamp_unix_s,lat,lon,alt,fix_type\n'
        '0,34,-118,100,4\n'
        '5,34,-118,100,4\n',
    )
    _write(
        attitude,
        'timestamp_unix_s,roll,pitch,yaw\n'
        '0,0,0,0\n'
        '5,0,0,0\n',
    )
    log = load_pose_log(
        str(gps),
        str(attitude),
        maximum_gps_gap_s=2.0,
        maximum_attitude_gap_s=0.5,
    )
    assert log.sample_at(2.5) is None
    assert log.unavailable_reason(2.5) == 'gps_gap'


def test_slerp_uses_shortest_quaternion_path():
    first = np.asarray([0.0, 0.0, 0.0, 1.0])
    same_rotation_negative = -first
    result = slerp_xyzw(first, same_rotation_negative, 0.5)
    np.testing.assert_allclose(result, first)


def test_gps_axes_are_east_north_up(tmp_path):
    gps = tmp_path / 'gps.csv'
    attitude = tmp_path / 'attitude.csv'
    _write(
        gps,
        'timestamp_unix_s,lat,lon,alt,fix_type\n'
        '0,0,0,0,3\n'
        '1,0,0.00001,0,3\n'
        '2,0.00001,0,1,3\n',
    )
    _write(
        attitude,
        'timestamp_unix_s,roll,pitch,yaw\n'
        '0,0,0,0\n'
        '2,0,0,0\n',
    )
    log = load_pose_log(
        str(gps), str(attitude), origin_samples=1,
        maximum_attitude_gap_s=3.0,
    )
    np.testing.assert_allclose(log.gps_position[0], 0.0, atol=1e-12)
    assert log.gps_position[1, 0] > 1.0
    assert abs(log.gps_position[1, 1]) < 1e-3
    assert log.gps_position[2, 1] > 1.0
    assert log.gps_position[2, 2] > 0.9


def test_zero_ned_frd_attitude_becomes_north_facing_enu_flu(tmp_path):
    attitude = tmp_path / 'attitude.csv'
    _write(
        attitude,
        'timestamp_unix_s,roll,pitch,yaw\n'
        '0,0,0,0\n'
        '1,0,0,0\n',
    )
    _, quaternions = load_attitude(attitude)
    expected = np.asarray([0.0, 0.0, np.sqrt(0.5), np.sqrt(0.5)])
    np.testing.assert_allclose(quaternions[0], expected, atol=1e-12)


def test_attitude_gap_reason(tmp_path):
    gps = tmp_path / 'gps.csv'
    attitude = tmp_path / 'attitude.csv'
    _write(
        gps,
        'timestamp_unix_s,lat,lon,alt,fix_type\n'
        '0,0,0,0,3\n'
        '1,0,0,0,3\n',
    )
    _write(
        attitude,
        'timestamp_unix_s,roll,pitch,yaw\n'
        '0,0,0,0\n'
        '1,0,0,0\n',
    )
    log = load_pose_log(str(gps), str(attitude))
    assert log.unavailable_reason(0.5) == 'attitude_gap'


def test_duplicate_attitude_timestamps_are_rejected(tmp_path):
    attitude = tmp_path / 'attitude.csv'
    _write(
        attitude,
        'timestamp_unix_s,roll,pitch,yaw\n'
        '1,0,0,0\n'
        '1,0,0,1\n',
    )
    with np.testing.assert_raises_regex(ValueError, 'unique attitude'):
        load_attitude(attitude)


def test_invalid_geodetic_coordinates_are_filtered(tmp_path):
    gps = tmp_path / 'gps.csv'
    attitude = tmp_path / 'attitude.csv'
    _write(
        gps,
        'timestamp_unix_s,lat,lon,alt,fix_type\n'
        '0,91,0,0,3\n'
        '1,0,0,0,3\n'
        '2,0,0,0,3\n',
    )
    _write(
        attitude,
        'timestamp_unix_s,roll,pitch,yaw\n'
        '0,0,0,0\n'
        '2,0,0,0\n',
    )
    log = load_pose_log(str(gps), str(attitude))
    np.testing.assert_allclose(log.gps_time, [1.0, 2.0])


def test_future_path_spatial_sampling_stops_at_log_end(tmp_path):
    gps = tmp_path / 'gps.csv'
    attitude = tmp_path / 'attitude.csv'
    _write(
        gps,
        'timestamp_unix_s,lat,lon,alt,fix_type\n'
        '0,0,0,0,3\n'
        '1,0,0.00001,0,3\n'
        '2,0,0.00002,0,3\n',
    )
    _write(
        attitude,
        'timestamp_unix_s,roll,pitch,yaw\n'
        '0,0,0,0\n'
        '1,0,0,0\n'
        '2,0,0,0\n',
    )
    log = load_pose_log(
        str(gps), str(attitude),
        maximum_attitude_gap_s=2.0,
    )
    samples = log.sample_future_within_radius(
        0.0, radius_m=10.0, step_m=0.5
    )
    assert samples[0].timestamp_s == 0.0
    assert samples[-1].timestamp_s == 2.0
    ground_steps = [
        np.linalg.norm(
            current.position_enu_m[:2] - previous.position_enu_m[:2]
        )
        for previous, current in zip(samples, samples[1:])
    ]
    np.testing.assert_allclose(ground_steps[:-1], 0.5, atol=1e-9)
    assert 0.0 < ground_steps[-1] <= 0.5
    assert samples[-1].position_enu_m[0] > samples[0].position_enu_m[0]


def test_future_path_sampling_rejects_invalid_configuration(tmp_path):
    gps = tmp_path / 'gps.csv'
    attitude = tmp_path / 'attitude.csv'
    _write(
        gps,
        'timestamp_unix_s,lat,lon,alt,fix_type\n'
        '0,0,0,0,3\n'
        '1,0,0,0,3\n',
    )
    _write(
        attitude,
        'timestamp_unix_s,roll,pitch,yaw\n'
        '0,0,0,0\n'
        '1,0,0,0\n',
    )
    log = load_pose_log(str(gps), str(attitude))
    with np.testing.assert_raises_regex(ValueError, 'step must be positive'):
        log.sample_future_within_radius(0.0, radius_m=1.0, step_m=0.0)
    with np.testing.assert_raises_regex(ValueError, 'radius must be positive'):
        log.sample_future_within_radius(0.0, radius_m=0.0, step_m=0.1)


def test_future_path_does_not_bridge_gps_gap(tmp_path):
    gps = tmp_path / 'gps.csv'
    attitude = tmp_path / 'attitude.csv'
    _write(
        gps,
        'timestamp_unix_s,lat,lon,alt,fix_type\n'
        '0,0,0,0,3\n'
        '1,0,0.00001,0,3\n'
        '5,0,0.00005,0,3\n'
        '6,0,0.00006,0,3\n',
    )
    _write(
        attitude,
        'timestamp_unix_s,roll,pitch,yaw\n'
        '0,0,0,0\n'
        '1,0,0,0\n'
        '2,0,0,0\n'
        '3,0,0,0\n'
        '4,0,0,0\n'
        '5,0,0,0\n'
        '6,0,0,0\n',
    )
    log = load_pose_log(str(gps), str(attitude))
    samples = log.sample_future_within_radius(
        0.0, radius_m=100.0, step_m=0.5
    )
    assert samples[0].timestamp_s == 0.0
    assert samples[-1].timestamp_s == 1.0
    assert all(sample.timestamp_s <= 1.0 for sample in samples)


def test_future_gps_path_is_not_cut_by_future_attitude_gap(tmp_path):
    gps = tmp_path / 'gps.csv'
    attitude = tmp_path / 'attitude.csv'
    _write(
        gps,
        'timestamp_unix_s,lat,lon,alt,fix_type\n'
        '0,0,0,0,3\n'
        '1,0,0.00001,0,3\n'
        '2,0,0.00002,0,3\n',
    )
    _write(
        attitude,
        'timestamp_unix_s,roll,pitch,yaw\n'
        '0,0,0,0\n'
        '2,0,0,0\n',
    )
    log = load_pose_log(str(gps), str(attitude))
    assert log.sample_at(1.0) is None
    samples = log.sample_future_within_radius(
        0.0, radius_m=10.0, step_m=0.5
    )
    assert samples[0].timestamp_s == 0.0
    assert samples[-1].timestamp_s == 2.0


def test_future_path_stops_at_xy_radius_boundary(tmp_path):
    gps = tmp_path / 'gps.csv'
    attitude = tmp_path / 'attitude.csv'
    _write(
        gps,
        'timestamp_unix_s,lat,lon,alt,fix_type\n'
        '0,0,0,0,3\n'
        '1,0,0.00001,0,3\n'
        '2,0,0.00002,0,3\n',
    )
    _write(
        attitude,
        'timestamp_unix_s,roll,pitch,yaw\n'
        '0,0,0,0\n'
        '2,0,0,0\n',
    )
    log = load_pose_log(str(gps), str(attitude), origin_samples=1)
    samples = log.sample_future_within_radius(
        0.0, radius_m=1.5, step_m=0.5
    )
    radii = [
        np.linalg.norm(
            sample.position_enu_m[:2] - samples[0].position_enu_m[:2]
        )
        for sample in samples
    ]
    np.testing.assert_allclose(radii, [0.0, 0.5, 1.0, 1.5], atol=1e-9)
    assert 1.0 < samples[-1].timestamp_s < 2.0


def test_future_path_does_not_reenter_radius_after_first_exit(tmp_path):
    gps = tmp_path / 'gps.csv'
    attitude = tmp_path / 'attitude.csv'
    _write(
        gps,
        'timestamp_unix_s,lat,lon,alt,fix_type\n'
        '0,0,0,0,3\n'
        '1,0,0.00002,0,3\n'
        '2,0,0,0,3\n',
    )
    _write(
        attitude,
        'timestamp_unix_s,roll,pitch,yaw\n'
        '0,0,0,0\n'
        '2,0,0,0\n',
    )
    log = load_pose_log(str(gps), str(attitude), origin_samples=1)
    samples = log.sample_future_within_radius(
        0.0, radius_m=1.0, step_m=0.25
    )
    assert samples[-1].timestamp_s < 1.0
    assert np.isclose(
        np.linalg.norm(
            samples[-1].position_enu_m[:2]
            - samples[0].position_enu_m[:2]
        ),
        1.0,
    )


def test_future_path_radius_ignores_height(tmp_path):
    gps = tmp_path / 'gps.csv'
    attitude = tmp_path / 'attitude.csv'
    _write(
        gps,
        'timestamp_unix_s,lat,lon,alt,fix_type\n'
        '0,0,0,0,3\n'
        '1,0,0,100,3\n'
        '2,0,0.00001,100,3\n',
    )
    _write(
        attitude,
        'timestamp_unix_s,roll,pitch,yaw\n'
        '0,0,0,0\n'
        '2,0,0,0\n',
    )
    log = load_pose_log(str(gps), str(attitude), origin_samples=1)
    samples = log.sample_future_within_radius(
        0.0, radius_m=2.0, step_m=0.5
    )
    assert samples[-1].timestamp_s == 2.0
    assert samples[-1].position_enu_m[2] > 99.0
    assert all(
        np.linalg.norm(
            sample.position_enu_m[:2] - samples[0].position_enu_m[:2]
        ) <= 2.0
        for sample in samples
    )
