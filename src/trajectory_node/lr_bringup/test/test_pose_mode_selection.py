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

"""Tests for selecting the session-wide point-cloud pose source."""

import importlib.util
from pathlib import Path

import pytest


LAUNCH_PATH = Path(__file__).parents[1] / 'launch' / 'rover.launch.py'
SPEC = importlib.util.spec_from_file_location('rover_launch', LAUNCH_PATH)
ROVER_LAUNCH = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(ROVER_LAUNCH)


def _write_valid_pair(tmp_path):
    gps = tmp_path / 'gps.csv'
    attitude = tmp_path / 'attitude.csv'
    gps.write_text(
        'timestamp_unix_s,lat,lon,alt,fix_type\n'
        '1,34,-118,100,3\n'
        '2,34,-118,100,3\n',
        encoding='utf-8',
    )
    attitude.write_text(
        'timestamp_unix_s,roll,pitch,yaw\n'
        '1,0,0,0\n'
        '2,0,0,0\n',
        encoding='utf-8',
    )
    return str(gps), str(attitude)


@pytest.mark.parametrize(
    ('svo_path', 'gps_path', 'attitude_path'),
    [
        ('live', '', ''),
        ('live', '/ignored/gps.csv', '/ignored/attitude.csv'),
        ('recording.svo2', '', ''),
        ('recording.svo2', '/only/gps.csv', ''),
        ('recording.svo2', '', '/only/attitude.csv'),
    ],
)
def test_live_and_missing_csv_use_zed_tf(
    svo_path, gps_path, attitude_path
):
    pose_source, _ = ROVER_LAUNCH._select_pose_mode(
        svo_path, gps_path, attitude_path
    )
    assert pose_source == 'tf'


def test_invalid_csv_uses_zed_tf(tmp_path):
    gps = tmp_path / 'gps.csv'
    attitude = tmp_path / 'attitude.csv'
    gps.write_text('not,the,required,schema\n', encoding='utf-8')
    attitude.write_text(
        'timestamp_unix_s,roll,pitch,yaw\n1,0,0,0\n2,0,0,0\n',
        encoding='utf-8',
    )
    pose_source, reason = ROVER_LAUNCH._select_pose_mode(
        'recording.svo2', str(gps), str(attitude)
    )
    assert pose_source == 'tf'
    assert 'invalid MAVLink CSV pair' in reason


def test_valid_csv_locks_session_to_mavlink_even_without_overlap(tmp_path):
    gps_path, attitude_path = _write_valid_pair(tmp_path)
    pose_source, _ = ROVER_LAUNCH._select_pose_mode(
        'uninspected_timeline.svo2', gps_path, attitude_path
    )
    assert pose_source == 'mavlink'
