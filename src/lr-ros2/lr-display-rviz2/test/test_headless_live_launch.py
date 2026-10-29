# Copyright 2026 Landfill Rover
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

"""The always-on launch must not start recording or simulated-time nodes."""

import importlib.util
from pathlib import Path

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.utilities import evaluate_parameters


def load():
    path = Path(__file__).resolve().parents[1] / 'launch/headless_live.launch.py'
    spec = importlib.util.spec_from_file_location('headless_live', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def context(**overrides):
    ctx = LaunchContext()
    ctx.launch_configurations.update(
        camera_model='zed2i', stream_address='192.0.2.1', stream_port='32000',
        mavros_fcu_url='udp://:24540@192.0.2.1:24550')
    ctx.launch_configurations.update(overrides)
    for action in load().generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(ctx)
    return ctx


def test_live_launch_is_wall_time_without_db_clock_or_prediction():
    ctx = context()
    nodes = load().launch_setup(ctx)
    assert [str(n.node_executable) for n in nodes] == [
        'mavros_node', 'mavros_pose', 'live_runtime', 'terrain_geometry_node']
    for node in nodes:
        params = evaluate_parameters(ctx, node._Node__parameters)
        assert params[-1]['use_sim_time'] is False
    assert evaluate_parameters(ctx, nodes[-1]._Node__parameters)[-1]['corridor.enabled'] is False


def test_prediction_requires_segmentation_and_uses_only_mavlink_path(tmp_path):
    with pytest.raises(ValueError, match='segmentation_model_path'):
        load().launch_setup(context(prediction_enabled='true'))
    model = tmp_path / 'model.pt'
    model.touch()
    ctx = context(prediction_enabled='true', segmentation_model_path=str(model))
    nodes = load().launch_setup(ctx)
    assert [str(n.node_executable) for n in nodes][-4:] == [
        'mavlink_future_path', 'prediction_bridge_node', 'prediction_node',
        'canonical_prediction_visualizer_node']
    for node in nodes[-3:]:
        params = evaluate_parameters(ctx, node._Node__parameters)[-1]
        assert params['use_sim_time'] is False and params['source_guard_enabled'] is True
    assert evaluate_parameters(ctx, nodes[-3]._Node__parameters)[-1][
        'trajectory.input_topic'] == '/lr/future_path/mavlink'


@pytest.mark.parametrize('override', [
    {'stream_port': '30001'}, {'mavros_fcu_url': ''}, {'mavlink_db_path': '/tmp/replay.db'},
    {'use_sim_time': 'true'}, {'future_path': 'true'},
])
def test_replay_arguments_or_invalid_endpoints_are_rejected(override):
    with pytest.raises(ValueError):
        load().launch_setup(context(**override))
