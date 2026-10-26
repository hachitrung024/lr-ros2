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

"""Behavioral contracts for the single NET/MAVROS backend architecture."""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters
import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]


def module():
    """Load the backend independently of ROS launch's executable discovery."""
    spec = importlib.util.spec_from_file_location(
        'headless_net', ROOT / 'launch/headless_zed_cam.launch.py',
    )
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


def context(**overrides):
    """Use real declared defaults and non-default explicit network endpoints."""
    ctx = LaunchContext()
    ctx.launch_configurations.update({
        'camera_model': 'zed2i', 'stream_address': '192.0.2.10', 'stream_port': '32000',
        'stream_svo_mode': 'true', 'mavros_fcu_url': 'udp://0.0.0.0:24540@192.0.2.10:24550',
    })
    ctx.launch_configurations.update(overrides)
    for entity in module().generate_launch_description().entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(ctx)
    return ctx


def text(ctx, value):
    """Evaluate substitutions exactly as ROS launch does."""
    return perform_substitutions(ctx, normalize_to_list_of_substitutions(value))


def includes(actions):
    """Collect package launch includes in pipeline order."""
    return [a for a in actions if isinstance(a, IncludeLaunchDescription)]


def executables(actions):
    """Inspect processes without starting camera hardware."""
    return [str(a.node_executable) for a in actions if isinstance(a, Node)]


def test_svo_always_uses_mavros_and_reference_only_db_with_one_tf_owner():
    """SVO can only arrive over NET and pose must come from received telemetry."""
    ctx = context(segmentation_model_path='/tmp/best.pt')
    actions = module().launch_setup(ctx)
    assert executables(actions) == ['mavlink_ground_truth_node', 'mavros_pose']
    zed = dict(includes(actions)[0].launch_arguments)
    assert zed['stream_address'] == '192.0.2.10'
    assert zed['stream_port'] == '32000'
    assert zed['svo_path'] == 'live'
    assert zed['publish_svo_clock'] == 'false'
    assert zed['use_sim_time'] == 'false'
    assert zed['publish_tf'] == zed['publish_map_tf'] == 'false'
    assert 'debug.use_pub_timestamps:=false' in zed['param_overrides']
    assert 'sensors.sensors_image_sync:=true' in zed['param_overrides']
    assert 'pos_tracking.pos_tracking_enabled:=false' in zed['param_overrides']
    gateway = next(a for a in includes(actions) if 'fcu_url' in dict(a.launch_arguments))
    assert text(ctx, dict(gateway.launch_arguments)['fcu_url']) == ctx.launch_configurations[
        'mavros_fcu_url'
    ]
    for node in (a for a in actions if isinstance(a, Node)):
        assert evaluate_parameters(ctx, node._Node__parameters)[0]['use_sim_time'] is True
    prediction = includes(actions)[-1]
    assert prediction.condition.evaluate(ctx)


def test_live_uses_net_tracking_and_wall_time_without_recording_nodes():
    """Live stereo retains perception without opening a DB or running replay MAVROS."""
    ctx = context(stream_svo_mode='false', mavros_fcu_url='',
                  segmentation_model_path='/tmp/best.pt')
    actions = module().launch_setup(ctx)
    assert not executables(actions)
    zed = dict(includes(actions)[0].launch_arguments)
    assert zed['stream_address'] == '192.0.2.10'
    assert zed['publish_tf'] == zed['publish_map_tf'] == 'true'
    assert 'pos_tracking.pos_tracking_enabled:=false' not in zed['param_overrides']
    for include in includes(actions)[1:]:
        assert text(ctx, dict(include.launch_arguments)['use_sim_time']) == 'false'
    assert not includes(actions)[-1].condition.evaluate(ctx)
    assert dict(includes(actions)[1].launch_arguments)['corridor_enabled'] == 'false'


@pytest.mark.parametrize('overrides, error', [
    ({'stream_address': ''}, 'stream_address'),
    ({'stream_port': '32001'}, 'stream_port'),
    ({'stream_port': '65536'}, 'stream_port'),
    ({'mavros_fcu_url': ''}, 'mavros_fcu_url'),
    ({'stream_svo_mode': 'false', 'mavros_fcu_url': '', 'future_path': 'true'}, 'future_path'),
    ({'mavlink_body_to_camera': '[0,1]'}, 'mavlink_body_to_camera'),
    ({'mavlink_max_gps_gap_s': 'nan'}, 'mavlink_max_gps_gap_s'),
    ({'future_path_step_m': '0'}, 'Future-path'),
    ({'future_path_horizon_s': '-1'}, 'Future-path'),
    ({'future_path_max_speed_mps': '0.1'}, 'Future-path'),
    ({'start_path_prediction_node': 'true'}, 'Prediction'),
])
def test_invalid_network_or_processing_configuration_starts_no_nodes(overrides, error):
    """Reject missing endpoints, invalid geometry and unavailable prediction inputs."""
    ctx = context(**overrides)
    actions = module().launch_setup(ctx)
    assert not executables(actions)
    assert not includes(actions)
    assert error in text(ctx, actions[0].msg)


@pytest.mark.parametrize('option, value', [
    ('svo_path', '/tmp/session.svo2'), ('publish_svo_clock', 'true'),
    ('mavlink_backend', 'db'), ('mavlink', 'true'), ('start_rviz', 'true'),
    ('future_path_cache_dir', '/tmp/cache'), ('use_sim_time', 'false'),
])
def test_removed_cli_paths_cannot_silently_start_a_different_architecture(option, value):
    """Old commands must fail explicitly instead of being ignored by ROS launch."""
    ctx = context(**{option: value})
    actions = module().launch_setup(ctx)
    assert not executables(actions)
    assert 'Removed backend arguments' in text(ctx, actions[0].msg)


def test_dynamic_prediction_and_perception_receive_shared_contracts():
    """Pass map, extrinsic and received pose into the same LR processing modules."""
    ctx = context(
        camera_name='rover', map_frame='world', prediction_profile='dynamic',
        segmentation_model_path='/tmp/best.pt', future_path_topic='/custom/future',
        mavlink_body_to_camera='[1,2,3,0,0,0]',
    )
    actions = module().launch_setup(ctx)
    adapter = next(a for a in actions if isinstance(a, Node)
                   and str(a.node_executable) == 'mavros_pose')
    params = evaluate_parameters(ctx, adapter._Node__parameters)[0]
    assert params['child_frame'] == 'rover_camera_link'
    assert list(params['body_to_camera']) == [1., 2., 3., 0., 0., 0.]
    assert params['map_frame'] == 'world'
    terrain, segmentation, prediction = includes(actions)[-3:]
    assert dict(terrain.launch_arguments)['trajectory_topic'] == '/custom/future'
    assert dict(segmentation.launch_arguments)['box_frame'] == 'world'
    arguments = dict(prediction.launch_arguments)
    assert text(ctx, arguments['prediction_profile']) == 'dynamic'
    assert arguments['pose_topic'] == '/lr/mavlink/pose'
    assert arguments['objects_topic'] == '/segmentation/tracked_footprints'
    assert arguments['objects_input_type'] == 'tracked_objects'


def test_rviz_presets_keep_durable_source_path_and_cycle_reference():
    """The host keeps existing display layers while processing stays headless."""
    config = yaml.safe_load((ROOT / 'rviz2/zed_stereo.rviz').read_text())

    def walk(displays):
        for display in displays:
            yield display
            yield from walk(display.get('Displays', []))

    paths = {d['Topic']['Value']: d for d in walk(config['Visualization Manager']['Displays'])
             if d.get('Class') == 'rviz_default_plugins/Path'}
    assert paths['/lr/future_path/ground_truth']['Enabled']
    assert paths['/lr/future_path/ground_truth']['Topic']['Durability Policy'] == 'Transient Local'
    assert not paths['/lr/path_prediction/reference_path']['Enabled']
