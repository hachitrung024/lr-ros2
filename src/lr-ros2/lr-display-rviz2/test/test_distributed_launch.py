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

"""Structural tests for the split headless and RViz launch entry points."""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters


LAUNCH_DIR = Path(__file__).resolve().parents[1] / 'launch'


def _load(name):
    path = LAUNCH_DIR / name
    spec = importlib.util.spec_from_file_location(name.replace('.', '_'), path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _text(context, value):
    return perform_substitutions(context, normalize_to_list_of_substitutions(value))


def test_headless_entry_point_has_only_net_arguments_and_no_ui():
    """Backend launch cannot open a local file/camera or start RViz."""
    module = _load('headless_zed_cam.launch.py')
    declarations = {e.name: e for e in module.generate_launch_description().entities
                    if isinstance(e, DeclareLaunchArgument)}
    for name in ('stream_address', 'stream_port', 'stream_svo_mode'):
        assert declarations[name].default_value is None
    for name in ('start_rviz', 'svo_path', 'mavlink', 'mavlink_backend', 'publish_svo_clock'):
        assert name not in declarations


def test_rviz_entry_point_starts_only_rviz_and_uses_remote_clock():
    """The laptop entry point must not duplicate any processing node."""
    module = _load('rviz_zed_cam.launch.py')
    context = LaunchContext()
    context.launch_configurations.update(
        {
            'camera_name': 'zed',
            'camera_model': 'zed2i',
            'use_sim_time': 'true',
            'svo_mode': 'true',
            'segmentation_enabled': 'true',
            'rviz_config': '/tmp/remote.rviz',
        }
    )

    actions = module.launch_setup(context)

    assert len(actions) == 1
    rviz = actions[0]
    assert isinstance(rviz, Node)
    assert str(rviz.node_executable) == 'rviz2'
    rviz._perform_substitutions(context)
    assert rviz.expanded_node_namespace == '/zed'
    assert [perform_substitutions(context, item) for item in rviz.cmd[1:3]] == [
        '-d',
        '/tmp/remote.rviz',
    ]
    assert rviz.expanded_remapping_rules == [
        ('/lr/future_path/ground_truth', '/lr/path_prediction/reference_path')
    ]


def test_rviz_entry_point_can_show_rgb_without_segmentation():
    """The RGB fallback remains available without backend segmentation."""
    module = _load('rviz_zed_cam.launch.py')
    context = LaunchContext()
    context.launch_configurations.update(
        {
            'camera_name': 'front',
            'camera_model': 'zed2i',
            'use_sim_time': 'false',
            'svo_mode': 'false',
            'segmentation_enabled': 'false',
            'prediction_enabled': 'false',
            'rviz_config': '/tmp/remote.rviz',
        }
    )

    rviz = module.launch_setup(context)[0]
    rviz._perform_substitutions(context)

    assert rviz.expanded_remapping_rules == [
        ('/segmentation/overlay', '/front/zed_node/rgb/color/rect/image')
    ]


def test_rviz_controls_can_target_host_source_independently_of_receiver():
    """The panel endpoint must not move image/result subscriptions to the source."""
    module = _load('rviz_zed_cam.launch.py')
    context = LaunchContext()
    context.launch_configurations.update({
        'camera_name': 'zed', 'camera_model': 'zed2i', 'use_sim_time': 'true',
        'svo_mode': 'true', 'segmentation_enabled': 'true', 'rviz_config': '/tmp/net.rviz',
        'svo_control_namespace': '/zed_source/zed_node',
    })
    rviz = module.launch_setup(context)[0]
    parameters = evaluate_parameters(context, rviz._Node__parameters)[0]
    assert parameters['svo_control_namespace'] == '/zed_source/zed_node'
    assert parameters['use_sim_time'] is True
    rviz._perform_substitutions(context)
    assert rviz.expanded_node_namespace == '/zed'
