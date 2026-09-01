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

"""Structural tests for display_zed_cam launch mode selection."""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.utilities import (
    normalize_to_list_of_substitutions,
    perform_substitutions,
)
from launch_ros.actions import Node


def _load_launch_module():
    path = (
        Path(__file__).resolve().parents[1]
        / "launch"
        / "display_zed_cam.launch.py"
    )
    spec = importlib.util.spec_from_file_location("display_zed_cam", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _context(**overrides):
    values = {
        "start_zed_node": "true",
        "camera_name": "zed",
        "camera_model": "zed2i",
        "svo_path": "/tmp/session.svo2",
        "svo_realtime": "true",
        "publish_svo_clock": "true",
        "future_path": "false",
        "future_path_cache_dir": "",
        "future_path_rebuild_cache": "false",
        "future_path_topic": "/lr/future_path/ground_truth",
        "future_path_radius_m": "15.0",
        "future_path_step_m": "0.2",
        "mavlink": "false",
        "mavlink_dir": "mavlink",
        "mavlink_db_path": "",
        "mavlink_pose_topic": "/lr/mavlink/pose",
        "mavlink_match_tolerance_s": "60.0",
        "mavlink_max_gps_gap_s": "1.5",
        "mavlink_body_to_camera": "[0,0,0,0,0,0]",
        "start_terrain_node": "false",
        "terrain_params_file": "/tmp/terrain.yaml",
        "map_frame": "map",
        "start_segmentation_node": "false",
        "segmentation_params_file": "/tmp/segmentation.yaml",
        "segmentation_model_path": "",
        "start_path_prediction_node": "auto",
        "prediction_params_file": "/tmp/path_prediction.yaml",
        "prediction_profile": "static",
    }
    values.update(overrides)
    context = LaunchContext()
    context.launch_configurations.update(values)
    return context


def _node_executables(actions):
    return [
        str(action.node_executable)
        for action in actions
        if isinstance(action, Node)
    ]


def _zed_arguments(actions):
    include = next(
        action
        for action in actions
        if isinstance(action, IncludeLaunchDescription)
    )
    return dict(include.launch_arguments)


def _substitution_text(context, value):
    return perform_substitutions(
        context, normalize_to_list_of_substitutions(value)
    )


def test_mavlink_false_preserves_zed_tf_and_has_no_mavlink_node():
    """The default mode leaves ZED localization behavior unchanged."""
    module = _load_launch_module()
    context = _context()

    actions = module.launch_setup(context)
    arguments = _zed_arguments(actions)

    assert "mavlink_pose_node" not in _node_executables(actions)
    assert arguments["publish_tf"] == "true"
    assert arguments["publish_map_tf"] == "true"
    assert "pos_tracking.pos_tracking_enabled:=false" not in (
        _substitution_text(context, arguments["param_overrides"])
    )


def test_mavlink_true_disables_zed_tracking_and_owns_future_path():
    """Enabled mode disables ZED TF and skips the VIO cache process."""
    module = _load_launch_module()
    context = _context(mavlink="true", future_path="true")

    actions = module.launch_setup(context)
    arguments = _zed_arguments(actions)
    executables = _node_executables(actions)

    assert "mavlink_pose_node" in executables
    assert "future_ground_truth_node" not in executables
    assert not any(type(action) is ExecuteProcess for action in actions)
    assert arguments["publish_tf"] == "false"
    assert arguments["publish_map_tf"] == "false"
    overrides = _substitution_text(context, arguments["param_overrides"])
    assert "pos_tracking.pos_tracking_enabled:=false" in overrides
    assert "depth.depth_stabilization:=0" in overrides


def test_prediction_auto_starts_with_all_three_inputs():
    """Auto mode starts prediction when path, terrain, and boxes are on."""
    module = _load_launch_module()
    context = _context(
        future_path="true",
        mavlink="true",
        start_terrain_node="true",
        segmentation_model_path="/tmp/best.pt",
    )

    actions = module.launch_setup(context)

    executables = _node_executables(actions)
    assert executables.count("path_risk_predictor_node") == 1
    assert "trajectory_adapter_node" not in executables
    assert "geometry_adapter_node" not in executables
    assert "tracked_objects_adapter_node" not in executables
    assert "prediction_node" not in executables
    assert "canonical_prediction_visualizer_node" not in executables


def test_dynamic_prediction_remains_one_integrated_node():
    """Dynamic mode derives acceleration inside the prediction node."""
    module = _load_launch_module()
    context = _context(
        future_path="true",
        mavlink="true",
        start_terrain_node="true",
        segmentation_model_path="/tmp/best.pt",
        prediction_profile="dynamic",
    )

    actions = module.launch_setup(context)

    executables = _node_executables(actions)
    assert executables.count("path_risk_predictor_node") == 1
    assert "rover_state_adapter_node" not in executables


def test_mavlink_live_and_missing_svo_clock_are_rejected():
    """Offline MAVLink replay requires an SVO clock producer."""
    module = _load_launch_module()
    live_context = _context(mavlink="true", svo_path="live")
    clock_context = _context(
        mavlink="true", publish_svo_clock="false"
    )

    live_actions = module.launch_setup(live_context)
    clock_actions = module.launch_setup(clock_context)

    live_message = _substitution_text(live_context, live_actions[0].msg)
    clock_message = _substitution_text(clock_context, clock_actions[0].msg)
    assert "requires an SVO file" in live_message
    assert "requires publish_svo_clock:=true" in clock_message


def test_invalid_mavlink_extrinsic_is_rejected():
    """A malformed body-to-camera transform fails during launch setup."""
    module = _load_launch_module()
    context = _context(
        mavlink="true", mavlink_body_to_camera="[0, 1]"
    )

    actions = module.launch_setup(context)
    message = _substitution_text(context, actions[0].msg)

    assert "mavlink_body_to_camera" in message
