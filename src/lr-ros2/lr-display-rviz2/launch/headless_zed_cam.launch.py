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

"""Jetson-only ZED NET receiver, MAVROS pose and LR processing."""

import ast
import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, LogInfo,
    OpaqueFunction, RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


REMOVED_ARGUMENTS = {
    'svo_path', 'svo_realtime', 'publish_svo_clock', 'use_sim_time',
    'mavlink', 'mavlink_backend', 'future_path_cache_dir', 'future_path_rebuild_cache',
    'mavlink_match_tolerance_s', 'start_rviz', 'start_zed_node', 'svo_control_namespace',
}
STEREO_MODELS = [
    'zed', 'zedm', 'zed2', 'zed2i', 'zedx', 'zedxm', 'zedxnano',
    'zedxhdr', 'zedxhdrmini', 'zedxhdrmax', 'virtual',
]


def _value(context, name):
    return LaunchConfiguration(name).perform(context).strip()


def _stop_launch(reason):
    def raise_error(_context):
        raise RuntimeError(reason)

    return [LogInfo(msg=TextSubstitution(text=f'ERROR: {reason}')),
            OpaqueFunction(function=raise_error)]


def _process_exited(event, context):
    if context.is_shutdown:
        return []
    return [EmitEvent(event=Shutdown(
        reason=f'Required replay process exited with code {event.returncode}.'
    ))]


def _include(package, filename, arguments, *, enabled='true'):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory(package), 'launch', filename,
        )),
        launch_arguments=arguments.items(),
        condition=IfCondition(TextSubstitution(text=enabled)),
    )


def launch_setup(context, *args, **kwargs):
    """Build one NET-only backend; SVO always uses received MAVROS telemetry."""
    removed = sorted(REMOVED_ARGUMENTS.intersection(context.launch_configurations))
    if removed:
        return _stop_launch('Removed backend arguments: ' + ', '.join(removed)
                            + '. Use host source, NET backend and RViz in separate terminals.')
    address = _value(context, 'stream_address')
    if not address:
        return _stop_launch('stream_address is required; the backend only accepts ZED NET.')
    port = _value(context, 'stream_port')
    try:
        number = int(port)
        if not 2 <= number <= 65534 or number % 2:
            raise ValueError
    except ValueError:
        return _stop_launch('stream_port must be even and in [2, 65534].')
    svo = _value(context, 'stream_svo_mode') == 'true'
    fcu_url = _value(context, 'mavros_fcu_url')
    if svo and not fcu_url:
        return _stop_launch('SVO NET requires an explicit mavros_fcu_url.')
    if not svo and fcu_url:
        return _stop_launch('Live NET does not use the telemetry replay MAVROS gateway.')
    future_option = _value(context, 'future_path')
    future = svo if future_option == 'auto' else future_option == 'true'
    if future and not svo:
        return _stop_launch('future_path requires stream_svo_mode:=true and its reference DB.')
    try:
        mount = [float(v) for v in ast.literal_eval(_value(context, 'mavlink_body_to_camera'))]
        if len(mount) != 6 or not all(math.isfinite(v) for v in mount):
            raise ValueError
        gap = float(_value(context, 'mavlink_max_gps_gap_s'))
        if not math.isfinite(gap) or gap <= 0:
            raise ValueError
    except (SyntaxError, ValueError, TypeError):
        return _stop_launch('Invalid mavlink_body_to_camera or mavlink_max_gps_gap_s.')
    path_values = {}
    try:
        for name in ('radius_m', 'step_m', 'horizon_s', 'stationary_speed_mps',
                     'max_speed_mps', 'smoothing_window_m', 'smoothing_max_deviation_m',
                     'display_hold_s'):
            path_values[name] = float(_value(context, 'future_path_' + name))
        if (not all(math.isfinite(v) and v >= 0 for v in path_values.values())
                or path_values['radius_m'] <= 0 or path_values['step_m'] <= 0
                or path_values['max_speed_mps'] <= path_values['stationary_speed_mps']):
            raise ValueError
    except ValueError:
        return _stop_launch('Future-path distance, step, horizon and speed limits are invalid.')
    camera = _value(context, 'camera_name') or 'zed'
    model = _value(context, 'camera_model')
    if model not in STEREO_MODELS:
        return _stop_launch('ZED NET LR processing requires a stereo camera model.')
    frame = _value(context, 'map_frame')
    pose_topic = _value(context, 'mavlink_pose_topic')
    path_topic = _value(context, 'future_path_topic')
    model_path = _value(context, 'segmentation_model_path')
    segmentation = bool(model_path) and _value(context, 'start_segmentation_node') != 'false'
    terrain = _value(context, 'start_terrain_node') == 'true'
    prediction_option = _value(context, 'start_path_prediction_node')
    prediction = (future and segmentation and terrain) if prediction_option == 'auto' else (
        prediction_option == 'true'
    )
    if prediction and not (future and segmentation and terrain):
        return _stop_launch('Prediction requires SVO future path, segmentation and terrain.')
    sim = TextSubstitution(text=str(svo).lower())
    overrides = 'debug.use_pub_timestamps:=false;sensors.sensors_image_sync:=true'
    if svo:
        overrides += ';pos_tracking.pos_tracking_enabled:=false;depth.depth_stabilization:=0'
    nodes = [_include('zed_wrapper', 'zed_camera.launch.py', {
        'camera_name': camera, 'camera_model': model,
        'stream_address': address, 'stream_port': port,
        'svo_path': 'live', 'publish_svo_clock': 'false', 'use_sim_time': 'false',
        'param_overrides': overrides,
        'publish_tf': str(not svo).lower(), 'publish_map_tf': str(not svo).lower(),
    })]
    if svo:
        reference = Node(
            package='lr_future_path', executable='mavlink_ground_truth_node',
            name='mavlink_ground_truth', output='screen', parameters=[{
                'mavlink_dir': _value(context, 'mavlink_dir'),
                'mavlink_db_path': _value(context, 'mavlink_db_path'),
                'future_path_enabled': future, 'future_path_topic': path_topic,
                'map_frame': frame, 'max_gps_gap_s': gap, 'body_to_camera': mount,
                'max_points': 1000, **path_values, 'use_sim_time': True,
            }],
        )
        adapter = Node(
            package='lr_mavros_bridge', executable='mavros_pose', output='screen',
            parameters=[{
                'use_sim_time': True, 'map_frame': frame,
                'child_frame': f'{camera}_camera_link', 'pose_topic': pose_topic,
                'body_to_camera': mount, 'gps_max_gap_s': gap,
                'audit_file': _value(context, 'mavros_audit_file'),
            }],
        )
        for node in (reference, adapter):
            nodes.extend([RegisterEventHandler(OnProcessExit(
                target_action=node, on_exit=_process_exited,
            )), node])
        nodes.append(_include('lr_mavros_bridge', 'mavros_replay.launch.py', {'fcu_url': fcu_url}))
    nodes.extend([
        _include('lr_terrain_geometry', 'terrain_geometry.launch.py', {
            'camera_name': camera,
            'terrain_params_file': LaunchConfiguration('terrain_params_file'),
            'trajectory_topic': path_topic, 'map_frame': frame, 'use_sim_time': sim,
            'corridor_enabled': str(future).lower(),
        }, enabled=str(terrain).lower()),
        _include('lr_segmentation', 'segmentation.launch.py', {
            'camera_name': camera,
            'segmentation_params_file': LaunchConfiguration('segmentation_params_file'),
            'model_path': model_path, 'box_frame': frame, 'box_up_axis': '[0.0, 0.0, 1.0]',
            'use_sim_time': sim,
        }, enabled=str(segmentation).lower()),
        _include('lr_path_prediction', 'path_prediction.launch.py', {
            'prediction_profile': LaunchConfiguration('prediction_profile'),
            'prediction_params_file': LaunchConfiguration('prediction_params_file'),
            'bridge_params_file': LaunchConfiguration('bridge_params_file'),
            'prediction_runtime_params_file': LaunchConfiguration(
                'prediction_runtime_params_file'),
            'rover_config': LaunchConfiguration('prediction_rover_config'),
            'path_topic': path_topic, 'terrain_topic': '/terrain_geometry/grid_map',
            'objects_topic': '/segmentation/tracked_footprints',
            'objects_input_type': 'tracked_objects', 'pose_topic': pose_topic,
            'map_frame': frame, 'use_sim_time': sim,
        }, enabled=str(prediction).lower()),
    ])
    return nodes


def generate_launch_description():
    """Declare only the network backend and LR processing options."""
    def config(package, filename):
        return os.path.join(get_package_share_directory(package), 'config', filename)

    return LaunchDescription([
        DeclareLaunchArgument('camera_name', default_value='zed'),
        DeclareLaunchArgument('camera_model', choices=STEREO_MODELS),
        DeclareLaunchArgument('stream_address', description='Host ZED NET sender address.'),
        DeclareLaunchArgument('stream_port', description='Host ZED NET sender port (even).'),
        DeclareLaunchArgument('stream_svo_mode', choices=['true', 'false'],
                              description='true: host SVO + MAVROS; false: host live camera.'),
        DeclareLaunchArgument('mavros_fcu_url', default_value='',
                              description='Required for SVO: explicit MAVROS transport URL.'),
        DeclareLaunchArgument('mavros_audit_file', default_value=''),
        DeclareLaunchArgument('mavlink_dir', default_value='/zed/lr-ros2/mavlink'),
        DeclareLaunchArgument('mavlink_db_path', default_value='',
                              description='Ground-truth reference and fixed origin only.'),
        DeclareLaunchArgument('mavlink_pose_topic', default_value='/lr/mavlink/pose'),
        DeclareLaunchArgument('mavlink_max_gps_gap_s', default_value='1.5'),
        DeclareLaunchArgument('mavlink_body_to_camera', default_value='[0,0,0,0,0,0]'),
        DeclareLaunchArgument('future_path', default_value='auto',
                              choices=['auto', 'true', 'false'],
                              description='auto publishes DB future ground truth for SVO only.'),
        DeclareLaunchArgument('future_path_topic', default_value='/lr/future_path/ground_truth'),
        DeclareLaunchArgument('future_path_radius_m', default_value='15.0'),
        DeclareLaunchArgument('future_path_step_m', default_value='0.2'),
        DeclareLaunchArgument('future_path_horizon_s', default_value='0.0'),
        DeclareLaunchArgument('future_path_stationary_speed_mps', default_value='0.1'),
        DeclareLaunchArgument('future_path_max_speed_mps', default_value='5.0'),
        DeclareLaunchArgument('future_path_smoothing_window_m', default_value='0.8'),
        DeclareLaunchArgument('future_path_smoothing_max_deviation_m', default_value='0.2'),
        DeclareLaunchArgument('future_path_display_hold_s', default_value='1.5'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('start_terrain_node', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('terrain_params_file',
                              default_value=config('lr_terrain_geometry',
                                                   'terrain_geometry.yaml')),
        DeclareLaunchArgument('start_segmentation_node', default_value='auto',
                              choices=['auto', 'true', 'false']),
        DeclareLaunchArgument('segmentation_model_path', default_value=''),
        DeclareLaunchArgument('segmentation_params_file',
                              default_value=config('lr_segmentation', 'segmentation.yaml')),
        DeclareLaunchArgument('start_path_prediction_node', default_value='auto',
                              choices=['auto', 'true', 'false']),
        DeclareLaunchArgument('prediction_profile', default_value='static',
                              choices=['static', 'dynamic']),
        DeclareLaunchArgument('bridge_params_file',
                              default_value=config('lr_prediction_bridge', 'bridge.yaml')),
        DeclareLaunchArgument('prediction_runtime_params_file',
                              default_value=config('prediction_ros', 'prediction.yaml')),
        DeclareLaunchArgument('prediction_params_file',
                              default_value=config('lr_path_prediction', 'path_prediction.yaml')),
        DeclareLaunchArgument('prediction_rover_config',
                              default_value=config('prediction_core', 'rover.reference.yaml')),
        OpaqueFunction(function=launch_setup),
    ])
