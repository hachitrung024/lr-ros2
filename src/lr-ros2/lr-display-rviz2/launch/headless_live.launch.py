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

"""Persistent NET + MAVLink backend using wall time and no recording database."""

import ast
import math
import os
import re
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    def value(name):
        return LaunchConfiguration(name).perform(context).strip()

    forbidden = {'mavlink_db_path', 'mavlink_dir', 'stream_svo_mode', 'svo_path', 'future_path',
                 'replay_clock_source', 'clock_udp_bind_address', 'clock_udp_bind_port',
                 'use_sim_time', 'start_path_prediction_node'}
    if forbidden.intersection(context.launch_configurations):
        raise ValueError('headless_live does not accept DB, replay or clock arguments')
    camera, model = value('camera_name'), value('camera_model')
    if model not in {'zed', 'zedm', 'zed2', 'zed2i', 'zedx', 'zedxm', 'zedxnano',
                     'zedxhdr', 'zedxhdrmini', 'zedxhdrmax', 'virtual'}:
        raise ValueError('camera_model must identify a supported stereo camera')
    if not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*', camera):
        raise ValueError('camera_name must be a ROS name without namespace separators')
    address = value('stream_address')
    port, fcu = int(value('stream_port')), value('mavros_fcu_url')
    if not address or not fcu or not 2 <= port <= 65534 or port % 2:
        raise ValueError('Explicit NET address, even port and mavros_fcu_url are required')
    source_profile = value('source_profile')
    if source_profile not in ('native', 'host_replay'):
        raise ValueError('source_profile must be native or host_replay')
    native = source_profile == 'native'
    clock_mode = value('camera_time_mode')
    if clock_mode not in ('unverified', 'system', 'offset', 'fcu_utc'):
        raise ValueError('camera_time_mode must be unverified, system, offset or fcu_utc')
    if clock_mode == 'fcu_utc' and not native:
        raise ValueError('fcu_utc requires source_profile native')
    clock_offset = int(value('camera_time_offset_ns'))
    if not -(2**63 - 1) <= clock_offset < 2**63:
        raise ValueError('camera_time_offset_ns must fit signed int64')
    ids = {name: int(value(name)) for name in ('target_system_id', 'target_component_id')}
    if not all(1 <= v <= 255 for v in ids.values()):
        raise ValueError('MAVLink target IDs must be 1..255')
    thresholds = {name: float(value(name)) for name in (
        'gps_max_age_s', 'gps_max_gap_s', 'attitude_max_age_s', 'max_speed_mps')}
    if not all(math.isfinite(v) and v > 0 for v in thresholds.values()):
        raise ValueError('Telemetry limits must be finite and positive')
    extrinsics = value('mavlink_body_to_camera')
    if native and not extrinsics:
        raise ValueError('native requires explicit mavlink_body_to_camera extrinsics')
    mount = ast.literal_eval(extrinsics or '[0,0,0,0,0,0]')
    if len(mount) != 6 or not all(isinstance(x, (int, float)) and math.isfinite(x) for x in mount):
        raise ValueError('mavlink_body_to_camera requires six finite XYZ/RPY values')
    mount = [float(x) for x in mount]
    model_path = value('segmentation_model_path')
    if model_path and not Path(model_path).is_file():
        raise ValueError('segmentation_model_path must be an existing model')
    frame = value('map_frame')
    prediction_value = value('prediction_enabled').lower()
    if prediction_value not in ('true', 'false'):
        raise ValueError('prediction_enabled must be true or false')
    prediction = prediction_value == 'true'
    if prediction and not model_path:
        raise ValueError('prediction_enabled requires segmentation_model_path')
    profile = value('prediction_profile')
    if profile not in ('static', 'dynamic'):
        raise ValueError('prediction_profile must be static or dynamic')
    rover_config = value('rover_config')
    if native and prediction and (not rover_config or not Path(rover_config).is_file()
                                  or Path(rover_config).name == 'rover.reference.yaml'):
        raise ValueError('native prediction requires a measured rover_config file')
    common = {'use_sim_time': False}
    path_topic = '/lr/future_path/mission' if native else '/lr/future_path/mavlink'

    def config(package, filename):
        return os.path.join(get_package_share_directory(package), 'config', filename)

    def node(package, executable, name=None, parameters=None, **options):
        return Node(package=package, executable=executable, name=name, output='screen',
                    respawn=True, respawn_delay=2.0, parameters=parameters or [common], **options)
    plugins = ['command', 'sys_status', 'sys_time', 'global_position', 'imu', 'lr_source_time']
    plugins += ['waypoint', 'home_position'] if native else ['lr_future_path']
    path_executable = 'mission_path' if native else 'mavlink_future_path'
    nodes = [
        node('mavros', 'mavros_node', parameters=[config('lr_mavros_bridge', 'live.yaml'), {
            **common, 'fcu_url': fcu, 'gcs_url': '', 'fcu_protocol': 'v2.0',
            **ids, 'plugin_allowlist': plugins,
        }]),
        node('lr_mavros_bridge', 'mavros_pose', parameters=[{
            **common, **thresholds, 'time_mode': 'live', 'map_frame': frame,
            'child_frame': camera + '_camera_link', 'body_to_camera': mount,
            'pose_topic': '/lr/mavlink/pose', 'audit_file': value('mavros_audit_file'),
        }]),
        node('lr_mavros_bridge', 'live_runtime', parameters=[{
            **common, **thresholds, **ids, 'source_profile': source_profile,
            'camera_time_mode': clock_mode, 'camera_time_offset_ns': clock_offset,
            'camera_name': camera, 'camera_model': model,
            'stream_address': address, 'stream_port': port, 'map_frame': frame,
        }]),
        node('lr_terrain_geometry', 'terrain_geometry_node', 'terrain_geometry', parameters=[
            config('lr_terrain_geometry', 'terrain_geometry.yaml'), {
                **common, 'frames.map_frame': frame, 'source_guard_enabled': True,
                'corridor.enabled': False,
                'input.point_cloud_topic': f'/{camera}/zed_node/point_cloud/cloud_registered',
            }], additional_env={'OPENBLAS_NUM_THREADS': '1'}),
    ]
    if model_path:
        sensor = f'/{camera}/zed_node/'
        nodes.extend([
            node('lr_segmentation', 'segmentation_node', 'segmentation', parameters=[
                config('lr_segmentation', 'segmentation.yaml'), {
                    **common, 'model.path': model_path, 'source_guard_enabled': True,
                    'input.image_topic': sensor + 'rgb/color/rect/image',
                    'source_guard_topic': '/lr/source/camera_generation',
                }]),
            node('lr_segmentation', 'box_estimator_3d_node', 'box_estimator_3d', parameters=[
                config('lr_segmentation', 'segmentation.yaml'), {
                    **common, 'source_guard_enabled': True, 'output.frame_id': frame,
                    'geometry.up_axis': [0.0, 0.0, 1.0],
                    'input.depth_topic': sensor + 'depth/depth_registered',
                    'input.camera_info_topic': sensor + 'rgb/color/rect/camera_info',
                }]),
        ])
    if prediction:
        guard = {**common, 'source_guard_enabled': True,
                 'live_path_max_source_age_sec': 2.0 if native else 1.0,
                 'live_path_topic': path_topic}
        nodes.extend([
            node('lr_mavros_bridge', path_executable, parameters=[{
                **common, **ids, 'map_frame': frame, 'body_to_camera': mount,
            }]),
            node('lr_prediction_bridge', 'prediction_bridge_node', parameters=[
                config('lr_prediction_bridge', 'bridge.yaml'), {
                    **guard, 'prediction_profile': profile, 'expected_frame_id': frame,
                    'trajectory.input_topic': path_topic,
                }]),
            node('prediction_ros', 'prediction_node', parameters=[
                config('prediction_ros', 'prediction.yaml'), {
                    **guard, 'prediction_profile': profile, 'expected_frame_id': frame,
                    'config_path': rover_config or config(
                        'prediction_core', 'rover.reference.yaml'),
                }]),
            node('lr_path_prediction', 'canonical_prediction_visualizer_node', parameters=[
                config('lr_path_prediction', 'path_prediction.yaml'), {
                    **guard, 'frames.map_frame': frame,
                }]),
        ])
    return nodes


def generate_launch_description():
    required = ('camera_model', 'stream_address', 'stream_port', 'mavros_fcu_url')
    defaults = {'camera_name': 'zed', 'map_frame': 'map', 'segmentation_model_path': '',
                'mavlink_body_to_camera': '', 'mavros_audit_file': '',
                'prediction_enabled': 'true', 'prediction_profile': 'static', 'rover_config': '',
                'source_profile': 'native', 'camera_time_mode': 'unverified',
                'camera_time_offset_ns': '0', 'target_system_id': '1', 'target_component_id': '1',
                'gps_max_age_s': '0.75', 'gps_max_gap_s': '1.5',
                'attitude_max_age_s': '0.25', 'max_speed_mps': '5.0'}
    return LaunchDescription([
        *(DeclareLaunchArgument(name) for name in required),
        *(DeclareLaunchArgument(name, default_value=v) for name, v in defaults.items()),
        OpaqueFunction(function=launch_setup),
    ])
