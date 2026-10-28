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
        raise ValueError('headless_live does not accept DB, replay, prediction or clock arguments')
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
    mount = ast.literal_eval(value('mavlink_body_to_camera'))
    if len(mount) != 6 or not all(isinstance(x, (int, float)) and math.isfinite(x) for x in mount):
        raise ValueError('mavlink_body_to_camera requires six finite XYZ/RPY values')
    mount = [float(x) for x in mount]
    model_path = value('segmentation_model_path')
    if model_path and not Path(model_path).is_file():
        raise ValueError('segmentation_model_path must be an existing model')
    frame = value('map_frame')
    common = {'use_sim_time': False}

    def config(package, filename):
        return os.path.join(get_package_share_directory(package), 'config', filename)

    def node(package, executable, name=None, parameters=None, **options):
        return Node(package=package, executable=executable, name=name, output='screen',
                    respawn=True, respawn_delay=2.0, parameters=parameters or [common], **options)
    nodes = [
        node('mavros', 'mavros_node', parameters=[config('lr_mavros_bridge', 'live.yaml'), {
            **common, 'fcu_url': fcu, 'gcs_url': '', 'fcu_protocol': 'v2.0',
            'tgt_system': 1, 'tgt_component': 1,
        }]),
        node('lr_mavros_bridge', 'mavros_pose', parameters=[{
            **common, 'time_mode': 'live', 'map_frame': frame,
            'child_frame': camera + '_camera_link', 'body_to_camera': mount,
            'pose_topic': '/lr/mavlink/pose', 'audit_file': value('mavros_audit_file'),
        }]),
        node('lr_mavros_bridge', 'live_runtime', parameters=[{
            **common, 'camera_name': camera, 'camera_model': model,
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
                }]),
            node('lr_segmentation', 'box_estimator_3d_node', 'box_estimator_3d', parameters=[
                config('lr_segmentation', 'segmentation.yaml'), {
                    **common, 'source_guard_enabled': True, 'output.frame_id': frame,
                    'geometry.up_axis': [0.0, 0.0, 1.0],
                    'input.depth_topic': sensor + 'depth/depth_registered',
                    'input.camera_info_topic': sensor + 'rgb/color/rect/camera_info',
                }]),
        ])
    return nodes


def generate_launch_description():
    required = ('camera_model', 'stream_address', 'stream_port', 'mavros_fcu_url')
    defaults = {'camera_name': 'zed', 'map_frame': 'map', 'segmentation_model_path': '',
                'mavlink_body_to_camera': '[0,0,0,0,0,0]', 'mavros_audit_file': ''}
    return LaunchDescription([
        *(DeclareLaunchArgument(name) for name in required),
        *(DeclareLaunchArgument(name, default_value=v) for name, v in defaults.items()),
        OpaqueFunction(function=launch_setup),
    ])
