# Copyright 2025 Stereolabs
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

"""Launch ZED, RViz, LR perception, and optional SVO future ground truth."""

import ast
import math
import os

from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import (
    LaunchConfiguration,
    TextSubstitution
)
from launch_ros.actions import Node


def _stop_launch(reason):
    def raise_error(_context):
        raise RuntimeError(reason)

    return [
        LogInfo(msg=TextSubstitution(text=f'ERROR: {reason}')),
        OpaqueFunction(function=raise_error),
    ]


def _cache_process_exited(event, _context, *, cache_spec, pipeline):
    from lr_future_path.cache import cache_is_valid

    if event.returncode != 0:
        return _stop_launch(
            'Future-path cache process failed with return code '
            f'{event.returncode}.'
        )
    if not cache_is_valid(cache_spec.path, cache_spec.identity):
        return _stop_launch(
            f'Future-path cache is invalid after preprocessing: '
            f'{cache_spec.path}'
        )
    return [
        LogInfo(msg=TextSubstitution(
            text=(
                f'Future-path cache ready: {cache_spec.path}; '
                'starting SVO pipeline.'))),
        *pipeline,
    ]


def _mavlink_process_exited(event, context):
    if context.is_shutdown or event.returncode == 0:
        return []
    reason = (
        'MAVLink pose node failed with return code '
        f'{event.returncode}; stopping the SVO pipeline.'
    )
    return [
        LogInfo(msg=TextSubstitution(text=f'ERROR: {reason}')),
        EmitEvent(event=Shutdown(reason=reason)),
    ]


def _six_floats(value, name):
    try:
        parsed = ast.literal_eval(value)
        result = [float(item) for item in parsed]
    except (SyntaxError, ValueError, TypeError) as exception:
        raise ValueError(
            f'{name} must be a list of six finite numbers.'
        ) from exception
    if len(result) != 6 or not all(math.isfinite(item) for item in result):
        raise ValueError(f'{name} must be a list of six finite numbers.')
    return result


def launch_setup(context, *args, **kwargs):
    """Resolve options and construct immediate or cache-first actions."""
    # Launch configuration variables
    start_zed_node = LaunchConfiguration('start_zed_node')
    camera_name = LaunchConfiguration('camera_name')
    camera_model = LaunchConfiguration('camera_model')

    svo_path = LaunchConfiguration('svo_path')
    svo_realtime = LaunchConfiguration('svo_realtime')
    publish_svo_clock = LaunchConfiguration('publish_svo_clock')
    start_terrain_node = LaunchConfiguration('start_terrain_node')
    terrain_params_file = LaunchConfiguration('terrain_params_file')
    map_frame = LaunchConfiguration('map_frame')
    start_segmentation_node = LaunchConfiguration('start_segmentation_node')
    segmentation_params_file = LaunchConfiguration('segmentation_params_file')
    segmentation_model_path = LaunchConfiguration('segmentation_model_path')
    future_path = LaunchConfiguration('future_path')
    future_path_cache_dir = LaunchConfiguration('future_path_cache_dir')
    future_path_rebuild_cache = LaunchConfiguration(
        'future_path_rebuild_cache')
    future_path_topic = LaunchConfiguration('future_path_topic')
    future_path_radius_m = LaunchConfiguration('future_path_radius_m')
    future_path_step_m = LaunchConfiguration('future_path_step_m')
    mavlink = LaunchConfiguration('mavlink')
    mavlink_dir = LaunchConfiguration('mavlink_dir')
    mavlink_db_path = LaunchConfiguration('mavlink_db_path')
    mavlink_pose_topic = LaunchConfiguration('mavlink_pose_topic')
    mavlink_match_tolerance_s = LaunchConfiguration(
        'mavlink_match_tolerance_s')
    mavlink_max_gps_gap_s = LaunchConfiguration(
        'mavlink_max_gps_gap_s')
    mavlink_body_to_camera = LaunchConfiguration(
        'mavlink_body_to_camera')
    start_path_prediction_node = LaunchConfiguration(
        'start_path_prediction_node')
    prediction_params_file = LaunchConfiguration(
        'prediction_params_file')
    prediction_profile = LaunchConfiguration('prediction_profile')
    prediction_rover_config = LaunchConfiguration(
        'prediction_rover_config')

    camera_name_val = camera_name.perform(context)
    camera_model_val = camera_model.perform(context)
    start_segmentation_val = start_segmentation_node.perform(context).lower()
    segmentation_model_path_val = (
        segmentation_model_path.perform(context).strip()
    )
    start_prediction_val = (
        start_path_prediction_node.perform(context).lower()
    )
    prediction_profile_val = prediction_profile.perform(context).lower()
    svo_mode_val = svo_path.perform(context) != 'live'
    future_path_val = future_path.perform(context).lower() == 'true'
    mavlink_val = mavlink.perform(context).lower() == 'true'

    if mavlink_val and not svo_mode_val:
        return _stop_launch(
            'mavlink:=true requires an SVO file; live mode is unsupported.')
    if (
        mavlink_val
        and publish_svo_clock.perform(context).lower() != 'true'
    ):
        return _stop_launch(
            'mavlink:=true requires publish_svo_clock:=true.')

    mavlink_match_tolerance_val = 60.0
    mavlink_max_gps_gap_val = 1.5
    mavlink_body_to_camera_val = [0.0] * 6
    future_path_radius_val = 15.0
    future_path_step_val = 0.2
    if future_path_val:
        try:
            future_path_radius_val = float(
                future_path_radius_m.perform(context))
            future_path_step_val = float(
                future_path_step_m.perform(context))
            if (
                future_path_radius_val <= 0.0
                or future_path_step_val <= 0.0
            ):
                raise ValueError
        except ValueError:
            return _stop_launch(
                'future_path_radius_m and future_path_step_m must be '
                'positive.')
    if mavlink_val:
        try:
            mavlink_match_tolerance_val = float(
                mavlink_match_tolerance_s.perform(context))
            mavlink_max_gps_gap_val = float(
                mavlink_max_gps_gap_s.perform(context))
            if (
                mavlink_match_tolerance_val < 0.0
                or mavlink_max_gps_gap_val <= 0.0
            ):
                raise ValueError
            mavlink_body_to_camera_val = _six_floats(
                mavlink_body_to_camera.perform(context),
                'mavlink_body_to_camera',
            )
        except ValueError as exception:
            return _stop_launch(f'Invalid MAVLink configuration: {exception}')

    # A model path is the complete signal that segmentation was requested.
    # Keep the explicit flag for compatibility, but let a non-empty path
    # enable the node so callers do not need to repeat start_segmentation_node.
    segmentation_warning = None
    if segmentation_model_path_val and start_segmentation_val != 'false':
        start_segmentation_val = 'true'
    elif not segmentation_model_path_val:
        if start_segmentation_val == 'true':
            segmentation_warning = LogInfo(msg=TextSubstitution(
                text=(
                    'No segmentation_model_path was provided; '
                    'disabling start_segmentation_node.')))
        start_segmentation_val = 'false'

    if start_prediction_val == 'auto':
        start_prediction_val = str(
            future_path_val
            and start_terrain_node.perform(context).lower() == 'true'
            and start_segmentation_val == 'true'
        ).lower()
    if (
        start_prediction_val == 'true'
        and prediction_profile_val == 'dynamic'
        and not mavlink_val
    ):
        return _stop_launch(
            'prediction_profile:=dynamic requires mavlink:=true so '
            '/lr/mavlink/pose can provide rover acceleration.')

    if (camera_name_val == ''):
        camera_name_val = 'zed'

    camera_type = ''
    if (camera_model_val == 'zed' or
        camera_model_val == 'zedm' or
        camera_model_val == 'zed2' or
        camera_model_val == 'zed2i' or
        camera_model_val == 'zedx' or
        camera_model_val == 'zedxm' or
            camera_model_val == 'virtual'):
        camera_type = 'stereo'
    else:  # 'zedxonegs' or 'zedxone4k')
        camera_type = 'mono'

    # RVIZ2 Configurations to be loaded by ZED Node
    config_rviz2 = os.path.join(
        get_package_share_directory('lr_display_rviz2'),
        'rviz2',
        'zed_' + camera_type + '.rviz'
    )

    # RVIZ2 node
    rviz_remappings = []
    if camera_type == 'stereo' and start_segmentation_val != 'true':
        rviz_remappings.append((
            '/segmentation/overlay',
            f'/{camera_name_val}/zed_node/rgb/color/rect/image'
        ))

    rviz2_node = Node(
        package='rviz2',
        namespace=camera_name_val,
        executable='rviz2',
        name=camera_model_val + '_rviz2',
        output='screen',
        arguments=[['-d'], [config_rviz2]],
        parameters=[{
            'use_sim_time': publish_svo_clock,
            'svo_mode': svo_mode_val,
        }],
        remappings=rviz_remappings
    )

    # ZED Wrapper launch file
    zed_wrapper_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            get_package_share_directory('zed_wrapper'),
            '/launch/zed_camera.launch.py'
        ]),
        launch_arguments={
            'camera_name': camera_name_val,
            'camera_model': camera_model_val,
            'svo_path': svo_path,
            'param_overrides': [
                TextSubstitution(text='svo.svo_realtime:='),
                svo_realtime,
                TextSubstitution(text=(
                    ';pos_tracking.pos_tracking_enabled:=false;'
                    'depth.depth_stabilization:=0'
                    if mavlink_val else '')),
            ],
            'publish_svo_clock': publish_svo_clock,
            'publish_tf': 'false' if mavlink_val else 'true',
            'publish_map_tf': 'false' if mavlink_val else 'true',
            # The ZED component is the /clock producer in SVO playback.  It
            # must not use simulated time itself: the wrapper waits for a
            # /clock message before grabbing, which would deadlock a clock
            # producer.  All consumers below use publish_svo_clock as their
            # use_sim_time value instead.
            'use_sim_time': 'false',
        }.items(),
        condition=IfCondition(start_zed_node)
    )

    nodes = [
        rviz2_node,
        zed_wrapper_launch
    ]
    mavlink_exit_handler = None
    if mavlink_val:
        mavlink_node = Node(
            package='lr_future_path',
            executable='mavlink_pose_node',
            name='mavlink_pose',
            output='screen',
            parameters=[{
                'mavlink_dir': mavlink_dir.perform(context),
                'mavlink_db_path': mavlink_db_path.perform(context),
                'pose_topic': mavlink_pose_topic.perform(context),
                'future_path_enabled': future_path_val,
                'future_path_topic': future_path_topic.perform(context),
                'map_frame': map_frame.perform(context),
                'child_frame': f'{camera_name_val}_camera_link',
                'match_tolerance_s': mavlink_match_tolerance_val,
                'max_gps_gap_s': mavlink_max_gps_gap_val,
                'body_to_camera': mavlink_body_to_camera_val,
                'radius_m': future_path_radius_val,
                'step_m': future_path_step_val,
                'max_points': 1000,
                'use_sim_time': True,
            }],
        )
        nodes.insert(0, mavlink_node)
        mavlink_exit_handler = RegisterEventHandler(
            OnProcessExit(
                target_action=mavlink_node,
                on_exit=_mavlink_process_exited,
            )
        )

    if segmentation_warning is not None:
        nodes.insert(0, segmentation_warning)

    if camera_type == 'stereo':
        terrain_node = Node(
            package='lr_terrain_geometry',
            executable='terrain_geometry_node',
            name='terrain_geometry',
            output='screen',
            parameters=[
                terrain_params_file,
                {
                    'input.point_cloud_topic': (
                        f'/{camera_name_val}/zed_node/'
                        'point_cloud/cloud_registered'
                    ),
                    'frames.map_frame': map_frame,
                    'use_sim_time': publish_svo_clock,
                }
            ],
            condition=IfCondition(start_terrain_node)
        )
        nodes.append(terrain_node)

        segmentation_node = Node(
            package='lr_segmentation',
            executable='segmentation_node',
            name='segmentation',
            output='screen',
            parameters=[
                segmentation_params_file,
                {
                    'input.image_topic': (
                        f'/{camera_name_val}/zed_node/'
                        'rgb/color/rect/image'
                    ),
                    'model.path': segmentation_model_path,
                    'use_sim_time': publish_svo_clock,
                }
            ],
            condition=IfCondition(
                TextSubstitution(text=start_segmentation_val))
        )
        nodes.append(segmentation_node)

        box_estimator_node = Node(
            package='lr_segmentation',
            executable='box_estimator_3d_node',
            name='box_estimator_3d',
            output='screen',
            parameters=[
                segmentation_params_file,
                {
                    'input.mask_topic': '/segmentation/instance_mask',
                    'input.depth_topic': (
                        f'/{camera_name_val}/zed_node/'
                        'depth/depth_registered'
                    ),
                    'input.camera_info_topic': (
                        f'/{camera_name_val}/zed_node/'
                        'rgb/color/rect/camera_info'
                    ),
                    'output.box_topic': '/segmentation/boxes_3d',
                    'output.frame_id': map_frame,
                    'geometry.up_axis': [0.0, 0.0, 1.0],
                    'use_sim_time': publish_svo_clock,
                }
            ],
            condition=IfCondition(
                TextSubstitution(text=start_segmentation_val))
        )
        nodes.append(box_estimator_node)

        prediction_condition = IfCondition(
            TextSubstitution(text=start_prediction_val))
        trajectory_adapter_node = Node(
            package='lr_prediction_bridge',
            executable='trajectory_adapter_node',
            name='trajectory_adapter_node',
            output='screen',
            parameters=[{
                'input_topic': future_path_topic,
                'expected_frame_id': map_frame,
                'force_frame_id_map': True,
                'horizon_steps': 20,
                'output_dt_sec': 0.25,
                'min_distance_from_start_m': 1.0,
                'minimum_cycle_period_sec': 0.25,
                'use_sim_time': publish_svo_clock,
            }],
            condition=prediction_condition,
        )
        geometry_adapter_node = Node(
            package='lr_prediction_bridge',
            executable='geometry_adapter_node',
            name='geometry_adapter_node',
            output='screen',
            parameters=[{
                'grid_map_topic': '/terrain_geometry/grid_map',
                'expected_frame_id': map_frame,
                'force_frame_id_map': True,
                'allow_flat_fallback': False,
                'use_sim_time': publish_svo_clock,
            }],
            condition=prediction_condition,
        )
        tracked_objects_adapter_node = Node(
            package='lr_prediction_bridge',
            executable='tracked_objects_adapter_node',
            name='tracked_objects_adapter_node',
            output='screen',
            parameters=[{
                'input_topic': '/segmentation/boxes_3d',
                'expected_frame_id': map_frame,
                'use_sim_time': publish_svo_clock,
            }],
            condition=prediction_condition,
        )
        prediction_node = Node(
            package='prediction_ros',
            executable='prediction_node',
            name='prediction_node',
            output='screen',
            parameters=[{
                'config_path': prediction_rover_config,
                'prediction_profile': prediction_profile,
                'expected_frame_id': map_frame,
                'require_full_geometry_coverage': False,
                'reuse_latest_inputs_per_cycle': True,
                'use_sim_time': publish_svo_clock,
            }],
            condition=prediction_condition,
        )
        prediction_visualizer_node = Node(
            package='lr_path_prediction',
            executable='canonical_prediction_visualizer_node',
            name='canonical_prediction_visualizer',
            output='screen',
            parameters=[
                prediction_params_file,
                {
                    'input.terrain_topic': '/terrain_geometry/grid_map',
                    'frames.map_frame': map_frame,
                    'use_sim_time': publish_svo_clock,
                }
            ],
            condition=prediction_condition,
        )
        nodes.extend([
            trajectory_adapter_node,
            geometry_adapter_node,
            tracked_objects_adapter_node,
            prediction_node,
            prediction_visualizer_node,
        ])
        if prediction_profile_val == 'dynamic':
            nodes.append(Node(
                package='lr_prediction_bridge',
                executable='rover_state_adapter_node',
                name='rover_state_adapter_node',
                output='screen',
                parameters=[{
                    'pose_topic': mavlink_pose_topic,
                    'expected_frame_id': map_frame,
                    'force_frame_id_map': True,
                    'use_sim_time': publish_svo_clock,
                }],
                condition=prediction_condition,
            ))

    if not future_path_val:
        return (
            [mavlink_exit_handler, *nodes]
            if mavlink_exit_handler is not None
            else nodes
        )

    if not svo_mode_val:
        return _stop_launch(
            'future_path:=true requires an SVO file; live mode is '
            'unsupported.')

    radius_m_val = future_path_radius_val
    step_m_val = future_path_step_val

    if mavlink_val:
        cache_note = LogInfo(msg=TextSubstitution(text=(
            'MAVLink supplies future ground truth directly; '
            'skipping the SVO VIO cache pass.')))
        return [mavlink_exit_handler, cache_note, *nodes]

    from lr_future_path.cache import build_cache_spec, cache_is_valid

    try:
        cache_spec = build_cache_spec(
            svo_path.perform(context),
            camera_model_val,
            future_path_cache_dir.perform(context),
        )
    except (FileNotFoundError, OSError, ValueError) as exception:
        return _stop_launch(f'Cannot resolve future-path cache: {exception}')

    future_node = Node(
        package='lr_future_path',
        executable='future_ground_truth_node',
        name='future_ground_truth',
        output='screen',
        parameters=[{
            'cache_path': str(cache_spec.path),
            'input_pose_topic': f'/{camera_name_val}/zed_node/pose',
            'output_topic': future_path_topic.perform(context),
            'radius_m': radius_m_val,
            'step_m': step_m_val,
            'max_gap_s': 1.0,
            'max_points': 1000,
            'use_sim_time': publish_svo_clock,
        }],
    )
    nodes.append(future_node)

    rebuild_cache = (
        future_path_rebuild_cache.perform(context).lower() == 'true')
    if (
        cache_is_valid(cache_spec.path, cache_spec.identity)
        and not rebuild_cache
    ):
        return [
            LogInfo(msg=TextSubstitution(
                text=f'Future-path cache hit: {cache_spec.path}')),
            *nodes,
        ]

    if start_zed_node.perform(context).lower() != 'true':
        return _stop_launch(
            'Future-path cache is missing, but start_zed_node is false. '
            'Build the cache first or let this launch start ZED.')

    cache_process = ExecuteProcess(
        cmd=[
            'ros2',
            'launch',
            'lr_future_path',
            'build_svo_pose_cache.launch.py',
            f'camera_name:={camera_name_val}',
            f'camera_model:={camera_model_val}',
            f'svo_path:={svo_path.perform(context)}',
            f'cache_path:={cache_spec.path}',
            'rebuild:=true' if rebuild_cache else 'rebuild:=false',
        ],
        output='screen',
    )
    cache_exit_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=cache_process,
            on_exit=lambda event, child_context: _cache_process_exited(
                event,
                child_context,
                cache_spec=cache_spec,
                pipeline=nodes,
            ),
        )
    )
    return [
        cache_exit_handler,
        LogInfo(msg=TextSubstitution(
            text=(
                f'Future-path cache miss: {cache_spec.path}; '
                'running headless SVO preprocessing pass.'))),
        cache_process,
    ]


def generate_launch_description():
    """Declare display pipeline launch arguments."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'start_zed_node',
                default_value='True',
                description=(
                    'Set to `False` to start only RVIZ2 if a ZED node is '
                    'already running.')),
            DeclareLaunchArgument(
                'camera_name',
                default_value=TextSubstitution(text='zed'),
                description=(
                    'The name of the camera. It can be different from the '
                    'camera model and it will be used as node `namespace`.')),
            DeclareLaunchArgument(
                'camera_model',
                description=(
                    '[REQUIRED] The model of the camera. Using a wrong camera '
                    'model can disable camera features.'),
                choices=[
                    'zed', 'zedm', 'zed2', 'zed2i', 'zedx', 'zedxm',
                    'zedxnano', 'zedxhdr', 'zedxhdrmini', 'zedxhdrmax',
                    'virtual', 'zedxonegs', 'zedxone4k', 'zedxonehdr'
                ]),
            DeclareLaunchArgument(
                'svo_path',
                default_value=TextSubstitution(text='live'),
                description=(
                    'Path to an input SVO file. Keep `live` to use the '
                    'camera.')),
            DeclareLaunchArgument(
                'svo_realtime',
                default_value='true',
                description=(
                    'Play an SVO at its original recorded frame rate, '
                    'skipping frames if needed.'),
                choices=['true', 'false']),
            DeclareLaunchArgument(
                'publish_svo_clock',
                default_value='false',
                description=(
                    'If set to `true` the node will act as a clock server '
                    'publishing the SVO timestamp. This is useful for node '
                    'synchronization')),
            DeclareLaunchArgument(
                'future_path',
                default_value='false',
                description=(
                    'Publish future ground-truth nav_msgs/Path from the '
                    'MAVLink session or an SVO VIO cache.'),
                choices=['true', 'false']),
            DeclareLaunchArgument(
                'future_path_cache_dir',
                default_value='',
                description=(
                    'Cache root. Empty stores .lr_future_path_cache beside '
                    'the SVO file.')),
            DeclareLaunchArgument(
                'future_path_rebuild_cache',
                default_value='false',
                description='Force a fresh headless SVO cache pass.',
                choices=['true', 'false']),
            DeclareLaunchArgument(
                'future_path_topic',
                default_value='/lr/future_path/ground_truth',
                description='Output topic for nav_msgs/msg/Path.'),
            DeclareLaunchArgument(
                'future_path_radius_m',
                default_value='15.0',
                description='Maximum XY look-ahead radius in metres.'),
            DeclareLaunchArgument(
                'future_path_step_m',
                default_value='0.2',
                description='Spatial downsampling step in metres.'),
            DeclareLaunchArgument(
                'mavlink',
                default_value='false',
                description=(
                    'Replace ZED dynamic localization TF with an offline '
                    'MAVLink SQLite session.'),
                choices=['true', 'false']),
            DeclareLaunchArgument(
                'mavlink_dir',
                default_value='mavlink',
                description=(
                    'Directory recursively searched for session_mavlink.db; '
                    'relative paths use the launch working directory.')),
            DeclareLaunchArgument(
                'mavlink_db_path',
                default_value='',
                description=(
                    'Explicit session_mavlink.db path; bypasses directory '
                    'discovery but is still validated.')),
            DeclareLaunchArgument(
                'mavlink_pose_topic',
                default_value='/lr/mavlink/pose',
                description='MAVLink-derived geometry_msgs/msg/PoseStamped.'),
            DeclareLaunchArgument(
                'mavlink_match_tolerance_s',
                default_value='60.0',
                description=(
                    'Maximum difference between SVO and MAVLink session '
                    'start times in seconds.')),
            DeclareLaunchArgument(
                'mavlink_max_gps_gap_s',
                default_value='1.5',
                description=(
                    'Suppress MAVLink pose/TF/path when valid GPS samples '
                    'are separated by a larger interval.')),
            DeclareLaunchArgument(
                'mavlink_body_to_camera',
                default_value='[0,0,0,0,0,0]',
                description=(
                    'Camera mounting transform [x,y,z,roll,pitch,yaw] in '
                    'ROS FLU metres/radians.')),
            DeclareLaunchArgument(
                'start_terrain_node',
                default_value='true',
                description=(
                    'Start the LR terrain geometry estimator for stereo '
                    'camera models.'),
                choices=['true', 'false']),
            DeclareLaunchArgument(
                'terrain_params_file',
                default_value=os.path.join(
                    get_package_share_directory('lr_terrain_geometry'),
                    'config',
                    'terrain_geometry.yaml'
                ),
                description='Terrain geometry ROS parameter file.'),
            DeclareLaunchArgument(
                'map_frame',
                default_value='map',
                description='World frame used by terrain geometry.'),
            DeclareLaunchArgument(
                'start_segmentation_node',
                default_value='auto',
                description=(
                    'Start segmentation, disable explicitly with false; '
                    'auto enables it when segmentation_model_path is '
                    'provided.'),
                choices=['auto', 'true', 'false']),
            DeclareLaunchArgument(
                'segmentation_params_file',
                default_value=os.path.join(
                    get_package_share_directory('lr_segmentation'),
                    'config',
                    'segmentation.yaml'
                ),
                description='Segmentation ROS parameter file.'),
            DeclareLaunchArgument(
                'segmentation_model_path',
                default_value='',
                description=(
                    'External Ultralytics instance-segmentation checkpoint.')),
            DeclareLaunchArgument(
                'start_path_prediction_node',
                default_value='auto',
                description=(
                    'Start 20-step terrain/object prediction. Auto enables '
                    'it when future path, terrain, and segmentation are '
                    'enabled.'),
                choices=['auto', 'true', 'false']),
            DeclareLaunchArgument(
                'prediction_params_file',
                default_value=os.path.join(
                    get_package_share_directory('lr_path_prediction'),
                    'config',
                    'path_prediction.yaml'
                ),
                description='Prediction visualization parameter file.'),
            DeclareLaunchArgument(
                'prediction_profile',
                default_value='static',
                description=(
                    'Canonical prediction profile. Dynamic additionally uses '
                    'finite-difference acceleration from MAVLink pose.'),
                choices=['static', 'dynamic']),
            DeclareLaunchArgument(
                'prediction_rover_config',
                default_value=os.path.join(
                    get_package_share_directory('prediction_core'),
                    'config',
                    'rover.reference.yaml'
                ),
                description=(
                    'Measured rover geometry/mass/CoM YAML. Bundled values '
                    'are references only and are not field-safe.')),
            OpaqueFunction(function=launch_setup)
        ]
    )
