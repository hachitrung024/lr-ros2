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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.logging import get_logger
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


LOGGER = get_logger('lr_bringup.rover')


def _value(context, name):
    return LaunchConfiguration(name).perform(context)


def _select_pose_mode(svo_path, gps_path, attitude_path):
    """Validate launch inputs and select either ZED TF or MAVLink CSV TF."""
    if svo_path == 'live':
        if gps_path or attitude_path:
            return 'tf', 'live mode ignores MAVLink CSV paths'
        return 'tf', 'live mode uses ZED positional tracking TF'

    if not gps_path and not attitude_path:
        return 'tf', 'SVO has no MAVLink CSV paths; using ZED TF'
    if not gps_path or not attitude_path:
        return 'tf', (
            'SVO requires both gps_path and attitude_path for MAVLink replay; '
            'using ZED TF'
        )

    try:
        from lr_mavlink_replay.csv_pose import load_pose_log
        load_pose_log(gps_path, attitude_path)
    except (FileNotFoundError, OSError, TypeError, ValueError) as exception:
        return 'tf', f'invalid MAVLink CSV pair ({exception}); using ZED TF'
    return 'mavlink', 'valid MAVLink CSV pair; using external ENU pose'


def _launch_setup(context):
    camera_model = _value(context, 'camera_model')
    camera_name = _value(context, 'camera_name')
    svo_path = _value(context, 'svo_path')
    publish_svo_clock = _value(context, 'publish_svo_clock')
    serial_number = _value(context, 'serial_number')
    camera_id = _value(context, 'camera_id')
    ros_params_override_path = _value(context, 'ros_params_override_path')
    param_overrides = _value(context, 'param_overrides')
    input_topic = _value(context, 'input_topic')
    output_topic = _value(context, 'output_topic')
    target_frame = _value(context, 'target_frame')
    transform_timeout_sec = float(_value(context, 'transform_timeout_sec'))
    use_rviz = LaunchConfiguration('use_rviz')

    gps_path = _value(context, 'gps_path')
    attitude_path = _value(context, 'attitude_path')
    position_topic = _value(context, 'mavlink_position_topic')
    attitude_topic = _value(context, 'mavlink_attitude_topic')
    future_path_topic = _value(context, 'mavlink_future_path_topic')
    trajectory_topic = _value(context, 'mavlink_trajectory_topic')
    frame_axes_topic = _value(context, 'mavlink_frame_axes_topic')
    future_path_radius_m = float(_value(context, 'future_path_radius_m'))
    future_path_step_m = float(_value(context, 'future_path_step_m'))
    future_path_frames = int(_value(context, 'future_path_frames'))
    svo_fps = float(_value(context, 'svo_fps'))
    base_frame = _value(context, 'mavlink_base_frame')
    extrinsic = {
        name: float(_value(context, name))
        for name in (
            'base_to_camera_x_m',
            'base_to_camera_y_m',
            'base_to_camera_z_m',
            'base_to_camera_roll_deg',
            'base_to_camera_pitch_deg',
            'base_to_camera_yaw_deg',
        )
    }

    pose_source, reason = _select_pose_mode(
        svo_path, gps_path, attitude_path
    )
    if pose_source == 'mavlink' or (
        svo_path == 'live' and not gps_path and not attitude_path
    ):
        LOGGER.info(reason)
    else:
        LOGGER.warning(reason)

    zed_publish_tf = 'false' if pose_source == 'mavlink' else 'true'
    zed_param_overrides = param_overrides
    if pose_source == 'mavlink':
        mandatory_overrides = (
            'pos_tracking.publish_tf:=false;'
            'pos_tracking.publish_map_tf:=false'
        )
        zed_param_overrides = ';'.join(
            value for value in (param_overrides, mandatory_overrides) if value
        )

    zed_wrapper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('zed_wrapper'),
                'launch',
                'zed_camera.launch.py',
            )
        ),
        launch_arguments={
            'camera_name': camera_name,
            'camera_model': camera_model,
            'svo_path': svo_path,
            'publish_svo_clock': publish_svo_clock,
            'serial_number': serial_number,
            'camera_id': camera_id,
            'ros_params_override_path': ros_params_override_path,
            'param_overrides': zed_param_overrides,
            'publish_tf': zed_publish_tf,
            'publish_map_tf': zed_publish_tf,
        }.items(),
    )

    actions = [zed_wrapper_launch]
    if pose_source == 'mavlink':
        actions.append(Node(
            package='lr_mavlink_replay',
            executable='mavlink_csv_pose_node',
            name='mavlink_csv_pose_node',
            output='screen',
            parameters=[{
                'use_sim_time': publish_svo_clock == 'true',
                'gps_path': gps_path,
                'attitude_path': attitude_path,
                'input_topic': input_topic,
                'position_topic': position_topic,
                'attitude_topic': attitude_topic,
                'future_path_topic': future_path_topic,
                'trajectory_topic': trajectory_topic,
                'frame_axes_topic': frame_axes_topic,
                'axis_length_m': float(_value(context, 'axis_length_m')),
                'future_path_radius_m': future_path_radius_m,
                'future_path_step_m': future_path_step_m,
                'future_path_frames': future_path_frames,
                'svo_fps': svo_fps,
                'world_frame': target_frame,
            }],
        ))

    transform_parameters = {
        'use_sim_time': publish_svo_clock == 'true',
        'input_topic': input_topic,
        'output_topic': output_topic,
        'target_frame': target_frame,
        'transform_timeout_sec': transform_timeout_sec,
        'pose_source': pose_source,
        'mavlink_position_topic': position_topic,
        'mavlink_attitude_topic': attitude_topic,
        'mavlink_base_frame': base_frame,
        'camera_link_frame': f'{camera_name}_camera_link',
        'accumulate_cloud': _value(context, 'accumulate_cloud') == 'true',
        'frame_step': int(_value(context, 'map_frame_step')),
        'point_stride': int(_value(context, 'map_point_stride')),
        'max_map_points': int(_value(context, 'max_map_points')),
    }
    transform_parameters.update(extrinsic)
    actions.append(Node(
        package='lr_pointcloud_transform',
        executable='pointcloud_transform_node',
        name='pointcloud_transform_node',
        output='screen',
        parameters=[transform_parameters],
    ))

    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('lr_display_rviz2'),
                'launch',
                'display_zed_cam.launch.py',
            )
        ),
        launch_arguments={
            'camera_name': camera_name,
            'camera_model': camera_model,
            'start_zed_node': 'false',
            'publish_svo_clock': publish_svo_clock,
            'pointcloud_topic': output_topic,
            'future_path_topic': future_path_topic,
            'trajectory_topic': trajectory_topic,
            'frame_axes_topic': frame_axes_topic,
        }.items(),
        condition=IfCondition(use_rviz),
    ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_model',
            description='ZED camera model.',
            choices=[
                'zed', 'zedm', 'zed2', 'zed2i', 'zedx', 'zedxm',
                'zedxnano', 'zedxhdr', 'zedxhdrmini', 'zedxhdrmax',
                'virtual', 'zedxonegs', 'zedxone4k', 'zedxonehdr',
            ],
        ),
        DeclareLaunchArgument(
            'camera_name',
            default_value='zed',
            description='Camera namespace and frame prefix.',
        ),
        DeclareLaunchArgument(
            'svo_path',
            default_value='live',
            description=(
                'Absolute SVO/SVO2 input path, or live for a physical camera.'
            ),
        ),
        DeclareLaunchArgument(
            'publish_svo_clock',
            default_value='false',
            choices=['true', 'false'],
            description='Publish and use the SVO timestamp on /clock.',
        ),
        DeclareLaunchArgument(
            'serial_number',
            default_value='0',
            description=(
                'Camera serial number. Zero selects the first available camera.'
            ),
        ),
        DeclareLaunchArgument(
            'camera_id',
            default_value='-1',
            description='Camera ID. Minus one selects the first available camera.',
        ),
        DeclareLaunchArgument(
            'ros_params_override_path',
            default_value='',
            description='Optional YAML file overriding ZED wrapper parameters.',
        ),
        DeclareLaunchArgument(
            'param_overrides',
            default_value='',
            description=(
                'Semicolon-separated inline ZED wrapper parameter overrides.'
            ),
        ),
        DeclareLaunchArgument(
            'input_topic',
            default_value=[
                '/', LaunchConfiguration('camera_name'),
                '/zed_node/point_cloud/cloud_registered',
            ],
            description='PointCloud2 topic transformed by the processing node.',
        ),
        DeclareLaunchArgument(
            'output_topic',
            default_value='/lr/point_cloud/cloud_in_map',
            description='Transformed PointCloud2 output topic.',
        ),
        DeclareLaunchArgument(
            'target_frame',
            default_value='map',
            description='TF frame into which the point cloud is transformed.',
        ),
        DeclareLaunchArgument(
            'transform_timeout_sec',
            default_value='0.5',
            description='Maximum time to wait for the matching TF transform.',
        ),
        DeclareLaunchArgument(
            'gps_path',
            default_value='',
            description='MAVLink GPS CSV used only with SVO replay.',
        ),
        DeclareLaunchArgument(
            'attitude_path',
            default_value='',
            description='MAVLink attitude CSV used only with SVO replay.',
        ),
        DeclareLaunchArgument(
            'mavlink_position_topic',
            default_value='/lr/mavlink/position_enu',
            description='Exact-cloud-time MAVLink ENU position topic.',
        ),
        DeclareLaunchArgument(
            'mavlink_attitude_topic',
            default_value='/lr/mavlink/attitude_enu',
            description='Exact-cloud-time MAVLink ENU/FLU attitude topic.',
        ),
        DeclareLaunchArgument(
            'mavlink_future_path_topic',
            default_value='/lr/mavlink/trajectory_future',
            description='Recorded future ENU trajectory for each cloud.',
        ),
        DeclareLaunchArgument(
            'mavlink_trajectory_topic',
            default_value='/lr/mavlink/trajectory',
            description='Accumulated historical MAVLink ENU trajectory.',
        ),
        DeclareLaunchArgument(
            'mavlink_frame_axes_topic',
            default_value='/lr/mavlink/frame_axes',
            description='Persistent RGB coordinate axes for individual frames.',
        ),
        DeclareLaunchArgument(
            'axis_length_m',
            default_value='0.5',
            description='Length in metres of each per-frame RGB axis.',
        ),
        DeclareLaunchArgument(
            'future_path_radius_m',
            default_value='15.0',
            description='XY radius of the recorded future path, in metres.',
        ),
        DeclareLaunchArgument(
            'future_path_step_m',
            default_value='0.2',
            description='Ground distance between future path poses.',
        ),
        DeclareLaunchArgument(
            'future_path_frames',
            default_value='200',
            description='Number of future SVO frames displayed as the orange path.',
        ),
        DeclareLaunchArgument(
            'svo_fps',
            default_value='15.0',
            description='SVO frame rate used to timestamp future trajectory poses.',
        ),
        DeclareLaunchArgument(
            'mavlink_base_frame',
            default_value='lr_base_link',
            description='Rover FLU base frame used by MAVLink pose.',
        ),
        DeclareLaunchArgument(
            'base_to_camera_x_m',
            default_value='0.0',
            description='Camera X translation in base FLU, in metres.',
        ),
        DeclareLaunchArgument(
            'base_to_camera_y_m',
            default_value='0.0',
            description='Camera Y translation in base FLU, in metres.',
        ),
        DeclareLaunchArgument(
            'base_to_camera_z_m',
            default_value='0.0',
            description='Camera Z translation in base FLU, in metres.',
        ),
        DeclareLaunchArgument(
            'base_to_camera_roll_deg',
            default_value='0.0',
            description='Fixed-axis camera roll in base FLU, in degrees.',
        ),
        DeclareLaunchArgument(
            'base_to_camera_pitch_deg',
            default_value='0.0',
            description='Fixed-axis camera pitch in base FLU, in degrees.',
        ),
        DeclareLaunchArgument(
            'base_to_camera_yaw_deg',
            default_value='0.0',
            description='Fixed-axis camera yaw in base FLU, in degrees.',
        ),
        DeclareLaunchArgument(
            'accumulate_cloud', default_value='true', choices=['true', 'false'],
            description='Accumulate transformed clouds instead of replacing each frame.',
        ),
        DeclareLaunchArgument(
            'map_frame_step', default_value='3',
            description='Keep one transformed cloud every N input frames.',
        ),
        DeclareLaunchArgument(
            'map_point_stride', default_value='8',
            description='Keep one point every N points in each selected cloud.',
        ),
        DeclareLaunchArgument(
            'max_map_points', default_value='500000',
            description='Maximum number of accumulated map points.',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            choices=['true', 'false'],
            description='Start the RGB and transformed-point-cloud RViz UI.',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
