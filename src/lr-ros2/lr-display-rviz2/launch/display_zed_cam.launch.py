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

import os

from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription,
    LogInfo
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    TextSubstitution
)
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):

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

    camera_name_val = camera_name.perform(context)
    camera_model_val = camera_model.perform(context)
    start_segmentation_val = start_segmentation_node.perform(context).lower()
    segmentation_model_path_val = segmentation_model_path.perform(context).strip()

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
        parameters=[{'use_sim_time': publish_svo_clock}],
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
                svo_realtime
            ],
            'publish_svo_clock': publish_svo_clock
        }.items(),
        condition=IfCondition(start_zed_node)
    )

    nodes = [
        rviz2_node,
        zed_wrapper_launch
    ]

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
                    'object_filter.enabled': False,
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
                    'input.depth_topic': (
                        f'/{camera_name_val}/zed_node/'
                        'depth/depth_registered'
                    ),
                    'input.camera_info_topic': (
                        f'/{camera_name_val}/zed_node/'
                        'rgb/color/rect/camera_info'
                    ),
                    'model.path': segmentation_model_path,
                    'use_sim_time': publish_svo_clock,
                }
            ],
            condition=IfCondition(TextSubstitution(text=start_segmentation_val))
        )
        nodes.append(segmentation_node)

    return nodes


def generate_launch_description():
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
                description='Path to an input SVO file. Keep `live` to use the camera.'),
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
                    'auto enables it when segmentation_model_path is provided.'),
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
            OpaqueFunction(function=launch_setup)
        ]
    )
