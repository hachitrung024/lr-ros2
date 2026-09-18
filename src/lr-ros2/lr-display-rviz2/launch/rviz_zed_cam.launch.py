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

"""Launch only the LR RViz UI for a pipeline running on another ROS 2 host."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


CAMERA_MODELS = [
    'zed',
    'zedm',
    'zed2',
    'zed2i',
    'zedx',
    'zedxm',
    'zedxnano',
    'zedxhdr',
    'zedxhdrmini',
    'zedxhdrmax',
    'virtual',
    'zedxonegs',
    'zedxone4k',
    'zedxonehdr',
]

STEREO_CAMERA_MODELS = {
    'zed',
    'zedm',
    'zed2',
    'zed2i',
    'zedx',
    'zedxm',
    'virtual',
}


def launch_setup(context, *args, **kwargs):
    """Resolve the preset and create the single local UI process."""
    camera_name = LaunchConfiguration('camera_name').perform(context).strip() or 'zed'
    camera_model = LaunchConfiguration('camera_model').perform(context)
    segmentation_enabled = (
        LaunchConfiguration('segmentation_enabled').perform(context).lower() == 'true'
    )
    camera_type = 'stereo' if camera_model in STEREO_CAMERA_MODELS else 'mono'

    rviz_config = LaunchConfiguration('rviz_config').perform(context).strip()
    if not rviz_config:
        rviz_config = os.path.join(
            get_package_share_directory('lr_display_rviz2'),
            'rviz2',
            f'zed_{camera_type}.rviz',
        )

    remappings = []
    if camera_type == 'stereo' and not segmentation_enabled:
        remappings.append(
            ('/segmentation/overlay', f'/{camera_name}/zed_node/rgb/color/rect/image')
        )

    return [
        Node(
            package='rviz2',
            namespace=camera_name,
            executable='rviz2',
            name=f'{camera_model}_rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'svo_mode': LaunchConfiguration('svo_mode'),
                }
            ],
            remappings=remappings,
        )
    ]


def generate_launch_description():
    """Declare the RViz-only launch arguments."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'camera_name',
                default_value='zed',
                description='Remote ZED camera namespace.',
            ),
            DeclareLaunchArgument(
                'camera_model',
                description='Remote ZED camera model.',
                choices=CAMERA_MODELS,
            ),
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='false',
                choices=['true', 'false'],
                description='Use the remote /clock published during SVO playback.',
            ),
            DeclareLaunchArgument(
                'svo_mode',
                default_value='false',
                choices=['true', 'false'],
                description='Show the remote SVO playback controls.',
            ),
            DeclareLaunchArgument(
                'segmentation_enabled',
                default_value='true',
                choices=['true', 'false'],
                description=(
                    'Subscribe to the remote segmentation overlay. False '
                    'shows the remote ZED RGB stream in the same dock.'
                ),
            ),
            DeclareLaunchArgument(
                'rviz_config',
                default_value='',
                description='Optional RViz config path on this UI host.',
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
