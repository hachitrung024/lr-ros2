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
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    camera_name = LaunchConfiguration('camera_name')
    camera_model = LaunchConfiguration('camera_model')
    start_zed_node = LaunchConfiguration('start_zed_node')
    svo_path = LaunchConfiguration('svo_path')
    publish_svo_clock = LaunchConfiguration('publish_svo_clock')
    serial_number = LaunchConfiguration('serial_number')
    camera_id = LaunchConfiguration('camera_id')
    ros_params_override_path = LaunchConfiguration('ros_params_override_path')
    param_overrides = LaunchConfiguration('param_overrides')
    pointcloud_topic = LaunchConfiguration('pointcloud_topic')
    rgb_topic = LaunchConfiguration('rgb_topic')
    future_path_topic = LaunchConfiguration('future_path_topic')
    trajectory_topic = LaunchConfiguration('trajectory_topic')
    frame_axes_topic = LaunchConfiguration('frame_axes_topic')
    rviz_config = LaunchConfiguration('rviz_config')

    default_rviz_config = os.path.join(
        get_package_share_directory('lr_display_rviz2'),
        'rviz',
        'zed_rgb_pointcloud.rviz',
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
            'param_overrides': param_overrides,
        }.items(),
        condition=IfCondition(start_zed_node),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        namespace=camera_name,
        name='rgb_pointcloud_rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': publish_svo_clock}],
        remappings=[
            ('zed_node/point_cloud/cloud_registered', pointcloud_topic),
            ('zed_node/rgb/color/rect/image', rgb_topic),
            ('/lr/mavlink/trajectory_future', future_path_topic),
            ('/lr/mavlink/trajectory', trajectory_topic),
            ('/lr/mavlink/frame_axes', frame_axes_topic),
        ],
    )

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
            'start_zed_node',
            default_value='true',
            choices=['true', 'false'],
            description='Start the ZED wrapper. Set false when it is already running.',
        ),
        DeclareLaunchArgument(
            'svo_path',
            default_value='live',
            description='Absolute SVO/SVO2 input path, or live for a physical camera.',
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
            description='Camera serial number. Zero selects the first available camera.',
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
            description='Semicolon-separated inline ZED wrapper parameter overrides.',
        ),
        DeclareLaunchArgument(
            'pointcloud_topic',
            default_value=[
                '/', camera_name, '/zed_node/point_cloud/cloud_registered',
            ],
            description='PointCloud2 topic displayed by RViz.',
        ),
        DeclareLaunchArgument(
            'rgb_topic',
            default_value=['/', camera_name, '/zed_node/rgb/color/rect/image'],
            description='Rectified RGB image topic displayed by RViz.',
        ),
        DeclareLaunchArgument(
            'future_path_topic',
            default_value='/lr/mavlink/trajectory_future',
            description='Future ENU trajectory displayed by RViz.',
        ),
        DeclareLaunchArgument(
            'trajectory_topic',
            default_value='/lr/mavlink/trajectory',
            description='Historical ENU trajectory displayed by RViz.',
        ),
        DeclareLaunchArgument(
            'frame_axes_topic',
            default_value='/lr/mavlink/frame_axes',
            description='Per-frame RGB coordinate axes displayed by RViz.',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz_config,
            description='Absolute path to an RViz2 configuration file.',
        ),
        zed_wrapper_launch,
        rviz_node,
    ])
