"""A restartable NET camera and its stamp relay; never owns map TF or clock."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes, SetRemap
from launch_ros.descriptions import ComposableNode


def setup(context):
    values = {name: LaunchConfiguration(name).perform(context) for name in (
        'camera_name', 'camera_model', 'stream_address', 'stream_port')}
    base = '/' + values['camera_name'] + '/zed_node/'
    topics = ('rgb/color/rect/image', 'depth/depth_registered', 'rgb/color/rect/camera_info',
              'depth/camera_info', 'point_cloud/cloud_registered', 'imu/data')
    camera = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory('zed_wrapper'), 'launch', 'zed_camera.launch.py')),
        launch_arguments={**values, 'svo_path': 'live', 'publish_svo_clock': 'false',
                          'use_sim_time': 'false', 'publish_tf': 'false', 'publish_map_tf': 'false',
                          'param_overrides': 'debug.use_pub_timestamps:=false;'
                          'general.sdk_use_monotonic_clock:=false;'
                          'sensors.sensors_image_sync:=true;'
                          'pos_tracking.pos_tracking_enabled:=false;depth.depth_stabilization:=0',
                          }.items())
    return [GroupAction([*(SetRemap(src=base + t, dst=base + '_raw/' + t) for t in topics), camera]),
            LoadComposableNodes(target_container='/' + values['camera_name'] + '/zed_container',
                                composable_node_descriptions=[ComposableNode(
                                    package='lr_mavros_bridge',
                                    plugin='lr_mavros_bridge::NetSensorRelay',
                                    name='net_sensor_relay', parameters=[{
                                        'camera_name': values['camera_name'], 'use_sim_time': False,
                                    }])])]


def generate_launch_description():
    return LaunchDescription([*(DeclareLaunchArgument(name) for name in (
        'camera_name', 'camera_model', 'stream_address', 'stream_port')), OpaqueFunction(function=setup)])
