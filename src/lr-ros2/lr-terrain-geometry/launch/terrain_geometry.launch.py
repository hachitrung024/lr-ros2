"""Launch the terrain estimator against an already-running ZED wrapper."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    del args, kwargs
    camera_name = LaunchConfiguration("camera_name").perform(context)
    point_cloud_topic = LaunchConfiguration("point_cloud_topic").perform(context)
    if not point_cloud_topic:
        point_cloud_topic = f"/{camera_name}/zed_node/point_cloud/cloud_registered"
    detections_topic = LaunchConfiguration("detections_topic").perform(context)
    if not detections_topic:
        detections_topic = "/segmentation/detections_2d"
    instance_masks_topic = LaunchConfiguration("instance_masks_topic").perform(
        context
    )
    if not instance_masks_topic:
        instance_masks_topic = "/segmentation/instance_masks"
    camera_info_topic = LaunchConfiguration("camera_info_topic").perform(context)
    if not camera_info_topic:
        camera_info_topic = (
            f"/{camera_name}/zed_node/rgb/color/rect/camera_info"
        )
    object_filter_enabled = (
        LaunchConfiguration("object_filter_enabled")
        .perform(context)
        .lower() == "true"
    )
    return [
        Node(
            package="lr_terrain_geometry",
            executable="terrain_geometry_node",
            name="terrain_geometry",
            output="screen",
            parameters=[
                LaunchConfiguration("terrain_params_file"),
                {
                    "input.point_cloud_topic": point_cloud_topic,
                    "frames.map_frame": LaunchConfiguration("map_frame"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "object_filter.enabled": object_filter_enabled,
                    "object_filter.detections_topic": detections_topic,
                    "object_filter.instance_masks_topic": instance_masks_topic,
                    "object_filter.camera_info_topic": camera_info_topic,
                },
            ],
        )
    ]


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("lr_terrain_geometry"),
        "config",
        "terrain_geometry.yaml",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("camera_name", default_value="zed"),
            DeclareLaunchArgument("point_cloud_topic", default_value=""),
            DeclareLaunchArgument(
                "object_filter_enabled",
                default_value="false",
                choices=["true", "false"],
            ),
            DeclareLaunchArgument("detections_topic", default_value=""),
            DeclareLaunchArgument("instance_masks_topic", default_value=""),
            DeclareLaunchArgument("camera_info_topic", default_value=""),
            DeclareLaunchArgument("map_frame", default_value="map"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "terrain_params_file",
                default_value=default_config,
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
