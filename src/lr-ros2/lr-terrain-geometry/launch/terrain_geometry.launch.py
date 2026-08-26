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
            DeclareLaunchArgument("map_frame", default_value="map"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "terrain_params_file",
                default_value=default_config,
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
