"""Launch segmentation against an already-running ZED wrapper."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    """Resolve camera-relative default input topics."""
    del args, kwargs
    camera_name = LaunchConfiguration("camera_name").perform(context)
    image_topic = LaunchConfiguration("image_topic").perform(context)
    if not image_topic:
        image_topic = f"/{camera_name}/zed_node/rgb/color/rect/image"
    depth_topic = LaunchConfiguration("depth_topic").perform(context)
    if not depth_topic:
        depth_topic = f"/{camera_name}/zed_node/depth/depth_registered"
    camera_info_topic = LaunchConfiguration("camera_info_topic").perform(context)
    if not camera_info_topic:
        camera_info_topic = (
            f"/{camera_name}/zed_node/rgb/color/rect/camera_info"
        )
    return [
        Node(
            package="lr_segmentation",
            executable="segmentation_node",
            name="segmentation",
            output="screen",
            parameters=[
                LaunchConfiguration("segmentation_params_file"),
                {
                    "input.image_topic": image_topic,
                    "input.depth_topic": depth_topic,
                    "input.camera_info_topic": camera_info_topic,
                    "model.path": LaunchConfiguration("model_path"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                },
            ],
        )
    ]


def generate_launch_description():
    """Declare the standalone segmentation launch interface."""
    default_config = os.path.join(
        get_package_share_directory("lr_segmentation"),
        "config",
        "segmentation.yaml",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("camera_name", default_value="zed"),
            DeclareLaunchArgument("image_topic", default_value=""),
            DeclareLaunchArgument("depth_topic", default_value=""),
            DeclareLaunchArgument("camera_info_topic", default_value=""),
            DeclareLaunchArgument("model_path"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "segmentation_params_file",
                default_value=default_config,
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
