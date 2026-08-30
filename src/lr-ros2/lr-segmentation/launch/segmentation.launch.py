"""Launch segmentation against an already-running ZED wrapper."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _default_model_path() -> str:
    env_path = os.environ.get("LR_SEGMENTATION_MODEL_PATH", "").strip()
    if env_path:
        return env_path
    return "/data/rover_workspace/lr-ros2/models/best.pt"


def launch_setup(context, *args, **kwargs):
    """Resolve camera-relative default input topics."""
    del args, kwargs
    camera_name = LaunchConfiguration("camera_name").perform(context)
    image_topic = LaunchConfiguration("image_topic").perform(context)
    if not image_topic:
        image_topic = f"/{camera_name}/zed_node/rgb/color/rect/image"
    model_path = LaunchConfiguration("model_path").perform(context).strip()
    model_device = LaunchConfiguration("model_device").perform(context).strip() or "0"
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
                    "model.path": model_path,
                    "model.device": ParameterValue(model_device, value_type=str),
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
            DeclareLaunchArgument(
                "model_path",
                default_value=_default_model_path(),
            ),
            DeclareLaunchArgument("model_device", default_value="0"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "segmentation_params_file",
                default_value=default_config,
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
