"""Launch the self-contained LR path-prediction node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Declare standalone path-prediction launch arguments."""
    default_config = os.path.join(
        get_package_share_directory("lr_path_prediction"),
        "config",
        "path_prediction.yaml",
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            "prediction_params_file", default_value=default_config
        ),
        DeclareLaunchArgument(
            "prediction_profile",
            default_value="static",
            choices=["static", "dynamic"],
        ),
        DeclareLaunchArgument(
            "path_topic", default_value="/lr/future_path/ground_truth"
        ),
        DeclareLaunchArgument(
            "terrain_topic", default_value="/terrain_geometry/grid_map"
        ),
        DeclareLaunchArgument(
            "objects_topic", default_value="/segmentation/boxes_3d"
        ),
        DeclareLaunchArgument(
            "pose_topic", default_value="/lr/mavlink/pose"
        ),
        DeclareLaunchArgument("map_frame", default_value="map"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(
            package="lr_path_prediction",
            executable="path_risk_predictor_node",
            name="path_risk_predictor",
            parameters=[
                LaunchConfiguration("prediction_params_file"),
                {
                    "input.path_topic": LaunchConfiguration("path_topic"),
                    "input.terrain_topic": LaunchConfiguration(
                        "terrain_topic"
                    ),
                    "input.objects_topic": LaunchConfiguration(
                        "objects_topic"
                    ),
                    "input.pose_topic": LaunchConfiguration("pose_topic"),
                    "frames.map_frame": LaunchConfiguration("map_frame"),
                    "prediction.profile": LaunchConfiguration(
                        "prediction_profile"
                    ),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                },
            ],
            output="screen",
        ),
    ])
