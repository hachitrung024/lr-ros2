"""Launch the canonical Prediction engine, LR adapters, and RViz output."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    """Declare standalone canonical path-prediction launch arguments."""
    visualizer_config = os.path.join(
        get_package_share_directory("lr_path_prediction"),
        "config",
        "path_prediction.yaml",
    )
    bridge_config = os.path.join(
        get_package_share_directory("lr_prediction_bridge"),
        "config",
        "bridge.yaml",
    )
    rover_config = os.path.join(
        get_package_share_directory("prediction_core"),
        "config",
        "rover.reference.yaml",
    )
    profile = LaunchConfiguration("prediction_profile")
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_frame = LaunchConfiguration("map_frame")
    return LaunchDescription([
        DeclareLaunchArgument(
            "prediction_params_file", default_value=visualizer_config
        ),
        DeclareLaunchArgument(
            "bridge_params_file", default_value=bridge_config
        ),
        DeclareLaunchArgument(
            "rover_config", default_value=rover_config
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
            package="lr_prediction_bridge",
            executable="trajectory_adapter_node",
            parameters=[
                LaunchConfiguration("bridge_params_file"),
                {
                    "input_topic": LaunchConfiguration("path_topic"),
                    "expected_frame_id": map_frame,
                    "use_sim_time": use_sim_time,
                },
            ],
            output="screen",
        ),
        Node(
            package="lr_prediction_bridge",
            executable="geometry_adapter_node",
            parameters=[
                LaunchConfiguration("bridge_params_file"),
                {
                    "grid_map_topic": LaunchConfiguration("terrain_topic"),
                    "expected_frame_id": map_frame,
                    "use_sim_time": use_sim_time,
                },
            ],
            output="screen",
        ),
        Node(
            package="lr_prediction_bridge",
            executable="tracked_objects_adapter_node",
            parameters=[
                LaunchConfiguration("bridge_params_file"),
                {
                    "input_topic": LaunchConfiguration("objects_topic"),
                    "expected_frame_id": map_frame,
                    "use_sim_time": use_sim_time,
                },
            ],
            output="screen",
        ),
        Node(
            package="lr_prediction_bridge",
            executable="rover_state_adapter_node",
            parameters=[
                LaunchConfiguration("bridge_params_file"),
                {
                    "pose_topic": LaunchConfiguration("pose_topic"),
                    "expected_frame_id": map_frame,
                    "use_sim_time": use_sim_time,
                },
            ],
            condition=IfCondition(PythonExpression([
                "'", profile, "' == 'dynamic'"
            ])),
            output="screen",
        ),
        Node(
            package="prediction_ros",
            executable="prediction_node",
            parameters=[{
                "config_path": LaunchConfiguration("rover_config"),
                "prediction_profile": profile,
                "expected_frame_id": map_frame,
                "require_full_geometry_coverage": False,
                "reuse_latest_inputs_per_cycle": True,
                "use_sim_time": use_sim_time,
            }],
            output="screen",
        ),
        Node(
            package="lr_path_prediction",
            executable="canonical_prediction_visualizer_node",
            name="canonical_prediction_visualizer",
            parameters=[
                LaunchConfiguration("prediction_params_file"),
                {
                    "input.terrain_topic": LaunchConfiguration(
                        "terrain_topic"
                    ),
                    "frames.map_frame": map_frame,
                    "use_sim_time": use_sim_time,
                },
            ],
            output="screen",
        ),
    ])
