"""Launch the canonical Prediction engine, LR adapters, and RViz output."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
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
    runtime_config = os.path.join(
        get_package_share_directory("prediction_ros"), "config", "prediction.yaml"
    )
    rover_config = os.path.join(
        get_package_share_directory("prediction_core"),
        "config",
        "rover.reference.yaml",
    )
    profile = LaunchConfiguration("prediction_profile")
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_frame = LaunchConfiguration("map_frame")
    return LaunchDescription(
        [
            DeclareLaunchArgument("prediction_params_file", default_value=visualizer_config),
            DeclareLaunchArgument("bridge_params_file", default_value=bridge_config),
            DeclareLaunchArgument("prediction_runtime_params_file", default_value=runtime_config),
            DeclareLaunchArgument("rover_config", default_value=rover_config),
            DeclareLaunchArgument(
                "prediction_profile",
                default_value="static",
                choices=["static", "dynamic"],
            ),
            DeclareLaunchArgument("path_topic", default_value="/lr/future_path/ground_truth"),
            DeclareLaunchArgument("terrain_topic", default_value="/terrain_geometry/grid_map"),
            DeclareLaunchArgument("objects_topic", default_value="/segmentation/boxes_3d"),
            DeclareLaunchArgument("pose_topic", default_value="/lr/mavlink/pose"),
            DeclareLaunchArgument("map_frame", default_value="map"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            Node(
                package="lr_prediction_bridge",
                executable="prediction_bridge_node",
                name="prediction_bridge_node",
                parameters=[
                    LaunchConfiguration("bridge_params_file"),
                    {
                        "trajectory.input_topic": LaunchConfiguration("path_topic"),
                        "geometry.grid_map_topic": LaunchConfiguration("terrain_topic"),
                        "tracked_objects.input_topic": LaunchConfiguration("objects_topic"),
                        "rover_state.pose_topic": LaunchConfiguration("pose_topic"),
                        "prediction_profile": profile,
                        "expected_frame_id": map_frame,
                        "use_sim_time": use_sim_time,
                    },
                ],
                output="screen",
            ),
            Node(
                package="prediction_ros",
                executable="prediction_node",
                name="prediction_node",
                parameters=[
                    LaunchConfiguration("prediction_runtime_params_file"),
                    {
                        "config_path": LaunchConfiguration("rover_config"),
                        "prediction_profile": profile,
                        "expected_frame_id": map_frame,
                        "use_sim_time": use_sim_time,
                    },
                ],
                output="screen",
            ),
            Node(
                package="lr_path_prediction",
                executable="canonical_prediction_visualizer_node",
                name="canonical_prediction_visualizer",
                parameters=[
                    LaunchConfiguration("prediction_params_file"),
                    {
                        "input.terrain_topic": LaunchConfiguration("terrain_topic"),
                        "frames.map_frame": map_frame,
                        "use_sim_time": use_sim_time,
                    },
                ],
                output="screen",
            ),
        ]
    )
