from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    config = Path(get_package_share_directory("prediction_ros")) / "config" / "prediction.yaml"
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_path",
                description="Absolute path to rover YAML config (e.g. .../config/rover.mock.yaml)",
            ),
            DeclareLaunchArgument(
                "prediction_profile",
                default_value="static",
                description="Runtime readiness profile: static | dynamic",
            ),
            Node(
                package="prediction_ros",
                executable="prediction_node",
                name="prediction_node",
                output="screen",
                parameters=[
                    str(config),
                    {
                        "config_path": LaunchConfiguration("config_path"),
                        "prediction_profile": LaunchConfiguration("prediction_profile"),
                    },
                ],
            ),
        ]
    )
