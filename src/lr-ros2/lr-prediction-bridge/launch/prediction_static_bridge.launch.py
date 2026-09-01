from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    config = Path(
        get_package_share_directory("lr_prediction_bridge")
    ) / "config" / "bridge.yaml"
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enable_empty_objects",
                default_value="false",
                description="SMOKE ONLY empty TrackedObjectArray bridge",
            ),
            DeclareLaunchArgument(
                "enable_tracked_objects_adapter",
                default_value="true",
                description="Convert terrain Detection3DArray to /tracked_objects",
            ),
            DeclareLaunchArgument(
                "allow_flat_fallback",
                default_value="false",
                description="If true, missing GridMap normals become (0,0,1)",
            ),
            DeclareLaunchArgument(
                "start_upstream_stub",
                default_value="false",
                description="Synthetic Path/pose/GridMap publisher for adapter smoke",
            ),
            Node(
                package="lr_prediction_bridge",
                executable="trajectory_adapter_node",
                name="trajectory_adapter_node",
                output="screen",
                parameters=[str(config)],
            ),
            Node(
                package="lr_prediction_bridge",
                executable="geometry_adapter_node",
                name="geometry_adapter_node",
                output="screen",
                parameters=[
                    str(config),
                    {"allow_flat_fallback": LaunchConfiguration("allow_flat_fallback")}
                ],
            ),
            Node(
                package="lr_prediction_bridge",
                executable="tracked_objects_adapter_node",
                name="tracked_objects_adapter_node",
                output="screen",
                parameters=[str(config)],
                condition=IfCondition(LaunchConfiguration("enable_tracked_objects_adapter")),
            ),
            Node(
                package="lr_prediction_bridge",
                executable="empty_objects_bridge_node",
                name="empty_objects_bridge_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_empty_objects")),
                parameters=[
                    str(config),
                    {
                        "enable_empty_objects": LaunchConfiguration(
                            "enable_empty_objects"
                        )
                    }
                ],
            ),
            Node(
                package="lr_prediction_bridge",
                executable="upstream_contract_stub",
                name="upstream_contract_stub",
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_upstream_stub")),
            ),
        ]
    )
