from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    config = PathJoinSubstitution(
        [FindPackageShare("lr_prediction_bridge"), "config", "bridge.yaml"]
    )
    static_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("lr_prediction_bridge"),
                    "launch",
                    "prediction_static_bridge.launch.py",
                ]
            )
        ),
        launch_arguments={
            "enable_empty_objects": LaunchConfiguration("enable_empty_objects"),
            "enable_tracked_objects_adapter": LaunchConfiguration(
                "enable_tracked_objects_adapter"
            ),
            "allow_flat_fallback": LaunchConfiguration("allow_flat_fallback"),
            "start_upstream_stub": LaunchConfiguration("start_upstream_stub"),
        }.items(),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("enable_empty_objects", default_value="false"),
            DeclareLaunchArgument("enable_tracked_objects_adapter", default_value="true"),
            DeclareLaunchArgument("allow_flat_fallback", default_value="false"),
            DeclareLaunchArgument("start_upstream_stub", default_value="false"),
            static_include,
            Node(
                package="lr_prediction_bridge",
                executable="rover_state_adapter_node",
                name="rover_state_adapter_node",
                output="screen",
                parameters=[config],
            ),
        ]
    )
