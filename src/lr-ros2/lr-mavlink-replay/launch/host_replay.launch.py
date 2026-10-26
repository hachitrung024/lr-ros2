"""One source terminal: SVO NET plus causal UDP telemetry."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    EmitEvent,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    telemetry = Node(
        package="lr_mavlink_replay",
        executable="telemetry_replay",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "db_path": LaunchConfiguration("mavlink_db_path"),
                "mavlink_dir": LaunchConfiguration("mavlink_dir"),
                "bind_address": LaunchConfiguration("mavlink_bind_address"),
                "bind_port": ParameterValue(
                    LaunchConfiguration("mavlink_bind_port"), value_type=int
                ),
                "destination": LaunchConfiguration("mavlink_destination"),
                "port": ParameterValue(
                    LaunchConfiguration("mavlink_destination_port"), value_type=int
                ),
            }
        ],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("svo_path"),
            DeclareLaunchArgument("stream_port", description="ZED NET UDP port (even)."),
            DeclareLaunchArgument("mavlink_db_path", default_value=""),
            DeclareLaunchArgument("mavlink_dir", default_value="/zed/lr-ros2/mavlink"),
            DeclareLaunchArgument("mavlink_bind_address", description="Host UDP bind address."),
            DeclareLaunchArgument("mavlink_bind_port", description="Host UDP bind port."),
            DeclareLaunchArgument("mavlink_destination", description="Jetson UDP address."),
            DeclareLaunchArgument("mavlink_destination_port", description="Jetson UDP port."),
            DeclareLaunchArgument("play_from_frame", default_value="0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    get_package_share_directory("lr_zed_stream")
                    + "/launch/stream_sender.launch.py"
                ),
                launch_arguments={
                    "svo_path": LaunchConfiguration("svo_path"),
                    "port": LaunchConfiguration("stream_port"),
                    "play_from_frame": LaunchConfiguration("play_from_frame"),
                }.items(),
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=telemetry,
                    on_exit=[EmitEvent(event=Shutdown(reason="MAVLink replay exited"))],
                )
            ),
            telemetry,
        ]
    )
