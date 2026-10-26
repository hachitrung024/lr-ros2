"""Minimal telemetry gateway; source /clock is the only clock producer."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mavros = Node(
        package="mavros",
        executable="mavros_node",
        output="screen",
        parameters=[
            get_package_share_directory("lr_mavros_bridge") + "/config/replay.yaml",
            {
                "fcu_url": LaunchConfiguration("fcu_url"),
                "gcs_url": "",
                "fcu_protocol": "v2.0",
                "tgt_system": 1,
                "tgt_component": 1,
            },
        ],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("fcu_url", description="MAVROS local/remote transport URL."),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=mavros,
                    on_exit=[EmitEvent(event=Shutdown(reason="MAVROS gateway exited"))],
                )
            ),
            mavros,
        ]
    )
