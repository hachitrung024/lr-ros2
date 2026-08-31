"""Run a headless ZED SVO pass and create a future-path pose cache."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

from lr_future_path.cache import (
    build_cache_spec,
    cache_is_valid,
)


def _value(context, name):
    return LaunchConfiguration(name).perform(context)


def _shutdown_after_recorder_exit(_event, context):
    if context.is_shutdown:
        return []
    return [
        EmitEvent(
            event=Shutdown(reason="SVO future-path cache pass finished")
        )
    ]


def _launch_setup(context):
    svo_path = _value(context, "svo_path")
    camera_model = _value(context, "camera_model")
    camera_name = _value(context, "camera_name") or "zed"
    cache_path = _value(context, "cache_path")
    rebuild = _value(context, "rebuild").lower() == "true"

    if svo_path == "live":
        raise RuntimeError("SVO pose cache cannot be built in live mode")
    if not cache_path:
        raise RuntimeError("cache_path is required")
    spec = build_cache_spec(svo_path, camera_model, os.path.dirname(cache_path))
    if os.path.realpath(cache_path) != os.path.realpath(spec.path):
        raise RuntimeError(
            f"cache_path fingerprint mismatch: expected {spec.path}"
        )
    if cache_is_valid(spec.path, spec.identity) and not rebuild:
        return [
            LogInfo(
                msg=TextSubstitution(
                    text=f"Future-path cache already valid: {spec.path}"
                )
            )
        ]

    pose_topic = f"/{camera_name}/zed_node/pose"
    status_topic = f"/{camera_name}/zed_node/status/svo"
    recorder = Node(
        package="lr_future_path",
        executable="svo_pose_cache_node",
        name="svo_pose_cache",
        output="screen",
        parameters=[
            {
                "svo_path": svo_path,
                "camera_model": camera_model,
                "cache_path": str(spec.path),
                "rebuild": rebuild,
                "pose_topic": pose_topic,
                "status_topic": status_topic,
                "use_sim_time": False,
            }
        ],
    )
    shutdown_when_recorder_exits = RegisterEventHandler(
        OnProcessExit(
            target_action=recorder,
            on_exit=_shutdown_after_recorder_exit,
        )
    )

    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("zed_wrapper"),
                "launch",
                "zed_camera.launch.py",
            )
        ),
        launch_arguments={
            "camera_name": camera_name,
            "camera_model": camera_model,
            "svo_path": svo_path,
            "publish_svo_clock": "true",
            "use_sim_time": "false",
            "publish_urdf": "true",
            "publish_tf": "false",
            "publish_map_tf": "false",
            "enable_ipc": "false",
            "node_log_type": "screen",
            "param_overrides": (
                "svo.svo_realtime:=false;"
                "svo.svo_loop:=false;"
                "svo.play_from_frame:=0;"
                "svo.use_svo_timestamps:=true;"
                "pos_tracking.pos_tracking_enabled:=true;"
                "pos_tracking.publish_odom_pose:=true"
            ),
        }.items(),
    )
    return [
        shutdown_when_recorder_exits,
        recorder,
        TimerAction(period=1.0, actions=[zed_launch]),
    ]


def generate_launch_description():
    """Declare cache-pass inputs and defer launch construction."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "camera_name",
                default_value="zed",
                description="ZED namespace used for pose and SVO status.",
            ),
            DeclareLaunchArgument(
                "camera_model",
                description="ZED camera model used to open the SVO.",
            ),
            DeclareLaunchArgument(
                "svo_path",
                description="Absolute path to the source SVO/SVO2 file.",
            ),
            DeclareLaunchArgument(
                "cache_path",
                description="Resolved rosbag2 cache directory.",
            ),
            DeclareLaunchArgument(
                "rebuild",
                default_value="false",
                choices=["true", "false"],
                description="Replace an existing cache after a successful pass.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
