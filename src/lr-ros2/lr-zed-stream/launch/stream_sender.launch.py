"""Publish SVO2 or a host-attached stereo camera over ZED Streaming NET."""

from dataclasses import fields

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from lr_zed_stream.source import SourceOptions


def generate_launch_description():
    """Keep source selection and encoder settings explicit and typed."""
    arguments = [
        DeclareLaunchArgument('camera_name', default_value='zed_source'),
        DeclareLaunchArgument(
            'enable_svo_controls', default_value='false', choices=['true', 'false'],
            description='Experimental: SDK 5.4.1 pause/seek NET has not passed acceptance tests.',
        ),
    ]
    parameters = {
        'use_sim_time': False,
        'enable_svo_controls': ParameterValue(
            LaunchConfiguration('enable_svo_controls'), value_type=bool
        ),
    }
    for field in fields(SourceOptions):
        value = field.default
        default = str(value).lower() if isinstance(value, bool) else str(value)
        arguments.append(DeclareLaunchArgument(field.name, default_value=default))
        parameters[field.name] = ParameterValue(
            LaunchConfiguration(field.name), value_type=type(value)
        )
    return LaunchDescription([
        *arguments,
        Node(
            package='lr_zed_stream', executable='stream_sender', name='zed_node',
            namespace=LaunchConfiguration('camera_name'), output='screen',
            parameters=[parameters],
        ),
    ])
