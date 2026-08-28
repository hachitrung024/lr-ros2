import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz')
    start_terrain = LaunchConfiguration('start_terrain')
    terrain_params_file = LaunchConfiguration('terrain_params_file')

    sim_config = os.path.join(
        get_package_share_directory('rover_sim_bringup'),
        'config',
        'sim.yaml',
    )
    terrain_default = os.path.join(
        get_package_share_directory('lr_terrain_geometry'),
        'config',
        'terrain_geometry.yaml',
    )
    display_launch = os.path.join(
        get_package_share_directory('lr_display_rviz2'),
        'launch',
        'display_zed_cam.launch.py',
    )

    bridge_nodes = [
        Node(
            package='rover_sim_bringup',
            executable='cmd_bridge',
            name='cmd_vel_bridge',
            parameters=[sim_config, {'use_sim_time': True}],
            output='screen',
        ),
        Node(
            package='rover_sim_bringup',
            executable='state_bridge',
            name='rover_state_bridge',
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
        Node(
            package='rover_sim_bringup',
            executable='contact_bridge',
            name='contact_wrench_bridge',
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
        Node(
            package='rover_sim_bringup',
            executable='camera_bridge',
            name='zed2i_bridge',
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
        Node(
            package='rover_sim_bringup',
            executable='rollover_monitor',
            name='rollover_monitor',
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
    ]

    depth_cloud = Node(
        package='depth_image_proc',
        executable='point_cloud_xyz_node',
        name='sim_depth_to_pointcloud',
        parameters=[{'use_sim_time': True}],
        remappings=[
            ('image_rect', '/zed2i/depth/image_raw'),
            ('camera_info', '/zed2i/depth/camera_info'),
            ('points', '/zed2i/point_cloud'),
        ],
        output='screen',
    )

    map_cloud = Node(
        package='lr_pointcloud_transform',
        executable='pointcloud_transform_node',
        name='sim_pointcloud_transform',
        parameters=[{
            'use_sim_time': True,
            'input_topic': '/zed2i/point_cloud',
            'output_topic': '/lr/point_cloud/cloud_in_map',
            'target_frame': 'map',
            'transform_timeout_sec': 0.5,
            'pose_source': 'tf',
            'accumulate_cloud': True,
            'frame_step': 3,
            'point_stride': 8,
            'max_map_points': 500000,
        }],
        output='screen',
    )

    terrain = Node(
        package='lr_terrain_geometry',
        executable='terrain_geometry_node',
        name='terrain_geometry',
        parameters=[
            terrain_params_file,
            {
                'use_sim_time': True,
                'input.point_cloud_topic': '/lr/point_cloud/cloud_in_map',
                'frames.map_frame': 'map',
            },
        ],
        condition=IfCondition(start_terrain),
        output='screen',
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(display_launch),
        launch_arguments={
            'camera_model': 'zed2i',
            'camera_name': 'zed2i',
            'start_zed_node': 'false',
            'publish_svo_clock': 'true',
            'pointcloud_topic': '/lr/point_cloud/cloud_in_map',
            'rgb_topic': '/zed2i/left/image_raw',
        }.items(),
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            choices=['true', 'false'],
        ),
        DeclareLaunchArgument(
            'start_terrain',
            default_value='true',
            choices=['true', 'false'],
        ),
        DeclareLaunchArgument(
            'terrain_params_file',
            default_value=terrain_default,
        ),
        *bridge_nodes,
        depth_cloud,
        map_cloud,
        terrain,
        rviz,
    ])
