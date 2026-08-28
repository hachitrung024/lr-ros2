# Copyright 2026 Landfill Rover Maintainers
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Integration test for MAVLink pose, extrinsic TF and point-cloud output."""

import math
import os
import signal
import struct
import subprocess
import time

from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PointStamped, QuaternionStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField


def _make_cloud(stamp):
    cloud = PointCloud2()
    cloud.header.stamp = stamp
    cloud.header.frame_id = 'test_camera'
    cloud.height = 1
    cloud.width = 1
    cloud.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    cloud.is_bigendian = False
    cloud.point_step = 12
    cloud.row_step = 12
    cloud.data = struct.pack('<fff', 1.0, 0.0, 0.0)
    cloud.is_dense = True
    return cloud


def _wait_until(node, predicate, timeout=8.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return True
    return False


def test_pose_and_extrinsic_transform_synthetic_cloud():
    """Confirm p_map = T_map_base * T_base_camera * p_camera."""
    executable = os.path.join(
        get_package_prefix('lr_pointcloud_transform'),
        'lib', 'lr_pointcloud_transform', 'pointcloud_transform_node',
    )
    command = [
        executable,
        '--ros-args',
        '-p', 'input_topic:=/test/mavlink_transform/cloud',
        '-p', 'output_topic:=/test/mavlink_transform/output',
        '-p', 'target_frame:=map',
        '-p', 'pose_source:=mavlink',
        '-p', 'mavlink_position_topic:=/test/mavlink_transform/position',
        '-p', 'mavlink_attitude_topic:=/test/mavlink_transform/attitude',
        '-p', 'mavlink_base_frame:=test_base',
        '-p', 'camera_link_frame:=test_camera',
        '-p', 'base_to_camera_x_m:=1.0',
        '-p', 'base_to_camera_y_m:=2.0',
        '-p', 'base_to_camera_z_m:=3.0',
        '-p', 'base_to_camera_yaw_deg:=90.0',
    ]
    process = subprocess.Popen(
        command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True
    )
    rclpy.init()
    node = Node('test_mavlink_transform_client')
    position_publisher = node.create_publisher(
        PointStamped, '/test/mavlink_transform/position', 10
    )
    attitude_publisher = node.create_publisher(
        QuaternionStamped, '/test/mavlink_transform/attitude', 10
    )
    cloud_publisher = node.create_publisher(
        PointCloud2, '/test/mavlink_transform/cloud', qos_profile_sensor_data
    )
    outputs = []
    node.create_subscription(
        PointCloud2,
        '/test/mavlink_transform/output',
        outputs.append,
        qos_profile_sensor_data,
    )

    try:
        connected = _wait_until(
            node,
            lambda: position_publisher.get_subscription_count() == 1
            and attitude_publisher.get_subscription_count() == 1
            and cloud_publisher.get_subscription_count() == 1,
        )
        assert connected

        stamp = node.get_clock().now().to_msg()
        position = PointStamped()
        position.header.stamp = stamp
        position.header.frame_id = 'map'
        position.point.x = 10.0
        position.point.y = 20.0
        position.point.z = 30.0

        attitude = QuaternionStamped()
        attitude.header.stamp = stamp
        attitude.header.frame_id = 'map'
        attitude.quaternion.z = math.sqrt(0.5)
        attitude.quaternion.w = math.sqrt(0.5)
        cloud = _make_cloud(stamp)

        for _ in range(10):
            position_publisher.publish(position)
            attitude_publisher.publish(attitude)
            rclpy.spin_once(node, timeout_sec=0.05)
            cloud_publisher.publish(cloud)
            if _wait_until(node, lambda: bool(outputs), timeout=0.2):
                break

        assert outputs
        output = outputs[-1]
        assert output.header.stamp == stamp
        assert output.header.frame_id == 'map'
        x_value, y_value, z_value = struct.unpack('<fff', output.data[:12])
        # Two +90 degree yaw rotations and both translations yield (7, 21, 33).
        assert math.isclose(x_value, 7.0, abs_tol=1e-5)
        assert math.isclose(y_value, 21.0, abs_tol=1e-5)
        assert math.isclose(z_value, 33.0, abs_tol=1e-5)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        process.send_signal(signal.SIGINT)
        try:
            process.communicate(timeout=5.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.communicate(timeout=5.0)
        assert process.returncode == 0
