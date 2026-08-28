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

"""ROS node publishing exact-time local ENU poses from MAVLink CSV logs."""

from __future__ import annotations

import time

from geometry_msgs.msg import Point, PointStamped, PoseStamped, QuaternionStamped
from nav_msgs.msg import Path
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from .csv_pose import load_pose_log


class MavlinkCsvPoseNode(Node):
    """Interpolate a recorded MAVLink base pose at each point-cloud timestamp."""

    def __init__(self) -> None:
        super().__init__('mavlink_csv_pose_node')
        gps_path = self.declare_parameter('gps_path', '').value
        attitude_path = self.declare_parameter('attitude_path', '').value
        input_topic = self.declare_parameter(
            'input_topic', '/zed/zed_node/point_cloud/cloud_registered'
        ).value
        position_topic = self.declare_parameter(
            'position_topic', '/lr/mavlink/position_enu'
        ).value
        attitude_topic = self.declare_parameter(
            'attitude_topic', '/lr/mavlink/attitude_enu'
        ).value
        future_path_topic = self.declare_parameter(
            'future_path_topic', '/lr/mavlink/trajectory_future'
        ).value
        trajectory_topic = self.declare_parameter(
            'trajectory_topic', '/lr/mavlink/trajectory'
        ).value
        frame_axes_topic = self.declare_parameter(
            'frame_axes_topic', '/lr/mavlink/frame_axes'
        ).value
        self._axis_length_m = float(self.declare_parameter(
            'axis_length_m', 0.5
        ).value)
        self._axis_line_width_m = float(self.declare_parameter(
            'axis_line_width_m', 0.035
        ).value)
        if self._axis_length_m <= 0.0 or self._axis_line_width_m <= 0.0:
            raise ValueError('axis length and line width must be positive')
        self._future_path_radius_m = float(self.declare_parameter(
            'future_path_radius_m', 10.0
        ).value)
        self._future_path_step_m = float(self.declare_parameter(
            'future_path_step_m', 0.2
        ).value)
        self._future_path_frames = int(self.declare_parameter(
            'future_path_frames', 200
        ).value)
        self._svo_fps = float(self.declare_parameter(
            'svo_fps', 15.0
        ).value)
        if self._future_path_frames < 0 or self._svo_fps <= 0.0:
            raise ValueError('future_path_frames must be >= 0 and svo_fps > 0')
        self._world_frame = self.declare_parameter(
            'world_frame', 'map'
        ).value
        origin_samples = self.declare_parameter('origin_samples', 20).value
        minimum_fix_type = self.declare_parameter('minimum_fix_type', 3).value
        maximum_gps_gap_s = self.declare_parameter(
            'maximum_gps_gap_s', 2.0
        ).value
        maximum_attitude_gap_s = self.declare_parameter(
            'maximum_attitude_gap_s', 0.5
        ).value

        self._pose_log = load_pose_log(
            gps_path,
            attitude_path,
            origin_samples=int(origin_samples),
            minimum_fix_type=int(minimum_fix_type),
            maximum_gps_gap_s=float(maximum_gps_gap_s),
            maximum_attitude_gap_s=float(maximum_attitude_gap_s),
        )
        self._pose_log.sample_future_within_radius(
            self._pose_log.gps_time[0],
            self._future_path_radius_m,
            self._future_path_step_m,
        )
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=30,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._position_publisher = self.create_publisher(
            PointStamped, position_topic, qos
        )
        self._attitude_publisher = self.create_publisher(
            QuaternionStamped, attitude_topic, qos
        )
        self._future_path_publisher = self.create_publisher(
            Path, future_path_topic, qos
        )
        path_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._trajectory_publisher = self.create_publisher(
            Path, trajectory_topic, path_qos
        )
        self._trajectory = Path()
        self._trajectory.header.frame_id = self._world_frame
        self._frame_axes_publisher = self.create_publisher(
            MarkerArray, frame_axes_topic, qos
        )
        self._frame_index = 0
        self._cloud_subscription = self.create_subscription(
            PointCloud2,
            input_topic,
            self._pointcloud_callback,
            qos_profile_sensor_data,
        )
        self._last_warning_time: dict[str, float] = {}

        origin = self._pose_log.origin
        self.get_logger().info(
            'MAVLink CSV pose replay: input=%s position=%s attitude=%s '
            'future_path=%s (%d frames ahead at %.3f FPS) '
            'ENU origin=(%.9f, %.9f, %.3f m)'
            % (
                input_topic,
                position_topic,
                attitude_topic,
                future_path_topic,
                self._future_path_frames,
                self._svo_fps,
                origin['latitude_deg'],
                origin['longitude_deg'],
                origin['altitude_m'],
            )
        )

    def _warn_throttled(self, reason: str, timestamp_s: float) -> None:
        now = time.monotonic()
        if now - self._last_warning_time.get(reason, float('-inf')) < 5.0:
            return
        self._last_warning_time[reason] = now
        self.get_logger().warning(
            f'No MAVLink pose at {timestamp_s:.6f}: {reason}; dropping cloud pose'
        )

    def _pointcloud_callback(self, cloud: PointCloud2) -> None:
        timestamp_s = (
            float(cloud.header.stamp.sec)
            + float(cloud.header.stamp.nanosec) * 1e-9
        )
        sample = self._pose_log.sample_at(timestamp_s)
        if sample is None:
            self._warn_throttled(
                self._pose_log.unavailable_reason(timestamp_s) or 'unknown',
                timestamp_s,
            )
            return

        position = PointStamped()
        position.header.stamp = cloud.header.stamp
        position.header.frame_id = self._world_frame
        position.point.x = float(sample.position_enu_m[0])
        position.point.y = float(sample.position_enu_m[1])
        position.point.z = float(sample.position_enu_m[2])

        attitude = QuaternionStamped()
        attitude.header.stamp = cloud.header.stamp
        attitude.header.frame_id = self._world_frame
        attitude.quaternion.x = float(sample.quaternion_enu_flu_xyzw[0])
        attitude.quaternion.y = float(sample.quaternion_enu_flu_xyzw[1])
        attitude.quaternion.z = float(sample.quaternion_enu_flu_xyzw[2])
        attitude.quaternion.w = float(sample.quaternion_enu_flu_xyzw[3])

        self._position_publisher.publish(position)
        self._attitude_publisher.publish(attitude)
        trajectory_pose = PoseStamped()
        trajectory_pose.header = position.header
        trajectory_pose.pose.position.x = position.point.x
        trajectory_pose.pose.position.y = position.point.y
        trajectory_pose.pose.position.z = position.point.z
        trajectory_pose.pose.orientation = attitude.quaternion
        self._trajectory.header.stamp = cloud.header.stamp
        self._trajectory.poses.append(trajectory_pose)
        self._trajectory_publisher.publish(self._trajectory)
        self._publish_frame_axes(position, attitude)
        self._future_path_publisher.publish(
            self._build_future_path(cloud, timestamp_s)
        )

    def _publish_frame_axes(
        self,
        position: PointStamped,
        attitude: QuaternionStamped,
    ) -> None:
        """Publish persistent RGB FLU axes for the current SVO frame."""
        q = attitude.quaternion
        x, y, z, w = q.x, q.y, q.z, q.w
        rotation = (
            (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w),
             2.0 * (x * z + y * w)),
            (2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z),
             2.0 * (y * z - x * w)),
            (2.0 * (x * z - y * w), 2.0 * (y * z + x * w),
             1.0 - 2.0 * (x * x + y * y)),
        )
        origin = position.point
        marker = Marker()
        marker.header = position.header
        marker.ns = 'mavlink_frame_axes'
        marker.id = self._frame_index
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = self._axis_line_width_m
        colors = (
            ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0),
            ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0),
            ColorRGBA(r=0.0, g=0.35, b=1.0, a=1.0),
        )
        for axis, color in enumerate(colors):
            marker.points.append(Point(x=origin.x, y=origin.y, z=origin.z))
            marker.points.append(Point(
                x=origin.x + rotation[0][axis] * self._axis_length_m,
                y=origin.y + rotation[1][axis] * self._axis_length_m,
                z=origin.z + rotation[2][axis] * self._axis_length_m,
            ))
            marker.colors.extend((color, color))
        self._frame_axes_publisher.publish(MarkerArray(markers=[marker]))
        self._frame_index += 1

    def _build_future_path(
        self,
        cloud: PointCloud2,
        timestamp_s: float,
    ) -> Path:
        path = Path()
        path.header.stamp = cloud.header.stamp
        path.header.frame_id = self._world_frame
        samples = []
        for frame_offset in range(self._future_path_frames + 1):
            sample_time = timestamp_s + frame_offset / self._svo_fps
            sample = self._pose_log.sample_at(sample_time)
            if sample is None:
                break
            samples.append(sample)
        for index, sample in enumerate(samples):
            pose = PoseStamped()
            pose.header.frame_id = self._world_frame
            if index == 0:
                pose.header.stamp = cloud.header.stamp
            else:
                total_nanoseconds = int(round(sample.timestamp_s * 1e9))
                pose.header.stamp.sec = total_nanoseconds // 1_000_000_000
                pose.header.stamp.nanosec = (
                    total_nanoseconds % 1_000_000_000
                )
            pose.pose.position.x = float(sample.position_enu_m[0])
            pose.pose.position.y = float(sample.position_enu_m[1])
            pose.pose.position.z = float(sample.position_enu_m[2])
            pose.pose.orientation.x = float(sample.quaternion_enu_flu_xyzw[0])
            pose.pose.orientation.y = float(sample.quaternion_enu_flu_xyzw[1])
            pose.pose.orientation.z = float(sample.quaternion_enu_flu_xyzw[2])
            pose.pose.orientation.w = float(sample.quaternion_enu_flu_xyzw[3])
            path.poses.append(pose)
        return path


def main(args=None) -> None:
    """Run the MAVLink CSV pose replay node."""
    rclpy.init(args=args)
    node = None
    try:
        node = MavlinkCsvPoseNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    except (FileNotFoundError, TypeError, ValueError) as exception:
        if node is not None:
            node.get_logger().fatal(str(exception))
        else:
            print(f'Failed to start mavlink_csv_pose_node: {exception}')
        raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
