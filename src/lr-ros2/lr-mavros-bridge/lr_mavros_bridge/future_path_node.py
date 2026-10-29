"""Receive LRFP snapshots from MAVROS and publish a session-scoped ENU Path."""

import json
import math
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geographic_msgs.msg import GeoPointStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String, UInt8MultiArray

from lr_future_path.mavlink import PoseSample, apply_body_to_camera, geodetic_to_enu
from lr_future_path.path_processing import StablePathProcessor
from lr_future_path.time_utils import nanoseconds_to_stamp

from .path_protocol import Assembler
from .source_guard import SourceGuard


class MavlinkFuturePath(Node):
    """No DB, clock or TF ownership; retain the backend's first-GPS origin."""

    def __init__(self, **kwargs):
        super().__init__('mavlink_future_path', **kwargs)
        self.frame = self.declare_parameter('map_frame', 'map').value
        self.mount = np.asarray(self.declare_parameter(
            'body_to_camera', [0., 0., 0., 0., 0., 0.]).value, dtype=float)
        self.radius = float(self.declare_parameter('radius_m', 15.).value)
        self.step = float(self.declare_parameter('step_m', .2).value)
        self.stale = float(self.declare_parameter('stale_after_sec', 1.).value)
        if not self.frame.strip() or self.mount.shape != (6,) or not np.isfinite(self.mount).all():
            raise ValueError('Invalid frame/extrinsics')
        if not all(math.isfinite(x) and x > 0 for x in (self.radius, self.step, self.stale)):
            raise ValueError('Path radius, step and stale interval must be positive')
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.output = self.create_publisher(Path, '/lr/future_path/mavlink', latched)
        self.diagnostics = self.create_publisher(DiagnosticArray, '/lr/future_path/diagnostics', 10)
        self.origin, self.state, self.state_seen = None, {}, 0.
        self.assembler, self.processor = Assembler(), StablePathProcessor()
        self.last_seen, self.last_stamp, self.last_offset, self.published = 0., 0, 0, False
        self.accepted, self.rejected = 0, 0
        self.status = 'waiting_source'
        # Guard callbacks only mark a pending reset. Processing stays in the
        # default mutually-exclusive callback group; publication is fenced again.
        self.generation = None
        self.guard = SourceGuard(self, lambda _: None)
        self.create_subscription(String, '/lr/source/time_state', self.on_time, latched)
        self.create_subscription(GeoPointStamped, '/lr/mavlink/origin', self.on_origin, latched)
        self.create_subscription(UInt8MultiArray, '/lr/mavlink/path_fragment', self.on_fragment, 256)
        self.create_timer(.1, self.expire)
        self.create_timer(1., self.report)

    def clear(self, reset_transport=False):
        if reset_transport:
            self.assembler.reset()
        self.processor.reset()
        self.last_seen, self.last_stamp = 0., 0
        if self.published:
            path = Path()
            path.header.frame_id = self.frame
            path.header.stamp = self.get_clock().now().to_msg()
            self.output.publish(path)
        self.published = False

    def on_time(self, message):
        try:
            state = json.loads(message.data)
            if state['version'] != 1 or not isinstance(state['offset_ns'], int):
                return
            self.state, self.state_seen = state, time.monotonic()
        except (ValueError, TypeError, KeyError):
            return

    def on_origin(self, message):
        lla = [message.position.latitude, message.position.longitude, message.position.altitude]
        if self.origin is None and message.header.frame_id == self.frame and np.isfinite(lla).all():
            self.origin = np.asarray(lla)

    def expire(self):
        generation = self.guard.token()
        if generation != self.generation:
            self.clear(reset_transport=True)
            self.generation = generation
        now = self.get_clock().now().nanoseconds
        active = (self.guard.ready and time.monotonic() - self.guard.seen < 2.
                  and self.state.get('sync_ready', False)
                  and time.monotonic() - self.state_seen < 2.)
        if not active or (self.published and (
                time.monotonic() - self.last_seen > self.stale or
                now - self.last_stamp > self.stale * 1e9)):
            self.clear()
            self.status = 'waiting_source' if not active else 'waiting_path'

    def on_fragment(self, message):
        self.expire()
        session = self.state.get('session_id', '')
        if not self.guard.ready or not self.state.get('sync_ready') or not session or (
                session != self.guard.session_id):
            return
        if time.monotonic() - self.state_seen > 2. or self.origin is None:
            self.status = 'waiting_origin_or_sync'
            return
        token = self.guard.token()
        try:
            snapshot = self.assembler.accept(message.data, session)
            if snapshot is None:
                return
            offset = int(self.state['offset_ns'])
            stamp_ns = snapshot.stamp_ns + offset
            age = self.get_clock().now().nanoseconds - stamp_ns
            # Validate before constructing builtin Time (signed 32-bit seconds).
            if not -500_000_000 <= age <= self.stale * 1e9:
                raise ValueError('Stale path reference')
            path = Path()
            path.header.frame_id = self.frame
            nanoseconds_to_stamp(stamp_ns, path.header.stamp)
            if not self.guard.valid(path.header.stamp, token):
                raise ValueError('Stale path or retired source')
            if stamp_ns <= self.last_stamp:
                raise ValueError('Non-increasing path reference')
            if not snapshot.points:
                # Keep the assembler sequence watermark when invalidating output.
                self.processor.reset()
                self.output.publish(path)
                self.published, self.status = False, 'empty_path'
                self.last_stamp = stamp_ns
                return
            lla = np.asarray([[p.lat_e7 / 1e7, p.lon_e7 / 1e7, p.alt_msl_mm / 1000.]
                              for p in snapshot.points])
            positions = geodetic_to_enu(lla, self.origin)
            samples = [apply_body_to_camera(PoseSample(
                p.stamp_ns + offset, position, np.asarray(p.quaternion)), self.mount)
                for p, position in zip(snapshot.points, positions)]
            stamps, xyz, quaternions = self.processor.process(
                [s.stamp_ns for s in samples], [s.position for s in samples],
                [s.orientation_xyzw for s in samples], radius_m=self.radius,
                step_m=self.step, max_points=128, smoothing_window_m=.8, max_deviation_m=.2)
            for stamp, position, quaternion in zip(stamps, xyz, quaternions):
                pose = PoseStamped()
                pose.header.frame_id = self.frame
                nanoseconds_to_stamp(int(stamp), pose.header.stamp)
                pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, position)
                (pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z,
                 pose.pose.orientation.w) = map(float, quaternion)
                path.poses.append(pose)
            if not self.guard.valid(path.header.stamp, token):
                return
            self.output.publish(path)
            self.last_seen, self.last_stamp, self.last_offset = time.monotonic(), stamp_ns, offset
            self.published = len(path.poses) >= 2
            self.status = 'ready' if self.published else 'empty_path'
            self.accepted += 1
        except (ValueError, TypeError, KeyError, OverflowError) as error:
            self.rejected += 1
            self.status = str(error)

    def report(self):
        message = DiagnosticArray(status=[DiagnosticStatus(
            name=self.get_name(), level=DiagnosticStatus.OK if self.status == 'ready' else DiagnosticStatus.WARN,
            message=self.status, values=[KeyValue(key=k, value=str(v)) for k, v in {
                'snapshots': self.accepted, 'rejected': self.rejected,
                'session_id': self.state.get('session_id', ''),
                'offset_ns': self.last_offset,
            }.items()])])
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame
        self.diagnostics.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = MavlinkFuturePath()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
