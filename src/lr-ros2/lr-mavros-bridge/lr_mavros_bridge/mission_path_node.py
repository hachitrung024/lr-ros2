"""Read complete FCU missions and publish a fresh local route for prediction."""

from copy import deepcopy
import ctypes
import json
import math
from pathlib import Path as FilePath
import time

from ament_index_python.packages import get_package_prefix
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geographic_msgs.msg import GeoPointStamped
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import HomePosition, WaypointList
from mavros_msgs.srv import WaypointPull
from nav_msgs.msg import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from lr_future_path.mavlink import PoseSample
from lr_future_path.time_utils import nanoseconds_to_stamp
from .mission import mission_path
from .source_guard import SourceGuard


class MissionPath(Node):
    def __init__(self, **kwargs):
        super().__init__('mission_path', **kwargs)
        self.frame = self.declare_parameter('map_frame', 'map').value
        self.mount = np.asarray(self.declare_parameter('body_to_camera', [0.] * 6).value)
        self.system = int(self.declare_parameter('target_system_id', 1).value)
        self.component = int(self.declare_parameter('target_component_id', 1).value)
        if not self.frame or self.mount.shape != (6,) or not np.isfinite(self.mount).all():
            raise ValueError('Invalid map frame or extrinsics')
        geoid = ctypes.CDLL(str(FilePath(get_package_prefix('lr_mavros_bridge')) /
                                'lib/liblr_geoid.so'))
        self.geoid = geoid.lr_geoid_height
        self.geoid.argtypes = [ctypes.c_double, ctypes.c_double]
        self.geoid.restype = ctypes.c_double
        if not math.isfinite(self.geoid(0., 0.)):
            raise RuntimeError('GeographicLib egm96-5 is required')
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.output = self.create_publisher(Path, '/lr/future_path/mission', qos)
        self.diag = self.create_publisher(DiagnosticArray, '/lr/future_path/diagnostics', 10)
        self.origin = self.home = self.pose = self.snapshot = None
        self.state, self.state_seen, self.epoch = {}, 0., None
        self.snapshot_seen = self.pose_seen = 0.
        self.candidate, self.candidate_seen = None, 0.
        self.pull_future = self.home_future = None
        self.home_requested = 0.
        self.pull_started = self.next_pull = 0.
        self.completed_count = None
        self.last_seq = self.last_stamp = None
        self.published = False
        self.path_count = 0
        self.report_started = time.monotonic()
        self.status = 'waiting_mavlink'
        self.guard = SourceGuard(self, self.clear)
        self.pull = self.create_client(WaypointPull, '/mavros/mission/pull')
        self.home_pull = self.create_client(Trigger, '/mavros/home_position/req_update')
        self.create_subscription(WaypointList, '/mavros/mission/waypoints', self.on_list, qos)
        self.create_subscription(HomePosition, '/mavros/home_position/home', self.on_home, qos)
        self.create_subscription(GeoPointStamped, '/lr/mavlink/origin', self.on_origin, qos)
        self.create_subscription(PoseStamped, '/lr/mavlink/pose', self.on_pose, 10)
        self.create_subscription(String, '/lr/source/time_state', self.on_state, qos)
        self.create_timer(.25, self.tick)
        self.create_timer(1., self.report)

    def clear(self, _reason=None):
        # All callbacks are serialized; no computation can publish across reset.
        self.pose = None
        self.last_stamp = None
        self.invalidate()

    def invalidate(self):
        if self.published:
            output = Path()
            output.header.frame_id = self.frame
            output.header.stamp = self.get_clock().now().to_msg()
            self.output.publish(output)
        self.published = False

    def reset_transport(self):
        if self.home_future is not None:
            self.home_pull.remove_pending_request(self.home_future)
        self.home_future = None
        self.home_requested = 0.
        if self.pull_future is not None:
            self.pull.remove_pending_request(self.pull_future)
        self.pull_future = None
        self.completed_count = None
        self.snapshot = self.candidate = self.home = None
        self.snapshot_seen = self.candidate_seen = self.next_pull = 0.
        self.last_seq = None
        self.clear()

    def on_state(self, message):
        try:
            state = json.loads(message.data)
            if state.get('version') != 1:
                return
            epoch = state.get('telemetry_epoch', 0)
            changed_plan = state.get('mission_id', 0) != self.state.get('mission_id', 0)
            if epoch != self.epoch or changed_plan or (
                    self.state.get('connected') and not state['connected']):
                self.reset_transport()
            self.epoch, self.state, self.state_seen = epoch, state, time.monotonic()
        except (ValueError, KeyError, TypeError):
            return

    def on_origin(self, message):
        values = [message.position.latitude, message.position.longitude, message.position.altitude]
        if self.origin is None and message.header.frame_id == self.frame and np.isfinite(values).all():
            self.origin = values

    def on_home(self, message):
        if not self.state.get('connected') or time.monotonic() - self.state_seen >= 2.:
            return
        p = message.geo
        if all(math.isfinite(v) for v in (p.latitude, p.longitude, p.altitude)):
            altitude = p.altitude - self.geoid(p.latitude, p.longitude)
            if self.home is not None and abs(self.home - altitude) > .01:
                self.invalidate()
            self.home = altitude

    def on_pose(self, message):
        if message.header.frame_id == self.frame and self.guard.valid(
                message.header.stamp, self.guard.token()):
            self.pose, self.pose_seen = message, time.monotonic()

    def on_list(self, message):
        # WaypointList is also published on progress changes. Such a message
        # must never renew the lifetime of the last successful download.
        self.candidate, self.candidate_seen = deepcopy(message), time.monotonic()
        if self.snapshot is not None and self.snapshot != message.waypoints:
            self.invalidate()
            self.snapshot = None
            self.next_pull = 0.

    def download(self, now):
        if self.pull_future is not None:
            if self.pull_future.done() and self.completed_count is None:
                try:
                    result = self.pull_future.result()
                    self.completed_count = result.wp_received if result.success else -1
                except Exception:
                    self.completed_count = -1
            if self.completed_count is not None:
                if self.completed_count >= 0 and self.candidate_seen >= self.pull_started and (
                        self.candidate is not None and
                        len(self.candidate.waypoints) == self.completed_count):
                    if self.snapshot != self.candidate.waypoints:
                        self.invalidate()
                    self.snapshot = deepcopy(self.candidate.waypoints)
                    self.snapshot_seen = now
                    self.pull_future = None
                    self.completed_count = None
                    self.next_pull = now + 10.
                elif self.completed_count < 0:
                    self.pull_future = None
                    self.completed_count = None
                    self.next_pull = now + 2.
            if self.pull_future is not None and now - self.pull_started > 20.:
                self.pull.remove_pending_request(self.pull_future)
                self.pull_future = None
                self.completed_count = None
                self.next_pull = now + 2.
        if self.pull_future is None and now >= self.next_pull and self.pull.service_is_ready():
            self.candidate = None
            self.pull_started = now
            self.pull_future = self.pull.call_async(WaypointPull.Request())

    def tick(self):
        now = time.monotonic()
        if not self.state.get('connected') or now - self.state_seen >= 2.:
            self.status = 'waiting_mavlink'
            if self.snapshot is not None or self.pull_future is not None:
                self.reset_transport()
            self.invalidate()
            return
        self.download(now)
        if self.home is None and self.snapshot and any(w.frame in (3, 6) for w in self.snapshot):
            if self.home_future is not None and (
                    self.home_future.done() or now-self.home_requested > 5.):
                self.home_pull.remove_pending_request(self.home_future)
                self.home_future = None
            if self.home_future is None and now-self.home_requested > 2. and (
                    self.home_pull.service_is_ready()):
                self.home_future = self.home_pull.call_async(Trigger.Request())
                self.home_requested = now
        try:
            if self.snapshot is None or now - self.snapshot_seen >= 30.:
                raise ValueError('waiting_complete_mission')
            if not 0 <= self.state.get('mission_age_sec', -1.) < 3. or (
                    self.state.get('mission_system_id') != self.system or
                    self.state.get('mission_component_id') != self.component):
                raise ValueError('waiting_mission_progress')
            # MAVLink 1 and FCUs without extension support report zero. A
            # known inactive state must not keep the last route alive.
            if self.state.get('mission_total', 0) == 65535:
                raise ValueError('no_mission')
            if self.state.get('mission_state', 0) not in (0, 3):
                raise ValueError('mission_inactive')
            if self.state.get('mission_mode', 0) not in (0, 1):
                raise ValueError('mission_suspended')
            seq = int(self.state['mission_seq'])
            if seq != self.last_seq:
                self.invalidate()
                self.last_seq = seq
            if self.origin is None or self.pose is None or now - self.pose_seen >= .75:
                raise ValueError('waiting_pose_or_origin')
            token = self.guard.token()
            if not self.guard.valid(self.pose.header.stamp, token):
                raise ValueError('waiting_fusion')
            p, q = self.pose.pose.position, self.pose.pose.orientation
            stamp = self.pose.header.stamp.sec * 1_000_000_000 + self.pose.header.stamp.nanosec
            if stamp == self.last_stamp:
                return
            current = PoseSample(stamp, np.array([p.x, p.y, p.z]), np.array([q.x, q.y, q.z, q.w]))
            stamps, positions, orientations = mission_path(
                self.snapshot, seq, self.origin, self.home, current, self.mount)
            output = Path()
            output.header = deepcopy(self.pose.header)
            for t, xyz, quat in zip(stamps, positions, orientations):
                item = PoseStamped()
                item.header.frame_id = self.frame
                nanoseconds_to_stamp(int(t), item.header.stamp)
                item.pose.position.x, item.pose.position.y, item.pose.position.z = map(float, xyz)
                (item.pose.orientation.x, item.pose.orientation.y,
                 item.pose.orientation.z, item.pose.orientation.w) = map(float, quat)
                output.poses.append(item)
            if self.guard.valid(output.header.stamp, token):
                self.output.publish(output)
                self.path_count += 1
                self.published, self.status, self.last_stamp = True, 'ready', stamp
        except (ValueError, TypeError, KeyError, OverflowError) as error:
            self.status = str(error)
            self.invalidate()

    def report(self):
        now = time.monotonic()
        message = DiagnosticArray(status=[DiagnosticStatus(
            name='lr/mission', level=DiagnosticStatus.OK if self.published else DiagnosticStatus.WARN,
            message=self.status, values=[KeyValue(key=k, value=str(v)) for k, v in {
                'mission_age_sec': now-self.snapshot_seen if self.snapshot_seen else -1,
                'progress_age_sec': self.state.get('mission_age_sec', -1),
                'mission_state': self.state.get('mission_state', 0),
                'mission_mode': self.state.get('mission_mode', 0),
                'mission_id': self.state.get('mission_id', 0),
                'path_hz': self.path_count / max(.001, now-self.report_started),
                'current_seq': self.last_seq, 'waypoints': len(self.snapshot or []),
                'target_system_id': self.system, 'target_component_id': self.component,
            }.items()])])
        message.header.stamp = self.get_clock().now().to_msg()
        self.diag.publish(message)
        self.path_count, self.report_started = 0, now


def main(args=None):
    rclpy.init(args=args)
    node = MissionPath()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
