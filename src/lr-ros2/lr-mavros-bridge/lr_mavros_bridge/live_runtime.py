"""Supervise independent camera and telemetry inputs with separate generations."""

import json
import math
import os
import signal
import subprocess
import sys
import time
import uuid

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import Header, String
from sensor_msgs.msg import NavSatFix, Imu
from geographic_msgs.msg import GeoPointStamped
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from mavros_msgs.srv import CommandLong


class LiveRuntime(Node):
    def __init__(self, **kwargs):
        super().__init__('live_runtime', **kwargs)
        def param(name, default):
            return self.declare_parameter(name, default).value
        self.options = {name: param(name, default) for name, default in (
            ('camera_name', 'zed'), ('camera_model', ''), ('stream_address', ''), ('stream_port', 0),
        )}
        self.profile = param('source_profile', 'native')
        self.clock_mode = param('camera_time_mode', 'unverified')
        self.camera_offset = int(param('camera_time_offset_ns', 0))
        if self.profile not in ('native', 'host_replay') or self.clock_mode not in (
                'unverified', 'system', 'offset', 'fcu_utc'):
            raise ValueError('Invalid source profile or camera time mode')
        self.target_ids = {name: int(param(name, 1)) for name in (
            'target_system_id', 'target_component_id')}
        self.gps_age = float(param('gps_max_age_s', .75))
        self.attitude_age = float(param('attitude_max_age_s', .25))
        self.camera_startup_timeout = float(param('camera_startup_timeout_sec', 15.))
        if not all(math.isfinite(v) and v > 0 for v in (
                self.gps_age, self.attitude_age, self.camera_startup_timeout)):
            raise ValueError('Input timeouts must be finite and positive')
        self.child, self.stopping = None, False
        self.stop_deadline = self.retry_at = 0.
        self.started = self.last_raw = time.monotonic()
        self.received_raw = False
        self.raw_stamp = self.raw_floor = 0
        self.last_frame = self.state_seen = 0.
        self.state, self.session, self.retired = {}, None, set()
        self.generation = self.camera_generation = 0
        self.runtime_id = uuid.uuid4().hex
        self.ready = self.camera_ready = self.was_synced = False
        self.start_ns = 0
        self.resume_at = 0.
        self.status = 'waiting_source'
        self.origin = None
        self.map_frame = param('map_frame', 'map')
        self.telemetry = {'gps': (0, 0.), 'attitude': (0, 0.)}
        self.counts = {'camera': 0, 'gps': 0, 'attitude': 0}
        self.report_at = self.report_started = time.monotonic()
        self.mapping = self.native_mapping()
        self.utc_client = self.create_client(CommandLong, '/mavros/cmd/command')
        self.utc_request = None
        self.utc_requested_at = self.next_utc_request = 0.
        self.utc_request_status = 'not_required'
        self.wall_sample = (self.get_clock().now().nanoseconds, time.monotonic())
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(String, '/lr/source/generation', qos)
        self.camera_publisher = self.create_publisher(String, '/lr/source/camera_generation', qos)
        self.diagnostics = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.create_subscription(String, '/lr/source/time_state', self.on_state, qos)
        self.create_subscription(GeoPointStamped, '/lr/mavlink/origin', self.on_origin, qos)
        self.create_subscription(Header, '/lr/net/raw_frame', self.on_raw, 10)
        self.create_subscription(Header, '/lr/net/frame_stamp', self.on_frame, 10)
        self.create_subscription(NavSatFix, '/mavros/global_position/raw/fix',
                                 self.on_gps, qos_profile_sensor_data)
        self.create_subscription(Imu, '/mavros/imu/data', self.on_attitude, qos_profile_sensor_data)
        self.create_timer(.1, self.poll)

    def native_mapping(self):
        return {'stamp_is_system': self.profile == 'native' and self.clock_mode in ('system', 'offset'),
                'stamp_offset_ns': self.camera_offset if self.clock_mode == 'offset' else 0,
                'record_anchor_ns': 0}

    def on_origin(self, message):
        values = [message.position.latitude, message.position.longitude, message.position.altitude]
        if self.origin is None and message.header.frame_id == self.map_frame and all(
                math.isfinite(v) for v in values):
            self.origin = values

    def boundary(self, status):
        self.generation += 1
        self.ready = False
        self.start_ns = self.get_clock().now().nanoseconds
        self.resume_at = time.monotonic() + .3
        self.status = status
        self.get_logger().info(f'Fusion generation {self.generation}: {status}')

    def camera_boundary(self, status):
        self.camera_generation += 1
        self.camera_ready = False
        self.raw_floor = self.raw_stamp
        self.boundary(status)

    def on_state(self, message):
        try:
            state = json.loads(message.data)
            synced = bool(state['connected'] and state['sync_ready'])
            offset = int(state['offset_ns'])
            session = state['session_id'] if self.profile == 'host_replay' else ''
            if self.profile == 'native' and self.clock_mode == 'fcu_utc' and synced and (
                    state.get('utc_ready') and 0 <= state.get('utc_age_sec', -1) < 2.):
                # Both anchors are measured/reported clocks, never packet arrival.
                mapped_offset = offset - int(state['utc_minus_boot_ns'])
                changed = (not self.mapping['stamp_is_system'] or
                           abs(mapped_offset-self.mapping['stamp_offset_ns']) > 100_000_000)
                if changed:
                    had_mapping = self.mapping['stamp_is_system']
                    self.camera_boundary('camera_clock_relation_changed')
                    if had_mapping and self.child is not None:
                        self.received_raw = False
                        self.raw_stamp = self.raw_floor = 0
                        self.stop_camera()
                self.mapping = {'stamp_is_system': True, 'stamp_offset_ns': mapped_offset,
                                'record_anchor_ns': 0}
            if self.profile == 'host_replay':
                if not session and self.session:
                    session = self.session
                    synced = False  # Retain the mapping, but require a fresh descriptor for fusion.
                if session in self.retired:
                    return
                if session != self.session:
                    if self.session:
                        self.retired.add(self.session)
                    self.session = session
                    self.camera_boundary('new_session')
                    if session and self.child is not None:
                        self.received_raw = False
                        self.last_frame = 0.
                        self.raw_stamp = self.raw_floor = 0
                        self.stop_camera()
                if session and state['session_id'] and synced:
                    mapping = {'stamp_is_system': True,
                               'stamp_offset_ns': int(state['boot_anchor_ns']) + offset
                               - int(state['record_anchor_ns']),
                               'record_anchor_ns': int(state['record_anchor_ns'])}
                    if not self.mapping['stamp_is_system']:
                        self.camera_boundary('camera_time_verified')
                    self.mapping = mapping
                synced = synced and bool(state['session_id'])
            if self.state and state.get('telemetry_epoch') != self.state.get('telemetry_epoch'):
                self.boundary('telemetry_clock_reset')
            if self.was_synced and not synced:
                self.boundary('waiting_mavlink_sync')
            if synced and not self.was_synced:
                self.boundary('sync_acquired')
            self.was_synced = synced
            self.state, self.state_seen = state, time.monotonic()
        except (ValueError, KeyError, TypeError):
            return

    def on_raw(self, message):
        stamp = message.stamp.sec * 1_000_000_000 + message.stamp.nanosec
        if stamp <= 0:
            return
        now = time.monotonic()
        if self.raw_stamp - 500_000_000 <= stamp <= self.raw_stamp:
            return
        if self.raw_stamp and (stamp < self.raw_stamp - 500_000_000 or abs(
                (stamp - self.raw_stamp) / 1e9 - (now - self.last_raw)) > 2.):
            self.raw_stamp = stamp
            self.camera_boundary('camera_clock_jump')
        if not self.received_raw:
            self.raw_floor = stamp
        if stamp >= self.raw_floor:
            self.raw_stamp = max(stamp, self.raw_stamp)
            self.last_raw, self.received_raw = now, True
            self.counts['camera'] += 1

    def on_frame(self, _message):
        self.last_frame = time.monotonic()

    def on_gps(self, message):
        valid = message.status.status >= 0 and all(math.isfinite(v) for v in (
            message.latitude, message.longitude, message.altitude))
        self.record_telemetry('gps', message.header, valid)

    def on_attitude(self, message):
        q, w = message.orientation, message.angular_velocity
        valid = all(math.isfinite(v) for v in (q.x, q.y, q.z, q.w, w.x, w.y, w.z))
        valid = valid and .5 <= math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w) <= 1.5
        self.record_telemetry('attitude', message.header, valid)

    def record_telemetry(self, key, header, valid):
        stamp = header.stamp.sec * 1_000_000_000 + header.stamp.nanosec
        self.counts[key] += 1
        if valid:
            self.telemetry[key] = (stamp, time.monotonic())

    def launch_camera(self):
        command = ['ros2', 'launch', 'lr_mavros_bridge', 'live_camera.launch.py']
        command.extend(f'{k}:={v}' for k, v in self.options.items())
        self.child = subprocess.Popen([
            sys.executable, '-m', 'lr_mavros_bridge.camera_child', str(os.getpid()), *command,
        ], start_new_session=True)
        self.started = self.last_raw = time.monotonic()
        self.received_raw = False
        self.get_logger().info(f'NET receiver started, PID={self.child.pid}')

    def stop_camera(self):
        if self.child is not None and self.child.poll() is None and not self.stopping:
            try:
                os.killpg(self.child.pid, signal.SIGINT)
            except ProcessLookupError:
                return
            self.stopping = True
            self.stop_deadline = time.monotonic() + 3.0

    def request_utc(self, now):
        if self.profile != 'native' or self.clock_mode != 'fcu_utc':
            return
        if self.utc_request is not None:
            if self.utc_request.done():
                try:
                    result = self.utc_request.result()
                    self.utc_request_status = 'accepted' if result.success else f'rejected:{result.result}'
                except Exception:
                    self.utc_request_status = 'request_failed'
                self.utc_request = None
            elif now-self.utc_requested_at > 15.:
                self.utc_client.remove_pending_request(self.utc_request)
                self.utc_request = None
                self.utc_request_status = 'request_timeout'
        if self.utc_request is not None or now < self.next_utc_request or not (
                self.state.get('connected') and now-self.state_seen < 2.):
            return
        if not self.utc_client.service_is_ready():
            self.utc_request_status = 'waiting_command_service'
            return
        request = CommandLong.Request()
        request.command, request.param1 = 512, 2.  # REQUEST_MESSAGE(SYSTEM_TIME), read only.
        # MAVROS CommandLong targets its configured UAS IDs; broadcast stays false.
        self.utc_request = self.utc_client.call_async(request)
        self.utc_requested_at, self.next_utc_request = now, now+1.

    def poll(self):
        now, wall = time.monotonic(), self.get_clock().now().nanoseconds
        self.request_utc(now)
        old_wall, old_steady = self.wall_sample
        if abs((wall - old_wall) / 1e9 - (now - old_steady)) > .1:
            self.camera_boundary('system_clock_jump')
        self.wall_sample = wall, now
        if self.child is not None and self.child.poll() is not None:
            self.child, self.stopping = None, False
            self.received_raw = False
            self.raw_stamp = self.raw_floor = 0
            self.retry_at = now + 2.
            self.camera_boundary('waiting_net')
        if self.stopping and now >= self.stop_deadline and self.child is not None:
            try:
                os.killpg(self.child.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        if self.child is None and now >= self.retry_at:
            self.launch_camera()
        silent = (now - self.last_raw > 5. if self.received_raw else
                  now - self.started > self.camera_startup_timeout)
        if self.child is not None and not self.stopping and silent:
            self.stop_camera()
        camera = (self.received_raw and self.child is not None and not self.stopping
                  and now - self.last_raw < 2.)
        if self.camera_ready and not camera:
            self.camera_boundary('waiting_net')
        self.camera_ready = camera
        mapped = self.raw_stamp + self.mapping['stamp_offset_ns']
        clock_ok = (self.mapping['stamp_is_system'] and mapped > 0
                    and -.5 <= (wall - mapped) / 1e9 <= 2.)
        if self.profile == 'native' and self.clock_mode == 'fcu_utc':
            clock_ok = clock_ok and bool(self.state.get('utc_ready')) and (
                now-self.state_seen < 2. and 0 <= self.state.get('utc_age_sec', -1) < 2.)
        # Stream liveness is separate from frame-causal estimation. Applying the
        # 250 ms attitude limit to wall time here resets the whole map at 4 Hz.
        # The pose estimator enforces GPS/attitude age against each IMAGE stamp.
        telemetry = all(stamp > 0 and now - seen < 2.
                        and -.5 <= (wall - stamp) / 1e9 <= 2.
                        for stamp, seen in self.telemetry.values())
        healthy = (camera and clock_ok and telemetry and self.was_synced
                   and now - self.state_seen < 2.)
        reason = ('waiting_net' if not camera else 'camera_time_unverified' if not clock_ok
                  else 'waiting_mavlink_sync' if not self.was_synced or now-self.state_seen >= 2.
                  else 'waiting_telemetry' if not telemetry else 'running')
        if self.ready and not healthy:
            self.boundary(reason)
        self.ready = healthy and now >= self.resume_at
        self.status = reason if not self.ready else 'running'
        common = {'version': 1, 'runtime_id': self.runtime_id,
                  'session_id': self.session or '', 'source_profile': self.profile,
                  'camera_time_mode': self.clock_mode}
        self.camera_publisher.publish(String(data=json.dumps({
            **common, **self.mapping, 'generation': self.camera_generation,
            'ready': camera, 'start_ns': self.raw_floor + self.mapping['stamp_offset_ns'],
            'latest_stamp_ns': mapped, 'raw_floor_ns': self.raw_floor,
            'status': 'running' if camera else 'waiting_net'})))
        self.publisher.publish(String(data=json.dumps({
            **common, 'generation': self.generation, 'ready': self.ready,
            'start_ns': self.start_ns, 'status': self.status, 'origin': self.origin,
            'camera_ready': camera, 'camera_time_verified': clock_ok,
            'telemetry_ready': telemetry})))
        if now >= self.report_at:
            self.report(camera, clock_ok, telemetry, now)
            self.report_at = now + 1.

    def report(self, camera, clock_ok, telemetry, now):
        elapsed = max(.001, now - self.report_started)
        values = {**self.state, **self.target_ids, 'camera_time_mode': self.clock_mode,
                  'system_time_request': self.utc_request_status,
                  'camera_stamp_offset_ns': self.mapping['stamp_offset_ns'],
                  'camera_age_sec': now-self.last_raw, 'source_profile': self.profile,
                  **{k+'_age_sec': (self.get_clock().now().nanoseconds-v[0])/1e9
                     if v[0] else -1. for k, v in self.telemetry.items()},
                  **{k+'_hz': v/elapsed for k, v in self.counts.items()}}
        statuses = [DiagnosticStatus(
            name='lr/'+name, level=DiagnosticStatus.OK if ready else DiagnosticStatus.WARN,
            message='ready' if ready else self.status,
            values=[KeyValue(key=k, value=str(v)) for k, v in values.items()])
            for name, ready in (('camera', camera), ('telemetry', telemetry),
                                ('clock', clock_ok and self.was_synced and now-self.state_seen < 2.),
                                ('fusion', self.ready))]
        output = DiagnosticArray(status=statuses)
        output.header.stamp = self.get_clock().now().to_msg()
        self.diagnostics.publish(output)
        self.counts = dict.fromkeys(self.counts, 0)
        self.report_started = now

    def destroy_node(self):
        self.stop_camera()
        if self.child is not None:
            try:
                self.child.wait(timeout=4)
            except subprocess.TimeoutExpired:
                os.killpg(self.child.pid, signal.SIGKILL)
                self.child.wait(timeout=2)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LiveRuntime()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
