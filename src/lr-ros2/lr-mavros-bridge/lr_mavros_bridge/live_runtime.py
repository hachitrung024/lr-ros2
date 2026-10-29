"""Keep NET reception alive and fence all processing at source boundaries."""

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
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Header, String
from geographic_msgs.msg import GeoPointStamped


class LiveRuntime(Node):
    def __init__(self):
        super().__init__('live_runtime')
        self.options = {name: self.declare_parameter(name, default).value for name, default in (
            ('camera_name', 'zed'), ('camera_model', ''), ('stream_address', ''), ('stream_port', 0),
        )}
        self.child = None
        self.stopping = False
        self.stop_deadline = 0.0
        self.retry_at = 0.0
        self.started = self.last_raw = time.monotonic()
        self.received_raw = False
        self.camera_startup_timeout = float(self.declare_parameter(
            'camera_startup_timeout_sec', 15.).value)
        if not math.isfinite(self.camera_startup_timeout) or self.camera_startup_timeout <= 0:
            raise ValueError('camera_startup_timeout_sec must be finite and positive')
        self.last_frame = 0.0
        self.state_seen = 0.0
        self.state = {}
        self.session = None
        self.retired = set()
        self.generation = 0
        self.runtime_id = uuid.uuid4().hex
        self.ready = False
        self.start_ns = 0
        self.resume_at = 0.0
        self.was_synced = False
        self.status = 'waiting_source'
        self.origin = None
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(String, '/lr/source/generation', qos)
        self.create_subscription(String, '/lr/source/time_state', self.on_state, qos)
        self.create_subscription(GeoPointStamped, '/lr/mavlink/origin', self.on_origin, qos)
        self.create_subscription(Header, '/lr/net/raw_frame', self.on_raw, 10)
        self.create_subscription(Header, '/lr/net/frame_stamp', self.on_frame, 10)
        self.create_timer(0.1, self.poll)

    def on_origin(self, message):
        values = [message.position.latitude, message.position.longitude, message.position.altitude]
        if self.origin is None and message.header.frame_id == self.map_frame and all(
                math.isfinite(v) for v in values):
            self.origin = values

    def boundary(self, status):
        self.generation += 1
        self.ready = False
        self.resume_at = time.monotonic() + 0.3
        self.status = status
        self.get_logger().info(f'Source generation {self.generation}: {status}')

    def on_state(self, message):
        try:
            state = json.loads(message.data)
            session = state['session_id']
            offset = int(state['offset_ns'])
            synced = bool(state['connected'] and state['sync_ready'] and offset > 0)
            if not session and self.session and not synced:
                # MAVROS may respawn before the next descriptor arrives.
                session = self.session
            if session in self.retired:
                return
            if session != self.session:
                if self.session and session:
                    self.retired.add(self.session)
                self.session = session
                self.start_ns = (int(state['boot_anchor_ns']) + offset) if session else (
                    self.get_clock().now().nanoseconds)
                self.boundary('new_session')
                if session and self.child is not None:
                    # NET frames carry no source session ID. Recreate the receiver
                    # before accepting images from a seek/restarted encoder.
                    self.received_raw = False
                    self.last_frame = 0.0
                    self.stop_camera()
            if self.was_synced and not synced:
                self.boundary('waiting_mavlink')
            if synced and not self.was_synced:
                self.start_ns = (int(state['boot_anchor_ns']) + offset) if session else (
                    self.get_clock().now().nanoseconds)
                self.resume_at = time.monotonic() + 0.3
            self.was_synced = synced
            self.state, self.state_seen = state, time.monotonic()
        except (ValueError, KeyError, TypeError):
            return

    def on_raw(self, _message):
        self.last_raw = time.monotonic()
        self.received_raw = True

    def on_frame(self, _message):
        self.last_frame = time.monotonic()

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

    def poll(self):
        now = time.monotonic()
        if self.child is not None and self.child.poll() is not None:
            self.child = None
            self.stopping = False
            self.retry_at = now + 2.0
            if self.ready:
                self.boundary('waiting_net')
        if self.stopping and now >= self.stop_deadline:
            try:
                os.killpg(self.child.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        if self.child is None and now >= self.retry_at and self.was_synced:
            self.launch_camera()
        # Loading the wrapper/SDK can exceed the steady-state 5 s watchdog,
        # especially while prediction processes are active. Require real raw
        # data before reporting readiness; give initialization its own bound.
        silent = (now - self.last_raw > 5.0 if self.received_raw else
                  now - self.started > self.camera_startup_timeout)
        if self.child is not None and not self.stopping and silent:
            self.stop_camera()
        healthy = (self.was_synced and self.received_raw and self.child is not None
                   and now - self.state_seen < 2.0
                   and now - self.last_raw < 2.0 and not self.stopping)
        if self.ready and not healthy:
            self.boundary('waiting_source')
        if healthy and now >= self.resume_at:
            self.ready = True
            self.status = 'running' if now - self.last_frame < 2.0 else 'waiting_frames_or_time_mapping'
        self.publisher.publish(String(data=json.dumps({
            'version': 1, 'runtime_id': self.runtime_id,
            'generation': self.generation, 'ready': self.ready,
            'start_ns': self.start_ns, 'session_id': self.session or '', 'status': self.status,
            'origin': self.origin,
        })))

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
