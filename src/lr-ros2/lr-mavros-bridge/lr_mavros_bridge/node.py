"""Adapt received MAVROS sensors to the existing LR pose and TF contract."""

from collections import deque
import ctypes
import json
from pathlib import Path
import time
from types import SimpleNamespace

from ament_index_python.packages import get_package_prefix
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geographic_msgs.msg import GeoPointStamped
from geometry_msgs.msg import PoseStamped, TransformStamped
import numpy as np
import rclpy
from rclpy.clock import Clock as RosClock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Header, String, UInt64
from tf2_ros import TransformBroadcaster

from lr_future_path.mavlink import apply_body_to_camera, geodetic_to_enu
from .estimator import Estimator, Gps, Attitude


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class MavrosPose(Node):
    def __init__(self):
        super().__init__("mavros_pose")

        def param(name, value):
            return self.declare_parameter(name, value).value

        self.live = param('time_mode', 'replay') == 'live'
        self.live_ready = False
        self.generation = None
        self.runtime_id = ''
        self.retired_runtimes = set()
        self.start_ns = 0
        self.map_frame = param("map_frame", "map")
        self.child_frame = param("child_frame", "zed_camera_link")
        self.body_to_camera = np.asarray(param("body_to_camera", [0.0] * 6))
        self.wait_s = param("telemetry_wait_s", 0.1)
        if self.body_to_camera.shape != (6,) or not np.all(np.isfinite(self.body_to_camera)):
            raise ValueError("body_to_camera must contain six finite XYZ/RPY values")
        if not np.isfinite(self.wait_s) or not 0 <= self.wait_s <= 1:
            raise ValueError("telemetry_wait_s must be in [0, 1]")
        self.estimator = Estimator(
            param("gps_max_age_s", 0.75),
            param("gps_max_gap_s", 1.5),
            param("max_speed_mps", 5.0),
            param("attitude_max_age_s", 0.25),
        )
        geoid = ctypes.CDLL(
            str(Path(get_package_prefix("lr_mavros_bridge")) / "lib/liblr_geoid.so")
        )
        self.geoid_height = geoid.lr_geoid_height
        self.geoid_height.argtypes = [ctypes.c_double, ctypes.c_double]
        self.geoid_height.restype = ctypes.c_double
        if not np.isfinite(self.geoid_height(0.0, 0.0)):
            raise RuntimeError(
                "Install GeographicLib egm96-5 geoid dataset before starting MAVROS"
            )
        self.origin = None
        self.epoch_us = 0
        self.clock_ns = 0
        self.last_clock_wall = time.monotonic()
        self.pending = deque()
        self.status = "waiting_clock"
        self.total = self.valid = 0
        audit_path = param("audit_file", "")
        self.audit = open(audit_path, "a" if self.live else "w") if audit_path else None
        self.pose_pub = self.create_publisher(
            PoseStamped, param("pose_topic", "/lr/mavlink/pose"), 10
        )
        self.tf = TransformBroadcaster(self)
        self.diag = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        if self.live:
            self.origin_pub = self.create_publisher(GeoPointStamped, '/lr/mavlink/origin', latched)
            self.create_subscription(String, '/lr/source/generation', self.on_generation, latched)
            self.create_subscription(Header, '/lr/net/frame_stamp', self.on_frame, 100)
        else:
            self.create_subscription(GeoPointStamped, "/lr/mavlink/origin", self.on_origin, latched)
            self.create_subscription(UInt64, "/mavros/replay_time/epoch_us", self.on_epoch, latched)
        self.create_subscription(
            NavSatFix, "/mavros/global_position/raw/fix", self.on_gps, qos_profile_sensor_data
        )
        self.create_subscription(
            Imu, "/mavros/imu/data", self.on_attitude, qos_profile_sensor_data
        )
        if not self.live:
            self.create_subscription(Clock, "/clock", self.on_clock, 100)
        self.create_timer(0.01, self.process, clock=RosClock(clock_type=ClockType.STEADY_TIME))
        self.create_timer(1.0, self.diagnostics, clock=RosClock(clock_type=ClockType.STEADY_TIME))

    def reset(self):
        self.pending.clear()
        self.estimator.reset()
        self.status = "waiting_samples"

    def on_generation(self, message):
        try:
            state = json.loads(message.data)
            generation = (state.get('runtime_id', ''), state['generation'])
            ready = state['ready']
            runtime = generation[0]
            if runtime in self.retired_runtimes:
                return
            if runtime == self.runtime_id and self.generation and generation < self.generation:
                return
            if runtime != self.runtime_id:
                if self.runtime_id:
                    self.retired_runtimes.add(self.runtime_id)
                self.runtime_id = runtime
            if generation != self.generation or (self.live_ready and not ready):
                self.reset()
                self.clock_ns = 0
            self.generation, self.live_ready = generation, ready
            self.start_ns = int(state['start_ns'])
            if self.origin is None and state.get('origin') is not None:
                origin = GeoPointStamped()
                origin.header.frame_id = self.map_frame
                origin.header.stamp = self.get_clock().now().to_msg()
                origin.position.latitude, origin.position.longitude, origin.position.altitude = (
                    state['origin'])
                self.on_origin(origin)
                self.origin_pub.publish(origin)
        except (KeyError, ValueError, TypeError):
            return

    def on_frame(self, message):
        if self.accepted_stamp(stamp_ns(message.stamp)):
            self.on_clock(SimpleNamespace(clock=message.stamp))

    def on_origin(self, message):
        origin = np.array(
            [message.position.latitude, message.position.longitude, message.position.altitude]
        )
        if not np.all(np.isfinite(origin)) or message.header.frame_id != self.map_frame:
            return
        if self.origin is None or not np.array_equal(self.origin, origin):
            self.reset()
        self.origin = origin

    def on_epoch(self, message):
        if message.data != self.epoch_us:
            self.reset()
        self.epoch_us = message.data

    def accepted_stamp(self, stamp):
        # Reject old-run packets after rewind; tolerate ordinary UDP/DDS arrival ordering.
        if self.live:
            now = self.get_clock().now().nanoseconds
            return self.live_ready and self.start_ns <= stamp <= now + 500_000_000 and (
                now - stamp <= 2_000_000_000)
        return (
            self.epoch_us > 0
            and self.clock_ns > 0
            and (self.epoch_us * 1000 <= stamp <= self.clock_ns + 500_000_000)
        )

    def on_gps(self, message):
        stamp = stamp_ns(message.header.stamp)
        if (not self.live and self.origin is None) or not self.accepted_stamp(stamp):
            return
        valid = message.status.status >= 0 and all(
            np.isfinite(v) for v in (message.latitude, message.longitude, message.altitude)
        )
        position = np.zeros(3)
        if valid:
            # MAVROS publishes ellipsoid height; legacy ground truth uses the DB's AMSL height.
            msl = message.altitude - self.geoid_height(message.latitude, message.longitude)
            if self.live and self.origin is None:
                self.origin = np.array([message.latitude, message.longitude, msl])
                origin = GeoPointStamped()
                origin.header.frame_id = self.map_frame
                origin.header.stamp = message.header.stamp
                origin.position.latitude, origin.position.longitude, origin.position.altitude = (
                    self.origin.tolist())
                self.origin_pub.publish(origin)
            position = geodetic_to_enu(
                np.array([[message.latitude, message.longitude, msl]]), self.origin
            )[0]
        self.estimator.add_gps(Gps(stamp, position, valid))

    def on_attitude(self, message):
        stamp = stamp_ns(message.header.stamp)
        if not self.accepted_stamp(stamp):
            return
        q, w = message.orientation, message.angular_velocity
        self.estimator.add_attitude(
            Attitude(stamp, np.array([q.x, q.y, q.z, q.w]), np.array([w.x, w.y, w.z]))
        )

    def on_clock(self, message):
        stamp = stamp_ns(message.clock)
        if stamp <= 0 or stamp == self.clock_ns:
            return
        if stamp < self.clock_ns:
            self.reset()
        self.clock_ns = stamp
        self.last_clock_wall = time.monotonic()
        self.pending.append((stamp, time.monotonic() + self.wait_s))
        while len(self.pending) > 100:
            self.pending.popleft()

    def process(self):
        now = time.monotonic()
        while self.pending and self.pending[0][1] <= now:
            stamp, _ = self.pending.popleft()
            self.total += 1
            if (not self.live and not self.epoch_us) or self.origin is None or (
                    self.live and not self.accepted_stamp(stamp)):
                self.status = "waiting_epoch_or_origin"
                continue
            sample, self.status, evidence = self.estimator.estimate(stamp)
            if sample is None:
                continue
            sample = apply_body_to_camera(sample, self.body_to_camera)
            message = PoseStamped()
            message.header.frame_id = self.map_frame
            message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
            message.pose.position.x, message.pose.position.y, message.pose.position.z = (
                sample.position.tolist()
            )
            q = sample.orientation_xyzw.tolist()
            message.pose.orientation.x, message.pose.orientation.y = q[:2]
            message.pose.orientation.z, message.pose.orientation.w = q[2:]
            self.pose_pub.publish(message)
            transform = TransformStamped()
            transform.header = message.header
            transform.child_frame_id = self.child_frame
            transform.transform.translation.x = message.pose.position.x
            transform.transform.translation.y = message.pose.position.y
            transform.transform.translation.z = message.pose.position.z
            transform.transform.rotation = message.pose.orientation
            self.tf.sendTransform(transform)
            self.valid += 1
            if self.audit:
                self.audit.write(
                    json.dumps(
                        dict(
                            stamp_ns=stamp,
                            position=sample.position.tolist(),
                            orientation=q,
                            **evidence,
                        )
                    )
                    + "\n"
                )
                self.audit.flush()

    def diagnostics(self):
        if time.monotonic() - self.last_clock_wall > 2.0:
            self.status = "frames_stopped" if self.live else "clock_stopped"
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus(
            name="lr/mavros_pose",
            hardware_id="mavlink_replay",
            level=DiagnosticStatus.OK if self.status == "valid" else DiagnosticStatus.WARN,
            message=self.status,
        )
        status.values = [
            KeyValue(key="frames", value=str(self.total)),
            KeyValue(key="valid_frames", value=str(self.valid)),
            KeyValue(key="epoch_us", value=str(self.epoch_us)),
        ]
        message.status = [status]
        self.diag.publish(message)

    def destroy_node(self):
        if self.audit:
            self.audit.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MavrosPose()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
