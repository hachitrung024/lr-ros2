"""One bridge node, with small compatibility wrappers for individual roles."""

from collections import deque
import math
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import Path
import rclpy
from rclpy.clock import Clock, ClockType, JumpThreshold
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from safety_perception_msgs.msg import GeometryArray, RoverState, TrackedObjectArray, Trajectory
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection3DArray

from lr_terrain_geometry.grid_map_sampling import GridMapSampler

from .conversions import (
    StateConverter,
    TrajectoryConverter,
    geometry_from_trajectory,
    objects_from_detections,
    require_frame,
    stamp_ns,
)
from .detection3d_conversion import rotation_matrix_from_tf, translation_from_tf


class PredictionBridgeNode(Node):
    """Convert LR inputs, sampling geometry without a local DDS round trip."""

    def __init__(self, *, role=None, **kwargs):
        super().__init__(f'{role}_adapter_node' if role else 'prediction_bridge_node', **kwargs)
        self._role = role
        self._frame = self.declare_parameter('expected_frame_id', 'map').value.strip()
        if not self._frame:
            raise ValueError('expected_frame_id must not be empty')
        # Accepted for old YAMLs; never permits relabelling coordinates.
        self.declare_parameter('force_frame_id_map', False)
        profile = self.declare_parameter('prediction_profile', 'static').value
        if profile not in ('static', 'dynamic'):
            raise ValueError('prediction_profile must be static or dynamic')
        reliable = QoSProfile(depth=10)
        sensor = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        grid_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._grids = deque(maxlen=128)
        self._active_trajectory = None
        self._last_geometry_key = None
        self._trajectory_converter = None
        self._state_converter = None
        self._tf_buffer = None
        self._pending_detections = deque(maxlen=30)
        self._last_warning_time = 0.0
        self._diag = self.create_publisher(DiagnosticArray, '/prediction/diagnostics', 10)
        roles = {role} if role else {'trajectory', 'geometry', 'tracked_objects'}
        if role is None and profile == 'dynamic':
            roles.add('rover_state')
        if 'trajectory' in roles:
            values = {
                name: self._param('trajectory', name, default)
                for name, default in (
                    ('horizon_steps', 20),
                    ('path_stride', 1),
                    ('output_dt_sec', 0.25),
                    ('min_distance_from_start_m', 1.0),
                    ('minimum_cycle_period_sec', 0.25),
                )
            }
            self._trajectory_converter = TrajectoryConverter(frame=self._frame, **values)
            self._trajectory_pub = self.create_publisher(
                Trajectory, self._param('trajectory', 'output_topic', '/trajectory'), reliable
            )
            self.create_subscription(
                Path,
                self._param('trajectory', 'input_topic', '/lr/future_path/ground_truth'),
                self._on_path,
                reliable,
            )
        if 'geometry' in roles:
            self._allow_flat = self._param('geometry', 'allow_flat_fallback', False)
            self._flat_conf = self._param('geometry', 'flat_fallback_confidence', 0.25)
            self._grid_max_age = self._param('geometry', 'max_geometry_age_sec', 2.0)
            if not 0 <= self._grid_max_age < float('inf'):
                raise ValueError('max_geometry_age_sec must be finite and non-negative')
            self._geometry_pub = self.create_publisher(
                GeometryArray, self._param('geometry', 'output_topic', '/geometry'), reliable
            )
            self.create_subscription(
                GridMap,
                self._param('geometry', 'grid_map_topic', '/terrain_geometry/grid_map'),
                self._on_grid,
                grid_qos,
            )
            if role == 'geometry':
                self.create_subscription(
                    Trajectory,
                    self._param('geometry', 'trajectory_topic', '/trajectory'),
                    self._on_trajectory,
                    reliable,
                )
        if 'tracked_objects' in roles:
            self._objects_input_type = self._param(
                'tracked_objects', 'input_type', 'detection3d'
            )
            if self._objects_input_type not in ('detection3d', 'tracked_objects'):
                raise ValueError(
                    'tracked_objects.input_type must be detection3d or tracked_objects'
                )
            self._tf_timeout = self._param('tracked_objects', 'tf_timeout_sec', 0.2)
            if not 0 <= self._tf_timeout < float('inf'):
                raise ValueError('tf_timeout_sec must be finite and non-negative')
            self._objects_pub = self.create_publisher(
                TrackedObjectArray,
                self._param('tracked_objects', 'output_topic', '/tracked_objects'),
                reliable,
            )
            objects_input_topic = self._param(
                'tracked_objects', 'input_topic', '/segmentation/boxes_3d'
            )
            if self._objects_input_type == 'detection3d':
                self._tf_buffer = Buffer(cache_time=Duration(seconds=2.0))
                self._tf_listener = TransformListener(self._tf_buffer, self)
                self.create_subscription(
                    Detection3DArray,
                    objects_input_topic,
                    self._on_detections,
                    sensor,
                )
                self._retry_clock = Clock(clock_type=ClockType.STEADY_TIME)
                self._retry_timer = self.create_timer(
                    0.01, self._retry_detections, clock=self._retry_clock
                )
            else:
                self.create_subscription(
                    TrackedObjectArray,
                    objects_input_topic,
                    self._on_tracked_objects,
                    sensor,
                )
        if 'rover_state' in roles:
            self._state_converter = StateConverter(
                frame=self._frame,
                min_dt_sec=self._param('rover_state', 'min_dt_sec', 1e-3),
                max_dt_sec=self._param('rover_state', 'max_dt_sec', 1.0),
            )
            self._state_pub = self.create_publisher(
                RoverState, self._param('rover_state', 'output_topic', '/rover/state'), reliable
            )
            self.create_subscription(
                PoseStamped,
                self._param('rover_state', 'pose_topic', '/lr/mavlink/pose'),
                self._on_pose,
                reliable,
            )
        self._roles = roles
        self._jump_handle = self.get_clock().create_jump_callback(
            JumpThreshold(
                min_forward=None, min_backward=Duration(nanoseconds=-1), on_clock_change=True
            ),
            post_callback=self._on_time_jump,
        )
        self.get_logger().info(f'Prediction bridge roles={sorted(roles)}, frame={self._frame}')

    def _param(self, role, name, default):
        return self.declare_parameter(name if self._role else f'{role}.{name}', default).value

    def _on_time_jump(self, _jump):
        self._grids.clear()
        self._pending_detections.clear()
        self._active_trajectory = None
        self._last_geometry_key = None
        if self._trajectory_converter:
            self._trajectory_converter.reset()
        if self._state_converter:
            self._state_converter.reset()
        if self._tf_buffer:
            self._tf_buffer.clear()
        self.get_logger().info('ROS time changed; bridge caches cleared')

    def _warn(self, reason):
        now = time.monotonic()
        if now - self._last_warning_time >= 1.0:
            self.get_logger().warning(str(reason))
            message = DiagnosticArray()
            message.header.stamp = self.get_clock().now().to_msg()
            message.status = [
                DiagnosticStatus(
                    name=self.get_name(),
                    level=DiagnosticStatus.WARN,
                    message=str(reason),
                    values=[KeyValue(key='frame', value=self._frame)],
                )
            ]
            self._diag.publish(message)
            self._last_warning_time = now

    def _on_path(self, message):
        try:
            trajectory = self._trajectory_converter.convert(message)
            if trajectory is not None:
                self._trajectory_pub.publish(trajectory)
                if 'geometry' in self._roles:
                    self._on_trajectory(trajectory)
        except ValueError as error:
            self._warn(error)

    def _on_trajectory(self, message):
        try:
            require_frame(message.header, self._frame)
        except ValueError as error:
            self._warn(error)
            return
        self._active_trajectory = message
        self._publish_geometry()

    def _on_grid(self, message):
        try:
            require_frame(message.header, self._frame)
            sampler = GridMapSampler(message)
        except (ValueError, IndexError, TypeError) as error:
            self._warn(error)
            return
        self._grids.append((stamp_ns(message.header.stamp), message.header, sampler))
        self._publish_geometry()

    def _publish_geometry(self):
        trajectory = self._active_trajectory
        if trajectory is None:
            return
        stamp = stamp_ns(trajectory.header.stamp)
        eligible = [g for g in self._grids if 0 <= stamp - g[0] <= self._grid_max_age * 1e9]
        grid = max(eligible, key=lambda g: g[0]) if eligible else None
        if grid is None and not self._allow_flat:
            self._warn('waiting: terrain at or before trajectory within max_geometry_age_sec')
            return
        key = (trajectory.trajectory_id, stamp, None if grid is None else grid[0])
        if key == self._last_geometry_key:
            return
        output = geometry_from_trajectory(
            trajectory,
            None if grid is None else grid[2],
            None if grid is None else grid[1],
            allow_flat=self._allow_flat,
            flat_confidence=self._flat_conf,
        )
        # Empty coverage is still reported to the runtime as unknown, not flat.
        self._geometry_pub.publish(output)
        self._last_geometry_key = key

    def _on_pose(self, message):
        try:
            self._state_pub.publish(self._state_converter.convert(message))
        except ValueError as error:
            self._warn(error)

    def _on_detections(self, message):
        self._convert_detections(message, time.monotonic() + self._tf_timeout)

    def _on_tracked_objects(self, message):
        try:
            _validate_tracked_objects(message, self._frame)
            self._objects_pub.publish(message)
        except ValueError as error:
            self._warn(error)

    def _retry_detections(self):
        pending = list(self._pending_detections)
        self._pending_detections.clear()
        for message, deadline in pending:
            self._convert_detections(message, deadline)

    def _convert_detections(self, message, deadline):
        transform = {}
        try:
            if not message.header.frame_id.strip():
                raise ValueError('Detection3DArray frame is empty')
            if message.header.frame_id != self._frame:
                # Do not block the single bridge executor waiting on its own TF subscription.
                tf = self._tf_buffer.lookup_transform(
                    self._frame, message.header.frame_id, Time.from_msg(message.header.stamp)
                )
                transform = dict(
                    transform_rotation=rotation_matrix_from_tf(tf.transform),
                    transform_translation=translation_from_tf(tf.transform),
                )
            output, stats = objects_from_detections(message, self._frame, **transform)
            if stats.skipped_invalid:
                # A malformed non-empty batch must not turn into evidence of no objects.
                self._warn(f'invalid detections: {stats.skipped_invalid}; batch dropped')
                return
            self._objects_pub.publish(output)
        except TransformException as error:
            if time.monotonic() < deadline:
                self._pending_detections.append((message, deadline))
            else:
                self._warn(error)
        except ValueError as error:
            self._warn(error)


def _validate_tracked_objects(message, expected_frame):
    """Reject malformed canonical batches before they reach Prediction."""
    require_frame(message.header, expected_frame)
    track_ids = set()
    for tracked_object in message.objects:
        if tracked_object.track_id in track_ids:
            raise ValueError(f'duplicate track ID {tracked_object.track_id}')
        track_ids.add(tracked_object.track_id)
        polygon = [
            (float(point.x), float(point.y))
            for point in tracked_object.footprint_polygon_xy
        ]
        if len(polygon) < 3 or len(set(polygon)) < 3:
            raise ValueError(f'track {tracked_object.track_id} has an invalid footprint')
        if not all(math.isfinite(value) for point in polygon for value in point):
            raise ValueError(f'track {tracked_object.track_id} has non-finite footprint data')
        area_twice = sum(
            first[0] * second[1] - first[1] * second[0]
            for first, second in zip(polygon, polygon[1:] + polygon[:1])
        )
        if abs(area_twice) <= 1e-9:
            raise ValueError(f'track {tracked_object.track_id} has a degenerate footprint')
        if not _polygon_is_simple(polygon):
            raise ValueError(f'track {tracked_object.track_id} footprint self-intersects')
        if tracked_object.confidence_valid and not math.isfinite(
            tracked_object.confidence
        ):
            raise ValueError(f'track {tracked_object.track_id} has invalid confidence')
        velocity = tracked_object.velocity
        if tracked_object.velocity_valid and not all(
            math.isfinite(value) for value in (velocity.x, velocity.y, velocity.z)
        ):
            raise ValueError(f'track {tracked_object.track_id} has invalid velocity')


def _polygon_is_simple(polygon):
    """Return whether a polygon has no intersecting non-adjacent edges."""
    count = len(polygon)
    for first in range(count):
        first_end = (first + 1) % count
        for second in range(first + 1, count):
            second_end = (second + 1) % count
            if first_end == second or second_end == first:
                continue
            if _segments_intersect(
                polygon[first],
                polygon[first_end],
                polygon[second],
                polygon[second_end],
            ):
                return False
    return True


def _segments_intersect(first, first_end, second, second_end):
    """Test closed 2D segments, including collinear contact."""
    def orientation(origin, end, point):
        return (end[0] - origin[0]) * (point[1] - origin[1]) - (
            end[1] - origin[1]
        ) * (point[0] - origin[0])

    values = (
        orientation(first, first_end, second),
        orientation(first, first_end, second_end),
        orientation(second, second_end, first),
        orientation(second, second_end, first_end),
    )
    epsilon = 1e-10
    if values[0] * values[1] < -epsilon and values[2] * values[3] < -epsilon:
        return True

    def on_segment(origin, end, point):
        return (
            min(origin[0], end[0]) - epsilon <= point[0]
            <= max(origin[0], end[0]) + epsilon
            and min(origin[1], end[1]) - epsilon <= point[1]
            <= max(origin[1], end[1]) + epsilon
        )

    return any(
        abs(value) <= epsilon and on_segment(origin, end, point)
        for value, origin, end, point in (
            (values[0], first, first_end, second),
            (values[1], first, first_end, second_end),
            (values[2], second, second_end, first),
            (values[3], second, second_end, first_end),
        )
    )


def run(role=None, argv=None):
    rclpy.init(args=argv)
    node = None
    try:
        node = PredictionBridgeNode(role=role)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        # Humble can raise while taking a subscription during context teardown.
        if rclpy.ok():
            raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(argv=None):
    run(argv=argv)
