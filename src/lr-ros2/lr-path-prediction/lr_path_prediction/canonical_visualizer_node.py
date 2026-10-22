"""Convert canonical PredictionOutput evidence into diagnostics and RViz."""

from __future__ import annotations

import math
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from nav_msgs.msg import Path
import rclpy
from rclpy.clock import Clock, ClockType, JumpThreshold
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from safety_perception_msgs.msg import (
    GeometryArray,
    PredictionOutput,
    Trajectory,
    TrajectoryPath,
)
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

from lr_terrain_geometry.grid_map_sampling import TerrainSample
from .presentation import StepPrediction
from .path_snapshot import stamp_ns, validate_snapshot
from .outputs import (
    predictions_to_diagnostics,
    predictions_to_markers,
)


_FOOTPRINT_INTERSECTION_EPSILON_M = 1e-9


def intersecting_collision_objects(candidates):
    """Keep only raw collision candidates whose XY footprints intersect."""
    return [
        item
        for item in candidates
        if float(item.min_distance_m) <= _FOOTPRINT_INTERSECTION_EPSILON_M
    ]


class CanonicalPredictionVisualizerNode(Node):
    """Join canonical cycle messages and render physical evidence in RViz."""

    def __init__(self, **node_kwargs) -> None:
        """Connect canonical Prediction topics to LR visualization outputs."""
        super().__init__("canonical_prediction_visualizer", **node_kwargs)
        self._trajectory_topic = str(
            self.declare_parameter("input.trajectory_topic", "/trajectory").value
        )
        self._geometry_topic = str(
            self.declare_parameter("input.geometry_topic", "/geometry").value
        )
        self._prediction_topic = str(
            self.declare_parameter("input.prediction_topic", "/predict_output").value
        )
        snapshot_topic = str(self.declare_parameter(
            'input.path_snapshot_topic', '/trajectory/path',
        ).value)
        self._max_prediction_age = float(self.declare_parameter(
            'visualization.max_prediction_age_sec', 1.0,
        ).value)
        self._max_path_age = float(self.declare_parameter(
            'visualization.max_path_age_sec', 1.5,
        ).value)
        self._max_geometry_age = float(self.declare_parameter(
            'visualization.max_geometry_age_sec', 2.0,
        ).value)
        if not all(math.isfinite(v) and v > 0 for v in
                   (self._max_prediction_age, self._max_path_age, self._max_geometry_age)):
            raise ValueError('Presentation age limits must be finite and positive')
        steps_topic = str(
            self.declare_parameter("output.steps_topic", "/lr/path_prediction/steps").value
        )
        markers_topic = str(
            self.declare_parameter("output.markers_topic", "/lr/path_prediction/markers").value
        )
        reference_path_topic = str(
            self.declare_parameter(
                "output.reference_path_topic",
                "/lr/path_prediction/reference_path",
            ).value
        )
        self._map_frame = str(self.declare_parameter("frames.map_frame", "map").value).strip()
        self._rollover_warning_margin = float(self.declare_parameter(
            "warning.rollover_normalized_margin", 0.20,
        ).value)
        self._rollover_critical_margin = float(self.declare_parameter(
            "critical.rollover_normalized_margin", 0.0,
        ).value)
        if not (math.isfinite(self._rollover_warning_margin)
                and math.isfinite(self._rollover_critical_margin)
                and self._rollover_critical_margin < self._rollover_warning_margin):
            raise ValueError("rollover margins must be finite with critical < warning")
        self._slope_warning_deg = float(self.declare_parameter("warning.slope_deg", 20.0).value)
        self._slope_critical_deg = float(self.declare_parameter("critical.slope_deg", 30.0).value)
        self._normal_length_m = float(
            self.declare_parameter("visualization.normal_length_m", 0.8).value
        )
        # Legacy parameters are accepted for existing YAML files. Source XYZ
        # and visibility are now shared, so these no longer alter presentation.
        self._marker_z_offset_m = float(
            self.declare_parameter("visualization.marker_z_offset_m", 0.0).value
        )
        self._hide_unknown_terrain = bool(
            self.declare_parameter(
                "visualization.hide_unknown_terrain", False
            ).value
        )
        self._label_height_m = float(
            self.declare_parameter("visualization.label_height_m", 0.45).value
        )
        self._collision_warning_height_m = float(
            self.declare_parameter("visualization.collision_warning_height_m", 1.0).value
        )
        self._collision_warning_triangle_size_m = float(
            self.declare_parameter("visualization.collision_warning_triangle_size_m", 0.9).value
        )
        self._collision_warning_line_width_m = float(
            self.declare_parameter("visualization.collision_warning_line_width_m", 0.08).value
        )
        self._collision_warning_symbol_scale_m = float(
            self.declare_parameter("visualization.collision_warning_symbol_scale_m", 0.60).value
        )
        self._validate_parameters(steps_topic, markers_topic, reference_path_topic)

        reliable = QoSProfile(depth=10)
        reliable.reliability = ReliabilityPolicy.RELIABLE
        sensor = QoSProfile(depth=10)
        sensor.reliability = ReliabilityPolicy.BEST_EFFORT
        output_qos = QoSProfile(depth=1)
        output_qos.reliability = ReliabilityPolicy.RELIABLE
        output_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self._steps_publisher = self.create_publisher(DiagnosticArray, steps_topic, output_qos)
        self._markers_publisher = self.create_publisher(MarkerArray, markers_topic, output_qos)
        self._reference_path_publisher = self.create_publisher(
            Path, reference_path_topic, output_qos
        )
        self._trajectory_subscription = self.create_subscription(
            Trajectory,
            self._trajectory_topic,
            self._on_trajectory,
            reliable,
        )
        self._geometry_subscription = self.create_subscription(
            GeometryArray,
            self._geometry_topic,
            self._on_geometry,
            sensor,
        )
        self._prediction_subscription = self.create_subscription(
            PredictionOutput,
            self._prediction_topic,
            self._on_prediction,
            reliable,
        )
        self._predictions = {}
        self._snapshots = {}
        self._displayed_cycle = -1
        self._display_key = None
        self._display_signature = None
        self._rendered_mask = 0
        self._invalid_before_ns = -1
        self._invalid_cycles = {}
        self._prediction_ready = False
        self._trajectories = {}
        self._geometries = {}
        self._snapshot_subscription = self.create_subscription(
            TrajectoryPath, snapshot_topic, self._on_path_snapshot,
            QoSProfile(depth=20, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self._last_warning_monotonic = 0.0
        self._status_subscription = self.create_subscription(
            DiagnosticArray, '/prediction/diagnostics', self._on_status, reliable
        )
        self._presentation_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._presentation_timer = self.create_timer(
            0.25, self._flush_pending, clock=self._presentation_clock
        )
        self._time_jump_handle = self.get_clock().create_jump_callback(
            JumpThreshold(
                min_forward=None,
                min_backward=Duration(nanoseconds=-1),
                on_clock_change=True,
            ),
            post_callback=self._on_time_jump,
        )
        self.get_logger().info(
            "canonical prediction visualization: %s + %s + %s -> %s + %s"
            % (
                self._trajectory_topic,
                self._geometry_topic,
                self._prediction_topic,
                markers_topic,
                reference_path_topic,
            )
        )

    def _validate_parameters(
        self,
        steps_topic: str,
        markers_topic: str,
        reference_path_topic: str,
    ) -> None:
        topics = (
            self._trajectory_topic,
            self._geometry_topic,
            self._prediction_topic,
            steps_topic,
            markers_topic,
            reference_path_topic,
            self._map_frame,
        )
        if any(not value.strip() for value in topics):
            raise ValueError("prediction topics and map frame must not be empty")
        values = (
            self._slope_warning_deg,
            self._slope_critical_deg,
            self._normal_length_m,
            self._marker_z_offset_m,
            self._label_height_m,
            self._collision_warning_height_m,
            self._collision_warning_triangle_size_m,
            self._collision_warning_line_width_m,
            self._collision_warning_symbol_scale_m,
        )
        if not all(math.isfinite(value) and value >= 0.0 for value in values):
            raise ValueError("visualization values must be finite and non-negative")
        if self._normal_length_m <= 0.0:
            raise ValueError("normal vector length must be positive")
        if (
            self._collision_warning_triangle_size_m <= 0.0
            or self._collision_warning_line_width_m <= 0.0
            or self._collision_warning_symbol_scale_m <= 0.0
        ):
            raise ValueError("collision warning dimensions must be positive")
        if self._slope_critical_deg < self._slope_warning_deg:
            raise ValueError("critical slope must be at least warning slope")

    def _on_trajectory(self, message: Trajectory) -> None:
        self._trajectories[int(message.trajectory_id)] = message
        self._trim_cache(self._trajectories)
        self._flush_pending()

    def _on_geometry(self, message: GeometryArray) -> None:
        self._geometries[int(message.source_trajectory_id)] = message
        self._trim_cache(self._geometries)
        self._flush_pending()

    def _on_prediction(self, message: PredictionOutput) -> None:
        if message.header.frame_id != self._map_frame:
            self._warn('Prediction frame mismatch')
            return
        if (stamp_ns(message.header.stamp) <= self._invalid_before_ns
                or stamp_ns(message.header.stamp) < self._invalid_cycles.get(
                    int(message.source_trajectory_id), -1,
                )):
            return
        self._predictions[int(message.source_trajectory_id)] = message
        self._trim_cache(self._predictions)
        self._prediction_ready = True
        self._flush_pending()

    def _on_path_snapshot(self, message: TrajectoryPath) -> None:
        try:
            validate_snapshot(message, self._map_frame)
        except ValueError as error:
            self._warn(str(error))
            return
        self._snapshots[int(message.trajectory_id)] = message
        self._trim_cache(self._snapshots)
        self._flush_pending()

    def _now_ns(self):
        return self.get_clock().now().nanoseconds

    def _fresh(self, snapshot, max_age):
        age = self._now_ns() - stamp_ns(snapshot.header.stamp)
        return -200_000_000 <= age <= max_age * 1e9

    def _fresh_geometry(self, cycle):
        geometry = self._geometries.get(cycle)
        return geometry if geometry is not None and self._fresh(
            geometry, self._max_geometry_age,
        ) else None

    def _flush_pending(self) -> None:
        available = {
            k for k, snapshot in self._snapshots.items()
            if k in self._trajectories and k >= self._displayed_cycle
            and self._fresh(snapshot, self._max_path_age)
        }
        complete = {
            k for k in available if self._prediction_ready
            and k in self._predictions and k in self._geometries
            and self._fresh(self._snapshots[k], self._max_prediction_age)
        }
        with_geometry = {k for k in available if self._fresh_geometry(k) is not None}
        if not available:
            if self._display_key is not None:
                self._clear_presentation()
            return
        # Keep a complete cycle together while it remains fresh. If prediction
        # is unavailable, the newest source path is still drawn without evidence.
        cycle = max(complete or with_geometry or available)
        evaluated = cycle in complete
        requested = int(self._steps_publisher.get_subscription_count() > 0) | (
            int(self._markers_publisher.get_subscription_count() > 0) << 1
        ) | (
            int(self._reference_path_publisher.get_subscription_count() > 0) << 2
        )
        key = (cycle, evaluated)
        signature = (key, id(self._fresh_geometry(cycle)),
                     id(self._predictions.get(cycle)) if evaluated else None)
        rendered = self._rendered_mask if signature == self._display_signature else 0
        if requested == 0 or rendered & requested == requested:
            return
        if self._render_cycle(cycle, evaluated):
            self._displayed_cycle = cycle
            self._display_key = key
            self._display_signature = signature
            self._rendered_mask = rendered | requested
        else:
            # A malformed newer cycle must not freeze an old warning on screen.
            self._snapshots.pop(cycle, None)
            self._flush_pending()

    def _on_status(self, message: DiagnosticArray) -> None:
        for status in message.status:
            if status.name == 'prediction_node' and status.level != DiagnosticStatus.OK:
                values = {v.key: v.value for v in status.values}
                if 'trajectory_id' in values:
                    try:
                        cycle = int(values['trajectory_id'])
                    except ValueError:
                        continue
                    # A new cycle waiting for asynchronous inputs does not
                    # invalidate a different, still-fresh completed cycle.
                    self._invalid_cycles[cycle] = max(
                        self._invalid_cycles.get(cycle, -1), stamp_ns(message.header.stamp),
                    )
                    self._trim_cache(self._invalid_cycles)
                    previous = self._predictions.get(cycle)
                    if previous is not None and stamp_ns(previous.header.stamp) <= (
                        self._invalid_cycles[cycle]
                    ):
                        self._predictions.pop(cycle)
                    self._flush_pending()
                    continue
                if self._predictions:
                    latest = self._predictions[max(self._predictions)].header.stamp
                    stamp = message.header.stamp
                    if (stamp.sec, stamp.nanosec) < (latest.sec, latest.nanosec):
                        return
                self._prediction_ready = False
                self._invalid_before_ns = max(
                    self._invalid_before_ns, stamp_ns(message.header.stamp),
                )
                self._predictions.clear()
                self._flush_pending()

    def _render_cycle(self, cycle_id, evaluated):
        trajectory = self._trajectories[cycle_id]
        snapshot = self._snapshots[cycle_id]
        try:
            mapping = validate_snapshot(snapshot, self._map_frame, trajectory)
        except ValueError as error:
            self._warn(str(error))
            return False
        # Terrain evidence remains useful when object/depth data is waiting.
        # It keeps its own source timestamp and expires independently.
        geometry = self._fresh_geometry(cycle_id)
        if geometry is not None and (
            geometry.header.frame_id != self._map_frame
            or geometry.source_trajectory_stamp != trajectory.header.stamp
        ):
            self._warn('Geometry does not match source path timestamp/frame')
            return False
        message = self._predictions[cycle_id] if evaluated else PredictionOutput()
        predictions = self._join(trajectory, geometry, message, snapshot, mapping)
        # Presentation products share the source trajectory timestamp.  The
        # runtime output stamp represents completion time and can lag the raw
        # future path by one or more SVO frames.
        header = Header(stamp=trajectory.header.stamp, frame_id=self._map_frame)
        if self._steps_publisher.get_subscription_count():
            self._steps_publisher.publish(
                predictions_to_diagnostics(
                    predictions,
                    header,
                    slope_warning_deg=self._slope_warning_deg,
                    slope_critical_deg=self._slope_critical_deg,
                    rollover_warning_margin=self._rollover_warning_margin,
                    rollover_critical_margin=self._rollover_critical_margin,
                )
            )
        if self._markers_publisher.get_subscription_count():
            self._markers_publisher.publish(
                predictions_to_markers(
                    predictions,
                    header,
                    slope_warning_deg=self._slope_warning_deg,
                    slope_critical_deg=self._slope_critical_deg,
                    rollover_warning_margin=self._rollover_warning_margin,
                    rollover_critical_margin=self._rollover_critical_margin,
                    normal_length_m=self._normal_length_m,
                    marker_z_offset_m=0.0,
                    label_height_m=self._label_height_m,
                    collision_warning_height_m=self._collision_warning_height_m,
                    collision_warning_triangle_size_m=(self._collision_warning_triangle_size_m),
                    collision_warning_line_width_m=(self._collision_warning_line_width_m),
                    collision_warning_symbol_scale_m=(self._collision_warning_symbol_scale_m),
                    hide_unknown_terrain=False,
                )
            )
        if self._reference_path_publisher.get_subscription_count():
            self._reference_path_publisher.publish(
                Path(header=snapshot.header, poses=snapshot.poses)
            )
        return True

    def _join(
        self,
        trajectory: Trajectory,
        geometry: GeometryArray | None,
        output: PredictionOutput,
        snapshot: TrajectoryPath,
        mapping: dict,
    ) -> list[StepPrediction]:
        geometry_by_id = {
            int(item.step_id): item for item in ([] if geometry is None else geometry.steps)
        }
        rollover_by_id = {int(item.step_id): item for item in output.rollover_steps}
        collision_by_id = {int(item.step_id): item for item in output.collision_steps}
        predictions = []
        distances = [0.0]
        for first, second in zip(snapshot.poses, snapshot.poses[1:]):
            a, b = first.pose.position, second.pose.position
            distances.append(distances[-1] + math.hypot(b.x - a.x, b.y - a.y))
        for step_index, step in enumerate(trajectory.steps, start=1):
            source_index = mapping[step.step_id]
            source_pose = snapshot.poses[source_index]
            position = source_pose.pose.position
            terrain = self._terrain_for_step(geometry_by_id.get(int(step.step_id)))
            collision = collision_by_id.get(int(step.step_id))
            candidates = [] if collision is None else collision.collision_objects
            objects = intersecting_collision_objects(candidates)
            rollover = rollover_by_id.get(int(step.step_id))
            moment = None if rollover is None else rollover.stability_moment
            zmp = None if rollover is None else rollover.zmp
            clearance = (
                math.inf
                if not candidates
                else min(float(item.min_distance_m) for item in candidates)
            )
            predictions.append(
                StepPrediction(
                    step_index=step_index,
                    source_pose_index=source_index,
                    position_xyz=(position.x, position.y, position.z),
                    distance_from_start_m=distances[source_index],
                    time_from_start_sec=(stamp_ns(source_pose.header.stamp)
                                         - stamp_ns(snapshot.header.stamp)) / 1e9,
                    terrain=terrain,
                    object_data_available=collision is not None,
                    object_collision=bool(objects),
                    object_ids=tuple(str(item.track_id) for item in objects),
                    nearest_object_clearance_m=clearance,
                    rover_yaw_rad=float(step.yaw),
                    predicted_roll_deg=(
                        math.nan if rollover is None else float(rollover.predicted_roll_deg)
                    ),
                    predicted_pitch_deg=(
                        math.nan if rollover is None else float(rollover.predicted_pitch_deg)
                    ),
                    static_stability_margin_m=(
                        math.nan if rollover is None else float(rollover.static_stability_margin_m)
                    ),
                    normalized_static_stability_margin=(
                        math.nan
                        if rollover is None
                        else float(rollover.normalized_static_stability_margin)
                    ),
                    dynamic_state_available=bool(
                        moment is not None and moment.acceleration_available
                    ),
                    stability_moment_valid=bool(moment is not None and moment.valid),
                    minimum_stability_moment_nm=(
                        math.nan
                        if moment is None or not moment.valid
                        else float(moment.minimum_stability_moment_nm)
                    ),
                    normalized_minimum_stability_moment=(
                        math.nan
                        if moment is None or not moment.valid
                        else float(moment.normalized_minimum_stability_moment)
                    ),
                    minimum_normalized_moment_edge=(
                        ""
                        if moment is None or not moment.valid
                        else str(moment.minimum_normalized_moment_edge)
                    ),
                    zmp_valid=bool(zmp is not None and zmp.valid),
                    zmp_xy=(
                        None if zmp is None or not zmp.valid else (float(zmp.x), float(zmp.y))
                    ),
                    zmp_margin_m=(
                        math.nan if zmp is None or not zmp.valid else float(zmp.margin_m)
                    ),
                    normalized_zmp_margin=(
                        math.nan if zmp is None or not zmp.valid else float(zmp.normalized_margin)
                    ),
                    nearest_zmp_edge=(
                        "" if zmp is None or not zmp.valid else str(zmp.nearest_edge)
                    ),
                )
            )
        return predictions

    def _terrain_for_step(self, geometry_step):
        if geometry_step is None:
            return TerrainSample(valid=False)
        vector = (
            float(geometry_step.normal.x),
            float(geometry_step.normal.y),
            float(geometry_step.normal.z),
        )
        norm = math.sqrt(sum(value * value for value in vector))
        if norm <= 1e-12 or not all(math.isfinite(value) for value in vector):
            return TerrainSample(valid=False)
        normal = tuple(value / norm for value in vector)
        if normal[2] < 0.0:
            normal = tuple(-value for value in normal)
        slope = math.degrees(math.acos(max(-1.0, min(1.0, normal[2]))))
        elevation = (
            float(geometry_step.elevation_m)
            if geometry_step.elevation_valid
            and math.isfinite(float(geometry_step.elevation_m))
            else 0.0
        )
        return TerrainSample(
            valid=True,
            elevation_m=elevation,
            slope_deg=slope,
            normal_xyz=normal,
        )

    @staticmethod
    def _trim_cache(cache) -> None:
        while len(cache) > 64:
            del cache[next(iter(cache))]

    def _on_time_jump(self, _time_jump) -> None:
        self._predictions.clear()
        self._snapshots.clear()
        self._displayed_cycle = -1
        self._invalid_before_ns = -1
        self._invalid_cycles.clear()
        self._prediction_ready = False
        self._trajectories.clear()
        self._geometries.clear()
        self._clear_presentation()

    def _clear_presentation(self):
        self._display_key = None
        self._display_signature = None
        self._rendered_mask = 0
        header = Header(
            stamp=self.get_clock().now().to_msg(),
            frame_id=self._map_frame,
        )
        self._steps_publisher.publish(DiagnosticArray(header=header))
        self._reference_path_publisher.publish(Path(header=header))
        self._markers_publisher.publish(
            MarkerArray(
                markers=[
                    Marker(
                        header=header,
                        action=Marker.DELETEALL,
                    )
                ]
            )
        )

    def _warn(self, message: str) -> None:
        now = time.monotonic()
        if now - self._last_warning_monotonic >= 5.0:
            self.get_logger().warning(message)
            self._last_warning_monotonic = now


def main(args=None) -> None:
    """Run the canonical prediction visualizer node."""
    rclpy.init(args=args)
    node = None
    try:
        node = CanonicalPredictionVisualizerNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
