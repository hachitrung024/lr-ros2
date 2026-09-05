"""Convert canonical PredictionOutput evidence into diagnostics and RViz."""

from __future__ import annotations

import math
import time

from diagnostic_msgs.msg import DiagnosticArray
from grid_map_msgs.msg import GridMap
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
)
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

from lr_terrain_geometry.grid_map_sampling import GridMapSampler, TerrainSample
from .presentation import StepPrediction
from .outputs import predictions_to_diagnostics, predictions_to_markers


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
        self._terrain_topic = str(
            self.declare_parameter("input.terrain_topic", "/terrain_geometry/grid_map").value
        )
        steps_topic = str(
            self.declare_parameter("output.steps_topic", "/lr/path_prediction/steps").value
        )
        markers_topic = str(
            self.declare_parameter("output.markers_topic", "/lr/path_prediction/markers").value
        )
        self._map_frame = str(self.declare_parameter("frames.map_frame", "map").value).strip()
        self._slope_warning_deg = float(self.declare_parameter("warning.slope_deg", 20.0).value)
        self._slope_critical_deg = float(self.declare_parameter("critical.slope_deg", 30.0).value)
        self._normal_length_m = float(
            self.declare_parameter("visualization.normal_length_m", 0.8).value
        )
        self._marker_z_offset_m = float(
            self.declare_parameter("visualization.marker_z_offset_m", 0.10).value
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
        self._validate_parameters(steps_topic, markers_topic)

        reliable = QoSProfile(depth=10)
        reliable.reliability = ReliabilityPolicy.RELIABLE
        sensor = QoSProfile(depth=10)
        sensor.reliability = ReliabilityPolicy.BEST_EFFORT
        terrain_qos = QoSProfile(depth=1)
        terrain_qos.reliability = ReliabilityPolicy.RELIABLE
        terrain_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        output_qos = QoSProfile(depth=1)
        output_qos.reliability = ReliabilityPolicy.RELIABLE
        output_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self._steps_publisher = self.create_publisher(DiagnosticArray, steps_topic, output_qos)
        self._markers_publisher = self.create_publisher(MarkerArray, markers_topic, output_qos)
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
        self._terrain_subscription = self.create_subscription(
            GridMap,
            self._terrain_topic,
            self._on_terrain,
            terrain_qos,
        )

        self._predictions = {}
        self._rendered = {}
        self._prediction_ready = False
        self._trajectories = {}
        self._geometries = {}
        self._terrain_sampler = None
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
            "canonical prediction visualization: %s + %s + %s -> %s"
            % (
                self._trajectory_topic,
                self._geometry_topic,
                self._prediction_topic,
                markers_topic,
            )
        )

    def _validate_parameters(self, steps_topic: str, markers_topic: str) -> None:
        topics = (
            self._trajectory_topic,
            self._geometry_topic,
            self._prediction_topic,
            self._terrain_topic,
            steps_topic,
            markers_topic,
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

    def _on_terrain(self, message: GridMap) -> None:
        try:
            self._terrain_sampler = GridMapSampler(message)
        except ValueError as error:
            self._warn(f"Ignoring malformed terrain GridMap: {error}")

    def _on_prediction(self, message: PredictionOutput) -> None:
        if message.header.frame_id != self._map_frame:
            self._warn('Prediction frame mismatch')
            return
        self._predictions[int(message.source_trajectory_id)] = message
        self._trim_cache(self._predictions)
        self._rendered = {
            key: value for key, value in self._rendered.items() if key in self._predictions
        }
        self._prediction_ready = True
        self._flush_pending()

    def _flush_pending(self) -> None:
        if not self._prediction_ready or not self._predictions:
            return
        cycle = max(self._predictions)
        if cycle not in self._trajectories or cycle not in self._geometries:
            return
        requested = int(self._steps_publisher.get_subscription_count() > 0) | (
            int(self._markers_publisher.get_subscription_count() > 0) << 1
        )
        rendered = self._rendered.get(cycle, 0)
        if requested == 0 or rendered & requested == requested:
            return
        self._render_prediction(self._predictions[cycle])
        self._rendered[cycle] = rendered | requested

    def _on_status(self, message: DiagnosticArray) -> None:
        for status in message.status:
            if status.name == 'prediction_node' and status.level != 0:
                if self._predictions:
                    latest = self._predictions[max(self._predictions)].header.stamp
                    stamp = message.header.stamp
                    if (stamp.sec, stamp.nanosec) < (latest.sec, latest.nanosec):
                        return
                self._prediction_ready = False
                self._clear_presentation()

    def _render_prediction(self, message: PredictionOutput) -> None:
        cycle_id = int(message.source_trajectory_id)
        trajectory = self._trajectories.get(cycle_id)
        if trajectory is None:
            self._warn(f"Prediction cycle {cycle_id} has no cached trajectory")
            return
        geometry = self._geometries.get(cycle_id)
        if message.header.frame_id != self._map_frame:
            self._warn(
                f"Prediction frame '{message.header.frame_id}' does not match "
                f"'{self._map_frame}'"
            )
            return
        predictions = self._join(trajectory, geometry, message)
        header = Header(
            stamp=message.header.stamp,
            frame_id=self._map_frame,
        )
        if self._steps_publisher.get_subscription_count():
            self._steps_publisher.publish(
                predictions_to_diagnostics(
                    predictions,
                    header,
                    slope_warning_deg=self._slope_warning_deg,
                    slope_critical_deg=self._slope_critical_deg,
                )
            )
        if self._markers_publisher.get_subscription_count():
            self._markers_publisher.publish(
                predictions_to_markers(
                    predictions,
                    header,
                    slope_warning_deg=self._slope_warning_deg,
                    slope_critical_deg=self._slope_critical_deg,
                    normal_length_m=self._normal_length_m,
                    marker_z_offset_m=self._marker_z_offset_m,
                    label_height_m=self._label_height_m,
                    collision_warning_height_m=self._collision_warning_height_m,
                    collision_warning_triangle_size_m=(self._collision_warning_triangle_size_m),
                    collision_warning_line_width_m=(self._collision_warning_line_width_m),
                    collision_warning_symbol_scale_m=(self._collision_warning_symbol_scale_m),
                )
            )

    def _join(
        self,
        trajectory: Trajectory,
        geometry: GeometryArray | None,
        output: PredictionOutput,
    ) -> list[StepPrediction]:
        geometry_by_id = {
            int(item.step_id): item for item in ([] if geometry is None else geometry.steps)
        }
        rollover_by_id = {int(item.step_id): item for item in output.rollover_steps}
        collision_by_id = {int(item.step_id): item for item in output.collision_steps}
        predictions = []
        distance = 0.0
        previous_xy = None
        for step_index, step in enumerate(trajectory.steps, start=1):
            xy = (float(step.x), float(step.y))
            if previous_xy is not None:
                distance += math.hypot(xy[0] - previous_xy[0], xy[1] - previous_xy[1])
            previous_xy = xy
            terrain = self._terrain_for_step(xy[0], xy[1], geometry_by_id.get(int(step.step_id)))
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
            elevation = terrain.elevation_m if terrain.valid else 0.0
            predictions.append(
                StepPrediction(
                    step_index=step_index,
                    source_pose_index=int(step.step_id),
                    position_xyz=(xy[0], xy[1], elevation),
                    distance_from_start_m=distance,
                    time_from_start_sec=math.nan,
                    terrain=terrain,
                    object_data_available=True,
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

    def _terrain_for_step(self, x_value, y_value, geometry_step):
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
        elevation = 0.0
        if self._terrain_sampler is not None:
            sampled = self._terrain_sampler.sample(x_value, y_value)
            if sampled.valid:
                elevation = sampled.elevation_m
        return TerrainSample(
            valid=True,
            elevation_m=elevation,
            slope_deg=slope,
            normal_xyz=normal,
        )

    @staticmethod
    def _trim_cache(cache) -> None:
        while len(cache) > 20:
            del cache[next(iter(cache))]

    def _on_time_jump(self, _time_jump) -> None:
        self._predictions.clear()
        self._rendered.clear()
        self._prediction_ready = False
        self._trajectories.clear()
        self._geometries.clear()
        self._terrain_sampler = None
        self._clear_presentation()

    def _clear_presentation(self):
        header = Header(
            stamp=self.get_clock().now().to_msg(),
            frame_id=self._map_frame,
        )
        self._steps_publisher.publish(DiagnosticArray(header=header))
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
