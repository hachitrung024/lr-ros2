"""ROS 2 node predicting terrain and obstacle risk for 20 future steps."""

from __future__ import annotations

import math
import time

from diagnostic_msgs.msg import DiagnosticArray
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import Path
import rclpy
from rclpy.clock import JumpThreshold
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Header
from vision_msgs.msg import Detection3DArray
from visualization_msgs.msg import Marker, MarkerArray

from .core import GridMapSampler, obstacles_from_message, predict_steps
from .outputs import predictions_to_diagnostics, predictions_to_markers


class PathRiskPredictorNode(Node):
    """Fuse a future path, terrain map, and static tracked object boxes."""

    def __init__(self, **node_kwargs) -> None:
        """Declare configuration and connect all prediction topics."""
        super().__init__("path_risk_predictor", **node_kwargs)
        self._path_topic = str(self.declare_parameter(
            "input.path_topic", "/lr/future_path/ground_truth"
        ).value)
        self._terrain_topic = str(self.declare_parameter(
            "input.terrain_topic", "/terrain_geometry/grid_map"
        ).value)
        self._objects_topic = str(self.declare_parameter(
            "input.objects_topic", "/segmentation/boxes_3d"
        ).value)
        steps_topic = str(self.declare_parameter(
            "output.steps_topic", "/lr/path_prediction/steps"
        ).value)
        markers_topic = str(self.declare_parameter(
            "output.markers_topic", "/lr/path_prediction/markers"
        ).value)
        self._map_frame = str(self.declare_parameter(
            "frames.map_frame", "map"
        ).value).strip()
        self._step_count = int(self.declare_parameter(
            "prediction.step_count", 20
        ).value)
        self._path_stride = int(self.declare_parameter(
            "prediction.path_stride", 1
        ).value)
        self._slope_warning_deg = float(self.declare_parameter(
            "warning.slope_deg", 20.0
        ).value)
        self._slope_critical_deg = float(self.declare_parameter(
            "critical.slope_deg", 30.0
        ).value)
        self._rover_radius_m = float(self.declare_parameter(
            "collision.rover_radius_m", 0.55
        ).value)
        self._collision_margin_m = float(self.declare_parameter(
            "collision.margin_m", 0.15
        ).value)
        self._max_terrain_age_sec = float(self.declare_parameter(
            "synchronization.max_terrain_age_sec", 2.0
        ).value)
        self._max_objects_age_sec = float(self.declare_parameter(
            "synchronization.max_objects_age_sec", 1.0
        ).value)
        self._normal_length_m = float(self.declare_parameter(
            "visualization.normal_length_m", 0.8
        ).value)
        self._marker_z_offset_m = float(self.declare_parameter(
            "visualization.marker_z_offset_m", 0.10
        ).value)
        self._label_height_m = float(self.declare_parameter(
            "visualization.label_height_m", 0.45
        ).value)
        self._validate_parameters(steps_topic, markers_topic)

        reliable_qos = QoSProfile(depth=1)
        reliable_qos.reliability = ReliabilityPolicy.RELIABLE
        reliable_qos.durability = DurabilityPolicy.VOLATILE
        terrain_qos = QoSProfile(depth=1)
        terrain_qos.reliability = ReliabilityPolicy.RELIABLE
        terrain_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        object_qos = QoSProfile(depth=1)
        object_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        object_qos.durability = DurabilityPolicy.VOLATILE
        output_qos = QoSProfile(depth=1)
        output_qos.reliability = ReliabilityPolicy.RELIABLE
        output_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self._steps_publisher = self.create_publisher(
            DiagnosticArray,
            steps_topic,
            output_qos,
        )
        self._markers_publisher = self.create_publisher(
            MarkerArray,
            markers_topic,
            output_qos,
        )
        self._path_subscription = self.create_subscription(
            Path,
            self._path_topic,
            self._on_path,
            reliable_qos,
        )
        self._terrain_subscription = self.create_subscription(
            GridMap,
            self._terrain_topic,
            self._on_terrain,
            terrain_qos,
        )
        self._objects_subscription = self.create_subscription(
            Detection3DArray,
            self._objects_topic,
            self._on_objects,
            object_qos,
        )

        self._latest_path = None
        self._terrain_sampler = None
        self._terrain_stamp_ns = 0
        self._terrain_frame = ""
        self._obstacles = []
        self._objects_stamp_ns = 0
        self._objects_frame = ""
        self._objects_received = False
        self._last_warning_monotonic = 0.0
        self._time_jump_handle = self.get_clock().create_jump_callback(
            JumpThreshold(
                min_forward=None,
                min_backward=Duration(nanoseconds=-1),
                on_clock_change=True,
            ),
            post_callback=self._on_time_jump,
        )
        self.get_logger().info(
            "20-step prediction: path=%s terrain=%s objects=%s "
            "steps=%s markers=%s"
            % (
                self._path_topic,
                self._terrain_topic,
                self._objects_topic,
                steps_topic,
                markers_topic,
            )
        )

    def _validate_parameters(
        self,
        steps_topic: str,
        markers_topic: str,
    ) -> None:
        for name, value in (
            ("input.path_topic", self._path_topic),
            ("input.terrain_topic", self._terrain_topic),
            ("input.objects_topic", self._objects_topic),
            ("output.steps_topic", steps_topic),
            ("output.markers_topic", markers_topic),
            ("frames.map_frame", self._map_frame),
        ):
            if not value.strip():
                raise ValueError(f"{name} must not be empty")
        if self._step_count < 1 or self._path_stride < 1:
            raise ValueError(
                "prediction step_count and path_stride must be positive"
            )
        finite_nonnegative = (
            ("warning.slope_deg", self._slope_warning_deg),
            ("critical.slope_deg", self._slope_critical_deg),
            ("collision.rover_radius_m", self._rover_radius_m),
            ("collision.margin_m", self._collision_margin_m),
            ("visualization.marker_z_offset_m", self._marker_z_offset_m),
            ("visualization.label_height_m", self._label_height_m),
        )
        for name, value in finite_nonnegative:
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and non-negative")
        for name, value in (
            (
                "synchronization.max_terrain_age_sec",
                self._max_terrain_age_sec,
            ),
            (
                "synchronization.max_objects_age_sec",
                self._max_objects_age_sec,
            ),
            ("visualization.normal_length_m", self._normal_length_m),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        if self._slope_critical_deg < self._slope_warning_deg:
            raise ValueError(
                "critical.slope_deg must be at least warning.slope_deg"
            )

    def _on_path(self, message: Path) -> None:
        self._latest_path = message
        self._publish_prediction()

    def _on_terrain(self, message: GridMap) -> None:
        try:
            sampler = GridMapSampler(message)
        except ValueError as error:
            self._warn(f"Ignoring malformed terrain GridMap: {error}")
            return
        self._terrain_sampler = sampler
        self._terrain_stamp_ns = _stamp_ns(message.header)
        self._terrain_frame = message.header.frame_id
        self._publish_prediction()

    def _on_objects(self, message: Detection3DArray) -> None:
        self._obstacles = obstacles_from_message(message)
        self._objects_stamp_ns = _stamp_ns(message.header)
        self._objects_frame = message.header.frame_id
        self._objects_received = True
        self._publish_prediction()

    def _publish_prediction(self) -> None:
        path = self._latest_path
        if path is None:
            return
        path_frame = path.header.frame_id
        if not path_frame and path.poses:
            path_frame = path.poses[0].header.frame_id
        if path_frame != self._map_frame:
            self._warn(
                f"Path frame '{path_frame}' does not match "
                f"'{self._map_frame}'"
            )
            self._publish_empty(path.header)
            return

        path_stamp_ns = _stamp_ns(path.header)
        terrain_available = (
            self._terrain_sampler is not None
            and self._terrain_frame == path_frame
            and _is_fresh(
                path_stamp_ns,
                self._terrain_stamp_ns,
                self._max_terrain_age_sec,
            )
        )
        object_data_available = (
            self._objects_received
            and self._objects_frame == path_frame
            and _is_fresh(
                path_stamp_ns,
                self._objects_stamp_ns,
                self._max_objects_age_sec,
            )
        )
        predictions = predict_steps(
            path,
            self._terrain_sampler if terrain_available else None,
            self._obstacles if object_data_available else [],
            object_data_available=object_data_available,
            step_count=self._step_count,
            path_stride=self._path_stride,
            rover_radius_m=self._rover_radius_m,
            collision_margin_m=self._collision_margin_m,
        )
        if len(predictions) < self._step_count:
            self._warn(
                f"Future path provides only {len(predictions)} of "
                f"{self._step_count} requested steps"
            )
        header = Header(stamp=path.header.stamp, frame_id=path_frame)
        self._steps_publisher.publish(predictions_to_diagnostics(
            predictions,
            header,
            slope_warning_deg=self._slope_warning_deg,
            slope_critical_deg=self._slope_critical_deg,
        ))
        self._markers_publisher.publish(predictions_to_markers(
            predictions,
            header,
            slope_warning_deg=self._slope_warning_deg,
            slope_critical_deg=self._slope_critical_deg,
            normal_length_m=self._normal_length_m,
            marker_z_offset_m=self._marker_z_offset_m,
            label_height_m=self._label_height_m,
        ))

    def _publish_empty(self, input_header: Header) -> None:
        header = Header(
            stamp=input_header.stamp,
            frame_id=self._map_frame,
        )
        self._steps_publisher.publish(DiagnosticArray(header=header))
        clear = Marker(header=header, action=Marker.DELETEALL)
        self._markers_publisher.publish(MarkerArray(markers=[clear]))

    def _on_time_jump(self, _time_jump) -> None:
        self._latest_path = None
        self._terrain_sampler = None
        self._terrain_stamp_ns = 0
        self._terrain_frame = ""
        self._obstacles = []
        self._objects_stamp_ns = 0
        self._objects_frame = ""
        self._objects_received = False
        now_header = Header(
            stamp=self.get_clock().now().to_msg(),
            frame_id=self._map_frame,
        )
        self._publish_empty(now_header)
        self.get_logger().info(
            "ROS time changed; cleared path-prediction input caches"
        )

    def _warn(self, message: str) -> None:
        now = time.monotonic()
        if now - self._last_warning_monotonic >= 5.0:
            self.get_logger().warning(message)
            self._last_warning_monotonic = now


def _stamp_ns(header: Header) -> int:
    return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)


def _is_fresh(path_stamp_ns: int, input_stamp_ns: int, limit_sec: float) -> bool:
    if path_stamp_ns <= 0 or input_stamp_ns <= 0:
        return False
    return abs(path_stamp_ns - input_stamp_ns) <= int(limit_sec * 1e9)


def main(args=None) -> None:
    """Run the path risk predictor node."""
    rclpy.init(args=args)
    node = None
    try:
        node = PathRiskPredictorNode()
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
