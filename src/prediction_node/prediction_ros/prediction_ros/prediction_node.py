"""Typed ROS 2 runtime node wrapping frozen PredictionRuntime / PredictionCore."""

from __future__ import annotations

import logging
from pathlib import Path

from prediction_core.config import load_config
from prediction_core.runtime import PredictionRuntime
from prediction_core.validation import PredictionProfile

from .adapters import RosAdapters

LOGGER = logging.getLogger(__name__)


class PredictionNode:
    """Thin typed-message ROS node; orchestration and physics stay in prediction_core."""

    def __init__(self) -> None:
        try:
            import rclpy
            from rclpy.node import Node
            from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
            from safety_perception_msgs.msg import (
                CollisionObject,
                CollisionStep,
                ExternalWrenchArray,
                GeometryArray,
                PredictionOutput,
                RoverState,
                RolloverStep,
                StabilityMomentEvidence,
                TrackedObjectArray,
                Trajectory,
                ZmpEvidence,
            )
        except ImportError as exc:  # pragma: no cover - needs built ROS workspace
            raise RuntimeError(
                "prediction_node requires rclpy and built safety_perception_msgs"
            ) from exc

        self._rclpy = rclpy
        self._types = {
            "trajectory": Trajectory,
            "objects": TrackedObjectArray,
            "geometry": GeometryArray,
            "state": RoverState,
            "wrenches": ExternalWrenchArray,
            "output": PredictionOutput,
            "collision_step": CollisionStep,
            "collision_object": CollisionObject,
            "rollover_step": RolloverStep,
            "stability_moment": StabilityMomentEvidence,
            "zmp": ZmpEvidence,
        }
        self._QoSProfile = QoSProfile
        self._ReliabilityPolicy = ReliabilityPolicy
        self._HistoryPolicy = HistoryPolicy

        class _PredictionNodeImpl(Node):
            def __init__(self, outer: "PredictionNode") -> None:
                super().__init__("prediction_node")
                outer._configure(self)

        self._node = _PredictionNodeImpl(self)

    def _configure(self, node) -> None:
        defaults = {
            "trajectory_topic": "/trajectory",
            "tracked_objects_topic": "/tracked_objects",
            "geometry_topic": "/geometry",
            "state_topic": "/rover/state",
            "external_wrench_topic": "/external_wrenches",
            "prediction_output_topic": "/predict_output",
            "expected_frame_id": "map",
            "config_path": "",
            "prediction_profile": "static",
            "require_full_geometry_coverage": False,
            "max_object_age_sec": -1.0,
            "max_geometry_age_sec": -1.0,
            "max_state_age_sec": -1.0,
        }
        for name, default in defaults.items():
            node.declare_parameter(name, default)

        config_path = node.get_parameter("config_path").value
        if not config_path:
            raise ValueError("config_path parameter is required")

        profile_raw = str(node.get_parameter("prediction_profile").value).strip().lower()
        try:
            profile = PredictionProfile(profile_raw)
        except ValueError as exc:
            raise ValueError(
                f"prediction_profile must be 'static' or 'dynamic', got {profile_raw!r}"
            ) from exc

        self.runtime = PredictionRuntime(
            load_config(Path(config_path)),
            profile=profile,
            expected_frame_id=node.get_parameter("expected_frame_id").value,
            require_full_geometry_coverage=node.get_parameter(
                "require_full_geometry_coverage"
            ).value,
            max_object_age_sec=self._optional_age(node, "max_object_age_sec"),
            max_geometry_age_sec=self._optional_age(node, "max_geometry_age_sec"),
            max_state_age_sec=self._optional_age(node, "max_state_age_sec"),
            logger=LOGGER.info,
        )
        node.get_logger().info(f"PredictionRuntime profile={profile.value}")

        reliable = self._QoSProfile(
            reliability=self._ReliabilityPolicy.RELIABLE,
            history=self._HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        sensor = self._QoSProfile(
            reliability=self._ReliabilityPolicy.BEST_EFFORT,
            history=self._HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        topic = lambda name: node.get_parameter(name).value
        node.create_subscription(
            self._types["trajectory"], topic("trajectory_topic"), self._trajectory_callback, reliable
        )
        node.create_subscription(
            self._types["objects"], topic("tracked_objects_topic"), self._objects_callback, sensor
        )
        node.create_subscription(
            self._types["geometry"], topic("geometry_topic"), self._geometry_callback, sensor
        )
        node.create_subscription(
            self._types["state"], topic("state_topic"), self._state_callback, sensor
        )
        node.create_subscription(
            self._types["wrenches"],
            topic("external_wrench_topic"),
            self._external_wrench_callback,
            sensor,
        )
        self._publisher = node.create_publisher(
            self._types["output"], topic("prediction_output_topic"), reliable
        )
        self._node = node

    @staticmethod
    def _optional_age(node, name: str) -> float | None:
        value = node.get_parameter(name).value
        return None if value < 0 else value

    def _publish_if_ready(self, result) -> None:
        for message in result.messages:
            LOGGER.info("%s", message)
        if result.output is None or result.cycle_key is None:
            return
        self._publisher.publish(
            RosAdapters.prediction_to_ros(
                result.output,
                source_trajectory_id=result.cycle_key.trajectory_id,
                frame_id=result.cycle_key.frame_id,
                output_type=self._types["output"],
                collision_step_type=self._types["collision_step"],
                collision_object_type=self._types["collision_object"],
                rollover_step_type=self._types["rollover_step"],
                stability_moment_type=self._types["stability_moment"],
                zmp_type=self._types["zmp"],
            )
        )

    def _trajectory_callback(self, msg) -> None:
        try:
            result = self.runtime.on_trajectory(
                RosAdapters.trajectory_from_ros(msg),
                trajectory_id=int(msg.trajectory_id),
            )
            self._publish_if_ready(result)
        except Exception:
            LOGGER.exception("trajectory callback failed")

    def _objects_callback(self, msg) -> None:
        try:
            result = self.runtime.on_objects(
                RosAdapters.objects_from_ros(msg),
                frame_id=msg.header.frame_id,
            )
            self._publish_if_ready(result)
        except Exception:
            LOGGER.exception("tracked objects callback failed")

    def _geometry_callback(self, msg) -> None:
        try:
            result = self.runtime.on_geometry(
                RosAdapters.geometry_from_ros(msg),
                frame_id=msg.header.frame_id,
                source_trajectory_id=int(msg.source_trajectory_id),
                source_trajectory_stamp=RosAdapters.timestamp(msg.source_trajectory_stamp),
            )
            self._publish_if_ready(result)
        except Exception:
            LOGGER.exception("geometry callback failed")

    def _state_callback(self, msg) -> None:
        try:
            result = self.runtime.on_state(
                RosAdapters.state_from_ros(msg),
                frame_id=msg.header.frame_id,
            )
            self._publish_if_ready(result)
        except Exception:
            LOGGER.exception("state callback failed")

    def _external_wrench_callback(self, msg) -> None:
        try:
            result = self.runtime.on_external_wrenches(
                RosAdapters.external_wrenches_from_ros(msg),
                frame_id=msg.header.frame_id,
            )
            self._publish_if_ready(result)
        except Exception:
            LOGGER.exception("external wrench callback failed")

    def spin(self) -> None:
        self._rclpy.spin(self._node)

    def destroy(self) -> None:
        self._node.destroy_node()


def main(argv: list[str] | None = None) -> None:
    import rclpy

    logging.basicConfig(level=logging.INFO)
    rclpy.init(args=argv)
    node = PredictionNode()
    try:
        node.spin()
    finally:
        node.destroy()
        rclpy.shutdown()
