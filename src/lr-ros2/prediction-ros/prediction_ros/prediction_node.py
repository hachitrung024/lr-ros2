"""Typed ROS 2 runtime node wrapping frozen PredictionRuntime / PredictionCore."""

from __future__ import annotations

import logging
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.clock import JumpThreshold
from rclpy.duration import Duration
from pathlib import Path

from prediction_core.config import load_config
from prediction_core.runtime import PredictionRuntime
from prediction_core.validation import PredictionProfile

from .adapters import RosAdapters
from .input_history import InputHistory, stamp_ns

LOGGER = logging.getLogger(__name__)


class PredictionNode:
    """Thin typed-message ROS node; orchestration and physics stay in prediction_core."""

    def __init__(self, **node_kwargs) -> None:
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
                super().__init__("prediction_node", **node_kwargs)
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
            "max_object_age_sec": 0.5,
            "max_geometry_age_sec": 2.0,
            "max_state_age_sec": 0.25,
            # A new trajectory is a new PredictionCore cycle. Re-inject the
            # latest asynchronous object/state samples after that reset.
            "reuse_latest_inputs_per_cycle": True,
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
            logger=LOGGER.debug,
        )
        self._reuse_latest_inputs = bool(node.get_parameter("reuse_latest_inputs_per_cycle").value)
        self._objects_history = InputHistory(self._optional_age(node, "max_object_age_sec"))
        self._state_history = InputHistory(self._optional_age(node, "max_state_age_sec"))
        self._geometries = {}
        self._active_trajectory = None
        self._injected = {}
        self._last_trajectory_stamp = None
        self._waiting = "waiting: trajectory"
        self._output_count = 0
        self._callback_ms = 0.0
        self._callback_ms_ema = 0.0
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

        def topic(name):
            return node.get_parameter(name).value

        node.create_subscription(
            self._types["trajectory"],
            topic("trajectory_topic"),
            self._trajectory_callback,
            reliable,
        )
        node.create_subscription(
            self._types["objects"],
            topic("tracked_objects_topic"),
            self._objects_callback,
            sensor,
        )
        node.create_subscription(
            self._types["geometry"],
            topic("geometry_topic"),
            self._geometry_callback,
            sensor,
        )
        node.create_subscription(
            self._types["state"],
            topic("state_topic"),
            self._state_callback,
            sensor,
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
        self._diagnostics = node.create_publisher(
            DiagnosticArray, "/prediction/diagnostics", reliable
        )
        self._diagnostic_timer = node.create_timer(1.0, self._publish_diagnostics)
        self._jump_handle = node.get_clock().create_jump_callback(
            JumpThreshold(
                min_forward=None, min_backward=Duration(nanoseconds=-1), on_clock_change=True
            ),
            post_callback=self._on_time_jump,
        )

    @staticmethod
    def _optional_age(node, name: str) -> float | None:
        value = node.get_parameter(name).value
        return None if value < 0 else value

    def _publish_if_ready(self, result) -> None:
        if result.output is None or result.cycle_key is None:
            return
        message = RosAdapters.prediction_to_ros(
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
        # PredictionCore uses wall time internally. ROS output must remain in
        # the node clock domain so SVO/sim-time consumers can synchronize it.
        message.header.stamp = self._node.get_clock().now().to_msg()
        self._publisher.publish(message)
        self._output_count += 1

    def _on_time_jump(self, _jump) -> None:
        self.runtime.reset()
        self._objects_history.clear()
        self._state_history.clear()
        self._geometries.clear()
        self._active_trajectory = None
        self._last_trajectory_stamp = None
        self._injected.clear()
        self._waiting = "waiting: trajectory after ROS time change"
        self._publish_diagnostics()

    def _publish_diagnostics(self) -> None:
        if not self._node.context.ok():
            return
        if self._active_trajectory is not None:
            age_ns = self._node.get_clock().now().nanoseconds - stamp_ns(
                self._active_trajectory.header.stamp
            )
            if age_ns > 500_000_000:
                self._waiting = "waiting: stale trajectory"
        message = DiagnosticArray()
        message.header.stamp = self._node.get_clock().now().to_msg()
        message.status = [
            DiagnosticStatus(
                name="prediction_node",
                level=DiagnosticStatus.OK if self._waiting == "ready" else DiagnosticStatus.WARN,
                message=self._waiting,
                values=[
                    KeyValue(key="output_count", value=str(self._output_count)),
                    KeyValue(key="callback_ms", value=str(self._callback_ms)),
                    KeyValue(key="callback_ms_ema", value=str(self._callback_ms_ema)),
                ],
            )
        ]
        self._diagnostics.publish(message)

    def _trajectory_callback(self, msg) -> None:
        try:
            stamp = stamp_ns(msg.header.stamp)
            if self._last_trajectory_stamp is not None and stamp < self._last_trajectory_stamp:
                self._on_time_jump(None)
            self._last_trajectory_stamp = stamp
            if not self._reuse_latest_inputs and self._active_trajectory is not None:
                self._objects_history.clear()
                self._state_history.clear()
            self._active_trajectory = msg
            self._injected.clear()
            self.runtime.on_trajectory(
                RosAdapters.trajectory_from_ros(msg), trajectory_id=int(msg.trajectory_id)
            )
            self._refresh_inputs()
        except Exception:
            LOGGER.exception("trajectory callback failed")

    def _objects_callback(self, msg) -> None:
        self._objects_history.append(msg)
        self._refresh_inputs()

    def _geometry_callback(self, msg) -> None:
        key = (
            msg.header.frame_id,
            int(msg.source_trajectory_id),
            stamp_ns(msg.source_trajectory_stamp),
        )
        self._geometries[key] = msg
        while len(self._geometries) > 20:
            del self._geometries[next(iter(self._geometries))]
        self._refresh_inputs()

    def _state_callback(self, msg) -> None:
        self._state_history.append(msg)
        self._refresh_inputs()

    def _refresh_inputs(self) -> None:
        trajectory = self._active_trajectory
        if trajectory is None:
            return
        started = time.monotonic()
        try:
            # Histories prevent a fast state stream from replacing a valid
            # cycle sample with a newer, future observation.
            objects, object_reason = self._objects_history.select(trajectory)
            state, state_reason = self._state_history.select(trajectory)
            geometry = self._geometries.get(
                (
                    trajectory.header.frame_id,
                    int(trajectory.trajectory_id),
                    stamp_ns(trajectory.header.stamp),
                )
            )
            if objects is not None and self._injected.get("objects") is not objects:
                result = self.runtime.on_objects(
                    RosAdapters.objects_from_ros(objects),
                    frame_id=objects.header.frame_id,
                    timestamp=RosAdapters.timestamp(objects.header.stamp),
                )
                self._injected["objects"] = objects
                self._publish_if_ready(result)
            if state is not None and self._injected.get("state") is not state:
                result = self.runtime.on_state(
                    RosAdapters.state_from_ros(state), frame_id=state.header.frame_id
                )
                self._injected["state"] = state
                self._publish_if_ready(result)
            if geometry is not None and self._injected.get("geometry") is not geometry:
                result = self.runtime.on_geometry(
                    RosAdapters.geometry_from_ros(geometry),
                    frame_id=geometry.header.frame_id,
                    source_trajectory_id=int(geometry.source_trajectory_id),
                    source_trajectory_stamp=RosAdapters.timestamp(
                        geometry.source_trajectory_stamp
                    ),
                )
                self._injected["geometry"] = geometry
                self._publish_if_ready(result)
            readiness = self.runtime.readiness()
            self._waiting = "ready" if readiness.ready else readiness.reason
            if objects is None:
                self._waiting += "; objects: " + object_reason
            if state is None and self.runtime.profile == PredictionProfile.DYNAMIC:
                self._waiting += "; state: " + state_reason
        except Exception as error:
            self._waiting = f"invalid input: {error}"
            LOGGER.exception("prediction input conversion failed")
        finally:
            self._callback_ms = (time.monotonic() - started) * 1000.0
            self._callback_ms_ema = (
                self._callback_ms
                if self._callback_ms_ema == 0
                else (0.9 * self._callback_ms_ema + 0.1 * self._callback_ms)
            )

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
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()
        if rclpy.ok():
            rclpy.shutdown()
