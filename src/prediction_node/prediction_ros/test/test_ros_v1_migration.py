"""ROS adapter + PredictionRuntime wiring tests (no rclpy required)."""

from __future__ import annotations

from types import SimpleNamespace as NS

from prediction_core.config import RoverConfig
from prediction_core.models import (
    DynamicStabilityEvidence,
    PredictionOutput,
    RolloverStep,
)
from prediction_core.runtime import PredictionRuntime
from prediction_core.validation import PredictionProfile
from prediction_ros.adapters import RosAdapters


def _header(stamp: float = 10.0, frame_id: str = "map"):
    return NS(stamp=NS(sec=int(stamp), nanosec=0), frame_id=frame_id)


def _vector(x: float = 0.0, y: float = 0.0, z: float = 0.0):
    return NS(x=x, y=y, z=z)


def test_acceleration_valid_false_maps_xyz_none() -> None:
    vector = _vector()
    message = NS(
        header=_header(),
        pose_valid=False,
        pose=NS(position=vector, orientation=NS(x=0.0, y=0.0, z=0.0, w=1.0)),
        twist_valid=False,
        twist=NS(linear=vector, angular=vector),
        acceleration_valid=False,
        acceleration=NS(linear=_vector(1.0, 2.0, 3.0), angular=vector),
    )
    state = RosAdapters.state_from_ros(message)
    assert state.acceleration_xyz is None
    assert state.acceleration_xy is None
    assert state.resolved_acceleration_xyz() is None


def test_acceleration_valid_zero_xyz_is_valid() -> None:
    vector = _vector()
    message = NS(
        header=_header(),
        pose_valid=False,
        pose=NS(position=vector, orientation=NS(x=0.0, y=0.0, z=0.0, w=1.0)),
        twist_valid=False,
        twist=NS(linear=vector, angular=vector),
        acceleration_valid=True,
        acceleration=NS(linear=_vector(0.0, 0.0, 0.0), angular=vector),
    )
    state = RosAdapters.state_from_ros(message)
    assert state.acceleration_xyz == (0.0, 0.0, 0.0)
    assert state.resolved_acceleration_xyz() == (0.0, 0.0, 0.0)


def test_acceleration_maps_full_xyz_not_xy_only() -> None:
    vector = _vector()
    message = NS(
        header=_header(),
        pose_valid=False,
        pose=NS(position=vector, orientation=NS(x=0.0, y=0.0, z=0.0, w=1.0)),
        twist_valid=True,
        twist=NS(linear=_vector(0.1, 0.2, 0.3), angular=_vector(0.4, 0.5, 0.6)),
        acceleration_valid=True,
        acceleration=NS(linear=_vector(1.0, 2.0, 3.0), angular=_vector(7.0, 8.0, 9.0)),
    )
    state = RosAdapters.state_from_ros(message)
    assert state.acceleration_xyz == (1.0, 2.0, 3.0)
    assert state.acceleration_xy == (1.0, 2.0)
    assert state.velocity_xyz == (0.1, 0.2, 0.3)
    assert state.angular_velocity_xyz == (0.4, 0.5, 0.6)
    assert state.angular_acceleration_xyz == (7.0, 8.0, 9.0)


def test_wrench_empty_list_vs_semantics() -> None:
    empty = RosAdapters.external_wrenches_from_ros(NS(header=_header(), wrenches=[]))
    assert empty == []

    item = NS(
        header=_header(),
        source="boom",
        wrench=NS(force=_vector(10.0, 0.0, 0.0), torque=_vector(0.0, 1.0, 0.0)),
        application_point=_vector(0.0, 0.0, 0.5),
        application_point_valid=True,
        confidence=0.9,
        confidence_valid=True,
    )
    filled = RosAdapters.external_wrenches_from_ros(NS(header=_header(), wrenches=[item]))
    assert len(filled) == 1
    assert filled[0].force_xyz == (10.0, 0.0, 0.0)
    assert filled[0].torque_xyz == (0.0, 1.0, 0.0)
    assert filled[0].application_point_xyz == (0.0, 0.0, 0.5)
    assert filled[0].confidence == 0.9


def test_wrench_application_point_invalid_maps_none() -> None:
    item = NS(
        header=_header(),
        source="arm",
        wrench=NS(force=_vector(1.0, 0.0, 0.0), torque=_vector()),
        application_point=_vector(9.0, 9.0, 9.0),
        application_point_valid=False,
        confidence=0.0,
        confidence_valid=False,
    )
    wrenches = RosAdapters.external_wrenches_from_ros(NS(header=_header(), wrenches=[item]))
    assert wrenches[0].application_point_xyz is None
    assert wrenches[0].confidence is None


def test_prediction_to_ros_maps_stability_moment_and_zmp_edges() -> None:
    class Output:
        def __init__(self):
            self.header = NS(stamp=NS(sec=0, nanosec=0), frame_id="")
            self.collision_steps = []
            self.rollover_steps = []

    class CollisionStep:
        def __init__(self):
            self.collision_objects = []

    class CollisionObject:
        pass

    class RolloverStepMessage:
        pass

    class StabilityMoment:
        pass

    class Zmp:
        pass

    dyn = DynamicStabilityEvidence(
        acceleration_available=True,
        external_wrench_available=False,
        external_wrench_included=False,
        effective_force_xyz_n=(0.0, 0.0, -981.0),
        effective_gravity_projection_xy=(0.0, 0.0),
        effective_ssm_m=0.3,
        normalized_effective_ssm=0.8,
        zmp_xy=(0.05, -0.02),
        zmp_margin_m=0.25,
        normalized_zmp_margin=0.7,
        edge_stability_moments_nm={"front": 100.0, "rear": 110.0, "left": 90.0, "right": 80.0},
        minimum_stability_moment_nm=80.0,
        normalized_minimum_stability_moment=0.85,
        critical_edge="right",
        valid=True,
        validity_reason="ok",
        normalized_edge_stability_moments={
            "front": 1.0,
            "rear": 1.0,
            "left": 0.9,
            "right": 0.85,
        },
        nearest_effective_edge="front",
        nearest_zmp_edge="front",
    )
    output = PredictionOutput(
        timestamp=10.0,
        source_trajectory_stamp=10.0,
        rollover_steps=[
            RolloverStep(
                1,
                2.0,
                3.0,
                0.4,
                0.8,
                "terrain",
                None,
                dynamic_stability=dyn,
            ),
        ],
    )
    message = RosAdapters.prediction_to_ros(
        output,
        source_trajectory_id=7,
        frame_id="map",
        output_type=Output,
        collision_step_type=CollisionStep,
        collision_object_type=CollisionObject,
        rollover_step_type=RolloverStepMessage,
        stability_moment_type=StabilityMoment,
        zmp_type=Zmp,
    )
    moment = message.rollover_steps[0].stability_moment
    zmp = message.rollover_steps[0].zmp
    assert moment.valid is True
    assert moment.minimum_stability_moment_nm == 80.0
    assert moment.normalized_minimum_stability_moment == 0.85
    assert moment.minimum_normalized_moment_edge == "right"
    assert moment.right_moment_nm == 80.0
    assert zmp.valid is True
    assert zmp.x == 0.05
    assert zmp.y == -0.02
    assert zmp.nearest_edge == "front"
    # Explicit edge semantic difference preserved in ROS output
    assert moment.minimum_normalized_moment_edge != zmp.nearest_edge


def test_prediction_to_ros_invalid_dynamic_sets_valid_false() -> None:
    class Output:
        def __init__(self):
            self.header = NS(stamp=NS(sec=0, nanosec=0), frame_id="")
            self.collision_steps = []
            self.rollover_steps = []

    class CollisionStep:
        def __init__(self):
            self.collision_objects = []

    class CollisionObject:
        pass

    class RolloverStepMessage:
        pass

    class StabilityMoment:
        pass

    class Zmp:
        pass

    dyn = DynamicStabilityEvidence(
        acceleration_available=False,
        external_wrench_available=False,
        external_wrench_included=False,
        effective_force_xyz_n=None,
        effective_gravity_projection_xy=None,
        effective_ssm_m=None,
        normalized_effective_ssm=None,
        zmp_xy=None,
        zmp_margin_m=None,
        normalized_zmp_margin=None,
        edge_stability_moments_nm=None,
        minimum_stability_moment_nm=None,
        normalized_minimum_stability_moment=None,
        critical_edge=None,
        valid=False,
        validity_reason="acceleration_xyz unavailable",
    )
    output = PredictionOutput(
        timestamp=10.0,
        source_trajectory_stamp=10.0,
        rollover_steps=[RolloverStep(0, 0.0, 0.0, 0.3, 1.0, "t", None, dynamic_stability=dyn)],
    )
    message = RosAdapters.prediction_to_ros(
        output,
        source_trajectory_id=1,
        frame_id="map",
        output_type=Output,
        collision_step_type=CollisionStep,
        collision_object_type=CollisionObject,
        rollover_step_type=RolloverStepMessage,
        stability_moment_type=StabilityMoment,
        zmp_type=Zmp,
    )
    assert message.rollover_steps[0].stability_moment.valid is False
    assert "unavailable" in message.rollover_steps[0].stability_moment.validity_reason
    assert message.rollover_steps[0].zmp.valid is False


def test_runtime_profile_dynamic_waits_for_acceleration(mock_config: RoverConfig) -> None:
    runtime = PredictionRuntime(mock_config, profile=PredictionProfile.DYNAMIC)
    from prediction_core.models import GeometryStep, Trajectory, TrajectoryStep

    traj = Trajectory(
        timestamp=100.0,
        frame_id="map",
        steps=[TrajectoryStep(0, 0.0, 0.0, 0.0), TrajectoryStep(1, 1.0, 0.0, 0.0)],
    )
    geometry = [
        GeometryStep(100.0, 0, "p0", (0.0, 0.0, 1.0)),
        GeometryStep(100.0, 1, "p1", (0.0, 0.0, 1.0)),
    ]
    runtime.on_trajectory(traj, trajectory_id=1)
    runtime.on_objects([], frame_id="map")
    waiting = runtime.on_geometry(
        geometry,
        frame_id="map",
        source_trajectory_id=1,
        source_trajectory_stamp=100.0,
    )
    assert waiting.output is None
    assert "missing rover state" in waiting.readiness.reasons

    # Invalid accel via ROS-mapped state shape
    vector = _vector()
    invalid_msg = NS(
        header=_header(100.5),
        pose_valid=False,
        pose=NS(position=vector, orientation=NS(x=0.0, y=0.0, z=0.0, w=1.0)),
        twist_valid=False,
        twist=NS(linear=vector, angular=vector),
        acceleration_valid=False,
        acceleration=NS(linear=vector, angular=vector),
    )
    still = runtime.on_state(RosAdapters.state_from_ros(invalid_msg), frame_id="map")
    assert still.output is None
    assert "rover acceleration unavailable" in still.readiness.reasons

    valid_msg = NS(
        header=_header(100.6),
        pose_valid=False,
        pose=NS(position=vector, orientation=NS(x=0.0, y=0.0, z=0.0, w=1.0)),
        twist_valid=False,
        twist=NS(linear=vector, angular=vector),
        acceleration_valid=True,
        acceleration=NS(linear=_vector(0.0, 1.0, 0.0), angular=vector),
    )
    first = runtime.on_state(RosAdapters.state_from_ros(valid_msg), frame_id="map")
    assert first.output is not None

    second = runtime.on_state(RosAdapters.state_from_ros(valid_msg), frame_id="map")
    assert second.output is None
    assert second.duplicate_cycle is True


def test_new_trajectory_clears_state_for_ros_wiring(mock_config: RoverConfig) -> None:
    runtime = PredictionRuntime(mock_config, profile="dynamic")
    from prediction_core.models import GeometryStep, RoverState, Trajectory, TrajectoryStep

    traj = Trajectory(
        timestamp=100.0,
        frame_id="map",
        steps=[TrajectoryStep(0, 0.0, 0.0, 0.0)],
    )
    geometry = [GeometryStep(100.0, 0, "p0", (0.0, 0.0, 1.0))]
    runtime.on_trajectory(traj, trajectory_id=1)
    runtime.on_objects([], frame_id="map")
    runtime.on_state(
        RoverState(timestamp=100.1, acceleration_xyz=(0.0, 0.0, 0.0)),
        frame_id="map",
    )
    runtime.on_external_wrenches([], frame_id="map")
    assert (
        runtime.on_geometry(
            geometry, frame_id="map", source_trajectory_id=1, source_trajectory_stamp=100.0
        ).output
        is not None
    )

    opened = runtime.on_trajectory(
        Trajectory(timestamp=200.0, frame_id="map", steps=[TrajectoryStep(0, 0.0, 0.0, 0.0)]),
        trajectory_id=2,
    )
    snap = runtime.snapshot()
    assert snap.state is None
    assert snap.external_wrenches is None
    assert snap.objects is None
    assert snap.geometry is None
    assert opened.output is None
    assert "missing rover state" in opened.readiness.reasons
