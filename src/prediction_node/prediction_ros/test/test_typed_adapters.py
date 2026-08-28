from types import SimpleNamespace as NS

from prediction_ros.adapters import RosAdapters
from prediction_core.models import PredictionOutput, RolloverStep


def _header(stamp: float = 10.0, frame_id: str = "map"):
    return NS(stamp=NS(sec=int(stamp), nanosec=0), frame_id=frame_id)


def test_ros_trajectory_maps_to_core() -> None:
    message = NS(
        header=_header(),
        trajectory_id=7,
        steps=[NS(step_id=2, x=1.0, y=2.0, yaw=0.5)],
    )
    trajectory = RosAdapters.trajectory_from_ros(message)
    assert trajectory.frame_id == "map"
    assert trajectory.steps[0].yaw == 0.5


def test_empty_ros_objects_is_a_valid_empty_batch() -> None:
    assert RosAdapters.objects_from_ros(NS(header=_header(), objects=[])) == []


def test_invalid_acceleration_maps_to_none_not_zero() -> None:
    vector = NS(x=0.0, y=0.0, z=0.0)
    message = NS(
        header=_header(),
        pose_valid=False,
        pose=NS(position=vector, orientation=NS(x=0.0, y=0.0, z=0.0, w=1.0)),
        twist_valid=False,
        twist=NS(linear=vector, angular=vector),
        acceleration_valid=False,
        acceleration=NS(linear=vector, angular=vector),
    )
    state = RosAdapters.state_from_ros(message)
    assert state.acceleration_xy is None
    assert state.acceleration_xyz is None
    assert state.velocity_xy is None
    assert state.angular_velocity_xyz is None


def test_geometry_confidence_validity_maps_to_optional() -> None:
    step = NS(
        step_id=1,
        plane_id="terrain",
        normal=NS(x=0.0, y=0.0, z=1.0),
        confidence=0.0,
        confidence_valid=False,
    )
    geometry = RosAdapters.geometry_from_ros(NS(header=_header(), steps=[step]))
    assert geometry[0].confidence is None


def test_prediction_output_maps_to_typed_message() -> None:
    class Output:
        def __init__(self):
            self.header = NS(stamp=NS(sec=0, nanosec=0), frame_id="")

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

    output = PredictionOutput(
        timestamp=10.0,
        source_trajectory_stamp=10.0,
        rollover_steps=[
            RolloverStep(1, 2.0, 3.0, 0.4, 0.8, "terrain", None),
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
    assert message.source_trajectory_id == 7
    assert message.rollover_steps[0].normalized_static_stability_margin == 0.8
    assert message.rollover_steps[0].confidence_valid is False
    assert message.rollover_steps[0].stability_moment.valid is False
    assert message.rollover_steps[0].zmp.valid is False
