"""Readiness must distinguish absent, stale, future and valid empty batches."""

from pathlib import Path

import pytest

from prediction_core.config import load_config
from prediction_core.models import GeometryStep, RoverState, Trajectory, TrajectoryStep
from prediction_core.runtime import PredictionRuntime


def runtime(profile='static'):
    node = PredictionRuntime(
        load_config(Path(__file__).parents[1] / 'config/rover.reference.yaml'),
        profile=profile,
        max_object_age_sec=0.5,
        max_geometry_age_sec=2.0,
        max_state_age_sec=0.25,
    )
    node.on_trajectory(
        Trajectory(
            10.0, 'map', [TrajectoryStep(0, 0.0, 0.0, 0.0), TrajectoryStep(1, 1.0, 0.0, 0.0)]
        ),
        trajectory_id=1,
    )
    return node


def geometry(node, timestamp=10.0, frame='map', cycle=1):
    return node.on_geometry(
        [GeometryStep(timestamp, 0, 'p', (0.0, 0.0, 1.0))],
        frame_id=frame,
        source_trajectory_id=cycle,
        source_trajectory_stamp=10.0,
    )


@pytest.mark.parametrize(
    'timestamp,reason', [(None, 'timestamp'), (9.0, 'stale'), (11.0, 'future')]
)
def test_invalid_empty_object_batch_cannot_clear_obstacles(timestamp, reason):
    node = runtime()
    node.on_objects([], frame_id='map', timestamp=timestamp)
    result = geometry(node)
    assert result.output is None
    assert reason in result.readiness.reason


def test_unknown_objects_differ_from_valid_empty_and_predict_once():
    node = runtime()
    assert geometry(node).output is None
    result = node.on_objects([], frame_id='map', timestamp=9.8)
    assert result.output is not None
    assert len(result.output.rollover_steps) == 1  # Partial terrain stays partial.
    assert node.try_predict().duplicate_cycle


@pytest.mark.parametrize(
    'timestamp,frame,cycle',
    [(7.0, 'map', 1), (10.1, 'map', 1), (10.0, 'odom', 1), (10.0, 'map', 2)],
)
def test_geometry_checks_sensor_age_frame_and_cycle(timestamp, frame, cycle):
    node = runtime()
    node.on_objects([], frame_id='map', timestamp=10.0)
    assert geometry(node, timestamp, frame, cycle).output is None


def test_dynamic_waits_for_acceleration_and_reset_clears_entire_timeline():
    node = runtime('dynamic')
    node.on_objects([], frame_id='map', timestamp=10.0)
    geometry(node)
    assert node.on_state(RoverState(10.0), frame_id='map').output is None
    assert (
        node.on_state(RoverState(9.0, acceleration_xyz=(0.0, 0.0, 0.0)), frame_id='map').output
        is None
    )
    assert (
        node.on_state(RoverState(10.1, acceleration_xyz=(0.0, 0.0, 0.0)), frame_id='map').output
        is None
    )
    assert (
        node.on_state(RoverState(10.0, acceleration_xyz=(0.0, 0.0, 0.0)), frame_id='map').output
        is not None
    )
    node.reset()
    assert node.snapshot().objects is None
    assert node.snapshot().state is None
    assert node.snapshot().geometry is None
    assert node.last_predicted_cycle is None
    node.on_trajectory(Trajectory(5.0, 'map', [TrajectoryStep(0, 0.0, 0.0, 0.0)]), trajectory_id=2)
    assert not node.readiness().ready
