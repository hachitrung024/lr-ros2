import json

import pytest

from prediction_ros.adapters import JsonAdapters
from prediction_ros.coordinator import PredictionCoordinator
from prediction_ros.test_support import (
    cache_ready_cycle,
    geometry_payload,
    load_geometry,
    load_objects,
    load_trajectory,
    objects_payload,
    trajectory_payload,
)
from prediction_ros.validation import InputValidator, ValidationConfig


def test_only_trajectory_no_prediction(cache, coordinator) -> None:
    cache.set_trajectory(load_trajectory(trajectory_payload()))
    result = coordinator.try_predict()
    assert result.output is None


def test_trajectory_and_objects_wait_for_geometry(cache, coordinator) -> None:
    cache.set_trajectory(load_trajectory(trajectory_payload()))
    objects, frame_id, source_stamp = load_objects(objects_payload())
    cache.set_objects(objects, frame_id=frame_id, source_trajectory_stamp=source_stamp)
    result = coordinator.try_predict()
    assert result.output is None


def test_objects_and_geometry_without_trajectory_no_prediction(cache, coordinator) -> None:
    geometry, geometry_frame, geometry_source = load_geometry(geometry_payload())
    cache.set_geometry(
        geometry,
        frame_id=geometry_frame,
        source_trajectory_stamp=geometry_source,
    )
    objects, objects_frame, objects_source = load_objects(objects_payload())
    cache.set_objects(objects, frame_id=objects_frame, source_trajectory_stamp=objects_source)
    result = coordinator.try_predict()
    assert result.output is None


def test_all_required_inputs_predict_once(cache, coordinator) -> None:
    cache_ready_cycle(cache)
    result = coordinator.try_predict()
    assert result.output is not None
    assert result.output.source_trajectory_stamp == 1000.0


def test_empty_objects_batch_is_valid(cache, coordinator) -> None:
    cache.set_trajectory(load_trajectory(trajectory_payload()))
    objects, frame_id, source_stamp = load_objects(objects_payload(objects=[]))
    cache.set_objects(objects, frame_id=frame_id, source_trajectory_stamp=source_stamp)
    geometry, geometry_frame, geometry_source = load_geometry(geometry_payload())
    cache.set_geometry(
        geometry,
        frame_id=geometry_frame,
        source_trajectory_stamp=geometry_source,
    )
    result = coordinator.try_predict()
    assert result.output is not None


def test_missing_objects_none_is_not_ready(cache, coordinator) -> None:
    cache.set_trajectory(load_trajectory(trajectory_payload()))
    geometry, geometry_frame, geometry_source = load_geometry(geometry_payload())
    cache.set_geometry(
        geometry,
        frame_id=geometry_frame,
        source_trajectory_stamp=geometry_source,
    )
    assert cache.snapshot().objects is None
    result = coordinator.try_predict()
    assert result.output is None


def test_missing_state_still_predicts(cache, coordinator) -> None:
    cache_ready_cycle(cache)
    result = coordinator.try_predict()
    assert result.output is not None
    assert cache.snapshot().state is None


def test_duplicate_cycle_not_predicted_twice(cache, coordinator) -> None:
    cache_ready_cycle(cache)
    first = coordinator.try_predict()
    second = coordinator.try_predict()
    assert first.output is not None
    assert second.output is None
    assert second.duplicate_cycle is True


def test_new_trajectory_rejects_old_geometry(cache, coordinator) -> None:
    cache_ready_cycle(cache, trajectory_stamp=1000.0, geometry_stamp=1000.0)
    first = coordinator.try_predict()
    assert first.output is not None

    cache.set_trajectory(load_trajectory(trajectory_payload(2000.0)))
    objects, objects_frame, objects_source = load_objects(objects_payload(2000.0, 2000.0))
    cache.set_objects(objects, frame_id=objects_frame, source_trajectory_stamp=objects_source)
    geometry, geometry_frame, geometry_source = load_geometry(geometry_payload(1000.0, 1000.0))
    cache.set_geometry(
        geometry,
        frame_id=geometry_frame,
        source_trajectory_stamp=geometry_source,
    )
    result = coordinator.try_predict()
    assert result.output is None
    assert result.validation is not None
    assert "geometry belong" in result.validation.reason


def test_geometry_for_new_trajectory_triggers_prediction(cache, coordinator) -> None:
    cache.set_trajectory(load_trajectory(trajectory_payload(2000.0)))
    objects, objects_frame, objects_source = load_objects(objects_payload(2000.0, 2000.0))
    cache.set_objects(objects, frame_id=objects_frame, source_trajectory_stamp=objects_source)
    geometry, geometry_frame, geometry_source = load_geometry(geometry_payload(1000.0))
    cache.set_geometry(
        geometry,
        frame_id=geometry_frame,
        source_trajectory_stamp=geometry_source,
    )
    assert coordinator.try_predict().output is None

    geometry_b, geometry_frame_b, geometry_source_b = load_geometry(
        geometry_payload(2000.0, 2000.0)
    )
    cache.set_geometry(
        geometry_b,
        frame_id=geometry_frame_b,
        source_trajectory_stamp=geometry_source_b,
    )
    result = coordinator.try_predict()
    assert result.output is not None
    assert result.output.source_trajectory_stamp == 2000.0


def test_frame_mismatch_blocks_prediction(cache, coordinator) -> None:
    cache.set_trajectory(load_trajectory(trajectory_payload()))
    objects, _, source_stamp = load_objects(objects_payload())
    cache.set_objects(objects, frame_id="odom", source_trajectory_stamp=source_stamp)
    geometry, _, geometry_source = load_geometry(geometry_payload())
    cache.set_geometry(
        geometry,
        frame_id="map",
        source_trajectory_stamp=geometry_source,
    )
    result = coordinator.try_predict()
    assert result.output is None
    assert result.validation is not None
    assert "frame mismatch" in result.validation.reason


def test_partial_geometry_coverage_runs_under_default_policy(cache, coordinator) -> None:
    cache.set_trajectory(load_trajectory(trajectory_payload(step_ids=[0, 1, 2, 3, 4])))
    objects, objects_frame, objects_source = load_objects(objects_payload())
    cache.set_objects(objects, frame_id=objects_frame, source_trajectory_stamp=objects_source)
    geometry, geometry_frame, geometry_source = load_geometry(
        geometry_payload(step_ids=[0, 1, 2])
    )
    cache.set_geometry(
        geometry,
        frame_id=geometry_frame,
        source_trajectory_stamp=geometry_source,
    )
    result = coordinator.try_predict()
    assert result.output is not None
    assert len(result.output.rollover_steps) == 3


def test_later_object_update_does_not_recompute_published_cycle(cache, coordinator) -> None:
    cache_ready_cycle(cache, trajectory_stamp=1000.0)
    assert coordinator.try_predict().output is not None
    objects, objects_frame, objects_source = load_objects(
        objects_payload(
            objects=[
                {
                    "track_id": "obj-1",
                    "class_name": "pipe",
                    "footprint_polygon_xy": [[0, 0], [1, 0], [1, 1]],
                }
            ]
        )
    )
    cache.set_objects(objects, frame_id=objects_frame, source_trajectory_stamp=objects_source)
    assert coordinator.try_predict().output is None
    assert coordinator.try_predict().duplicate_cycle is True


def test_integration_event_sequence(cache, coordinator) -> None:
    objects, objects_frame, objects_source = load_objects(objects_payload(900.0, 900.0))
    cache.set_objects(objects, frame_id=objects_frame, source_trajectory_stamp=objects_source)
    assert coordinator.try_predict().output is None

    geometry, geometry_frame, geometry_source = load_geometry(geometry_payload(900.0, 900.0))
    cache.set_geometry(
        geometry,
        frame_id=geometry_frame,
        source_trajectory_stamp=geometry_source,
    )
    assert coordinator.try_predict().output is None

    cache.set_trajectory(load_trajectory(trajectory_payload(1000.0)))
    assert coordinator.try_predict().output is None

    geometry_b, geometry_frame_b, geometry_source_b = load_geometry(
        geometry_payload(1000.0, 1000.0)
    )
    cache.set_geometry(
        geometry_b,
        frame_id=geometry_frame_b,
        source_trajectory_stamp=geometry_source_b,
    )
    objects_b, objects_frame_b, objects_source_b = load_objects(
        objects_payload(1000.0, 1000.0)
    )
    cache.set_objects(objects_b, frame_id=objects_frame_b, source_trajectory_stamp=objects_source_b)
    result = coordinator.try_predict()
    assert result.output is not None
    assert result.output.source_trajectory_stamp == 1000.0


def test_prediction_output_json_roundtrip(cache, coordinator) -> None:
    cache_ready_cycle(cache)
    output = coordinator.try_predict().output
    assert output is not None
    payload = json.loads(JsonAdapters.prediction_to_json(output, "map"))
    assert payload["source_trajectory_stamp"] == 1000.0
    assert "collision_steps" in payload
    assert "rollover_steps" in payload


def test_strict_geometry_mode_blocks_partial_coverage(cache, coordinator) -> None:
    from prediction_ros.coordinator import PredictionCoordinator

    strict_validator = InputValidator(
        ValidationConfig(expected_frame_id="map", require_full_geometry_coverage=True)
    )
    strict_coordinator = PredictionCoordinator(
        coordinator.core,
        cache,
        strict_validator,
    )
    cache.set_trajectory(load_trajectory(trajectory_payload(step_ids=[0, 1, 2, 3])))
    objects, objects_frame, objects_source = load_objects(objects_payload())
    cache.set_objects(objects, frame_id=objects_frame, source_trajectory_stamp=objects_source)
    geometry, geometry_frame, geometry_source = load_geometry(
        geometry_payload(step_ids=[0, 1])
    )
    cache.set_geometry(
        geometry,
        frame_id=geometry_frame,
        source_trajectory_stamp=geometry_source,
    )
    result = strict_coordinator.try_predict()
    assert result.output is None
    assert "coverage" in (result.validation.reason if result.validation else "")


def test_trajectory_id_not_timestamp_defines_cycle(cache, coordinator) -> None:
    trajectory = load_trajectory(trajectory_payload(1000.0))
    cache.set_trajectory(trajectory, trajectory_id=42)
    objects, frame_id, _ = load_objects(objects_payload())
    cache.set_objects(objects, frame_id=frame_id)
    geometry, geometry_frame, geometry_source = load_geometry(geometry_payload())
    cache.set_geometry(
        geometry,
        frame_id=geometry_frame,
        source_trajectory_id=42,
        source_trajectory_stamp=geometry_source,
    )
    assert coordinator.try_predict().output is not None

    cache.set_trajectory(trajectory, trajectory_id=43)
    cache.set_objects(objects, frame_id=frame_id)
    cache.set_geometry(
        geometry,
        frame_id=geometry_frame,
        source_trajectory_id=43,
        source_trajectory_stamp=geometry_source,
    )
    assert coordinator.try_predict().output is not None


def test_external_wrenches_are_cached_but_do_not_block_static_prediction(cache, coordinator) -> None:
    cache_ready_cycle(cache)
    from prediction_ros.cache import ExternalWrenchData

    cache.set_external_wrenches(
        [
            ExternalWrenchData(
                source="gazebo",
                frame_id="map",
                force_xyz=(1.0, 0.0, 0.0),
                torque_xyz=(0.0, 0.0, 0.0),
                application_point_xyz=None,
                confidence=None,
            )
        ],
        frame_id="map",
    )
    assert coordinator.try_predict().output is not None
