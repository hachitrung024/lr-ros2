import json

import pytest

from prediction_ros.adapters import JsonAdapters
from prediction_ros.test_support import (
    geometry_payload,
    objects_payload,
    trajectory_payload,
)


def test_trajectory_from_payload_maps_steps() -> None:
    trajectory = JsonAdapters.trajectory_from_payload(trajectory_payload(1234.5))
    assert trajectory.timestamp == 1234.5
    assert trajectory.frame_id == "map"
    assert len(trajectory.steps) == 3
    assert trajectory.steps[0].step_id == 0


def test_objects_from_payload_accepts_empty_list() -> None:
    objects, header, source_stamp = JsonAdapters.objects_from_payload(objects_payload(objects=[]))
    assert objects == []
    assert header.frame_id == "map"
    assert source_stamp == 1000.0


def test_geometry_from_payload_maps_normals() -> None:
    geometry, header, source_stamp = JsonAdapters.geometry_from_payload(geometry_payload())
    assert len(geometry) == 3
    assert geometry[0].normal_xyz == (0.0, 0.0, 1.0)
    assert header.frame_id == "map"
    assert source_stamp == 1000.0


def test_state_from_payload_optional_fields() -> None:
    payload = {
        "header": {"stamp": {"sec": 1000, "nanosec": 0}, "frame_id": "map"},
        "timestamp": 1000.0,
    }
    state = JsonAdapters.state_from_payload(payload)
    assert state.timestamp == 1000.0
    assert state.x is None
    assert state.roll is None


def test_prediction_json_roundtrip_fields() -> None:
    from prediction_core.models import PredictionOutput

    output = PredictionOutput(
        timestamp=1000.0,
        source_trajectory_stamp=1000.0,
        collision_steps=[],
        rollover_steps=[],
    )
    payload = json.loads(JsonAdapters.prediction_to_json(output, "map"))
    assert payload["header"]["frame_id"] == "map"
    assert payload["source_trajectory_stamp"] == 1000.0
    assert payload["collision_steps"] == []
    assert payload["rollover_steps"] == []


def test_parse_json_string_rejects_non_object() -> None:
    with pytest.raises(ValueError, match="object"):
        JsonAdapters.parse_json_string("[]")
