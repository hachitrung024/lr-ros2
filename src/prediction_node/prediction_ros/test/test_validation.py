import pytest

from prediction_core.config import PredictionConfig, RoverConfig

from prediction_ros.test_support import (
    geometry_payload,
    load_geometry,
    load_objects,
    load_trajectory,
    objects_payload,
    trajectory_payload,
)
from prediction_ros.validation import InputValidator, ValidationConfig


def test_invalid_com_config_rejected_by_core_config() -> None:
    with pytest.raises(ValueError, match="CoM"):
        RoverConfig(
            mass_kg=100.0,
            body_length_m=1.05,
            body_width_m=0.90,
            body_height_m=0.50,
            support_length_m=0.75,
            support_width_m=0.88,
            ground_clearance_m=0.15,
            com_x_m=0.375,
            com_y_m=0.0,
            com_height_m=0.33,
            prediction=PredictionConfig(),
        )


def test_validator_rejects_wrong_geometry_trajectory_id(cache) -> None:
    cache.set_trajectory(load_trajectory(trajectory_payload(2000.0)), trajectory_id=42)
    objects, objects_frame, _ = load_objects(objects_payload(2000.0))
    cache.set_objects(objects, frame_id=objects_frame)
    geometry, geometry_frame, geometry_source = load_geometry(geometry_payload(2000.0, 2000.0))
    cache.set_geometry(
        geometry,
        frame_id=geometry_frame,
        source_trajectory_id=41,
        source_trajectory_stamp=geometry_source,
    )
    validator = InputValidator(ValidationConfig())
    result = validator.inputs_compatible(cache.snapshot())
    assert not result.ok
    assert "trajectory_id=41" in result.reason
