"""Tests for canonical raw-evidence presentation semantics."""

from types import SimpleNamespace

from lr_path_prediction.canonical_visualizer_node import (
    intersecting_collision_objects,
)


def test_exclamation_requires_actual_footprint_intersection():
    """A positive-clearance safety-margin candidate must remain gray."""
    intersecting = SimpleNamespace(min_distance_m=0.0)
    touching_with_roundoff = SimpleNamespace(min_distance_m=1e-10)
    margin_only = SimpleNamespace(min_distance_m=0.05)

    result = intersecting_collision_objects([
        margin_only,
        intersecting,
        touching_with_roundoff,
    ])

    assert result == [intersecting, touching_with_roundoff]
