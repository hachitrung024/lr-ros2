"""Exact footprint-to-footprint collision-candidate prediction."""

from __future__ import annotations

import math

from shapely.geometry import Polygon
from shapely.validation import explain_validity

from .config import RoverConfig
from .geometry_utils import rover_rectangle
from .models import CollisionObject, CollisionStep, TrackedObject, Trajectory


DISTANCE_COMPARISON_EPSILON_M = 1e-9


def validated_object_polygon(tracked_object: TrackedObject) -> Polygon:
    """Construct a non-empty, positive-area, valid Shapely polygon."""
    polygon = Polygon(tracked_object.footprint_polygon_xy)
    if not polygon.is_valid:
        raise ValueError(
            f"object {tracked_object.track_id!r} polygon is invalid: "
            f"{explain_validity(polygon)}"
        )
    if polygon.is_empty or polygon.area <= 1e-12:
        raise ValueError(f"object {tracked_object.track_id!r} polygon has zero area")
    return polygon


class CollisionPredictor:
    """Evaluate every trajectory rectangle against every object polygon."""

    def __init__(self, config: RoverConfig) -> None:
        self.config = config

    def predict(
        self, trajectory: Trajectory, tracked_objects: list[TrackedObject]
    ) -> list[CollisionStep]:
        ids = [obj.track_id for obj in tracked_objects]
        if len(ids) != len(set(ids)):
            raise ValueError("tracked object IDs must be unique within a prediction input")
        object_polygons = [
            (tracked_object, validated_object_polygon(tracked_object))
            for tracked_object in tracked_objects
        ]
        distances = trajectory.cumulative_distances_m()
        collision_steps: list[CollisionStep] = []
        margin = self.config.prediction.collision_margin_m
        for step, distance_along_route in zip(trajectory.steps, distances):
            rover = rover_rectangle(
                step, self.config.body_length_m, self.config.body_width_m
            )
            candidates: list[CollisionObject] = []
            for tracked_object, object_polygon in object_polygons:
                min_distance = float(rover.distance(object_polygon))
                if not math.isfinite(min_distance):
                    raise ValueError(
                        f"distance to object {tracked_object.track_id!r} is not finite"
                    )
                if min_distance <= margin + DISTANCE_COMPARISON_EPSILON_M:
                    candidates.append(
                        CollisionObject(
                            object_id=tracked_object.track_id,
                            object_class=tracked_object.class_name,
                            min_distance_m=min_distance,
                            confidence_or_validity=tracked_object.confidence,
                        )
                    )
            if candidates:
                collision_steps.append(
                    CollisionStep(
                        step_id=step.step_id,
                        distance_along_route_m=distance_along_route,
                        collision_objects=candidates,
                    )
                )
        return collision_steps
