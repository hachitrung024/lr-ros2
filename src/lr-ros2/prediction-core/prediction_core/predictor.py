"""Composition root for collision and rollover prediction."""

from __future__ import annotations

import math
import time

from .collision import CollisionPredictor
from .config import RoverConfig
from .models import (
    ExternalWrench,
    GeometryStep,
    PredictionOutput,
    RoverState,
    TrackedObject,
    Trajectory,
)
from .rollover import RolloverPredictor


class PredictionCore:
    """Stable ROS-independent API consumed by future adapters."""

    def __init__(self, config: RoverConfig) -> None:
        self.config = config
        self.collision_predictor = CollisionPredictor(config)
        self.rollover_predictor = RolloverPredictor(config)

    def predict(
        self,
        trajectory: Trajectory,
        tracked_objects: list[TrackedObject],
        geometry: list[GeometryStep],
        state: RoverState | None = None,
        external_wrenches: list[ExternalWrench] | None = None,
    ) -> PredictionOutput:
        """Produce safety evidence.

        ``external_wrenches=None`` means wrench data was not supplied.
        ``external_wrenches=[]`` means an explicit empty wrench set.
        Static collision/rollover baselines still run when dynamic inputs are absent.
        """
        return PredictionOutput(
            timestamp=time.time(),
            source_trajectory_stamp=trajectory.timestamp,
            collision_steps=self.collision_predictor.predict(trajectory, tracked_objects),
            rollover_steps=self.rollover_predictor.predict(
                trajectory,
                geometry,
                state=state,
                external_wrenches=external_wrenches,
            ),
        )

    @staticmethod
    def stale_input_warnings(
        trajectory: Trajectory,
        tracked_objects: list[TrackedObject],
        geometry: list[GeometryStep],
        *,
        max_object_age_s: float | None = None,
        max_geometry_age_s: float | None = None,
    ) -> list[str]:
        """Optional V1 validation hook; no production timeouts are assumed.

        Ages are relative to the source trajectory timestamp. Inputs from the
        future are not classified as stale. Passing None disables each check.
        """
        warnings: list[str] = []
        if max_object_age_s is not None:
            if not math.isfinite(max_object_age_s) or max_object_age_s < 0:
                raise ValueError("max_object_age_s must be finite and non-negative")
            for tracked_object in tracked_objects:
                age = trajectory.timestamp - tracked_object.timestamp
                if age > max_object_age_s:
                    warnings.append(
                        f"object {tracked_object.track_id!r} is stale by policy: age={age:.3f}s"
                    )
        if max_geometry_age_s is not None:
            if not math.isfinite(max_geometry_age_s) or max_geometry_age_s < 0:
                raise ValueError("max_geometry_age_s must be finite and non-negative")
            for geometry_step in geometry:
                age = trajectory.timestamp - geometry_step.timestamp
                if age > max_geometry_age_s:
                    warnings.append(
                        f"geometry step {geometry_step.step_id} is stale by policy: age={age:.3f}s"
                    )
        return warnings
