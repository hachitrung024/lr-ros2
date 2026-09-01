"""Trajectory-cycle prediction coordinator (ROS-independent)."""

from __future__ import annotations

from dataclasses import dataclass
from threading import RLock
from typing import Callable

from .cache import PredictionInputCache, PredictionSnapshot
from .models import PredictionOutput
from .predictor import PredictionCore
from .validation import InputValidator, PredictionReadiness, ValidationResult


@dataclass(frozen=True)
class CycleKey:
    frame_id: str
    trajectory_id: int


@dataclass(frozen=True)
class CoordinatorResult:
    output: PredictionOutput | None
    validation: ValidationResult | None = None
    readiness: PredictionReadiness | None = None
    cycle_key: CycleKey | None = None
    duplicate_cycle: bool = False


class PredictionCoordinator:
    """Coordinate cache snapshots and invoke PredictionCore once per cycle."""

    def __init__(
        self,
        core: PredictionCore,
        cache: PredictionInputCache,
        validator: InputValidator,
        *,
        logger: Callable[[str], None] | None = None,
    ) -> None:
        self.core = core
        self.cache = cache
        self.validator = validator
        self._logger = logger or (lambda message: None)
        self._lock = RLock()
        self._last_predicted_cycle: CycleKey | None = None

    @property
    def last_predicted_cycle(self) -> CycleKey | None:
        return self._last_predicted_cycle

    def try_predict(self) -> CoordinatorResult:
        snapshot = self.cache.snapshot()
        readiness = self.validator.evaluate_readiness(snapshot)
        validation = ValidationResult.from_readiness(readiness)
        if not readiness.ready:
            self._logger(f"Prediction waiting: {readiness.reason}")
            return CoordinatorResult(
                output=None,
                validation=validation,
                readiness=readiness,
            )

        assert snapshot.trajectory is not None
        assert snapshot.objects is not None
        assert snapshot.geometry is not None
        assert snapshot.trajectory_id is not None
        cycle_key = CycleKey(snapshot.trajectory.frame_id, snapshot.trajectory_id)

        with self._lock:
            if self._last_predicted_cycle == cycle_key:
                self._logger(
                    "Prediction skipped: cycle already published "
                    f"({cycle_key.frame_id}, {cycle_key.trajectory_id})"
                )
                return CoordinatorResult(
                    output=None,
                    validation=validation,
                    readiness=readiness,
                    cycle_key=cycle_key,
                    duplicate_cycle=True,
                )

        external_wrenches = None
        if snapshot.external_wrenches is not None:
            from .models import ExternalWrench

            external_wrenches = [
                ExternalWrench(
                    source=item.source,
                    force_xyz=item.force_xyz,
                    torque_xyz=item.torque_xyz,
                    application_point_xyz=item.application_point_xyz,
                    confidence=item.confidence,
                    frame_id=item.frame_id,
                )
                for item in snapshot.external_wrenches
            ]

        output = self.core.predict(
            trajectory=snapshot.trajectory,
            tracked_objects=snapshot.objects,
            geometry=snapshot.geometry,
            state=snapshot.state,
            external_wrenches=external_wrenches,
        )

        with self._lock:
            self._last_predicted_cycle = cycle_key

        self._logger(
            "Prediction published cycle "
            f"{cycle_key.frame_id} trajectory_id={cycle_key.trajectory_id}: "
            f"{len(output.collision_steps)} collision steps, "
            f"{len(output.rollover_steps)} rollover steps"
        )
        return CoordinatorResult(
            output=output,
            validation=validation,
            readiness=readiness,
            cycle_key=cycle_key,
        )

    def reset_cycle_tracking(self) -> None:
        with self._lock:
            self._last_predicted_cycle = None

    @staticmethod
    def cycle_key_from_snapshot(snapshot: PredictionSnapshot) -> CycleKey | None:
        if snapshot.trajectory is None or snapshot.trajectory_id is None:
            return None
        return CycleKey(snapshot.trajectory.frame_id, snapshot.trajectory_id)
