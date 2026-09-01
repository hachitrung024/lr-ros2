"""Input compatibility / readiness checks (ROS-independent)."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum

from .cache import PredictionSnapshot


class PredictionProfile(str, Enum):
    """Runtime readiness profile controlling when a trajectory cycle is consumed.

    Prediction runs at most once per trajectory cycle, so the profile decides
    which inputs must exist before that single prediction is taken.
    """

    STATIC = "static"
    DYNAMIC = "dynamic"


@dataclass(frozen=True)
class ValidationConfig:
    expected_frame_id: str = "map"
    require_full_geometry_coverage: bool = False
    max_object_age_sec: float | None = None
    max_geometry_age_sec: float | None = None
    max_state_age_sec: float | None = None
    timestamp_tolerance_sec: float = 1e-3
    profile: PredictionProfile = PredictionProfile.STATIC


@dataclass(frozen=True)
class PredictionReadiness:
    """Diagnostic readiness result for debugging upstream data gaps."""

    ready: bool
    reasons: tuple[str, ...] = ()

    @property
    def reason(self) -> str:
        if self.ready:
            return self.reasons[0] if self.reasons else "ready"
        return "; ".join(self.reasons) if self.reasons else "not ready"


@dataclass(frozen=True)
class ValidationResult:
    """Compatibility result used by the coordinator (backward compatible)."""

    ok: bool
    reason: str = ""
    reasons: tuple[str, ...] = field(default_factory=tuple)

    @classmethod
    def from_readiness(cls, readiness: PredictionReadiness) -> "ValidationResult":
        return cls(ok=readiness.ready, reason=readiness.reason, reasons=readiness.reasons)


class InputValidator:
    """Validate that cached inputs belong to the active trajectory cycle."""

    def __init__(self, config: ValidationConfig | None = None) -> None:
        self.config = config or ValidationConfig()

    def evaluate_readiness(self, snapshot: PredictionSnapshot) -> PredictionReadiness:
        reasons: list[str] = []

        if snapshot.trajectory is None or snapshot.trajectory_id is None:
            reasons.append("missing trajectory")
        if snapshot.objects is None:
            reasons.append("missing tracked objects batch")
        if snapshot.geometry is None:
            reasons.append("missing geometry batch")

        if self.config.profile == PredictionProfile.DYNAMIC:
            if snapshot.state is None:
                reasons.append("missing rover state")
            elif snapshot.state.resolved_acceleration_xyz() is None:
                reasons.append("rover acceleration unavailable")

        if reasons:
            return PredictionReadiness(False, tuple(reasons))

        assert snapshot.trajectory is not None
        assert snapshot.trajectory_id is not None
        assert snapshot.objects is not None
        assert snapshot.geometry is not None

        trajectory = snapshot.trajectory
        if trajectory.frame_id != self.config.expected_frame_id:
            reasons.append("trajectory frame mismatch")

        for label, frame_id in (
            ("objects", snapshot.objects_frame_id),
            ("geometry", snapshot.geometry_frame_id),
            ("state", snapshot.state_frame_id),
            ("external wrenches", snapshot.external_wrenches_frame_id),
        ):
            if frame_id is not None and frame_id != self.config.expected_frame_id:
                reasons.append(f"{label} frame mismatch")

        if snapshot.geometry_source_trajectory_id != snapshot.trajectory_id:
            reasons.append(
                f"geometry belongs to trajectory_id={snapshot.geometry_source_trajectory_id}; "
                f"waiting for trajectory_id={snapshot.trajectory_id}"
            )
        if (
            snapshot.geometry_source_trajectory_stamp is not None
            and abs(snapshot.geometry_source_trajectory_stamp - trajectory.timestamp)
            > self.config.timestamp_tolerance_sec
        ):
            reasons.append("geometry source timestamp does not match active trajectory")

        if self.config.max_object_age_sec is not None and snapshot.objects:
            age = trajectory.timestamp - min(obj.timestamp for obj in snapshot.objects)
            if age > self.config.max_object_age_sec:
                reasons.append("stale tracked objects")

        if self.config.max_geometry_age_sec is not None and snapshot.geometry:
            age = trajectory.timestamp - min(step.timestamp for step in snapshot.geometry)
            if age > self.config.max_geometry_age_sec:
                reasons.append("stale geometry")

        if self.config.max_state_age_sec is not None and snapshot.state is not None:
            age = trajectory.timestamp - snapshot.state.timestamp
            if age > self.config.max_state_age_sec:
                reasons.append("stale state")

        trajectory_step_ids = {step.step_id for step in trajectory.steps}
        geometry_step_ids = {step.step_id for step in snapshot.geometry}
        if not geometry_step_ids & trajectory_step_ids:
            reasons.append("geometry step_ids unrelated to active trajectory")

        if self.config.require_full_geometry_coverage and not trajectory_step_ids.issubset(
            geometry_step_ids
        ):
            reasons.append("geometry coverage incomplete")

        if reasons:
            return PredictionReadiness(False, tuple(reasons))
        return PredictionReadiness(True, ("compatible",))

    def inputs_compatible(self, snapshot: PredictionSnapshot) -> ValidationResult:
        return ValidationResult.from_readiness(self.evaluate_readiness(snapshot))
