"""ROS-independent prediction runtime facade."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

from .cache import ExternalWrenchData, PredictionInputCache, PredictionSnapshot
from .config import RoverConfig
from .coordinator import CoordinatorResult, CycleKey, PredictionCoordinator
from .events import (
    ExternalWrenchEvent,
    GeometryEvent,
    ObjectsEvent,
    StateEvent,
    TrajectoryEvent,
)
from .models import GeometryStep, PredictionOutput, RoverState, TrackedObject, Trajectory
from .predictor import PredictionCore
from .validation import (
    InputValidator,
    PredictionProfile,
    PredictionReadiness,
    ValidationConfig,
)


@dataclass(frozen=True)
class RuntimeResult:
    """Outcome of handling one runtime input event."""

    output: PredictionOutput | None
    readiness: PredictionReadiness
    cycle_key: CycleKey | None = None
    duplicate_cycle: bool = False
    messages: tuple[str, ...] = ()


def _waiting_messages(readiness: PredictionReadiness) -> list[str]:
    messages: list[str] = []
    for reason in readiness.reasons:
        if reason == "compatible":
            continue
        if reason == "missing trajectory":
            messages.append("waiting: trajectory")
        elif reason == "missing tracked objects batch":
            messages.append("waiting: objects")
        elif reason == "missing geometry batch":
            messages.append("waiting: geometry")
        elif reason == "missing rover state":
            messages.append("waiting: rover state")
        elif reason == "rover acceleration unavailable":
            messages.append("waiting: rover acceleration (acceleration_xyz)")
        elif "geometry belongs to trajectory_id" in reason:
            messages.append(f"waiting: matching geometry ({reason})")
        else:
            messages.append(f"not ready: {reason}")
    return messages


def _coerce_profile(profile: PredictionProfile | str) -> PredictionProfile:
    if isinstance(profile, PredictionProfile):
        return profile
    try:
        return PredictionProfile(str(profile).strip().lower())
    except ValueError as exc:
        raise ValueError(
            f"unsupported prediction profile {profile!r}; expected 'static' or 'dynamic'"
        ) from exc


class PredictionRuntime:
    """Event-driven cache → readiness → once-per-cycle PredictionCore facade.

    Profiles
    --------
    ``static`` (default): predict when trajectory + objects + matching geometry
    are ready. RoverState and external wrenches are optional.

    ``dynamic``: additionally require RoverState with valid ``acceleration_xyz``
    (``None`` is not zero). External wrenches remain optional (``None`` ≠ ``[]``).
    """

    def __init__(
        self,
        config: RoverConfig,
        *,
        profile: PredictionProfile | str = PredictionProfile.STATIC,
        expected_frame_id: str = "map",
        require_full_geometry_coverage: bool = False,
        max_object_age_sec: float | None = None,
        max_geometry_age_sec: float | None = None,
        max_state_age_sec: float | None = None,
        logger: Callable[[str], None] | None = None,
    ) -> None:
        self._logger = logger or (lambda message: None)
        self.profile = _coerce_profile(profile)
        self.cache = PredictionInputCache()
        self.core = PredictionCore(config)
        self.validator = InputValidator(
            ValidationConfig(
                expected_frame_id=expected_frame_id,
                require_full_geometry_coverage=require_full_geometry_coverage,
                max_object_age_sec=max_object_age_sec,
                max_geometry_age_sec=max_geometry_age_sec,
                max_state_age_sec=max_state_age_sec,
                profile=self.profile,
            )
        )
        self.coordinator = PredictionCoordinator(
            self.core,
            self.cache,
            self.validator,
            logger=self._logger,
        )
        self._active_cycle_id: int | None = None
        self._logger(f"prediction profile: {self.profile.value}")

    @property
    def last_predicted_cycle(self) -> CycleKey | None:
        return self.coordinator.last_predicted_cycle

    def readiness(self) -> PredictionReadiness:
        return self.validator.evaluate_readiness(self.cache.snapshot())

    def snapshot(self) -> PredictionSnapshot:
        return self.cache.snapshot()

    def on_trajectory(self, trajectory: Trajectory, *, trajectory_id: int) -> RuntimeResult:
        messages = [
            f"prediction profile: {self.profile.value}",
            f"received trajectory cycle={trajectory_id}",
        ]
        previous = self._active_cycle_id
        if previous is not None and previous != trajectory_id:
            messages.append(
                "new cycle clears cycle-bound objects/geometry/state/wrenches "
                f"(previous cycle={previous})"
            )
        self.cache.set_trajectory(trajectory, trajectory_id=trajectory_id)
        self._active_cycle_id = int(trajectory_id)
        for message in messages:
            self._logger(message)
        return self._after_update(extra_messages=tuple(messages))

    def on_objects(self, objects: list[TrackedObject], *, frame_id: str) -> RuntimeResult:
        message = (
            f"received objects batch count={len(objects)}"
            if objects
            else "received empty objects batch"
        )
        self.cache.set_objects(objects, frame_id=frame_id)
        self._logger(message)
        return self._after_update(extra_messages=(message,))

    def on_geometry(
        self,
        geometry: list[GeometryStep],
        *,
        frame_id: str,
        source_trajectory_id: int,
        source_trajectory_stamp: float | None = None,
    ) -> RuntimeResult:
        message = f"received geometry for cycle={source_trajectory_id} steps={len(geometry)}"
        self.cache.set_geometry(
            geometry,
            frame_id=frame_id,
            source_trajectory_id=source_trajectory_id,
            source_trajectory_stamp=source_trajectory_stamp,
        )
        self._logger(message)
        return self._after_update(extra_messages=(message,))

    def on_state(self, state: RoverState, *, frame_id: str) -> RuntimeResult:
        message = f"received state stamp={state.timestamp}"
        self.cache.set_state(state, frame_id=frame_id)
        self._logger(message)
        return self._after_update(extra_messages=(message,))

    def on_external_wrenches(
        self,
        wrenches: list[ExternalWrenchData],
        *,
        frame_id: str,
    ) -> RuntimeResult:
        message = (
            f"received external wrenches count={len(wrenches)}"
            if wrenches
            else "received empty external wrenches batch"
        )
        self.cache.set_external_wrenches(wrenches, frame_id=frame_id)
        self._logger(message)
        return self._after_update(extra_messages=(message,))

    def handle_event(
        self,
        event: TrajectoryEvent | ObjectsEvent | GeometryEvent | StateEvent | ExternalWrenchEvent,
    ) -> RuntimeResult:
        if isinstance(event, TrajectoryEvent):
            return self.on_trajectory(event.trajectory, trajectory_id=event.trajectory_id)
        if isinstance(event, ObjectsEvent):
            return self.on_objects(event.objects, frame_id=event.frame_id)
        if isinstance(event, GeometryEvent):
            return self.on_geometry(
                event.geometry,
                frame_id=event.frame_id,
                source_trajectory_id=event.source_trajectory_id,
                source_trajectory_stamp=event.source_trajectory_stamp,
            )
        if isinstance(event, StateEvent):
            return self.on_state(event.state, frame_id=event.frame_id)
        if isinstance(event, ExternalWrenchEvent):
            return self.on_external_wrenches(event.wrenches, frame_id=event.frame_id)
        raise TypeError(f"unsupported event type: {type(event)!r}")

    def try_predict(self) -> RuntimeResult:
        return self._after_update()

    def _after_update(self, *, extra_messages: tuple[str, ...] = ()) -> RuntimeResult:
        result: CoordinatorResult = self.coordinator.try_predict()
        readiness = result.readiness or self.validator.evaluate_readiness(self.cache.snapshot())
        messages = list(extra_messages)

        if result.duplicate_cycle:
            cycle = result.cycle_key.trajectory_id if result.cycle_key else "?"
            messages.append(f"duplicate cycle: prediction already completed for cycle={cycle}")
        elif result.output is not None:
            messages.append("prediction started")
            messages.append("prediction completed")
        else:
            messages.extend(_waiting_messages(readiness))

        for message in messages[len(extra_messages) :]:
            self._logger(message)

        return RuntimeResult(
            output=result.output,
            readiness=readiness,
            cycle_key=result.cycle_key,
            duplicate_cycle=result.duplicate_cycle,
            messages=tuple(messages),
        )
