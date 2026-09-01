"""Thread-safe input cache for trajectory-cycle prediction (ROS-independent)."""

from __future__ import annotations

from dataclasses import dataclass
from threading import RLock

from .models import GeometryStep, RoverState, TrackedObject, Trajectory


@dataclass(frozen=True)
class ExternalWrenchData:
    """Optional dynamic input; unused by V1 static prediction algorithms."""

    source: str
    frame_id: str
    force_xyz: tuple[float, float, float]
    torque_xyz: tuple[float, float, float]
    application_point_xyz: tuple[float, float, float] | None
    confidence: float | None


@dataclass(frozen=True)
class PredictionSnapshot:
    trajectory: Trajectory | None
    trajectory_id: int | None
    objects: list[TrackedObject] | None
    geometry: list[GeometryStep] | None
    state: RoverState | None
    objects_frame_id: str | None
    geometry_frame_id: str | None
    state_frame_id: str | None
    geometry_source_trajectory_id: int | None
    geometry_source_trajectory_stamp: float | None
    external_wrenches: list[ExternalWrenchData] | None
    external_wrenches_frame_id: str | None


class PredictionInputCache:
    """Cache latest inputs and expose immutable snapshots.

    Semantics:
    - ``objects is None`` means tracking batch not received.
    - ``objects == []`` means tracking completed with no detections.
    - ``external_wrenches is None`` means wrench topic/sample not supplied.
    - ``external_wrenches == []`` means explicitly empty wrench batch.
    - A new trajectory clears cycle-bound objects, geometry, state, and wrenches.
      State/wrench are cleared so a prior cycle cannot satisfy dynamic readiness
      for a new trajectory ID (Python V1 deterministic policy).
    """

    def __init__(self) -> None:
        self._lock = RLock()
        self._trajectory: Trajectory | None = None
        self._trajectory_id: int | None = None
        self._objects: list[TrackedObject] | None = None
        self._geometry: list[GeometryStep] | None = None
        self._state: RoverState | None = None
        self._objects_frame_id: str | None = None
        self._geometry_frame_id: str | None = None
        self._state_frame_id: str | None = None
        self._geometry_source_trajectory_id: int | None = None
        self._geometry_source_trajectory_stamp: float | None = None
        self._external_wrenches: list[ExternalWrenchData] | None = None
        self._external_wrenches_frame_id: str | None = None

    def set_trajectory(self, trajectory: Trajectory, *, trajectory_id: int | None = None) -> None:
        with self._lock:
            self._trajectory = trajectory
            self._trajectory_id = int(trajectory.timestamp) if trajectory_id is None else int(trajectory_id)
            self._objects = None
            self._geometry = None
            self._objects_frame_id = None
            self._geometry_frame_id = None
            self._geometry_source_trajectory_id = None
            self._geometry_source_trajectory_stamp = None
            # Clear dynamic inputs so prior-cycle state/wrench cannot satisfy
            # dynamic readiness for the new trajectory cycle.
            self._state = None
            self._state_frame_id = None
            self._external_wrenches = None
            self._external_wrenches_frame_id = None

    def set_objects(
        self,
        objects: list[TrackedObject],
        *,
        frame_id: str,
        source_trajectory_stamp: float | None = None,
    ) -> None:
        del source_trajectory_stamp  # accepted for API compatibility; objects are cycle-cleared
        with self._lock:
            self._objects = objects
            self._objects_frame_id = frame_id

    def set_geometry(
        self,
        geometry: list[GeometryStep],
        *,
        frame_id: str,
        source_trajectory_id: int | None = None,
        source_trajectory_stamp: float | None = None,
    ) -> None:
        with self._lock:
            self._geometry = geometry
            self._geometry_frame_id = frame_id
            self._geometry_source_trajectory_id = (
                int(source_trajectory_stamp)
                if source_trajectory_id is None and source_trajectory_stamp is not None
                else (None if source_trajectory_id is None else int(source_trajectory_id))
            )
            self._geometry_source_trajectory_stamp = source_trajectory_stamp

    def set_state(self, state: RoverState, *, frame_id: str) -> None:
        with self._lock:
            self._state = state
            self._state_frame_id = frame_id

    def set_external_wrenches(
        self,
        wrenches: list[ExternalWrenchData],
        *,
        frame_id: str,
    ) -> None:
        with self._lock:
            self._external_wrenches = wrenches
            self._external_wrenches_frame_id = frame_id

    def snapshot(self) -> PredictionSnapshot:
        with self._lock:
            return PredictionSnapshot(
                trajectory=self._trajectory,
                trajectory_id=self._trajectory_id,
                objects=self._objects,
                geometry=self._geometry,
                state=self._state,
                objects_frame_id=self._objects_frame_id,
                geometry_frame_id=self._geometry_frame_id,
                state_frame_id=self._state_frame_id,
                geometry_source_trajectory_id=self._geometry_source_trajectory_id,
                geometry_source_trajectory_stamp=self._geometry_source_trajectory_stamp,
                external_wrenches=self._external_wrenches,
                external_wrenches_frame_id=self._external_wrenches_frame_id,
            )
