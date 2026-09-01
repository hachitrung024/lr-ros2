"""Pure-Python runtime events (no ROS message types)."""

from __future__ import annotations

from dataclasses import dataclass

from .cache import ExternalWrenchData
from .models import GeometryStep, RoverState, TrackedObject, Trajectory


@dataclass(frozen=True)
class TrajectoryEvent:
    trajectory_id: int
    trajectory: Trajectory


@dataclass(frozen=True)
class ObjectsEvent:
    objects: list[TrackedObject]
    frame_id: str
    timestamp: float | None = None


@dataclass(frozen=True)
class GeometryEvent:
    geometry: list[GeometryStep]
    frame_id: str
    source_trajectory_id: int
    source_trajectory_stamp: float | None = None


@dataclass(frozen=True)
class StateEvent:
    state: RoverState
    frame_id: str


@dataclass(frozen=True)
class ExternalWrenchEvent:
    wrenches: list[ExternalWrenchData]
    frame_id: str
