"""Internal prediction contracts, independent of ROS message definitions."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import TypeAlias


Identifier: TypeAlias = int | str
Point2: TypeAlias = tuple[float, float]
Vector2: TypeAlias = tuple[float, float]
Vector3: TypeAlias = tuple[float, float, float]


def _require_finite(name: str, *values: float) -> None:
    if not all(math.isfinite(value) for value in values):
        raise ValueError(f"{name} must contain only finite values")


def _require_optional_finite(name: str, value: float | None) -> None:
    if value is not None:
        _require_finite(name, value)


@dataclass(frozen=True)
class TrajectoryStep:
    step_id: int
    x: float
    y: float
    yaw: float

    def __post_init__(self) -> None:
        _require_finite("trajectory step coordinates/yaw", self.x, self.y, self.yaw)


@dataclass(frozen=True)
class Trajectory:
    timestamp: float
    frame_id: str
    steps: list[TrajectoryStep]

    def __post_init__(self) -> None:
        _require_finite("trajectory timestamp", self.timestamp)
        if not self.frame_id.strip():
            raise ValueError("trajectory frame_id must not be empty")
        if not self.steps:
            raise ValueError("trajectory must contain at least one step")
        ids = [step.step_id for step in self.steps]
        if len(ids) != len(set(ids)):
            raise ValueError("trajectory step_id values must be unique")

    def cumulative_distances_m(self) -> list[float]:
        """Return planar route distance at every step, measured from steps[0]."""
        distances = [0.0]
        for previous, current in zip(self.steps, self.steps[1:]):
            distances.append(
                distances[-1] + math.hypot(current.x - previous.x, current.y - previous.y)
            )
        return distances


@dataclass(frozen=True)
class TrackedObject:
    timestamp: float
    track_id: Identifier
    class_name: str
    footprint_polygon_xy: list[Point2]
    height_m: float | None = None
    velocity_xy: Vector2 | None = None
    confidence: float | None = None

    def __post_init__(self) -> None:
        _require_finite("tracked object timestamp", self.timestamp)
        if not isinstance(self.track_id, (int, str)) or isinstance(self.track_id, bool):
            raise ValueError("track_id must be an int or str")
        if not self.class_name.strip():
            raise ValueError("tracked object class_name must not be empty")
        if len(self.footprint_polygon_xy) < 3:
            raise ValueError(f"object {self.track_id!r} polygon requires at least 3 points")
        for point in self.footprint_polygon_xy:
            if len(point) != 2:
                raise ValueError(f"object {self.track_id!r} polygon points must be (x, y)")
            _require_finite(f"object {self.track_id!r} polygon", *point)
        if len(set(self.footprint_polygon_xy)) < 3:
            raise ValueError(f"object {self.track_id!r} polygon requires 3 unique points")
        _require_optional_finite("tracked object height_m", self.height_m)
        if self.height_m is not None and self.height_m < 0:
            raise ValueError("tracked object height_m must be non-negative")
        if self.velocity_xy is not None:
            _require_finite("tracked object velocity_xy", *self.velocity_xy)
        _require_optional_finite("tracked object confidence", self.confidence)


@dataclass(frozen=True)
class GeometryStep:
    timestamp: float
    step_id: int
    plane_id: Identifier
    normal_xyz: Vector3
    centroid_xyz: Vector3 | None = None
    confidence: float | None = None

    def __post_init__(self) -> None:
        _require_finite("geometry timestamp", self.timestamp)
        _require_finite("terrain normal", *self.normal_xyz)
        if math.sqrt(sum(component * component for component in self.normal_xyz)) <= 1e-12:
            raise ValueError(f"geometry step {self.step_id} has a zero normal")
        if self.centroid_xyz is not None:
            _require_finite("geometry centroid_xyz", *self.centroid_xyz)
        _require_optional_finite("geometry confidence", self.confidence)


@dataclass(frozen=True)
class RoverState:
    """Kinematic rover sample in the common ``map`` frame.

    Acceleration semantics
    ----------------------
    ``acceleration_xyz`` is the kinematic CoM acceleration in ``map`` (m/s²)
    **without gravity**. A stationary rover reports ``(0, 0, 0)``. Gravity is
    introduced separately by Prediction as ``g_world = (0, 0, -9.80665)``.

    Do **not** feed raw accelerometer specific-force into ``acceleration_xyz``.

    ``acceleration_xyz is None`` means acceleration is unavailable. That is
    distinct from a valid zero acceleration ``(0, 0, 0)``. Dynamic metrics that
    require acceleration must be marked unavailable when the value is ``None``;
    they must never silently substitute zero.

    ``acceleration_xy`` is retained for compatibility and is **not** promoted to
    a 3-D acceleration (missing ``az`` would invent data).
    """

    timestamp: float
    x: float | None = None
    y: float | None = None
    yaw: float | None = None
    roll: float | None = None
    pitch: float | None = None
    velocity_xy: Vector2 | None = None
    acceleration_xy: Vector2 | None = None
    angular_velocity_xyz: Vector3 | None = None
    velocity_xyz: Vector3 | None = None
    acceleration_xyz: Vector3 | None = None
    angular_acceleration_xyz: Vector3 | None = None

    def __post_init__(self) -> None:
        _require_finite("rover state timestamp", self.timestamp)
        for name in ("x", "y", "yaw", "roll", "pitch"):
            _require_optional_finite(f"rover state {name}", getattr(self, name))
        for name in (
            "velocity_xy",
            "acceleration_xy",
            "angular_velocity_xyz",
            "velocity_xyz",
            "acceleration_xyz",
            "angular_acceleration_xyz",
        ):
            value = getattr(self, name)
            if value is not None:
                _require_finite(f"rover state {name}", *value)

    def resolved_acceleration_xyz(self) -> Vector3 | None:
        """Return authoritative 3-D kinematic acceleration, or ``None`` if unknown."""
        return self.acceleration_xyz


@dataclass(frozen=True)
class ExternalWrench:
    """External force/torque sample in the common ``map`` frame.

    Semantics:
    - ``force_xyz`` in newtons, ``torque_xyz`` in newton-metres (free couple)
    - ``application_point_xyz`` in metres in ``map`` when known
    - missing application point ⇒ force moment-arm contribution is unavailable
      (free torque may still be applied); do not invent a point at the CoM
    """

    source: str
    force_xyz: Vector3
    torque_xyz: Vector3
    application_point_xyz: Vector3 | None = None
    confidence: float | None = None
    timestamp: float | None = None
    frame_id: str = "map"

    def __post_init__(self) -> None:
        if not self.source.strip():
            raise ValueError("external wrench source must not be empty")
        if not self.frame_id.strip():
            raise ValueError("external wrench frame_id must not be empty")
        _require_finite("external wrench force_xyz", *self.force_xyz)
        _require_finite("external wrench torque_xyz", *self.torque_xyz)
        if self.application_point_xyz is not None:
            _require_finite("external wrench application_point_xyz", *self.application_point_xyz)
        _require_optional_finite("external wrench confidence", self.confidence)
        _require_optional_finite("external wrench timestamp", self.timestamp)


@dataclass(frozen=True)
class CollisionObject:
    object_id: Identifier
    object_class: str
    min_distance_m: float
    confidence_or_validity: float | None


@dataclass(frozen=True)
class CollisionStep:
    step_id: int
    distance_along_route_m: float
    collision_objects: list[CollisionObject]


@dataclass(frozen=True)
class CriticalTipEvidence:
    """Secondary diagnostic: ideal geometric tip angles for flat configured support.

    Hierarchy role: SECONDARY DIAGNOSTIC (vehicle/config property, not a route-step
    primary metric). Geometric limits ``atan(reference_margin / h)`` only — not
    Decision thresholds.

    ``critical_edge`` is the edge with the smallest tip angle (legacy name).
    Prefer ``minimum_tip_angle_edge`` in new code.
    """

    front_deg: float
    rear_deg: float
    left_deg: float
    right_deg: float
    minimum_deg: float
    critical_edge: str

    @property
    def minimum_tip_angle_edge(self) -> str:
        """Alias for ``critical_edge``: edge with the smallest tip angle."""
        return self.critical_edge


@dataclass(frozen=True)
class DynamicStabilityEvidence:
    """Dynamic rollover evidence under the point-mass / translational model.

    Hierarchy (Prediction Python V1)
    --------------------------------
    - PRIMARY DYNAMIC: Stability Moment (``edge_stability_moments_nm``,
      ``minimum_stability_moment_nm``, ``normalized_minimum_stability_moment``,
      ``minimum_normalized_moment_edge`` / legacy ``critical_edge``)
    - OPTIONAL DIAGNOSTIC: Point-mass ZMP (``zmp_*``)
    - SECONDARY DIAGNOSTIC: Effective-gravity SSM (``effective_*``)

    Effective SSM ≈ point-mass ZMP margin when gravity + translational
    acceleration are the only loads and ``external_wrenches=[]``. They diverge
    when external forces/torques are included.

    Edge naming
    -----------
    - ``nearest_effective_edge`` / ``nearest_zmp_edge``: min **raw** support margin
    - ``critical_edge`` / ``minimum_normalized_moment_edge``: min **normalized**
      stability moment (not necessarily the nearest geometric edge)

    Model scope: gravity, translational acceleration, external force with known
    application point, external free torque. No ``-Iα`` / ``-ω×Iω``.
    """

    acceleration_available: bool
    external_wrench_available: bool
    external_wrench_included: bool
    effective_force_xyz_n: Vector3 | None
    effective_gravity_projection_xy: Vector2 | None
    effective_ssm_m: float | None
    normalized_effective_ssm: float | None
    zmp_xy: Vector2 | None
    zmp_margin_m: float | None
    normalized_zmp_margin: float | None
    edge_stability_moments_nm: dict[str, float] | None
    minimum_stability_moment_nm: float | None
    normalized_minimum_stability_moment: float | None
    critical_edge: str | None
    valid: bool
    validity_reason: str
    assumptions: tuple[str, ...] = ()
    normalized_edge_stability_moments: dict[str, float] | None = None
    nearest_effective_edge: str | None = None
    nearest_zmp_edge: str | None = None

    @property
    def minimum_normalized_moment_edge(self) -> str | None:
        """Alias for ``critical_edge``: most depleted normalized stability edge."""
        return self.critical_edge


@dataclass(frozen=True)
class RolloverStep:
    """One trajectory-step rollover evidence package (Prediction Python V1).

    Primary baseline (always when geometry matches):
      ``predicted_roll_deg``, ``predicted_pitch_deg``,
      ``static_stability_margin_m``, ``normalized_static_stability_margin``,
      ``nearest_static_edge``

    Nested:
      ``critical_tip`` — secondary diagnostic
      ``dynamic_stability`` — primary dynamic + optional ZMP + secondary effective SSM
    """

    step_id: int
    predicted_roll_deg: float
    predicted_pitch_deg: float
    static_stability_margin_m: float
    normalized_static_stability_margin: float
    terrain_id: Identifier
    confidence_or_validity: float | None
    critical_tip: CriticalTipEvidence | None = None
    dynamic_stability: DynamicStabilityEvidence | None = None
    nearest_static_edge: str | None = None


@dataclass(frozen=True)
class PredictionOutput:
    timestamp: float
    source_trajectory_stamp: float
    collision_steps: list[CollisionStep] = field(default_factory=list)
    rollover_steps: list[RolloverStep] = field(default_factory=list)

