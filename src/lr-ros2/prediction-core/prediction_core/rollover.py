"""Terrain-attitude R0, static SSM R1, and extended rollover evidence."""

from __future__ import annotations

import logging
import math

import numpy as np

from .config import RoverConfig
from .geometry_utils import (
    StabilityMargins,
    normalized_static_stability_margin,
    project_point_on_support_along_direction_xy,
    projected_com_on_support_xy,
    support_edge_margins,
    terrain_frame,
    terrain_roll_pitch_rad,
)
from .models import (
    CriticalTipEvidence,
    DynamicStabilityEvidence,
    ExternalWrench,
    GeometryStep,
    RolloverStep,
    RoverState,
    Trajectory,
    Vector3,
)


LOGGER = logging.getLogger(__name__)

# Gravitational acceleration in the common map / ENU frame (+Z up).
# This is NOT accelerometer specific force.
GRAVITY_WORLD_M_S2: Vector3 = (0.0, 0.0, -9.80665)
_EDGE_ORDER = ("front", "rear", "left", "right")
_FZ_MIN = 1e-6


def compute_critical_tip_evidence(config: RoverConfig) -> CriticalTipEvidence:
    """Ideal geometric tip angles for the flat configured support rectangle + CoM."""
    height = config.com_height_m
    if not math.isfinite(height) or height <= 0.0:
        raise ValueError("com_height_m must be finite and positive for tip angles")
    reference = support_edge_margins(
        (config.com_x_m, config.com_y_m),
        config.support_length_m,
        config.support_width_m,
    )
    margins = {
        "front": reference.front_m,
        "rear": reference.rear_m,
        "left": reference.left_m,
        "right": reference.right_m,
    }
    if any(not math.isfinite(value) or value <= 0.0 for value in margins.values()):
        raise ValueError("reference support margins must be finite and positive")
    angles = {
        edge: math.degrees(math.atan(value / height)) for edge, value in margins.items()
    }
    critical_edge = min(angles, key=angles.get)
    return CriticalTipEvidence(
        front_deg=angles["front"],
        rear_deg=angles["rear"],
        left_deg=angles["left"],
        right_deg=angles["right"],
        minimum_deg=angles[critical_edge],
        critical_edge=critical_edge,
    )


def _edge_point_and_axis(config: RoverConfig) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    half_length = config.support_length_m / 2.0
    half_width = config.support_width_m / 2.0
    up = np.array([0.0, 0.0, 1.0])
    outward = {
        "front": np.array([1.0, 0.0, 0.0]),
        "rear": np.array([-1.0, 0.0, 0.0]),
        "left": np.array([0.0, 1.0, 0.0]),
        "right": np.array([0.0, -1.0, 0.0]),
    }
    points = {
        "front": np.array([half_length, 0.0, 0.0]),
        "rear": np.array([-half_length, 0.0, 0.0]),
        "left": np.array([0.0, half_width, 0.0]),
        "right": np.array([0.0, -half_width, 0.0]),
    }
    return {edge: (points[edge], np.cross(outward[edge], up)) for edge in _EDGE_ORDER}


def _to_support(rotation_rover_to_world: np.ndarray, vector_world: np.ndarray) -> np.ndarray:
    return rotation_rover_to_world.T @ vector_world


def _accumulate_external_wrenches(
    wrenches: list[ExternalWrench],
    rotation_rover_to_world: np.ndarray,
    origin_world: np.ndarray,
    assumptions: list[str],
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    force_s = np.zeros(3)
    moment_s = np.zeros(3)
    force_world = np.zeros(3)
    for wrench in wrenches:
        force_w = np.asarray(wrench.force_xyz, dtype=float)
        torque_w = np.asarray(wrench.torque_xyz, dtype=float)
        force_local = _to_support(rotation_rover_to_world, force_w)
        torque_local = _to_support(rotation_rover_to_world, torque_w)
        moment_s = moment_s + torque_local
        if float(np.linalg.norm(force_w)) == 0.0:
            continue
        if wrench.application_point_xyz is None:
            assumptions.append(
                f"external force from {wrench.source!r} omitted from moment/ZMP arm "
                "(application_point_xyz missing; free torque still included)"
            )
            continue
        point_local = rotation_rover_to_world.T @ (
            np.asarray(wrench.application_point_xyz, dtype=float) - origin_world
        )
        force_s = force_s + force_local
        force_world = force_world + force_w
        moment_s = moment_s + np.cross(point_local, force_local)
    return force_s, moment_s, force_world


def compute_dynamic_stability_evidence(
    config: RoverConfig,
    *,
    normal_xyz: Vector3,
    yaw: float,
    step_x: float,
    step_y: float,
    state: RoverState | None,
    external_wrenches: list[ExternalWrench] | None,
    reference_margins: StabilityMargins,
) -> DynamicStabilityEvidence:
    """Effective-gravity SSM, edge stability moments, and point-mass ZMP."""
    assumptions: list[str] = [
        "map/ENU frame with g_world=(0,0,-9.80665) m/s^2",
        "acceleration_xyz is kinematic CoM acceleration excluding gravity",
        "angular_velocity_xyz is not consumed (no inertia tensor configured)",
        "stability moment and ZMP omit rotational inertia (-I alpha, -w x Iw)",
        "ZMP is point-mass/translational, not full rigid-body ZMP",
        "canonical FASM and LTR are not implemented",
    ]

    acceleration_xyz = None if state is None else state.resolved_acceleration_xyz()
    acceleration_available = acceleration_xyz is not None
    external_available = external_wrenches is not None

    if not acceleration_available:
        assumptions.append(
            "acceleration_xyz unavailable; effective SSM / ZMP / inertial moment omitted "
            "(None is never treated as zero)"
        )
        return DynamicStabilityEvidence(
            acceleration_available=False,
            external_wrench_available=external_available,
            external_wrench_included=False,
            effective_force_xyz_n=None,
            effective_gravity_projection_xy=None,
            effective_ssm_m=None,
            normalized_effective_ssm=None,
            zmp_xy=None,
            zmp_margin_m=None,
            normalized_zmp_margin=None,
            edge_stability_moments_nm=None,
            minimum_stability_moment_nm=None,
            normalized_minimum_stability_moment=None,
            critical_edge=None,
            valid=False,
            validity_reason="acceleration_xyz unavailable",
            assumptions=tuple(assumptions),
            normalized_edge_stability_moments=None,
            nearest_effective_edge=None,
            nearest_zmp_edge=None,
        )

    assert acceleration_xyz is not None
    mass = config.mass_kg
    g_world = np.asarray(GRAVITY_WORLD_M_S2, dtype=float)
    a_world = np.asarray(acceleration_xyz, dtype=float)
    g_eff_world = g_world - a_world

    rotation = terrain_frame(normal_xyz, yaw)
    origin_world = np.array([step_x, step_y, 0.0], dtype=float)
    com_s = np.array([config.com_x_m, config.com_y_m, config.com_height_m], dtype=float)

    projected = project_point_on_support_along_direction_xy(
        normal_xyz,
        yaw,
        (config.com_x_m, config.com_y_m, config.com_height_m),
        (float(g_eff_world[0]), float(g_eff_world[1]), float(g_eff_world[2])),
    )
    if projected is None:
        assumptions.append("g_eff nearly parallel to support plane; effective SSM undefined")
        effective_projection = None
        effective_ssm_m = None
        normalized_effective_ssm = None
        nearest_effective_edge = None
        projection_ok = False
    else:
        effective_projection = projected
        eff_margins = support_edge_margins(
            projected,
            config.support_length_m,
            config.support_width_m,
        )
        effective_ssm_m = eff_margins.minimum_m()
        normalized_effective_ssm = normalized_static_stability_margin(
            eff_margins, reference_margins
        )
        nearest_effective_edge = eff_margins.limiting_edge()
        projection_ok = True

    force_gravity_s = _to_support(rotation, mass * g_world)
    force_inertial_s = _to_support(rotation, -mass * a_world)
    force_total_s = force_gravity_s + force_inertial_s
    moment_total_s = np.cross(com_s, force_gravity_s + force_inertial_s)
    effective_force_world = mass * g_eff_world

    if external_wrenches is None:
        assumptions.append("external_wrenches=None; external loads not included")
        external_included = False
    else:
        assumptions.append(
            "external_wrenches explicitly supplied "
            f"(count={len(external_wrenches)}; [] means known-empty)"
        )
        force_ext_s, moment_ext_s, force_ext_world = _accumulate_external_wrenches(
            external_wrenches, rotation, origin_world, assumptions
        )
        force_total_s = force_total_s + force_ext_s
        moment_total_s = moment_total_s + moment_ext_s
        effective_force_world = effective_force_world + force_ext_world
        external_included = True

    fz = float(force_total_s[2])
    if abs(fz) < _FZ_MIN:
        zmp_xy = None
        zmp_margin_m = None
        normalized_zmp_margin = None
        nearest_zmp_edge = None
        zmp_ok = False
        assumptions.append("|Fz| too small to define a meaningful support ZMP")
    else:
        # r_zmp=(x,y,0): Mx=y*Fz, My=-x*Fz => x=-My/Fz, y=Mx/Fz
        zmp_xy = (-float(moment_total_s[1]) / fz, float(moment_total_s[0]) / fz)
        zmp_margins = support_edge_margins(
            zmp_xy,
            config.support_length_m,
            config.support_width_m,
        )
        zmp_margin_m = zmp_margins.minimum_m()
        normalized_zmp_margin = normalized_static_stability_margin(
            zmp_margins, reference_margins
        )
        nearest_zmp_edge = zmp_margins.limiting_edge()
        zmp_ok = True

    edge_defs = _edge_point_and_axis(config)
    edge_moments: dict[str, float] = {}
    for edge, (point, axis) in edge_defs.items():
        moment_at_edge = moment_total_s + np.cross(-point, force_total_s)
        edge_moments[edge] = float(np.dot(moment_at_edge, axis))

    gravity_mag = abs(float(GRAVITY_WORLD_M_S2[2]))
    reference_moment = {
        "front": mass * gravity_mag * reference_margins.front_m,
        "rear": mass * gravity_mag * reference_margins.rear_m,
        "left": mass * gravity_mag * reference_margins.left_m,
        "right": mass * gravity_mag * reference_margins.right_m,
    }
    normalized_moments = {
        edge: edge_moments[edge] / reference_moment[edge] for edge in _EDGE_ORDER
    }
    # Critical edge is the least-normalized restoring moment (not the smallest N·m),
    # so a wider track does not permanently dominate a shorter wheelbase.
    critical_edge = min(normalized_moments, key=normalized_moments.get)
    minimum_moment = edge_moments[critical_edge]
    normalized_minimum = normalized_moments[critical_edge]

    valid = projection_ok and zmp_ok
    if valid:
        reason = "dynamic evidence computed from acceleration and wrench semantics"
    elif not projection_ok and not zmp_ok:
        reason = "effective projection and ZMP undefined"
    elif not projection_ok:
        reason = "effective-gravity projection undefined"
    else:
        reason = "ZMP undefined"

    return DynamicStabilityEvidence(
        acceleration_available=True,
        external_wrench_available=external_available,
        external_wrench_included=external_included,
        effective_force_xyz_n=(
            float(effective_force_world[0]),
            float(effective_force_world[1]),
            float(effective_force_world[2]),
        ),
        effective_gravity_projection_xy=effective_projection,
        effective_ssm_m=effective_ssm_m,
        normalized_effective_ssm=normalized_effective_ssm,
        zmp_xy=zmp_xy,
        zmp_margin_m=zmp_margin_m,
        normalized_zmp_margin=normalized_zmp_margin,
        edge_stability_moments_nm=edge_moments,
        minimum_stability_moment_nm=minimum_moment,
        normalized_minimum_stability_moment=normalized_minimum,
        critical_edge=critical_edge,
        valid=valid,
        validity_reason=reason,
        assumptions=tuple(assumptions),
        normalized_edge_stability_moments=dict(normalized_moments),
        nearest_effective_edge=nearest_effective_edge,
        nearest_zmp_edge=nearest_zmp_edge,
    )


class RolloverPredictor:
    """Predict terrain-following attitude, static SSM, and extended evidence."""

    def __init__(self, config: RoverConfig) -> None:
        self.config = config
        self.last_missing_step_ids: list[int] = []
        self._last_logged_missing_step_ids: tuple[int, ...] = ()
        self._critical_tip = compute_critical_tip_evidence(config)

    def predict(
        self,
        trajectory: Trajectory,
        geometry: list[GeometryStep],
        state: RoverState | None = None,
        external_wrenches: list[ExternalWrench] | None = None,
    ) -> list[RolloverStep]:
        geometry_by_step: dict[int, GeometryStep] = {}
        for geometry_step in geometry:
            if geometry_step.step_id in geometry_by_step:
                raise ValueError(f"duplicate GeometryStep for step_id {geometry_step.step_id}")
            geometry_by_step[geometry_step.step_id] = geometry_step

        self.last_missing_step_ids = []
        output: list[RolloverStep] = []
        reference_margins = support_edge_margins(
            (self.config.com_x_m, self.config.com_y_m),
            self.config.support_length_m,
            self.config.support_width_m,
        )
        for step in trajectory.steps:
            terrain = geometry_by_step.get(step.step_id)
            if terrain is None:
                self.last_missing_step_ids.append(step.step_id)
                continue
            roll_rad, pitch_rad = terrain_roll_pitch_rad(terrain.normal_xyz, step.yaw)
            projected_com = projected_com_on_support_xy(
                terrain.normal_xyz,
                step.yaw,
                self.config.com_x_m,
                self.config.com_y_m,
                self.config.com_height_m,
            )
            current_margins = support_edge_margins(
                projected_com,
                self.config.support_length_m,
                self.config.support_width_m,
            )
            margin = current_margins.minimum_m()
            normalized_margin = normalized_static_stability_margin(
                current_margins, reference_margins
            )
            dynamic = compute_dynamic_stability_evidence(
                self.config,
                normal_xyz=terrain.normal_xyz,
                yaw=step.yaw,
                step_x=step.x,
                step_y=step.y,
                state=state,
                external_wrenches=external_wrenches,
                reference_margins=reference_margins,
            )
            output.append(
                RolloverStep(
                    step_id=step.step_id,
                    predicted_roll_deg=math.degrees(roll_rad),
                    predicted_pitch_deg=math.degrees(pitch_rad),
                    static_stability_margin_m=margin,
                    normalized_static_stability_margin=normalized_margin,
                    terrain_id=terrain.plane_id,
                    confidence_or_validity=terrain.confidence,
                    critical_tip=self._critical_tip,
                    dynamic_stability=dynamic,
                    nearest_static_edge=current_margins.limiting_edge(),
                )
            )
        missing_step_ids = tuple(self.last_missing_step_ids)
        if missing_step_ids and missing_step_ids != self._last_logged_missing_step_ids:
            LOGGER.warning(
                "missing geometry for %d trajectory steps: %s",
                len(missing_step_ids),
                self.last_missing_step_ids,
            )
        self._last_logged_missing_step_ids = missing_step_ids
        return output
