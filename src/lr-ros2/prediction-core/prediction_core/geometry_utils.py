"""Vector and polygon helpers shared by collision and rollover predictors."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np
from numpy.typing import NDArray
from shapely.geometry import Polygon

from .models import TrajectoryStep, Vector3


GROUND_NORMAL_NZ_EPSILON = 1e-6


@dataclass(frozen=True)
class StabilityMargins:
    """Signed support-edge margins in the terrain-aligned rover frame."""

    front_m: float
    rear_m: float
    left_m: float
    right_m: float

    def minimum_m(self) -> float:
        return min(self.front_m, self.rear_m, self.left_m, self.right_m)

    def limiting_edge(self) -> str:
        """Edge with the smallest signed raw margin (nearest support edge)."""
        values = {
            "front": self.front_m,
            "rear": self.rear_m,
            "left": self.left_m,
            "right": self.right_m,
        }
        return min(values, key=values.get)


def rover_rectangle(
    step: TrajectoryStep, length_m: float, width_m: float, expansion_m: float = 0.0
) -> Polygon:
    """Build a physical or expanded rectangular footprint in the map frame."""
    if length_m <= 0 or width_m <= 0:
        raise ValueError("rover rectangle dimensions must be positive")
    if expansion_m < 0 or not math.isfinite(expansion_m):
        raise ValueError("rectangle expansion must be finite and non-negative")
    half_length = length_m / 2 + expansion_m
    half_width = width_m / 2 + expansion_m
    cosine, sine = math.cos(step.yaw), math.sin(step.yaw)
    corners = []
    for local_x, local_y in (
        (half_length, half_width),
        (-half_length, half_width),
        (-half_length, -half_width),
        (half_length, -half_width),
    ):
        corners.append(
            (
                step.x + cosine * local_x - sine * local_y,
                step.y + sine * local_x + cosine * local_y,
            )
        )
    return Polygon(corners)


def normalized_upward_normal(normal_xyz: Vector3) -> NDArray[np.float64]:
    """Normalize a terrain normal and canonicalize it to the upward hemisphere."""
    normal = np.asarray(normal_xyz, dtype=float)
    if not np.all(np.isfinite(normal)):
        raise ValueError("terrain normal must contain only finite values")
    magnitude = float(np.linalg.norm(normal))
    if magnitude <= 1e-12:
        raise ValueError("terrain normal must be non-zero")
    normal /= magnitude
    if normal[2] < 0:
        normal = -normal
    if abs(float(normal[2])) <= GROUND_NORMAL_NZ_EPSILON:
        raise ValueError("terrain normal is near-vertical and invalid as a V1 ground plane")
    return normal


def terrain_roll_pitch_rad(normal_xyz: Vector3, yaw: float) -> tuple[float, float]:
    """Return signed terrain roll and pitch relative to rover heading.

    Positive pitch means terrain rises in rover +X (forward), so the rover is
    nose-up. Positive roll follows the right-hand rule about rover +X: terrain
    rises toward rover +Y (left), so the left side is higher. Yaw is positive
    counter-clockwise about world +Z, from world +X toward world +Y.
    """
    if not math.isfinite(yaw):
        raise ValueError("yaw must be finite")
    normal = normalized_upward_normal(normal_xyz)
    forward = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    left = np.array([-math.sin(yaw), math.cos(yaw), 0.0])
    # On n.x = constant, vertical rise per horizontal metre along d is
    # dz/ds = -(n dot d) / nz. atan2 preserves the intended slope signs.
    pitch = math.atan2(-float(np.dot(normal, forward)), float(normal[2]))
    roll = math.atan2(-float(np.dot(normal, left)), float(normal[2]))
    return roll, pitch


def terrain_frame(normal_xyz: Vector3, yaw: float) -> NDArray[np.float64]:
    """Return rover-to-world rotation columns [forward, left, terrain-up]."""
    normal = normalized_upward_normal(normal_xyz)
    cosine, sine = math.cos(yaw), math.sin(yaw)
    # The forward tangent retains the requested yaw in horizontal projection.
    forward = np.array(
        [cosine, sine, -(float(normal[0]) * cosine + float(normal[1]) * sine) / normal[2]],
        dtype=float,
    )
    forward /= np.linalg.norm(forward)
    left = np.cross(normal, forward)
    left /= np.linalg.norm(left)
    return np.column_stack((forward, left, normal))


def project_point_on_support_along_direction_xy(
    normal_xyz: Vector3,
    yaw: float,
    point_xyz: Vector3,
    direction_world_xyz: Vector3,
    *,
    min_normal_component: float = 1e-9,
) -> tuple[float, float] | None:
    """Project a point onto the support plane along an arbitrary world direction.

    Coordinates of the returned point are in the terrain-aligned rover frame
    (support plane ``z = 0``). Returns ``None`` when ``direction`` is nearly
    parallel to the support plane (no stable intersection).
    """
    rotation_rover_to_world = terrain_frame(normal_xyz, yaw)
    direction_world = np.asarray(direction_world_xyz, dtype=float)
    if not np.all(np.isfinite(direction_world)):
        raise ValueError("projection direction must contain only finite values")
    if float(np.linalg.norm(direction_world)) <= 1e-12:
        return None
    direction_rover = rotation_rover_to_world.T @ direction_world
    normal_component = float(direction_rover[2])
    if abs(normal_component) <= min_normal_component:
        return None
    point = np.asarray(point_xyz, dtype=float)
    parameter = -float(point[2]) / normal_component
    projected = point + parameter * direction_rover
    return float(projected[0]), float(projected[1])


def projected_com_on_support_xy(
    normal_xyz: Vector3,
    yaw: float,
    com_x_m: float,
    com_y_m: float,
    com_height_m: float,
) -> tuple[float, float]:
    """Project the Center of Mass along world gravity onto the support plane.

    Coordinates are expressed in the terrain-aligned rover frame, where the
    support plane is z=0. This is a vector line-plane intersection, not an
    attitude-specific shortcut.
    """
    projected = project_point_on_support_along_direction_xy(
        normal_xyz,
        yaw,
        (com_x_m, com_y_m, com_height_m),
        (0.0, 0.0, -1.0),
    )
    if projected is None:
        raise ValueError("gravity does not intersect the rover support plane")
    return projected


def signed_distance_to_support_rectangle(
    point_xy: tuple[float, float], length_m: float, width_m: float
) -> float:
    """Return the minimum signed margin to a centered support rectangle edge.

    This is the V1 static-stability definition: positive inside, zero on a
    tipping edge, negative outside. It intentionally returns the minimum of
    the four directional edge margins rather than a corner Euclidean distance.
    """
    return support_edge_margins(point_xy, length_m, width_m).minimum_m()


def support_edge_margins(
    point_xy: tuple[float, float], length_m: float, width_m: float
) -> StabilityMargins:
    """Return signed margins to front, rear, left, and right support edges."""
    x, y = point_xy
    half_length, half_width = length_m / 2, width_m / 2
    return StabilityMargins(
        front_m=half_length - x,
        rear_m=x + half_length,
        left_m=half_width - y,
        right_m=y + half_width,
    )


def normalized_static_stability_margin(
    current_margins: StabilityMargins, reference_margins: StabilityMargins
) -> float:
    """Return the minimum edge-wise current/reference signed-margin ratio.

    Reference margins describe the configured CoM's flat-terrain position and
    must all be strictly inside the support polygon. The result is intentionally
    not clamped: zero is a tipping edge and negative values are beyond it.
    """
    reference_values = (
        reference_margins.front_m,
        reference_margins.rear_m,
        reference_margins.left_m,
        reference_margins.right_m,
    )
    if any(value <= 0 or not math.isfinite(value) for value in reference_values):
        raise ValueError("reference CoM margins must be finite and positive")
    return min(
        current_margins.front_m / reference_margins.front_m,
        current_margins.rear_m / reference_margins.rear_m,
        current_margins.left_m / reference_margins.left_m,
        current_margins.right_m / reference_margins.right_m,
    )

