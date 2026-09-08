"""Pure trajectory sampling and rigid-transform utilities."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class PoseSeries:
    """Strictly timestamped poses loaded from the cache bag."""

    stamps_ns: np.ndarray
    positions: np.ndarray
    orientations_xyzw: np.ndarray
    frame_id: str

    def __post_init__(self) -> None:
        """Validate array shapes and timestamp ordering."""
        count = len(self.stamps_ns)
        if count < 2:
            raise ValueError("pose cache must contain at least two poses")
        if self.positions.shape != (count, 3):
            raise ValueError("positions must have shape (N, 3)")
        if self.orientations_xyzw.shape != (count, 4):
            raise ValueError("orientations must have shape (N, 4)")
        if np.any(np.diff(self.stamps_ns) <= 0):
            raise ValueError("pose timestamps must be strictly increasing")

    def nearest_index(
        self, stamp_ns: int, tolerance_ns: int
    ) -> int | None:
        """Find the closest cached sample within the supplied tolerance."""
        insertion = int(np.searchsorted(self.stamps_ns, stamp_ns))
        candidates = []
        if insertion < len(self.stamps_ns):
            candidates.append(insertion)
        if insertion > 0:
            candidates.append(insertion - 1)
        if not candidates:
            return None
        index = min(
            candidates,
            key=lambda item: (
                abs(int(self.stamps_ns[item]) - stamp_ns),
                int(self.stamps_ns[item]) > stamp_ns,
            ),
        )
        if abs(int(self.stamps_ns[index]) - stamp_ns) > tolerance_ns:
            return None
        return index

    def future_indices(
        self,
        current_index: int,
        *,
        radius_m: float,
        step_m: float,
        max_gap_ns: int,
        max_points: int,
        max_horizon_ns: int | None = None,
    ) -> list[int]:
        """Sample forward within along-path distance and time budgets."""
        if radius_m <= 0.0:
            raise ValueError("radius_m must be positive")
        if step_m <= 0.0:
            raise ValueError("step_m must be positive")
        if max_gap_ns <= 0:
            raise ValueError("max_gap_ns must be positive")
        if max_points < 1:
            raise ValueError("max_points must be at least one")
        if max_horizon_ns is not None and max_horizon_ns <= 0:
            raise ValueError("max_horizon_ns must be positive when provided")
        if not 0 <= current_index < len(self.stamps_ns):
            raise IndexError("current_index is outside the pose series")

        selected = [current_index]
        last_selected = current_index
        path_distance = 0.0
        for index in range(current_index + 1, len(self.stamps_ns)):
            if (
                int(self.stamps_ns[index] - self.stamps_ns[index - 1])
                > max_gap_ns
            ):
                break
            if (
                max_horizon_ns is not None
                and int(self.stamps_ns[index] - self.stamps_ns[current_index])
                > max_horizon_ns
            ):
                break
            distance_from_selected = float(
                np.linalg.norm(
                    self.positions[index, :2]
                    - self.positions[last_selected, :2]
                )
            )
            if distance_from_selected < step_m:
                continue
            if path_distance + distance_from_selected > radius_m:
                break
            selected.append(index)
            last_selected = index
            path_distance += distance_from_selected
            if len(selected) >= max_points:
                return selected
        return selected


def normalize_quaternion(quaternion_xyzw: np.ndarray) -> np.ndarray:
    """Return a normalized quaternion in ROS XYZW order."""
    quaternion = np.asarray(quaternion_xyzw, dtype=np.float64)
    norm = float(np.linalg.norm(quaternion))
    if norm < 1e-12:
        raise ValueError("quaternion norm is zero")
    return quaternion / norm


def quaternion_multiply(
    left: np.ndarray, right: np.ndarray
) -> np.ndarray:
    """Compose two quaternions in ROS XYZW order."""
    lx, ly, lz, lw = normalize_quaternion(left)
    rx, ry, rz, rw = normalize_quaternion(right)
    return normalize_quaternion(
        np.array(
            [
                lw * rx + lx * rw + ly * rz - lz * ry,
                lw * ry - lx * rz + ly * rw + lz * rx,
                lw * rz + lx * ry - ly * rx + lz * rw,
                lw * rw - lx * rx - ly * ry - lz * rz,
            ],
            dtype=np.float64,
        )
    )


def quaternion_inverse(quaternion_xyzw: np.ndarray) -> np.ndarray:
    """Invert a unit quaternion in ROS XYZW order."""
    x, y, z, w = normalize_quaternion(quaternion_xyzw)
    return np.array([-x, -y, -z, w], dtype=np.float64)


def rotate_vector(
    quaternion_xyzw: np.ndarray, vector: np.ndarray
) -> np.ndarray:
    """Rotate a 3D vector by a quaternion."""
    q = normalize_quaternion(quaternion_xyzw)
    xyz = q[:3]
    value = np.asarray(vector, dtype=np.float64)
    return (
        value
        + 2.0 * q[3] * np.cross(xyz, value)
        + 2.0 * np.cross(xyz, np.cross(xyz, value))
    )


def reanchor_pose(
    cached_current_position: np.ndarray,
    cached_current_orientation: np.ndarray,
    cached_future_position: np.ndarray,
    cached_future_orientation: np.ndarray,
    runtime_position: np.ndarray,
    runtime_orientation: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Apply a cached relative pose to the current runtime map pose."""
    cached_inverse = quaternion_inverse(cached_current_orientation)
    relative_position = rotate_vector(
        cached_inverse,
        np.asarray(cached_future_position)
        - np.asarray(cached_current_position),
    )
    relative_orientation = quaternion_multiply(
        cached_inverse, cached_future_orientation
    )
    runtime_orientation = normalize_quaternion(runtime_orientation)
    output_position = np.asarray(runtime_position) + rotate_vector(
        runtime_orientation, relative_position
    )
    output_orientation = quaternion_multiply(
        runtime_orientation, relative_orientation
    )
    return output_position, output_orientation
