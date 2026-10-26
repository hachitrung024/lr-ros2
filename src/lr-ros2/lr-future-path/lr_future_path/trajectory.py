"""Quaternion operations shared by DB ground truth and MAVROS pose."""

import numpy as np


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
