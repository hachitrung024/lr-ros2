"""Shared helpers for Prediction upstream adapters."""

from __future__ import annotations

import math
from typing import Iterable, Sequence


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Extract yaw (rotation about +Z) from a geometry_msgs quaternion."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def subsample_indices(n: int, *, horizon_steps: int, stride: int) -> list[int]:
    """Pick deterministic indices from a Path of length n for Prediction steps.

    Returns indices into the original Path poses. Always includes 0 when n > 0.
    """
    if n <= 0:
        return []
    if horizon_steps < 1:
        raise ValueError("horizon_steps must be >= 1")
    if stride < 1:
        raise ValueError("stride must be >= 1")

    indices = list(range(0, n, stride))
    if indices[-1] != n - 1:
        indices.append(n - 1)
    if len(indices) > horizon_steps:
        # Keep first + evenly spaced remaining including last.
        if horizon_steps == 1:
            return [0]
        kept = [0]
        inner = horizon_steps - 2
        if inner > 0:
            span = len(indices) - 2
            for k in range(1, inner + 1):
                pos = 1 + (k * span) // (inner + 1)
                kept.append(indices[pos])
        kept.append(indices[-1])
        # Deduplicate while preserving order.
        out: list[int] = []
        for idx in kept:
            if not out or out[-1] != idx:
                out.append(idx)
        return out
    return indices


def finite_difference(
    previous: Sequence[float],
    current: Sequence[float],
    dt: float,
) -> tuple[float, ...]:
    if dt <= 0.0 or not math.isfinite(dt):
        raise ValueError("dt must be finite and > 0")
    return tuple((c - p) / dt for p, c in zip(previous, current))
