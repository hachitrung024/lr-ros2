"""Bound terrain work to complete cells along a world-coordinate polyline."""

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class CorridorGrid:
    """Small lookup grid for fitting cells and their point-sampling halo."""

    origin: np.ndarray
    cell_size_m: float
    fit_mask: np.ndarray
    halo_m: float

    def allows_cell(self, key: tuple[int, int]) -> bool:
        """Return whether a world cell may be fitted or published."""
        x, y = np.asarray(key) - self.origin
        return bool(
            0 <= x < self.fit_mask.shape[0]
            and 0 <= y < self.fit_mask.shape[1]
            and self.fit_mask[x, y]
        )

    def point_mask(self, points_xy: np.ndarray) -> np.ndarray:
        """Select points in corridor cells, including neighboring fit support."""
        selected = self._lookup(points_xy)
        if self.halo_m > 0.0:
            outside = np.flatnonzero(~selected)
            points = points_xy[outside]
            for dx, dy in ((-1, -1), (-1, 1), (1, -1), (1, 1)):
                selected[outside] |= self._lookup(points + np.array([dx, dy]) * self.halo_m)
        return selected

    def _lookup(self, points_xy: np.ndarray) -> np.ndarray:
        indices = np.floor(points_xy / self.cell_size_m).astype(np.int64) - self.origin
        inside = np.all((indices >= 0) & (indices < self.fit_mask.shape), axis=1)
        selected = np.zeros(len(indices), dtype=bool)
        selected[inside] = self.fit_mask[indices[inside, 0], indices[inside, 1]]
        return selected


class TrajectoryCorridor:
    """A swept disk around path segments, including bends and endpoints."""

    def __init__(self, points_xy, half_width_m: float) -> None:
        points = np.asarray(points_xy, dtype=np.float64)
        if points.ndim != 2 or points.shape[1] != 2 or not 1 <= len(points) <= 256:
            raise ValueError("path must contain 1 to 256 XY positions")
        if not np.isfinite(points).all():
            raise ValueError("path positions must be finite")
        if not math.isfinite(half_width_m) or half_width_m <= 0.0:
            raise ValueError("corridor half width must be finite and positive")
        self._points = points.copy()
        self.half_width_m = float(half_width_m)

    def grid(
        self, rover_xy, radius_m: float, cell_size_m: float, halo_m: float,
    ) -> CorridorGrid:
        """Rasterize conservative cell coverage once, before filtering a cloud."""
        rover_xy = np.asarray(rover_xy, dtype=np.float64)
        origin = np.floor((rover_xy - radius_m) / cell_size_m).astype(np.int64)
        end = np.floor((rover_xy + radius_m) / cell_size_m).astype(np.int64)
        xx, yy = np.meshgrid(
            np.arange(origin[0], end[0] + 1),
            np.arange(origin[1], end[1] + 1),
            indexing="ij",
        )
        centers = (np.column_stack((xx.ravel(), yy.ravel())) + 0.5) * cell_size_m
        distances_squared = np.full(len(centers), np.inf)
        # Work scales with the local grid and path, never cloud size x path size.
        starts = self._points[:-1] if len(self._points) > 1 else self._points
        ends = self._points[1:] if len(self._points) > 1 else self._points
        for start, end_point in zip(starts, ends):
            segment = end_point - start
            length_squared = float(segment @ segment)
            delta = centers - start
            if length_squared > 0.0:
                fraction = np.clip((delta @ segment) / length_squared, 0.0, 1.0)
                delta -= fraction[:, None] * segment
            distances_squared = np.minimum(
                distances_squared, np.einsum("ij,ij->i", delta, delta)
            )
        # A circumscribed disk retains every cell intersecting the corridor.
        # Extra boundary cells preserve complete central samples for fitting.
        reach = self.half_width_m + cell_size_m / math.sqrt(2.0)
        return CorridorGrid(
            origin=origin,
            cell_size_m=cell_size_m,
            fit_mask=(distances_squared <= reach ** 2).reshape(xx.shape),
            # PlaneFitter reads only the eight immediately neighboring cells.
            halo_m=min(halo_m, cell_size_m),
        )
