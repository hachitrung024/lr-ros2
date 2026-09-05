"""Shared GridMap sampling for prediction adapters and presentation."""

from dataclasses import dataclass
import math

import numpy as np
from grid_map_msgs.msg import GridMap


@dataclass(frozen=True)
class TerrainSample:
    """Terrain values at one XY query position."""

    valid: bool
    elevation_m: float = math.nan
    slope_deg: float = math.nan
    normal_xyz: tuple[float, float, float] | None = None
    confidence: float | None = None
    plane_id: str = ""


class GridMapSampler:
    """Read world-coordinate terrain samples from a GridMap message."""

    def __init__(self, message: GridMap) -> None:
        """Decode terrain layers and map geometry once per input message."""
        if not math.isfinite(message.info.resolution):
            raise ValueError("GridMap resolution must be finite")
        self._resolution = float(message.info.resolution)
        if self._resolution <= 0.0:
            raise ValueError("GridMap resolution must be positive")
        self._layers = {name: self._decode_layer(message, name) for name in message.layers}
        for required in ("elevation", "slope_deg", "state"):
            if required not in self._layers:
                raise ValueError(f"GridMap is missing layer '{required}'")

        shapes = {layer.shape for layer in self._layers.values()}
        if len(shapes) != 1:
            raise ValueError("GridMap layers do not have a common shape")
        self._rows, self._columns = next(iter(shapes))
        if self._rows < 1 or self._columns < 1:
            raise ValueError("GridMap layers must not be empty")

        pose = message.info.pose
        values = (
            pose.position.x,
            pose.position.y,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("GridMap pose must be finite")
        self._center_x = float(pose.position.x)
        self._center_y = float(pose.position.y)
        self._yaw = _quaternion_yaw(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        self._length_x = self._rows * self._resolution
        self._length_y = self._columns * self._resolution
        self._outer_start = int(message.outer_start_index) % self._rows
        self._inner_start = int(message.inner_start_index) % self._columns

    @staticmethod
    def _decode_layer(message: GridMap, name: str) -> np.ndarray:
        index = message.layers.index(name)
        try:
            layer = message.data[index]
        except IndexError as error:
            raise ValueError(f"GridMap has no data for layer '{name}'") from error
        if len(layer.layout.dim) < 2:
            raise ValueError(f"GridMap layer '{name}' has no 2D layout")
        columns = int(layer.layout.dim[0].size)
        rows = int(layer.layout.dim[1].size)
        if rows <= 0 or columns <= 0:
            raise ValueError(f"GridMap layer '{name}' has invalid dimensions")
        offset = int(layer.layout.data_offset)
        count = rows * columns
        values = np.asarray(layer.data, dtype=np.float64)
        if offset < 0 or values.size < offset + count:
            raise ValueError(f"GridMap layer '{name}' has incomplete data")
        return values[offset:offset + count].reshape(
            (rows, columns),
            order="F",
        )

    def sample(self, x_m: float, y_m: float) -> TerrainSample:
        """Return the accepted terrain cell containing a world XY point."""
        if not math.isfinite(x_m) or not math.isfinite(y_m):
            return TerrainSample(valid=False)
        delta_x = float(x_m) - self._center_x
        delta_y = float(y_m) - self._center_y
        cosine = math.cos(self._yaw)
        sine = math.sin(self._yaw)
        local_x = cosine * delta_x + sine * delta_y
        local_y = -sine * delta_x + cosine * delta_y
        logical_row = int(math.floor((self._length_x * 0.5 - local_x) / self._resolution))
        logical_column = int(math.floor((self._length_y * 0.5 - local_y) / self._resolution))
        if not (0 <= logical_row < self._rows and 0 <= logical_column < self._columns):
            return TerrainSample(valid=False)
        row = (logical_row + self._outer_start) % self._rows
        column = (logical_column + self._inner_start) % self._columns

        state = float(self._layers["state"][row, column])
        elevation = float(self._layers["elevation"][row, column])
        slope = float(self._layers["slope_deg"][row, column])
        if state != 1.0 or not math.isfinite(elevation + slope):
            return TerrainSample(valid=False)

        normal = None
        normal_names = ("normal_x", "normal_y", "normal_z")
        if all(name in self._layers for name in normal_names):
            vector = np.asarray(
                [self._layers[name][row, column] for name in normal_names],
                dtype=np.float64,
            )
            norm = float(np.linalg.norm(vector))
            if np.isfinite(vector).all() and norm > 1e-8:
                vector /= norm
                if vector[2] < 0.0:
                    vector *= -1.0
                normal = tuple(float(value) for value in vector)
        return TerrainSample(
            valid=True,
            elevation_m=elevation,
            slope_deg=slope,
            normal_xyz=normal,
            confidence=(
                float(self._layers["inlier_ratio"][row, column])
                if "inlier_ratio" in self._layers
                and math.isfinite(float(self._layers["inlier_ratio"][row, column]))
                else None
            ),
            plane_id=f"terrain-r{row}-c{column}",
        )


def _quaternion_yaw(x: float, y: float, z: float, w: float) -> float:
    quaternion = np.asarray([x, y, z, w], dtype=np.float64)
    norm = float(np.linalg.norm(quaternion))
    if not np.isfinite(quaternion).all() or norm < 1e-9:
        raise ValueError("quaternion must be finite and non-zero")
    x_value, y_value, z_value, w_value = quaternion / norm
    return math.atan2(
        2.0 * (w_value * z_value + x_value * y_value),
        1.0 - 2.0 * (y_value * y_value + z_value * z_value),
    )
