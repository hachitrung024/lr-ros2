"""Pure geometry for terrain and obstacle prediction along a future path."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import Path
from vision_msgs.msg import Detection3DArray


@dataclass(frozen=True)
class TerrainSample:
    """Terrain values at one XY query position."""

    valid: bool
    elevation_m: float = math.nan
    slope_deg: float = math.nan
    normal_xyz: tuple[float, float, float] | None = None


@dataclass(frozen=True)
class Obstacle:
    """A tracked object's oriented XY footprint."""

    object_id: str
    center_xy: tuple[float, float]
    size_xy: tuple[float, float]
    yaw_rad: float


@dataclass(frozen=True)
class StepPrediction:
    """Predicted terrain and collision state at one future path pose."""

    step_index: int
    source_pose_index: int
    position_xyz: tuple[float, float, float]
    distance_from_start_m: float
    time_from_start_sec: float
    terrain: TerrainSample
    object_data_available: bool
    object_collision: bool
    object_ids: tuple[str, ...]
    nearest_object_clearance_m: float


class GridMapSampler:
    """Read world-coordinate terrain samples from a GridMap message."""

    def __init__(self, message: GridMap) -> None:
        """Decode terrain layers and map geometry once per input message."""
        if not math.isfinite(message.info.resolution):
            raise ValueError("GridMap resolution must be finite")
        self._resolution = float(message.info.resolution)
        if self._resolution <= 0.0:
            raise ValueError("GridMap resolution must be positive")
        self._layers = {
            name: self._decode_layer(message, name)
            for name in message.layers
        }
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
            raise ValueError(
                f"GridMap has no data for layer '{name}'"
            ) from error
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
        logical_row = int(math.floor(
            (self._length_x * 0.5 - local_x) / self._resolution
        ))
        logical_column = int(math.floor(
            (self._length_y * 0.5 - local_y) / self._resolution
        ))
        if not (
            0 <= logical_row < self._rows
            and 0 <= logical_column < self._columns
        ):
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
        )


def obstacles_from_message(message: Detection3DArray) -> list[Obstacle]:
    """Extract finite, positive oriented XY boxes from detections."""
    obstacles = []
    for index, detection in enumerate(message.detections):
        center = detection.bbox.center.position
        size = detection.bbox.size
        orientation = detection.bbox.center.orientation
        values = (
            center.x,
            center.y,
            size.x,
            size.y,
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        if not all(math.isfinite(float(value)) for value in values):
            continue
        if float(size.x) <= 0.0 or float(size.y) <= 0.0:
            continue
        try:
            yaw = _quaternion_yaw(
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
            )
        except ValueError:
            continue
        obstacles.append(Obstacle(
            object_id=detection.id or str(index),
            center_xy=(float(center.x), float(center.y)),
            size_xy=(float(size.x), float(size.y)),
            yaw_rad=yaw,
        ))
    return obstacles


def predict_steps(
    path: Path,
    terrain: GridMapSampler | None,
    obstacles: list[Obstacle],
    *,
    object_data_available: bool,
    step_count: int,
    path_stride: int,
    rover_radius_m: float,
    collision_margin_m: float,
) -> list[StepPrediction]:
    """Evaluate terrain and static-object collision at future path poses."""
    if step_count < 1 or path_stride < 1:
        raise ValueError("step_count and path_stride must be positive")
    if rover_radius_m < 0.0 or collision_margin_m < 0.0:
        raise ValueError("collision dimensions must not be negative")
    if len(path.poses) < 2:
        return []

    source_indices = list(range(path_stride, len(path.poses), path_stride))[
        :step_count
    ]
    cumulative_distances = _path_distances(path)
    initial_stamp_ns = _pose_stamp_ns(path, 0)
    predictions = []
    collision_radius = float(rover_radius_m + collision_margin_m)
    for step_index, source_index in enumerate(source_indices, start=1):
        pose = path.poses[source_index]
        position = pose.pose.position
        xyz = (float(position.x), float(position.y), float(position.z))
        terrain_sample = (
            terrain.sample(xyz[0], xyz[1])
            if terrain is not None
            else TerrainSample(valid=False)
        )
        collision_ids = []
        nearest_clearance = math.inf
        if object_data_available:
            for obstacle in obstacles:
                clearance = _obstacle_clearance(
                    xyz[0],
                    xyz[1],
                    obstacle,
                    collision_radius,
                )
                nearest_clearance = min(nearest_clearance, clearance)
                if clearance <= 0.0:
                    collision_ids.append(obstacle.object_id)
        target_stamp_ns = _pose_stamp_ns(path, source_index)
        time_from_start = math.nan
        if initial_stamp_ns > 0 and target_stamp_ns >= initial_stamp_ns:
            time_from_start = (target_stamp_ns - initial_stamp_ns) * 1e-9
        predictions.append(StepPrediction(
            step_index=step_index,
            source_pose_index=source_index,
            position_xyz=xyz,
            distance_from_start_m=float(cumulative_distances[source_index]),
            time_from_start_sec=time_from_start,
            terrain=terrain_sample,
            object_data_available=object_data_available,
            object_collision=bool(collision_ids),
            object_ids=tuple(collision_ids),
            nearest_object_clearance_m=nearest_clearance,
        ))
    return predictions


def _path_distances(path: Path) -> np.ndarray:
    distances = np.zeros(len(path.poses), dtype=np.float64)
    for index in range(1, len(path.poses)):
        previous = path.poses[index - 1].pose.position
        current = path.poses[index].pose.position
        distances[index] = distances[index - 1] + math.hypot(
            float(current.x - previous.x),
            float(current.y - previous.y),
        )
    return distances


def _pose_stamp_ns(path: Path, index: int) -> int:
    stamp = path.poses[index].header.stamp
    value = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    if value == 0:
        stamp = path.header.stamp
        value = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    return value


def _obstacle_clearance(
    x_m: float,
    y_m: float,
    obstacle: Obstacle,
    rover_radius_m: float,
) -> float:
    delta_x = float(x_m) - obstacle.center_xy[0]
    delta_y = float(y_m) - obstacle.center_xy[1]
    cosine = math.cos(obstacle.yaw_rad)
    sine = math.sin(obstacle.yaw_rad)
    local_x = cosine * delta_x + sine * delta_y
    local_y = -sine * delta_x + cosine * delta_y
    outside_x = max(abs(local_x) - obstacle.size_xy[0] * 0.5, 0.0)
    outside_y = max(abs(local_y) - obstacle.size_xy[1] * 0.5, 0.0)
    return math.hypot(outside_x, outside_y) - rover_radius_m


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
