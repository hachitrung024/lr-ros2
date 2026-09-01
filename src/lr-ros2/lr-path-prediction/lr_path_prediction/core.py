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
class RoverModel:
    """Physical geometry used by collision and rollover prediction."""

    mass_kg: float = 100.0
    body_length_m: float = 1.05
    body_width_m: float = 0.90
    support_length_m: float = 0.75
    support_width_m: float = 0.88
    com_x_m: float = 0.0
    com_y_m: float = 0.0
    com_height_m: float = 0.33
    collision_margin_m: float = 0.20

    def validate(self) -> None:
        """Reject non-physical values before prediction starts."""
        values = (
            self.mass_kg,
            self.body_length_m,
            self.body_width_m,
            self.support_length_m,
            self.support_width_m,
            self.com_x_m,
            self.com_y_m,
            self.com_height_m,
            self.collision_margin_m,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("all rover model values must be finite")
        if any(
            value <= 0.0
            for value in (
                self.mass_kg,
                self.body_length_m,
                self.body_width_m,
                self.support_length_m,
                self.support_width_m,
                self.com_height_m,
            )
        ):
            raise ValueError(
                "rover mass, dimensions, and CoM height must be positive"
            )
        if self.collision_margin_m < 0.0:
            raise ValueError("collision margin must not be negative")
        if (
            abs(self.com_x_m) >= self.support_length_m * 0.5
            or abs(self.com_y_m) >= self.support_width_m * 0.5
        ):
            raise ValueError("configured CoM must be inside the support rectangle")


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
    rover_yaw_rad: float = math.nan
    predicted_roll_deg: float = math.nan
    predicted_pitch_deg: float = math.nan
    static_stability_margin_m: float = math.nan
    normalized_static_stability_margin: float = math.nan
    nearest_static_edge: str = ""
    dynamic_state_available: bool = False
    effective_stability_margin_m: float = math.nan
    normalized_effective_stability_margin: float = math.nan
    nearest_effective_edge: str = ""


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
    rover: RoverModel,
    acceleration_world_xyz: tuple[float, float, float] | None = None,
) -> list[StepPrediction]:
    """Evaluate collision and rollover evidence at future path poses."""
    if step_count < 1 or path_stride < 1:
        raise ValueError("step_count and path_stride must be positive")
    rover.validate()
    if acceleration_world_xyz is not None:
        acceleration = np.asarray(acceleration_world_xyz, dtype=np.float64)
        if acceleration.shape != (3,) or not np.isfinite(acceleration).all():
            raise ValueError("acceleration must contain three finite values")
    if len(path.poses) < 2:
        return []

    source_indices = list(range(path_stride, len(path.poses), path_stride))[
        :step_count
    ]
    cumulative_distances = _path_distances(path)
    initial_stamp_ns = _pose_stamp_ns(path, 0)
    predictions = []
    for step_index, source_index in enumerate(source_indices, start=1):
        pose = path.poses[source_index]
        position = pose.pose.position
        xyz = (float(position.x), float(position.y), float(position.z))
        yaw = _path_pose_yaw(path, source_index)
        terrain_sample = (
            terrain.sample(xyz[0], xyz[1])
            if terrain is not None
            else TerrainSample(valid=False)
        )
        collision_ids = []
        nearest_clearance = math.inf
        if object_data_available:
            for obstacle in obstacles:
                clearance = _rectangle_distance(
                    _rectangle_vertices(
                        xyz[0],
                        xyz[1],
                        rover.body_length_m,
                        rover.body_width_m,
                        yaw,
                    ),
                    _rectangle_vertices(
                        obstacle.center_xy[0],
                        obstacle.center_xy[1],
                        obstacle.size_xy[0],
                        obstacle.size_xy[1],
                        obstacle.yaw_rad,
                    ),
                )
                nearest_clearance = min(nearest_clearance, clearance)
                if clearance <= rover.collision_margin_m + 1e-9:
                    collision_ids.append(obstacle.object_id)
        rollover = _rollover_evidence(
            terrain_sample,
            yaw,
            rover,
            acceleration_world_xyz,
        )
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
            rover_yaw_rad=yaw,
            predicted_roll_deg=rollover[0],
            predicted_pitch_deg=rollover[1],
            static_stability_margin_m=rollover[2],
            normalized_static_stability_margin=rollover[3],
            nearest_static_edge=rollover[4],
            dynamic_state_available=acceleration_world_xyz is not None,
            effective_stability_margin_m=rollover[5],
            normalized_effective_stability_margin=rollover[6],
            nearest_effective_edge=rollover[7],
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


def _path_pose_yaw(path: Path, index: int) -> float:
    orientation = path.poses[index].pose.orientation
    try:
        return _quaternion_yaw(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
    except ValueError:
        previous_index = max(0, index - 1)
        next_index = min(len(path.poses) - 1, index + 1)
        previous = path.poses[previous_index].pose.position
        following = path.poses[next_index].pose.position
        delta_x = float(following.x - previous.x)
        delta_y = float(following.y - previous.y)
        if math.hypot(delta_x, delta_y) <= 1e-9:
            return 0.0
        return math.atan2(delta_y, delta_x)


def _rectangle_vertices(
    center_x: float,
    center_y: float,
    length_m: float,
    width_m: float,
    yaw_rad: float,
) -> np.ndarray:
    half_length = length_m * 0.5
    half_width = width_m * 0.5
    local = np.asarray([
        [half_length, half_width],
        [-half_length, half_width],
        [-half_length, -half_width],
        [half_length, -half_width],
    ], dtype=np.float64)
    cosine = math.cos(yaw_rad)
    sine = math.sin(yaw_rad)
    rotation = np.asarray([[cosine, -sine], [sine, cosine]])
    return local @ rotation.T + np.asarray([center_x, center_y])


def _rectangle_distance(first: np.ndarray, second: np.ndarray) -> float:
    if _rectangles_intersect(first, second):
        return 0.0
    distance = math.inf
    for point in first:
        for index in range(4):
            distance = min(
                distance,
                _point_segment_distance(
                    point,
                    second[index],
                    second[(index + 1) % 4],
                ),
            )
    for point in second:
        for index in range(4):
            distance = min(
                distance,
                _point_segment_distance(
                    point,
                    first[index],
                    first[(index + 1) % 4],
                ),
            )
    return float(distance)


def _rectangles_intersect(first: np.ndarray, second: np.ndarray) -> bool:
    for polygon in (first, second):
        for index in range(2):
            edge = polygon[(index + 1) % 4] - polygon[index]
            axis = np.asarray([-edge[1], edge[0]], dtype=np.float64)
            first_projection = first @ axis
            second_projection = second @ axis
            if (
                float(np.max(first_projection))
                < float(np.min(second_projection)) - 1e-12
                or float(np.max(second_projection))
                < float(np.min(first_projection)) - 1e-12
            ):
                return False
    return True


def _point_segment_distance(
    point: np.ndarray,
    start: np.ndarray,
    end: np.ndarray,
) -> float:
    segment = end - start
    length_squared = float(segment @ segment)
    if length_squared <= 1e-18:
        return float(np.linalg.norm(point - start))
    fraction = float((point - start) @ segment) / length_squared
    fraction = min(1.0, max(0.0, fraction))
    projection = start + fraction * segment
    return float(np.linalg.norm(point - projection))


def _rollover_evidence(
    terrain: TerrainSample,
    yaw: float,
    rover: RoverModel,
    acceleration_world_xyz: tuple[float, float, float] | None,
) -> tuple[float, float, float, float, str, float, float, str]:
    unavailable = (math.nan, math.nan, math.nan, math.nan, "")
    if not terrain.valid or terrain.normal_xyz is None:
        return (*unavailable, math.nan, math.nan, "")
    try:
        normal = _normalized_upward_normal(terrain.normal_xyz)
    except ValueError:
        return (*unavailable, math.nan, math.nan, "")
    forward = np.asarray([math.cos(yaw), math.sin(yaw), 0.0])
    left = np.asarray([-math.sin(yaw), math.cos(yaw), 0.0])
    pitch = math.atan2(-float(normal @ forward), float(normal[2]))
    roll = math.atan2(-float(normal @ left), float(normal[2]))
    reference = _support_margins(
        (rover.com_x_m, rover.com_y_m),
        rover.support_length_m,
        rover.support_width_m,
    )
    try:
        projected = _project_com(
            normal,
            yaw,
            rover,
            np.asarray([0.0, 0.0, -9.80665]),
        )
    except ValueError:
        return (*unavailable, math.nan, math.nan, "")
    current = _support_margins(
        projected,
        rover.support_length_m,
        rover.support_width_m,
    )
    static_margin, static_edge = _minimum_margin(current)
    static_normalized = _normalized_margin(current, reference)
    if acceleration_world_xyz is None:
        effective_margin = math.nan
        effective_normalized = math.nan
        effective_edge = ""
    else:
        effective_gravity = (
            np.asarray([0.0, 0.0, -9.80665])
            - np.asarray(acceleration_world_xyz, dtype=np.float64)
        )
        try:
            effective_projection = _project_com(
                normal,
                yaw,
                rover,
                effective_gravity,
            )
        except ValueError:
            return (
                math.degrees(roll),
                math.degrees(pitch),
                static_margin,
                static_normalized,
                static_edge,
                math.nan,
                math.nan,
                "",
            )
        effective_margins = _support_margins(
            effective_projection,
            rover.support_length_m,
            rover.support_width_m,
        )
        effective_margin, effective_edge = _minimum_margin(
            effective_margins
        )
        effective_normalized = _normalized_margin(
            effective_margins,
            reference,
        )
    return (
        math.degrees(roll),
        math.degrees(pitch),
        static_margin,
        static_normalized,
        static_edge,
        effective_margin,
        effective_normalized,
        effective_edge,
    )


def _normalized_upward_normal(values: tuple[float, float, float]) -> np.ndarray:
    normal = np.asarray(values, dtype=np.float64)
    magnitude = float(np.linalg.norm(normal))
    if not np.isfinite(normal).all() or magnitude <= 1e-12:
        raise ValueError("terrain normal must be finite and non-zero")
    normal /= magnitude
    if normal[2] < 0.0:
        normal *= -1.0
    if abs(float(normal[2])) <= 1e-6:
        raise ValueError("near-vertical terrain normal is unsupported")
    return normal


def _terrain_frame(normal: np.ndarray, yaw: float) -> np.ndarray:
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    forward = np.asarray([
        cosine,
        sine,
        -(normal[0] * cosine + normal[1] * sine) / normal[2],
    ])
    forward /= np.linalg.norm(forward)
    left = np.cross(normal, forward)
    left /= np.linalg.norm(left)
    return np.column_stack((forward, left, normal))


def _project_com(
    normal: np.ndarray,
    yaw: float,
    rover: RoverModel,
    direction_world: np.ndarray,
) -> tuple[float, float]:
    rotation = _terrain_frame(normal, yaw)
    direction = rotation.T @ direction_world
    if abs(float(direction[2])) <= 1e-9:
        raise ValueError("projection direction does not meet support plane")
    point = np.asarray([
        rover.com_x_m,
        rover.com_y_m,
        rover.com_height_m,
    ])
    projected = point - point[2] / direction[2] * direction
    return float(projected[0]), float(projected[1])


def _support_margins(
    point_xy: tuple[float, float],
    length_m: float,
    width_m: float,
) -> dict[str, float]:
    x_value, y_value = point_xy
    return {
        "front": length_m * 0.5 - x_value,
        "rear": x_value + length_m * 0.5,
        "left": width_m * 0.5 - y_value,
        "right": y_value + width_m * 0.5,
    }


def _minimum_margin(margins: dict[str, float]) -> tuple[float, str]:
    edge = min(margins, key=margins.get)
    return float(margins[edge]), edge


def _normalized_margin(
    margins: dict[str, float],
    reference: dict[str, float],
) -> float:
    return min(margins[edge] / reference[edge] for edge in reference)


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
