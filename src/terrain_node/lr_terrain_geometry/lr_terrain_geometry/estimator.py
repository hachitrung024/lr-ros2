from __future__ import annotations

import math
import logging
from collections import deque
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

LOGGER = logging.getLogger(__name__)


@dataclass(frozen=True)
class TerrainGeometryConfig:
    # Map lifetime and reset policy.
    enabled: bool = True
    radius_m: float = 12.0
    cell_size_m: float = 1.0
    ttl_seconds: float = 60.0
    retention_hysteresis_m: float = 1.0
    pose_jump_threshold_m: float = 3.0
    pose_jump_rotation_deg: float = 45.0

    # Input point filtering in the camera-local processing volume.
    min_distance: float = 0.35
    max_distance: float = 15.0
    min_forward_m: Optional[float] = 1.0
    max_forward_m: Optional[float] = 10.0
    min_lateral_m: Optional[float] = -3.0
    max_lateral_m: Optional[float] = 3.0
    min_height_m: float = -10.0
    max_height_m: float = 10.0
    # Point accumulation and fit workload limits.
    max_ingest_points_per_frame: int = 12000
    max_points_per_cell_per_frame: int = 200
    max_points_per_cell: int = 1500
    point_window_seconds: float = 10.0
    voxel_size_m: float = 0.05
    fit_every_frames: int = 5
    max_cells_fit_per_cycle: int = 100
    max_central_fit_points: int = 1200
    max_neighbor_fit_points: int = 300
    neighbor_halo_m: float = 0.10
    ransac_iterations: int = 160
    robust_refinement_iterations: int = 3

    # ------------------------------------------------------------------
    # PLANE ACCEPTANCE - main tuning block.
    # A cell is fitted only after both accumulation requirements are met.
    min_accumulation_frames: int = 3
    min_cell_points: int = 80
    # Point-to-plane inlier distance. Increase for noisier depth data.
    distance_threshold: float = 0.035
    # A plane must satisfy BOTH the absolute and relative inlier limits.
    min_cell_inliers: int = 10
    min_cell_inlier_ratio: float = 0.15
    # Minor principal spread of central inliers in world X/Z.
    min_cell_principal_stddev: float = 0.10
    # RMSE is calculated from central inliers only.
    max_cell_rmse: float = 0.035
    max_plane_slope_deg: float = 55.0
    # Scale point/inlier thresholds by cell area and coverage by cell length.
    auto_scale_cell_thresholds: bool = True
    cell_reference_size_m: float = 1.0

    # TEMPORAL ACCEPTANCE - confirmation, rejection and surface switching.
    confirmation_good_fits: int = 2
    rejection_bad_fits: int = 2
    plane_consistency_normal_deg: float = 5.0
    plane_consistency_height_m: float = 0.10
    plane_ema_alpha: float = 0.25
    random_seed: int = 7
    print_status: bool = False


@dataclass(frozen=True)
class EffectiveCellThresholds:
    min_points: int
    min_inliers: int
    min_principal_stddev: float
    max_cells_fit: int


@dataclass
class PointBatch:
    timestamp: float
    frame_index: int
    points_world: np.ndarray


@dataclass
class LocalTerrainCell:
    key: tuple[int, int]
    first_seen: float
    last_point_seen: float
    batches: deque[PointBatch] = field(default_factory=deque)
    point_count: int = 0
    observation_count: int = 0
    last_fit_frame: Optional[int] = None
    last_fit_timestamp: Optional[float] = None
    last_successful_fit: Optional[float] = None
    plane: Optional[dict] = None
    candidate_plane: Optional[dict] = None
    candidate_good_fits: int = 0
    consecutive_bad_fits: int = 0
    rejected_plane: Optional[dict] = None
    rejection_reason: Optional[str] = None


@dataclass(frozen=True)
class TerrainResult:
    accepted_planes: tuple[dict, ...]
    rejected_planes: tuple[dict, ...]
    debug_cells: tuple[dict, ...]
    fit_cycle: bool
    display_changed: bool
    reset_reason: Optional[str]
    cell_count: int
    point_count: int


class TerrainGeometryEstimator:
    """Accumulate camera points in world-aligned cells before fitting planes."""

    def __init__(
        self,
        config: Optional[TerrainGeometryConfig] = None,
    ) -> None:
        self.config = (
            config if config is not None else TerrainGeometryConfig()
        )
        self._validate_config()
        self._rng = np.random.default_rng(self.config.random_seed)
        self._cells: dict[tuple[int, int], LocalTerrainCell] = {}
        self._last_pose_matrix: Optional[np.ndarray] = None
        self._last_timestamp: Optional[float] = None
        self._frame_index = 0
        self.last_update_changed = False
        self.last_update_reset_reason: Optional[str] = None
        self.last_reset_reason: Optional[str] = None

    @property
    def cell_count(self) -> int:
        return len(self._cells)

    @property
    def point_count(self) -> int:
        return sum(cell.point_count for cell in self._cells.values())

    def log_parameters(self) -> None:
        if not self.config.enabled:
            LOGGER.info("Terrain geometry estimation disabled")
            return

        thresholds = self._effective_cell_thresholds()
        LOGGER.info(
            "Terrain: radius=%.2fm cell=%.2fm ttl=%.1fs window=%.1fs "
            "fit_every=%d min_frames=%d min_points=%d min_inliers=%d "
            "inlier_ratio=%.2f rmse<=%.3fm slope<=%.1fdeg",
            self.config.radius_m,
            self.config.cell_size_m,
            self.config.ttl_seconds,
            self.config.point_window_seconds,
            self.config.fit_every_frames,
            self.config.min_accumulation_frames,
            thresholds.min_points,
            thresholds.min_inliers,
            self.config.min_cell_inlier_ratio,
            self.config.max_cell_rmse,
            self.config.max_plane_slope_deg,
        )

    def clear(self, reason: Optional[str] = None) -> None:
        if self._cells:
            self.last_update_changed = True
        self._cells.clear()
        self.last_update_reset_reason = reason
        self.last_reset_reason = reason

    def reset(self, reason: Optional[str] = None) -> None:
        self.clear(reason)
        self._last_pose_matrix = None
        self._last_timestamp = None
        self._frame_index = 0
        self._rng = np.random.default_rng(self.config.random_seed)

    def update(
        self,
        points_sensor,
        sensor_to_map,
        timestamp_seconds: float,
    ) -> TerrainResult:
        self.last_update_changed = False
        self.last_update_reset_reason = None

        timestamp = float(timestamp_seconds)
        if not math.isfinite(timestamp):
            raise ValueError("timestamp_seconds must be finite")

        if not self.config.enabled:
            return TerrainResult((), (), (), False, False, None, 0, 0)

        if (
            self._last_timestamp is not None
            and timestamp + 1e-6 < self._last_timestamp
        ):
            self.reset("timestamp moved backwards")
        self._last_timestamp = timestamp

        pose_matrix = self._validated_pose(sensor_to_map)
        if pose_matrix is None:
            self._prune_point_window(timestamp)
            self._prune_ttl(timestamp)
            return self._build_update(fit_cycle=False)

        if (
            self._last_pose_matrix is not None
            and self._pose_is_discontinuous(
                self._last_pose_matrix,
                pose_matrix,
            )
        ):
            self.clear("pose discontinuity")
        self._last_pose_matrix = pose_matrix.copy()

        self._frame_index += 1
        self._prune_point_window(timestamp)
        points_sensor = self._extract_valid_points(
            points_sensor,
            pose_matrix,
        )
        if points_sensor.shape[0] > 0:
            points_world = (
                points_sensor @ pose_matrix[:3, :3].T
                + pose_matrix[:3, 3]
            )
            self._accumulate_world_points(
                points_world,
                pose_matrix[:3, 3],
                timestamp,
            )

        fit_cycle = self._frame_index % self.config.fit_every_frames == 0
        if fit_cycle:
            self._fit_eligible_cells(
                pose_matrix[:3, 3],
                timestamp,
            )
            # Refresh visibility as the rolling radius moves even when no cell
            # had enough new observations to be refitted this cycle.
            self.last_update_changed = True

        self._prune_radius(pose_matrix[:3, 3])
        self._prune_ttl(timestamp)
        return self._build_update(fit_cycle=fit_cycle)

    def snapshot(
        self,
        rover_position: Optional[np.ndarray] = None,
    ) -> list[dict]:
        if rover_position is None:
            if self._last_pose_matrix is None:
                return []
            rover_position = self._last_pose_matrix[:3, 3]

        cells = self._visible_cells(rover_position)
        return [cell.plane for cell in cells if cell.plane is not None]

    def rejected_snapshot(
        self,
        rover_position: Optional[np.ndarray] = None,
    ) -> list[dict]:
        if rover_position is None:
            if self._last_pose_matrix is None:
                return []
            rover_position = self._last_pose_matrix[:3, 3]

        cells = self._visible_cells(rover_position)
        return [
            cell.rejected_plane
            for cell in cells
            if cell.rejected_plane is not None
        ]

    def debug_snapshot(
        self,
        rover_position: Optional[np.ndarray] = None,
    ) -> list[dict]:
        if rover_position is None:
            if self._last_pose_matrix is None:
                return []
            rover_position = self._last_pose_matrix[:3, 3]

        debug_cells = []
        for cell in self._visible_cells(rover_position):
            if cell.plane is not None or cell.rejected_plane is not None:
                continue
            points = self._cell_points(cell)
            if points.shape[0] == 0:
                continue
            debug_cells.append({
                "column_key": cell.key,
                "status": (
                    "rejected"
                    if cell.rejection_reason is not None
                    else "unchecked"
                ),
                "footprint_world": self._horizontal_footprint(
                    cell.key,
                    float(np.median(points[:, 2])),
                ),
                "observation_count": cell.observation_count,
                "point_count": cell.point_count,
            })
        return debug_cells

    def _build_update(self, *, fit_cycle: bool) -> TerrainResult:
        return TerrainResult(
            accepted_planes=tuple(self.snapshot()),
            rejected_planes=tuple(self.rejected_snapshot()),
            debug_cells=tuple(self.debug_snapshot()),
            fit_cycle=fit_cycle,
            display_changed=self.last_update_changed,
            reset_reason=self.last_update_reset_reason,
            cell_count=self.cell_count,
            point_count=self.point_count,
        )

    def _validate_config(self) -> None:
        numeric_values = (
            self.config.radius_m,
            self.config.cell_size_m,
            self.config.ttl_seconds,
            self.config.retention_hysteresis_m,
            self.config.pose_jump_threshold_m,
            self.config.pose_jump_rotation_deg,
            self.config.distance_threshold,
            self.config.min_distance,
            self.config.max_distance,
            self.config.min_height_m,
            self.config.max_height_m,
            self.config.min_cell_inlier_ratio,
            self.config.min_cell_principal_stddev,
            self.config.max_cell_rmse,
            self.config.max_plane_slope_deg,
            self.config.cell_reference_size_m,
            self.config.point_window_seconds,
            self.config.voxel_size_m,
            self.config.neighbor_halo_m,
            self.config.plane_consistency_normal_deg,
            self.config.plane_consistency_height_m,
            self.config.plane_ema_alpha,
        )
        optional_values = (
            self.config.min_forward_m,
            self.config.max_forward_m,
            self.config.min_lateral_m,
            self.config.max_lateral_m,
        )
        if not all(math.isfinite(value) for value in numeric_values):
            raise ValueError("terrain map parameters must be finite")
        if not all(
            value is None or math.isfinite(value)
            for value in optional_values
        ):
            raise ValueError("terrain map optional bounds must be finite")
        optional_ranges = (
            (
                "forward",
                self.config.min_forward_m,
                self.config.max_forward_m,
            ),
            (
                "lateral",
                self.config.min_lateral_m,
                self.config.max_lateral_m,
            ),
        )
        for name, low, high in optional_ranges:
            if low is not None and high is not None and high < low:
                raise ValueError(f"{name} range must be increasing")
        if self.config.radius_m <= 0.0:
            raise ValueError("radius_m must be greater than zero")
        if self.config.cell_size_m <= 0.0:
            raise ValueError("cell_size_m must be greater than zero")
        if self.config.ttl_seconds < 0.0:
            raise ValueError("ttl_seconds must be non-negative")
        if self.config.retention_hysteresis_m < 0.0:
            raise ValueError("retention_hysteresis_m must be non-negative")
        if self.config.pose_jump_threshold_m <= 0.0:
            raise ValueError("pose_jump_threshold_m must be greater than zero")
        if not 0.0 < self.config.pose_jump_rotation_deg <= 180.0:
            raise ValueError("pose_jump_rotation_deg must be in (0, 180]")
        if self.config.distance_threshold <= 0.0:
            raise ValueError("distance_threshold must be greater than zero")
        if self.config.min_distance < 0.0:
            raise ValueError("min_distance must be non-negative")
        if self.config.max_distance <= self.config.min_distance:
            raise ValueError("max_distance must be greater than min_distance")
        if self.config.max_height_m <= self.config.min_height_m:
            raise ValueError("column height range must be increasing")
        if not 0.0 <= self.config.min_cell_inlier_ratio <= 1.0:
            raise ValueError("min_cell_inlier_ratio must be in [0, 1]")
        if self.config.min_cell_principal_stddev < 0.0:
            raise ValueError("min_cell_principal_stddev must be non-negative")
        if self.config.max_cell_rmse <= 0.0:
            raise ValueError("max_cell_rmse must be greater than zero")
        if not 0.0 <= self.config.max_plane_slope_deg <= 90.0:
            raise ValueError("max_plane_slope_deg must be in [0, 90]")
        if self.config.cell_reference_size_m <= 0.0:
            raise ValueError("cell_reference_size_m must be greater than zero")
        if self.config.point_window_seconds <= 0.0:
            raise ValueError("point_window_seconds must be greater than zero")
        if self.config.voxel_size_m <= 0.0:
            raise ValueError("voxel_size_m must be greater than zero")
        if self.config.neighbor_halo_m < 0.0:
            raise ValueError("neighbor_halo_m must be non-negative")
        if not 0.0 <= self.config.plane_consistency_normal_deg <= 180.0:
            raise ValueError(
                "plane_consistency_normal_deg must be in [0, 180]"
            )
        if self.config.plane_consistency_height_m < 0.0:
            raise ValueError(
                "plane_consistency_height_m must be non-negative"
            )
        if not 0.0 < self.config.plane_ema_alpha <= 1.0:
            raise ValueError("plane_ema_alpha must be in (0, 1]")

        positive_integer_fields = {
            "ransac_iterations": self.config.ransac_iterations,
            "min_cell_points": self.config.min_cell_points,
            "min_cell_inliers": self.config.min_cell_inliers,
            "max_ingest_points_per_frame": (
                self.config.max_ingest_points_per_frame
            ),
            "max_points_per_cell_per_frame": (
                self.config.max_points_per_cell_per_frame
            ),
            "max_points_per_cell": self.config.max_points_per_cell,
            "min_accumulation_frames": self.config.min_accumulation_frames,
            "fit_every_frames": self.config.fit_every_frames,
            "max_cells_fit_per_cycle": self.config.max_cells_fit_per_cycle,
            "max_central_fit_points": self.config.max_central_fit_points,
            "max_neighbor_fit_points": self.config.max_neighbor_fit_points,
            "robust_refinement_iterations": (
                self.config.robust_refinement_iterations
            ),
            "confirmation_good_fits": self.config.confirmation_good_fits,
            "rejection_bad_fits": self.config.rejection_bad_fits,
        }
        for name, value in positive_integer_fields.items():
            if not isinstance(value, int) or value <= 0:
                raise ValueError(f"{name} must be a positive integer")

    def _effective_cell_thresholds(self) -> EffectiveCellThresholds:
        if self.config.auto_scale_cell_thresholds:
            length_scale = (
                self.config.cell_size_m
                / self.config.cell_reference_size_m
            )
        else:
            length_scale = 1.0
        area_scale = length_scale * length_scale
        return EffectiveCellThresholds(
            min_points=max(
                3,
                int(round(self.config.min_cell_points * area_scale)),
            ),
            min_inliers=max(
                3,
                int(round(self.config.min_cell_inliers * area_scale)),
            ),
            min_principal_stddev=max(
                0.0,
                self.config.min_cell_principal_stddev * length_scale,
            ),
            max_cells_fit=max(
                1,
                int(round(
                    self.config.max_cells_fit_per_cycle / area_scale
                )),
            ),
        )

    def _validated_pose(self, pose_matrix) -> Optional[np.ndarray]:
        try:
            matrix = np.asarray(pose_matrix, dtype=np.float64)
        except (TypeError, ValueError):
            return None
        if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
            return None
        return matrix

    def _pose_is_discontinuous(
        self,
        previous: np.ndarray,
        current: np.ndarray,
    ) -> bool:
        translation_delta = float(np.linalg.norm(
            current[:3, 3] - previous[:3, 3]
        ))
        if translation_delta > self.config.pose_jump_threshold_m:
            return True

        relative_rotation = previous[:3, :3].T @ current[:3, :3]
        cosine = float(np.clip(
            (np.trace(relative_rotation) - 1.0) * 0.5,
            -1.0,
            1.0,
        ))
        rotation_delta_deg = math.degrees(math.acos(cosine))
        return rotation_delta_deg > self.config.pose_jump_rotation_deg

    def _extract_valid_points(
        self,
        point_cloud,
        pose_matrix: np.ndarray,
    ) -> np.ndarray:
        data = (
            point_cloud.get_data()
            if hasattr(point_cloud, "get_data")
            else point_cloud
        )
        values = np.asarray(data)
        if values.ndim < 2 or values.shape[-1] < 3:
            raise ValueError("point cloud must contain XYZ values")
        points = np.asarray(values[..., :3], dtype=np.float64).reshape(-1, 3)
        points = points[np.isfinite(points).all(axis=1)]
        if points.shape[0] == 0:
            return points

        distances = np.linalg.norm(points, axis=1)
        mask = (
            (distances >= self.config.min_distance)
            & (distances <= self.config.max_distance)
        )
        points_local = points @ self._camera_to_gravity_rotation(
            pose_matrix[:3, :3]
        ).T
        mask &= points_local[:, 2] >= self.config.min_height_m
        mask &= points_local[:, 2] <= self.config.max_height_m
        if self.config.min_forward_m is not None:
            mask &= points_local[:, 0] >= self.config.min_forward_m
        if self.config.max_forward_m is not None:
            mask &= points_local[:, 0] <= self.config.max_forward_m
        if self.config.min_lateral_m is not None:
            mask &= points_local[:, 1] >= self.config.min_lateral_m
        if self.config.max_lateral_m is not None:
            mask &= points_local[:, 1] <= self.config.max_lateral_m
        points = points[mask]

        if points.shape[0] > self.config.max_ingest_points_per_frame:
            # The flattened ZED cloud preserves image order. Evenly spaced
            # indices retain coverage across the image instead of allowing
            # dense near-camera regions to dominate a random sample.
            indices = np.linspace(
                0,
                points.shape[0] - 1,
                self.config.max_ingest_points_per_frame,
                dtype=np.int64,
            )
            points = points[indices]
        return points

    def _camera_to_gravity_rotation(
        self,
        rotation_camera_to_world: np.ndarray,
    ) -> np.ndarray:
        up_world = np.asarray([0.0, 0.0, 1.0], dtype=np.float64)
        forward_world = rotation_camera_to_world[:, 0].copy()
        forward_world = forward_world.copy()
        forward_world[2] = 0.0
        forward_length = float(np.linalg.norm(forward_world))
        if forward_length < 1e-6:
            left_world = rotation_camera_to_world[:, 1].copy()
            left_world[2] = 0.0
            left_length = float(np.linalg.norm(left_world))
            if left_length < 1e-6:
                left_world = np.asarray([0.0, 1.0, 0.0])
            else:
                left_world /= left_length
            forward_world = np.cross(left_world, up_world)
        else:
            forward_world /= forward_length
            left_world = np.cross(up_world, forward_world)
            left_world /= max(float(np.linalg.norm(left_world)), 1e-6)

        rotation_gravity_to_world = np.column_stack((
            forward_world,
            left_world,
            up_world,
        ))
        return rotation_gravity_to_world.T @ rotation_camera_to_world

    def _accumulate_world_points(
        self,
        points_world: np.ndarray,
        rover_position: np.ndarray,
        timestamp: float,
    ) -> None:
        rover_xy = rover_position[[0, 1]]
        radius_squared = self.config.radius_m * self.config.radius_m
        radius_mask = np.sum(
            np.square(points_world[:, [0, 1]] - rover_xy),
            axis=1,
        ) <= radius_squared
        points_world = points_world[radius_mask]
        if points_world.shape[0] == 0:
            return

        points_world = self._voxel_downsample(points_world)
        if points_world.shape[0] == 0:
            return

        cell_coordinates = np.floor(
            points_world[:, [0, 1]] / self.config.cell_size_m
        ).astype(np.int64)
        unique_cells, inverse = np.unique(
            cell_coordinates,
            axis=0,
            return_inverse=True,
        )

        for cell_index, coordinates in enumerate(unique_cells):
            cell_points = points_world[inverse == cell_index]
            if (
                cell_points.shape[0]
                > self.config.max_points_per_cell_per_frame
            ):
                indices = np.linspace(
                    0,
                    cell_points.shape[0] - 1,
                    self.config.max_points_per_cell_per_frame,
                    dtype=np.int64,
                )
                cell_points = cell_points[indices]

            key = (int(coordinates[0]), int(coordinates[1]))
            cell = self._cells.get(key)
            if cell is None:
                cell = LocalTerrainCell(
                    key=key,
                    first_seen=timestamp,
                    last_point_seen=timestamp,
                )
                self._cells[key] = cell
                self.last_update_changed = True

            was_pending = cell.plane is None and cell.rejected_plane is None
            batch = PointBatch(
                timestamp=timestamp,
                frame_index=self._frame_index,
                points_world=np.asarray(cell_points, dtype=np.float32),
            )
            cell.batches.append(batch)
            cell.point_count += int(cell_points.shape[0])
            cell.observation_count += 1
            cell.last_point_seen = timestamp
            self._trim_cell_to_point_cap(cell)
            if was_pending:
                self.last_update_changed = True

    def _voxel_downsample(self, points_world: np.ndarray) -> np.ndarray:
        if points_world.shape[0] <= 1:
            return points_world
        voxel_keys = np.floor(
            points_world / self.config.voxel_size_m
        ).astype(np.int64)
        _, first_indices = np.unique(
            voxel_keys,
            axis=0,
            return_index=True,
        )
        return points_world[first_indices]

    def _trim_cell_to_point_cap(self, cell: LocalTerrainCell) -> None:
        excess = cell.point_count - self.config.max_points_per_cell
        while excess > 0 and cell.batches:
            batch = cell.batches[0]
            batch_count = int(batch.points_world.shape[0])
            if batch_count <= excess:
                cell.batches.popleft()
                cell.point_count -= batch_count
                excess -= batch_count
                continue
            batch.points_world = batch.points_world[excess:].copy()
            cell.point_count -= excess
            excess = 0

    def _fit_eligible_cells(
        self,
        rover_position: np.ndarray,
        timestamp: float,
    ) -> None:
        thresholds = self._effective_cell_thresholds()
        eligible = []
        for cell in self._cells.values():
            previous_fit_frame = (
                -1
                if cell.last_fit_frame is None
                else cell.last_fit_frame
            )
            new_frame_count = sum(
                batch.frame_index > previous_fit_frame
                for batch in cell.batches
            )
            if (
                cell.point_count < thresholds.min_points
                or new_frame_count < self.config.min_accumulation_frames
            ):
                continue
            distance = float(np.linalg.norm(
                np.asarray(self._grid_center_xy(cell.key))
                - rover_position[[0, 1]]
            ))
            priority = (
                0 if cell.last_fit_frame is None else 1,
                -1 if cell.last_fit_frame is None else cell.last_fit_frame,
                distance,
                cell.key,
            )
            eligible.append((priority, cell))

        eligible.sort(key=lambda item: item[0])
        for _, cell in eligible[:thresholds.max_cells_fit]:
            central_points, fit_points = self._fit_points_for_cell(cell)
            result = self._fit_plane_world(
                fit_points,
                central_points,
                cell,
                thresholds,
            )
            cell.last_fit_frame = self._frame_index
            cell.last_fit_timestamp = timestamp
            self._apply_fit_result(cell, result, timestamp)
            self.last_update_changed = True

        if self.config.print_status:
            accepted_count = sum(
                cell.plane is not None for cell in self._cells.values()
            )
            point_count = sum(
                cell.point_count for cell in self._cells.values()
            )
            print(
                "\r"
                f"World cells: {len(self._cells):3d}"
                f" | planes: {accepted_count:3d}"
                f" | points: {point_count:7d}       ",
                end="",
                flush=True,
            )

    def _fit_points_for_cell(
        self,
        cell: LocalTerrainCell,
    ) -> tuple[np.ndarray, np.ndarray]:
        central_points = self._evenly_sample_points(
            self._cell_points(cell),
            self.config.max_central_fit_points,
        )
        if (
            self.config.neighbor_halo_m <= 0.0
            or self.config.max_neighbor_fit_points <= 0
        ):
            return central_points, central_points

        size = self.config.cell_size_m
        halo = self.config.neighbor_halo_m
        x0 = cell.key[0] * size
        x1 = x0 + size
        y0 = cell.key[1] * size
        y1 = y0 + size
        neighbor_points = []
        for offset_x in (-1, 0, 1):
            for offset_y in (-1, 0, 1):
                if offset_x == 0 and offset_y == 0:
                    continue
                neighbor = self._cells.get((
                    cell.key[0] + offset_x,
                    cell.key[1] + offset_y,
                ))
                if neighbor is None:
                    continue
                points = self._cell_points(neighbor)
                if points.shape[0] == 0:
                    continue
                mask = (
                    (points[:, 0] >= x0 - halo)
                    & (points[:, 0] <= x1 + halo)
                    & (points[:, 1] >= y0 - halo)
                    & (points[:, 1] <= y1 + halo)
                )
                if np.any(mask):
                    neighbor_points.append(points[mask])

        if not neighbor_points:
            return central_points, central_points
        halo_points = self._evenly_sample_points(
            np.concatenate(neighbor_points, axis=0),
            self.config.max_neighbor_fit_points,
        )
        return central_points, np.concatenate(
            (central_points, halo_points),
            axis=0,
        )

    @staticmethod
    def _evenly_sample_points(
        points: np.ndarray,
        limit: int,
    ) -> np.ndarray:
        if points.shape[0] <= limit:
            return points
        indices = np.linspace(
            0,
            points.shape[0] - 1,
            limit,
            dtype=np.int64,
        )
        return points[indices]

    def _apply_fit_result(
        self,
        cell: LocalTerrainCell,
        result: Optional[dict],
        timestamp: float,
    ) -> None:
        if result is None or result["rejection_reason"] is not None:
            cell.candidate_plane = None
            cell.candidate_good_fits = 0
            cell.consecutive_bad_fits += 1
            cell.rejected_plane = result
            cell.rejection_reason = (
                "fit" if result is None else result["rejection_reason"]
            )
            if cell.consecutive_bad_fits >= self.config.rejection_bad_fits:
                cell.plane = None
                cell.last_successful_fit = None
            return

        cell.consecutive_bad_fits = 0
        cell.rejected_plane = None
        cell.rejection_reason = None
        if cell.plane is not None and self._planes_are_consistent(
            cell.plane,
            result,
        ):
            cell.plane = self._blend_planes(
                cell.plane,
                result,
                self.config.plane_ema_alpha,
            )
            cell.last_successful_fit = timestamp
            cell.candidate_plane = None
            cell.candidate_good_fits = 0
            return

        if (
            cell.candidate_plane is not None
            and self._planes_are_consistent(cell.candidate_plane, result)
        ):
            cell.candidate_plane = self._blend_planes(
                cell.candidate_plane,
                result,
                0.5,
            )
            cell.candidate_good_fits += 1
        else:
            cell.candidate_plane = self._copy_plane(result)
            cell.candidate_good_fits = 1

        if (
            cell.candidate_good_fits
            >= self.config.confirmation_good_fits
        ):
            cell.plane = self._copy_plane(cell.candidate_plane)
            cell.last_successful_fit = timestamp
            cell.candidate_plane = None
            cell.candidate_good_fits = 0

    def _planes_are_consistent(self, first: dict, second: dict) -> bool:
        first_normal = np.asarray(
            first["normal_world"],
            dtype=np.float64,
        )
        second_normal = np.asarray(
            second["normal_world"],
            dtype=np.float64,
        )
        first_normal /= max(float(np.linalg.norm(first_normal)), 1e-9)
        second_normal /= max(float(np.linalg.norm(second_normal)), 1e-9)
        cosine = float(np.clip(
            np.dot(first_normal, second_normal),
            -1.0,
            1.0,
        ))
        normal_delta_deg = math.degrees(math.acos(cosine))
        height_delta = abs(float(
            np.asarray(first["grid_center_world"])[2]
            - np.asarray(second["grid_center_world"])[2]
        ))
        return (
            normal_delta_deg <= self.config.plane_consistency_normal_deg
            and height_delta <= self.config.plane_consistency_height_m
        )

    @staticmethod
    def _copy_plane(plane: dict) -> dict:
        copied = dict(plane)
        for name in (
            "center_world",
            "normal_world",
            "footprint_world",
            "grid_center_world",
        ):
            if copied.get(name) is not None:
                copied[name] = np.asarray(copied[name]).copy()
        return copied

    def _blend_planes(
        self,
        previous: dict,
        current: dict,
        alpha: float,
    ) -> dict:
        alpha = float(np.clip(alpha, 0.0, 1.0))
        old_fraction = 1.0 - alpha
        blended = self._copy_plane(current)

        previous_normal = np.asarray(
            previous["normal_world"],
            dtype=np.float64,
        )
        current_normal = np.asarray(
            current["normal_world"],
            dtype=np.float64,
        )
        if float(np.dot(previous_normal, current_normal)) < 0.0:
            current_normal = -current_normal
        normal = old_fraction * previous_normal + alpha * current_normal
        normal /= max(float(np.linalg.norm(normal)), 1e-9)

        previous_center = np.asarray(
            previous["grid_center_world"],
            dtype=np.float64,
        )
        current_center = np.asarray(
            current["grid_center_world"],
            dtype=np.float64,
        )
        center = current_center.copy()
        center[2] = (
            old_fraction * previous_center[2]
            + alpha * current_center[2]
        )

        for name in (
            "inlier_ratio",
            "rmse",
            "inlier_principal_stddev_minor",
            "inlier_principal_stddev_major",
        ):
            blended[name] = (
                old_fraction * float(previous[name])
                + alpha * float(current[name])
            )
        for name in ("inlier_count", "sample_count"):
            blended[name] = int(round(
                old_fraction * int(previous[name])
                + alpha * int(current[name])
            ))

        key = tuple(current["column_key"])
        blended["center_world"] = center.astype(np.float32)
        blended["grid_center_world"] = center.astype(np.float32)
        blended["normal_world"] = normal.astype(np.float32)
        blended["slope_deg"] = self._slope_from_world_normal(normal)
        blended["footprint_world"] = self._plane_footprint(
            key,
            center,
            normal,
        )
        blended["rejection_reason"] = None
        blended["show_label"] = False
        return blended

    def _fit_plane_world(
        self,
        fit_points_world: np.ndarray,
        central_points_world: np.ndarray,
        cell: LocalTerrainCell,
        thresholds: EffectiveCellThresholds,
    ) -> Optional[dict]:
        sample = np.asarray(fit_points_world, dtype=np.float64)
        central_sample = np.asarray(
            central_points_world,
            dtype=np.float64,
        )
        sample_count = int(central_sample.shape[0])
        if sample.shape[0] < 3 or sample_count < 3:
            return None

        prior_plane = cell.plane or cell.candidate_plane
        prior_normal = None
        if prior_plane is not None:
            prior_normal = np.asarray(
                prior_plane["normal_world"],
                dtype=np.float64,
            )
            prior_normal /= max(float(np.linalg.norm(prior_normal)), 1e-9)

        iteration_count = self.config.ransac_iterations
        point_count = sample.shape[0]
        hypothesis_indices = self._rng.integers(
            0,
            point_count,
            size=(iteration_count, 3),
        )
        duplicate_rows = (
            (hypothesis_indices[:, 0] == hypothesis_indices[:, 1])
            | (hypothesis_indices[:, 0] == hypothesis_indices[:, 2])
            | (hypothesis_indices[:, 1] == hypothesis_indices[:, 2])
        )
        while np.any(duplicate_rows):
            hypothesis_indices[duplicate_rows] = self._rng.integers(
                0,
                point_count,
                size=(int(np.count_nonzero(duplicate_rows)), 3),
            )
            duplicate_rows = (
                (hypothesis_indices[:, 0] == hypothesis_indices[:, 1])
                | (hypothesis_indices[:, 0] == hypothesis_indices[:, 2])
                | (hypothesis_indices[:, 1] == hypothesis_indices[:, 2])
            )

        p0 = sample[hypothesis_indices[:, 0]]
        p1 = sample[hypothesis_indices[:, 1]]
        p2 = sample[hypothesis_indices[:, 2]]
        normals = np.cross(p1 - p0, p2 - p0)
        normal_lengths = np.linalg.norm(normals, axis=1)
        valid_hypotheses = normal_lengths >= 1e-6
        normals[valid_hypotheses] /= normal_lengths[
            valid_hypotheses,
            None,
        ]
        normals[~valid_hypotheses] = 0.0
        flip_mask = normals[:, 2] < 0.0
        normals[flip_mask] *= -1.0
        offsets = -np.einsum("ij,ij->i", normals, p0)

        hypothesis_distances = np.abs(
            central_sample @ normals.T + offsets[None, :]
        )
        hypothesis_inliers = (
            hypothesis_distances < self.config.distance_threshold
        )
        hypothesis_counts = np.count_nonzero(
            hypothesis_inliers,
            axis=0,
        )
        squared_error = np.sum(
            np.square(hypothesis_distances)
            * hypothesis_inliers,
            axis=0,
        )
        hypothesis_rmse = np.sqrt(
            squared_error / np.maximum(hypothesis_counts, 1)
        )
        if prior_normal is None:
            prior_deltas = np.zeros(iteration_count, dtype=np.float64)
        else:
            prior_deltas = np.arccos(np.clip(
                normals @ prior_normal,
                -1.0,
                1.0,
            ))
        slopes = np.degrees(np.arccos(np.clip(
            np.abs(normals[:, 2]),
            -1.0,
            1.0,
        )))

        def select_hypothesis(mask: np.ndarray) -> Optional[int]:
            candidates = np.flatnonzero(
                mask & (hypothesis_counts > 0)
            )
            if candidates.size == 0:
                return None
            order = np.lexsort((
                prior_deltas[candidates],
                hypothesis_rmse[candidates],
                -hypothesis_counts[candidates],
            ))
            return int(candidates[order[0]])

        best_index = select_hypothesis(
            valid_hypotheses
            & (slopes <= self.config.max_plane_slope_deg)
        )
        if best_index is None:
            # Preserve rejected-slope diagnostics when no acceptable
            # hypothesis exists, without allowing a steep dominant object to
            # hide a valid terrain hypothesis when one is available.
            best_index = select_hypothesis(valid_hypotheses)
        if best_index is None:
            return None
        best_normal = normals[best_index]
        best_d = float(offsets[best_index])

        distances = np.abs(sample @ best_normal + best_d)
        inliers = sample[distances < self.config.distance_threshold]
        if inliers.shape[0] < 3:
            return None

        center, normal, d = self._robust_refine_plane(
            inliers,
            best_normal,
        )
        distances = np.abs(sample @ normal + d)
        inlier_mask = distances < self.config.distance_threshold
        refined_inliers = sample[inlier_mask]
        if refined_inliers.shape[0] >= 3:
            center, normal, d = self._robust_refine_plane(
                refined_inliers,
                normal,
            )
        if refined_inliers.shape[0] < 3:
            return None

        if normal[2] < 0.0:
            normal = -normal
            d = -d
        central_distances = np.abs(central_sample @ normal + d)
        central_inlier_mask = (
            central_distances < self.config.distance_threshold
        )
        central_inliers = central_sample[central_inlier_mask]
        if central_inliers.shape[0] < 3:
            return None
        inlier_count = int(central_inliers.shape[0])
        inlier_ratio = inlier_count / float(sample_count)
        minor_stddev, major_stddev = self._principal_stddev_xy(
            central_inliers
        )
        slope_deg = self._slope_from_world_normal(normal)
        rmse = float(np.sqrt(np.mean(np.square(
            central_distances[central_inlier_mask]
        ))))

        rejection_reasons = []
        if (
            inlier_count < thresholds.min_inliers
            or inlier_ratio < self.config.min_cell_inlier_ratio
        ):
            rejection_reasons.append("inliers")
        if minor_stddev < thresholds.min_principal_stddev:
            rejection_reasons.append("coverage")
        if rmse > self.config.max_cell_rmse:
            rejection_reasons.append("rmse")
        if slope_deg > self.config.max_plane_slope_deg:
            rejection_reasons.append("slope")

        grid_x, grid_y = self._grid_center_xy(cell.key)
        if abs(float(normal[2])) >= 1e-6:
            grid_z = float(
                center[2]
                - (
                    normal[0] * (grid_x - center[0])
                    + normal[1] * (grid_y - center[1])
                ) / normal[2]
            )
        else:
            grid_z = float(np.median(refined_inliers[:, 2]))
        grid_center = np.asarray(
            [grid_x, grid_y, grid_z],
            dtype=np.float64,
        )
        footprint = self._plane_footprint(
            cell.key,
            grid_center,
            normal,
        )
        half_size = self.config.cell_size_m * 0.5
        rejection_reason = (
            ", ".join(rejection_reasons)
            if rejection_reasons
            else None
        )

        return {
            "center_world": grid_center.astype(np.float32),
            "normal_world": normal.astype(np.float32),
            "slope_deg": slope_deg,
            "inlier_count": inlier_count,
            "sample_count": sample_count,
            "inlier_ratio": inlier_ratio,
            "rmse": rmse,
            "inlier_principal_stddev_minor": minor_stddev,
            "inlier_principal_stddev_major": major_stddev,
            "extents": (half_size, half_size),
            "rejection_reason": rejection_reason,
            "column_key": cell.key,
            "footprint_world": footprint,
            "footprint_local": None,
            "grid_center_world": grid_center.astype(np.float32),
            "observation_count": cell.observation_count,
            "last_seen": cell.last_point_seen,
            "show_label": rejection_reason is not None,
        }

    def _cell_points(self, cell: LocalTerrainCell) -> np.ndarray:
        if not cell.batches:
            return np.empty((0, 3), dtype=np.float32)
        return np.concatenate(
            [batch.points_world for batch in cell.batches],
            axis=0,
        )

    def _prune_point_window(self, timestamp: float) -> None:
        for cell in self._cells.values():
            removed = False
            while (
                cell.batches
                and timestamp - cell.batches[0].timestamp
                > self.config.point_window_seconds
            ):
                batch = cell.batches.popleft()
                cell.point_count -= int(batch.points_world.shape[0])
                removed = True
            if removed and cell.plane is None:
                self.last_update_changed = True

    def _prune_radius(self, rover_position: np.ndarray) -> None:
        retention_radius = (
            self.config.radius_m + self.config.retention_hysteresis_m
        )
        retention_squared = retention_radius * retention_radius
        rover_xy = rover_position[[0, 1]]
        keys_to_remove = [
            key
            for key in self._cells
            if float(np.sum(np.square(
                np.asarray(self._grid_center_xy(key)) - rover_xy
            ))) > retention_squared
        ]
        self._remove_cells(keys_to_remove)

    def _prune_ttl(self, timestamp: float) -> None:
        if self.config.ttl_seconds <= 0.0:
            self._remove_empty_unfitted_cells()
            return

        keys_to_remove = [
            key
            for key, cell in self._cells.items()
            if timestamp - cell.last_point_seen > self.config.ttl_seconds
        ]
        self._remove_cells(keys_to_remove)
        self._remove_empty_unfitted_cells()

    def _remove_empty_unfitted_cells(self) -> None:
        keys_to_remove = [
            key
            for key, cell in self._cells.items()
            if (
                cell.point_count == 0
                and cell.plane is None
                and cell.rejected_plane is None
            )
        ]
        self._remove_cells(keys_to_remove)

    def _remove_cells(self, keys) -> None:
        removed = False
        for key in keys:
            if key in self._cells:
                del self._cells[key]
                removed = True
        if removed:
            self.last_update_changed = True

    def _visible_cells(
        self,
        rover_position: np.ndarray,
    ) -> list[LocalTerrainCell]:
        rover_xy = np.asarray(rover_position, dtype=np.float64)[[0, 1]]
        radius_squared = self.config.radius_m * self.config.radius_m
        visible = [
            cell
            for cell in self._cells.values()
            if float(np.sum(np.square(
                np.asarray(self._grid_center_xy(cell.key)) - rover_xy
            ))) <= radius_squared
        ]
        visible.sort(key=lambda cell: cell.key)
        return visible

    def _world_key(self, point_world: np.ndarray) -> tuple[int, int]:
        return (
            math.floor(float(point_world[0]) / self.config.cell_size_m),
            math.floor(float(point_world[1]) / self.config.cell_size_m),
        )

    def _grid_center_xy(
        self,
        key: tuple[int, int],
    ) -> tuple[float, float]:
        size = self.config.cell_size_m
        return (
            (key[0] + 0.5) * size,
            (key[1] + 0.5) * size,
        )

    def _horizontal_footprint(
        self,
        key: tuple[int, int],
        height: float,
    ) -> np.ndarray:
        size = self.config.cell_size_m
        x0 = key[0] * size
        x1 = x0 + size
        y0 = key[1] * size
        y1 = y0 + size
        return np.asarray(
            [
                [x0, y0, height],
                [x1, y0, height],
                [x1, y1, height],
                [x0, y1, height],
            ],
            dtype=np.float32,
        )

    def _plane_footprint(
        self,
        key: tuple[int, int],
        center: np.ndarray,
        normal: np.ndarray,
    ) -> np.ndarray:
        if abs(float(normal[2])) < 1e-6:
            return self._horizontal_footprint(key, float(center[2]))
        size = self.config.cell_size_m
        x0 = key[0] * size
        x1 = x0 + size
        y0 = key[1] * size
        y1 = y0 + size
        corners = []
        for x, y in ((x0, y0), (x1, y0), (x1, y1), (x0, y1)):
            z = center[2] - (
                normal[0] * (x - center[0])
                + normal[1] * (y - center[1])
            ) / normal[2]
            corners.append((x, y, z))
        return np.asarray(corners, dtype=np.float32)

    def _normalize(self, vector: np.ndarray) -> Optional[np.ndarray]:
        length = float(np.linalg.norm(vector))
        if length < 1e-6:
            return None
        return vector / length

    def _refine_plane(
        self,
        points: np.ndarray,
        initial_normal: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray, float]:
        center = points.mean(axis=0)
        _, _, vh = np.linalg.svd(points - center, full_matrices=False)
        normal = vh[-1]
        normal /= np.linalg.norm(normal)
        if np.dot(normal, initial_normal) < 0.0:
            normal = -normal
        d = -float(np.dot(normal, center))
        return center, normal, d

    def _robust_refine_plane(
        self,
        points: np.ndarray,
        initial_normal: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray, float]:
        center, normal, d = self._refine_plane(points, initial_normal)
        huber_delta = self.config.distance_threshold
        for _ in range(self.config.robust_refinement_iterations):
            residuals = np.abs(points @ normal + d)
            weights = np.ones(points.shape[0], dtype=np.float64)
            outside = residuals > huber_delta
            weights[outside] = huber_delta / np.maximum(
                residuals[outside],
                1e-12,
            )
            weight_sum = float(np.sum(weights))
            if weight_sum <= 1e-9:
                break
            center = np.sum(
                points * weights[:, None],
                axis=0,
            ) / weight_sum
            centered = points - center
            covariance = (
                (centered * weights[:, None]).T @ centered
            ) / weight_sum
            _, eigenvectors = np.linalg.eigh(covariance)
            updated_normal = eigenvectors[:, 0]
            updated_normal /= max(
                float(np.linalg.norm(updated_normal)),
                1e-9,
            )
            if float(np.dot(updated_normal, normal)) < 0.0:
                updated_normal = -updated_normal
            normal = updated_normal
            d = -float(np.dot(normal, center))
        return center, normal, d

    def _principal_stddev_xy(
        self,
        points_world: np.ndarray,
    ) -> tuple[float, float]:
        xy = np.asarray(points_world[:, [0, 1]], dtype=np.float64)
        centered = xy - np.mean(xy, axis=0)
        covariance = centered.T @ centered / xy.shape[0]
        eigenvalues = np.maximum(np.linalg.eigvalsh(covariance), 0.0)
        deviations = np.sqrt(eigenvalues)
        return float(deviations[0]), float(deviations[1])

    def _slope_from_world_normal(self, normal_world: np.ndarray) -> float:
        normal = normal_world / np.linalg.norm(normal_world)
        cosine = abs(float(normal[2]))
        return math.degrees(math.acos(float(np.clip(cosine, -1.0, 1.0))))
