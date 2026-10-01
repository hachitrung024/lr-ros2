"""Configuration and snapshots for the ROS-independent terrain estimator."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from typing import Optional

import numpy as np


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
