"""Presentation types for canonical prediction evidence; no physics engine."""

from dataclasses import dataclass
import math

from lr_terrain_geometry.grid_map_sampling import TerrainSample


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
    stability_moment_valid: bool = False
    minimum_stability_moment_nm: float = math.nan
    normalized_minimum_stability_moment: float = math.nan
    minimum_normalized_moment_edge: str = ""
    zmp_valid: bool = False
    zmp_xy: tuple[float, float] | None = None
    zmp_margin_m: float = math.nan
    normalized_zmp_margin: float = math.nan
    nearest_zmp_edge: str = ""
