# LR Terrain Geometry

ROS 2 Humble Python node that accumulates ZED point clouds into world-aligned
cells and estimates local terrain planes. The package uses REP-103 coordinates
(`X` forward, `Y` left, `Z` up) and has no dependency on the ZED SDK Python API.

```bash
ros2 launch lr_terrain_geometry terrain_geometry.launch.py camera_name:=zed
```

The node publishes `~/grid_map`, `~/markers`, `~/heatmap`, `/diagnostics`, and
exposes the `~/reset` service. The heatmap is an `rgb8` image intended for an
RViz Image dock. Processing parameters are read-only after startup and are
configured in `config/terrain_geometry.yaml`.

`estimator.py` owns rolling cells and temporal acceptance. Configuration and
snapshot types live in `terrain_types.py`; deterministic RANSAC and robust
refinement live in `plane_fitting.py`. Existing imports from `estimator` remain
supported. Point grouping uses a stable sort, preserving the per-cell sample
order and point caps while avoiding a full point scan for every cell.

Trajectory filtering is enabled by default. The node subscribes to
`nav_msgs/Path` on `input.trajectory_topic` (default
`/lr/future_path/ground_truth`). The display launch passes its `future_path_topic`
to terrain as well as prediction. A standalone launch can override the YAML
topic with `trajectory_topic:=/my/path`.

For a fresh path in the map frame, terrain keeps cells intersecting a corridor
around its line segments, including bends, endpoints and a stationary single
pose. `corridor.half_width_m` defaults to 1 m on each side. Set it to cover the
vehicle footprint and a localization margin; the default covers the reference
1.05 m by 0.90 m rover with approximately 0.3 m beyond its half diagonal.
This parameter is independent of the prediction rover configuration.

The corridor is an additional restriction inside the existing camera ROI,
distance and rolling-radius limits. Cell selection is conservative: a cell's
half diagonal expands the corridor so boundary cells retain all their central
samples. The narrow `terrain.neighbor_halo_m` support strip is also retained.
Filtering happens before the frame point cap and voxel accumulation. Only
corridor cells are fitted and published; other cached cells keep their normal
point-window/TTL lifetime for reuse if the path changes. Cells without an
accepted plane remain unknown. Path changes still need the normal accumulation
and confirmation period for newly observed cells.

A bounded buffer matches paths to cloud timestamps. The default maximum path
age is `corridor.stale_after_sec: 1.0`; a path may lead the cloud by at most
`corridor.future_tolerance_sec: 0.2`. Age uses message timestamps, so pausing
replay does not expire the corridor. Missing, stale, too-far-ahead, empty,
non-finite, oversized (over 256 poses), or mismatched-frame paths suspend
ingestion and fitting and publish unknown terrain. Cached cells are hidden;
they keep their normal TTL for recovery. TF failures preserve the corridor
restriction and cannot expose off-path cells. Full ROI processing is available
only with `corridor.enabled: false`. Pose frames must be empty or match the map frame; paths
are not transformed from other frames. Reset and backwards time jumps clear
the path buffer.

Path input runs in its own callback group with a short buffer lock. Cloud
fitting, marker generation and heatmap serialization cannot block this input
or make an actively published path appear stale under RViz load. Estimator
updates and resets remain serialized. The 1 s freshness limit is unchanged.

Diagnostics expose `corridor_status`, `trajectory_age_sec`, `roi_points`,
`corridor_points` (both before the frame cap), and `fitted_cells` for the last
cloud. ZED depth/point-cloud generation and ROS cloud decoding still process
the original input; the reduction applies to terrain accumulation and fitting.
