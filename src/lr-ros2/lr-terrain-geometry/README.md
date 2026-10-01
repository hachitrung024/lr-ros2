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
