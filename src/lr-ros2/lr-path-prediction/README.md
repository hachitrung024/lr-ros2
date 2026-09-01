# LR Path Prediction

`path_risk_predictor` evaluates the first 20 future poses from a
`nav_msgs/Path`. For every step it samples terrain slope and normal from a
`grid_map_msgs/GridMap`, then checks the rover's circular XY footprint against
the current `vision_msgs/Detection3DArray` boxes. Objects are assumed static
over the prediction horizon.

## Inputs

- `/lr/future_path/ground_truth` (`nav_msgs/msg/Path`)
- `/terrain_geometry/grid_map` (`grid_map_msgs/msg/GridMap`)
- `/segmentation/boxes_3d` (`vision_msgs/msg/Detection3DArray`)

The inputs must use the configured `map` frame. A recent empty detection array
means that no object was detected; missing or stale detection data is reported
as unknown.

## Outputs

- `/lr/path_prediction/steps` (`diagnostic_msgs/msg/DiagnosticArray`) contains
  one status per step, including pose, distance/time offset, slope, normal,
  object IDs, collision state, and nearest object clearance.
- `/lr/path_prediction/markers` (`visualization_msgs/msg/MarkerArray`) contains
  colored path points, compact slope labels, collision marks, and normal
  arrows. A label contains only the slope when available and adds `!` when the
  rover footprint intersects a 3D-box footprint.

Colors are gray by default, orange for slope warning, red for critical slope,
and magenta for object collision.

## Standalone launch

```bash
ros2 launch lr_path_prediction path_prediction.launch.py \
  use_sim_time:=true
```

The main `display_zed_cam.launch.py` starts this node automatically when the
future path, terrain, and segmentation are all enabled. Parameters such as the
20-step count, path stride, slope thresholds, rover radius, collision margin,
and input age limits are in `config/path_prediction.yaml`.
