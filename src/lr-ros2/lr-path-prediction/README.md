# LR Path Prediction

`lr_path_prediction` is a self-contained ROS 2 prediction node integrated into
the LR perception pipeline. The `prediction-rover/` repository was used only
as an algorithm reference and may be removed without affecting this package.

The node evaluates up to 20 future path poses using:

- exact oriented rover-body and object-box footprint distance;
- collision candidates within the configured safety margin;
- terrain slope and normal;
- terrain-relative roll and pitch;
- static stability margin from the support polygon and center of mass;
- optional effective stability margin from current kinematic acceleration.

Objects are treated as static because the current segmentation output has no
tracked velocity.

## Inputs

- `/lr/future_path/ground_truth` (`nav_msgs/msg/Path`)
- `/terrain_geometry/grid_map` (`grid_map_msgs/msg/GridMap`)
- `/segmentation/boxes_3d` (`vision_msgs/msg/Detection3DArray`)
- `/lr/mavlink/pose` (`geometry_msgs/msg/PoseStamped`), dynamic profile only

All inputs must use the configured `map` frame. A recent empty detection array
means no object was detected; missing or stale data is reported as unknown.

## Outputs

- `/lr/path_prediction/steps` (`diagnostic_msgs/msg/DiagnosticArray`) publishes
  one status per step with pose, terrain, collision, and stability evidence.
- `/lr/path_prediction/markers` (`visualization_msgs/msg/MarkerArray`) publishes
  the 20 points, slope labels, `!` collision marks, and normal arrows for RViz.

Points are gray by default, orange/red for slope warnings, and magenta for
object collisions. Labels contain only the slope and append `!` on collision.

## Standalone launch

```bash
ros2 launch lr_path_prediction path_prediction.launch.py \
  use_sim_time:=true
```

The main `display_zed_cam.launch.py` starts this node automatically when future
path, terrain, and segmentation are enabled. All rover geometry, CoM, collision
margin, warning, and visualization settings are in
`config/path_prediction.yaml`. The bundled rover values are references only and
must be replaced with measured/CAD values before field use.

The default `prediction_profile:=static` needs no robot state. The optional
`dynamic` profile estimates acceleration inside this same node from stamped
MAVLink poses; it does not start an adapter or another prediction engine.
