# LR Path Prediction

`lr_path_prediction` is the diagnostic and RViz layer for the canonical
Prediction pipeline. The safety calculation itself runs in `prediction_core`
through the `prediction_ros/prediction_node` executable.

The standalone launch starts three processes: the consolidated LR prediction
bridge, canonical Prediction node and this visualizer. Only the dynamic
profile enables the rover-state conversion inside the bridge. It is also
included by the display launch, so runtime configuration has one source.

```bash
ros2 launch lr_path_prediction path_prediction.launch.py \
  use_sim_time:=true \
  prediction_profile:=static
```

## Data flow

| Input to bridge | Canonical input to Prediction |
|---|---|
| `/lr/future_path/ground_truth` | `/trajectory` |
| `/terrain_geometry/grid_map` | `/geometry` |
| `/segmentation/boxes_3d` | `/tracked_objects` |
| `/lr/mavlink/pose` | `/rover/state`, dynamic only |

The canonical engine publishes `safety_perception_msgs/msg/PredictionOutput`
on `/predict_output`. This package joins it with `/trajectory` and `/geometry`
and publishes:

- `/lr/path_prediction/steps` (`diagnostic_msgs/msg/DiagnosticArray`);
- `/lr/path_prediction/markers` (`visualization_msgs/msg/MarkerArray`);
- `/lr/path_prediction/reference_path` (`nav_msgs/msg/Path`).

The reference path and markers are emitted only after trajectory, geometry,
and prediction with the same `trajectory_id` have been joined. They share the
source trajectory timestamp and exactly the same displayed XYZ positions.
Terrain elevation is carried in each `GeometryStep`, so presentation never
mixes a prediction cycle with a newer rolling GridMap.

RViz displays up to 20 future points. A point is gray by default, receives a
slope label only when terrain is available, and has a terrain-normal arrow when
its normal is known. When multiple rover footprints intersect object
footprints, only the nearest future step is highlighted: a large `!` inside a
triangle is centered above that point. Positive-clearance candidates inside
the configured safety margin remain raw evidence in `/predict_output`; they do
not add a warning or change the UI color. Slope labels and colors are preserved
independently from the collision warning.

By default, RViz stops the visible path at the first step without valid terrain
instead of drawing that step at z=0. Diagnostics still retain every step and
report TERRAIN UNKNOWN. Set visualization.hide_unknown_terrain to false in the
presentation parameters to restore the previous gray-point behavior.

## Profiles

`static` requires trajectory, tracked objects, and matching terrain geometry.
It produces collision evidence, predicted roll/pitch, Static SSM, and
normalized Static SSM without requiring robot velocity.

`dynamic` additionally requires `/rover/state`. The LR adapter estimates
map-frame velocity and kinematic acceleration by finite differences of stamped
MAVLink poses. The core then adds Stability Moment and point-mass ZMP evidence.
Missing acceleration remains unknown; it is never silently interpreted as
zero.

## Configuration

`config/path_prediction.yaml` controls only the visualization topics,
slope-display thresholds, and marker dimensions.

The physics configuration is owned by `prediction_core`. Pass measured rover
parameters to the standalone launch with:

```bash
rover_config:=/path/to/measured-rover.yaml
```

The default `prediction_core/config/rover.reference.yaml` contains reference
values only and is not suitable for a field safety decision.

The canonical output is evidence, not a Decision Node: it does not assign
severity or publish Stop/Go commands. Detected objects are currently treated
as static because this pipeline does not publish object velocity. Numeric box
IDs persist across frames until tracker reset; they are not globally unique.


The old `lr_path_prediction.core` and `.node` physics implementation has been
removed. Presentation dataclasses live in `.presentation`; GridMap sampling is
shared with the bridge through `lr_terrain_geometry.grid_map_sampling`.
Physics regression tests are in `prediction_core/test`, sampling/state tests
in `lr_prediction_bridge/test`, and terrain sampling tests in
`lr_terrain_geometry/test`.

The visualizer joins asynchronous messages by trajectory ID, buffers results
that arrive before trajectory/geometry, clears warnings when evidence is
unavailable, and renders markers only for subscribers.
