# Landfill Rover ROS 2

## Full launch command

This command plays an SVO, selects its matching MAVLink session, publishes the
future ground-truth path, and starts segmentation, terrain geometry, canonical
prediction, and RViz:

```bash
ros2 launch lr_display_rviz2 display_zed_cam.launch.py \
  camera_model:=zed2i \
  publish_svo_clock:=true \
  segmentation_model_path:=/path/to/best.pt \
  mavlink:=true \
  future_path:=true \
  svo_path:=/path/to/recording.svo2
```

The prediction launch arguments are optional. The command above uses the
default `prediction_profile:=static`. To run the dynamic model, append:

```bash
prediction_profile:=dynamic
```

Dynamic prediction currently requires `mavlink:=true` because its rover-state
adapter derives velocity and acceleration from `/lr/mavlink/pose`.

By default, `mavlink_db_path` is empty and the launch recursively searches
`mavlink_dir` for `session_mavlink.db`. To select a database explicitly, use:

```bash
mavlink_db_path:=/path/to/session_mavlink.db
```

When `future_path:=true` and `mavlink:=true`, the future path is read directly
from the MAVLink database. The SVO VIO pose-cache pass is therefore skipped.

## Canonical prediction pipeline

The complete runtime Prediction implementation from the provided
`prediction-rover` source is now integrated as normal ROS 2 packages under
`src/lr-ros2`:

| Package | Responsibility |
|---|---|
| `safety_perception_msgs` | Canonical typed ROS messages |
| `prediction_core` | ROS-independent collision and rollover physics |
| `prediction_ros` | Runtime node that publishes raw safety evidence |
| `lr_prediction_bridge` | Adapters from LR pipeline messages to canonical messages |
| `lr_path_prediction` | Diagnostic and RViz presentation of canonical output |

The top-level `prediction-rover/` directory is reference source only and is
ignored by `colcon`. It may be removed after this integration without changing
the build or runtime pipeline.

The adapters perform these conversions:

| LR input | Canonical topic |
|---|---|
| `/lr/future_path/ground_truth` (`nav_msgs/Path`) | `/trajectory` (`safety_perception_msgs/Trajectory`) |
| `/terrain_geometry/grid_map` (`grid_map_msgs/GridMap`) | `/geometry` (`safety_perception_msgs/GeometryArray`) |
| `/segmentation/boxes_3d` (`vision_msgs/Detection3DArray`) | `/tracked_objects` (`safety_perception_msgs/TrackedObjectArray`) |
| `/lr/mavlink/pose` (`geometry_msgs/PoseStamped`) | `/rover/state` (`safety_perception_msgs/RoverState`), dynamic only |

For up to 20 future steps, the canonical engine computes discrete oriented
footprint collision candidates, terrain-relative roll and pitch, Static SSM,
and normalized Static SSM. The dynamic profile additionally computes edge
Stability Moments and point-mass ZMP when acceleration is available.

`/predict_output` is raw safety evidence. It intentionally does not assign
risk severity, apply Stop/Go policy, or command the rover. The visualization
node derives only UI warnings from that evidence:

- gray points when no warning is available;
- a slope label when terrain geometry exists;
- one large triangle/`!` above the nearest actual footprint intersection;
- terrain-normal arrows;
- orange/red points for configured slope display thresholds; only the nearest
  actual intersection is highlighted in magenta.

Steps outside the current terrain GridMap remain gray. Current segmentation
does not provide persistent tracks or object velocity, so detected objects are
treated as static within each prediction cycle.

The raw engine also reports objects within `collision_margin_m` as collision
candidates. A positive-clearance candidate does not add `!` or change the RViz
color; it remains available in `/predict_output` for a downstream Decision
Node.

## Building

```bash
cd /path/to/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select \
  safety_perception_msgs \
  prediction_core \
  prediction_ros \
  lr_prediction_bridge \
  lr_future_path \
  lr_segmentation \
  lr_terrain_geometry \
  lr_path_prediction \
  lr_display_rviz2
source install/setup.bash
```

No installation from `prediction-rover/` is required.

## Rover configuration

The bundled rover values are references only. Before field use, replace them
with measured/CAD mass, body dimensions, support polygon, center of mass,
ground clearance, and collision margin. Supply the resulting file with:

```bash
prediction_rover_config:=/path/to/rover.yaml
```

`prediction_params_file` configures only diagnostic/RViz presentation, such as
slope thresholds and marker dimensions. It does not configure the physics
model.

## Main outputs

```text
/lr/mavlink/pose
/lr/future_path/ground_truth
/segmentation/overlay
/segmentation/boxes_3d
/terrain_geometry/grid_map
/terrain_geometry/markers
/trajectory
/geometry
/tracked_objects
/rover/state                  # dynamic profile
/predict_output               # canonical raw safety evidence
/lr/path_prediction/steps     # DiagnosticArray presentation
/lr/path_prediction/markers   # RViz MarkerArray presentation
```

`/lr/future_path/ground_truth` is a recorded future trajectory for offline
visualization and evaluation. It is not a planned path and must not be used to
control the rover.

## Other camera modes

SVO with ZED pose, without MAVLink or future prediction:

```bash
ros2 launch lr_display_rviz2 display_zed_cam.launch.py \
  camera_model:=zed2i \
  svo_path:=/path/to/recording.svo2 \
  publish_svo_clock:=true \
  mavlink:=false \
  future_path:=false \
  segmentation_model_path:=/path/to/best.pt
```

Future path from ZED VIO:

```bash
ros2 launch lr_display_rviz2 display_zed_cam.launch.py \
  camera_model:=zed2i \
  svo_path:=/path/to/recording.svo2 \
  publish_svo_clock:=true \
  mavlink:=false \
  future_path:=true \
  future_path_cache_dir:=/path/to/future_path_cache
```

Live camera:

```bash
ros2 launch lr_display_rviz2 display_zed_cam.launch.py \
  camera_model:=zed2i \
  svo_path:=live \
  mavlink:=false \
  future_path:=false \
  segmentation_model_path:=/path/to/best.pt
```

MAVLink and the future ground-truth path are supported only with SVO playback,
not with a live camera. MAVLink publishes `map -> zed_camera_link` when
`camera_name:=zed`. If exactly one session matching the SVO timestamp cannot be
found, the launch stops instead of falling back to ZED localization.
