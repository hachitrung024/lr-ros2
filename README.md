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

Prediction is built as normal ROS 2 packages under `src/lr-ros2`.

| Package | Responsibility |
|---|---|
| `safety_perception_msgs` | Canonical typed ROS messages |
| `prediction_core` | ROS-independent collision and rollover physics |
| `prediction_ros` | Runtime node that publishes raw safety evidence |
| `lr_prediction_bridge` | Adapters from LR pipeline messages to canonical messages |
| `lr_path_prediction` | Diagnostic and RViz presentation of canonical output |

The dynamic display pipeline has seven LR application nodes: `mavlink_pose`,
`segmentation`, `box_estimator_3d`, `terrain_geometry`,
`prediction_bridge_node`, `prediction_node`, and
`canonical_prediction_visualizer`. ZED, its component container, robot state
publisher, and optional RViz are additional infrastructure.

One `prediction_bridge_node` handles the four conversions below. Geometry is
sampled directly from the trajectory in that process. The four old adapter
executables remain available for standalone use and share the same converters.

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

Steps outside the current terrain GridMap remain gray. The box estimator has
Kalman tracking with persistent numeric IDs until its tracker resets. Its
`Detection3DArray` output has no object velocity, so `velocity_valid` stays false
and objects are treated as static within each prediction cycle. Class/score
metadata is currently `unknown`/1.0 in that box output, not the YOLO class score.

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

Run `source install/setup.bash` again after rebuilding new executables.
For pause/resume with the default `svo_realtime:=true`, build the existing
ZED SDK 5.4 realtime-pause patch once:

```bash
CMAKE_BUILD_PARALLEL_LEVEL=2 bash scripts/build_zed_realtime_pause.sh
source install/setup.bash
```

This script restores the wrapper source after building the patched component.
The stereo RViz preset uses the LR overlay/boxes/markers and SVO control panel;
it does not require the unused ZED object/body display or Nav2 panel plugins.

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

## Replay configuration and synchronization

The original full command remains valid. To run without the RViz process, add
`start_rviz:=false`. Overlays, heatmaps, and markers are generated only when
subscribed; masks, boxes, GridMap and raw prediction remain available.

| Launch argument | Owns |
|---|---|
| `bridge_params_file` | Bridge input/output topics and trajectory sampling |
| `prediction_runtime_params_file` | Runtime input age limits and readiness |
| `prediction_params_file` | Diagnostics/RViz presentation |
| `prediction_rover_config` | Rover physics parameters |

YAML is loaded first; explicit launch topic/frame/profile/time settings override
it. The display launch includes the same `lr_path_prediction` launch used for
standalone prediction. The bridge YAML uses the `prediction_bridge_node` section
and role prefixes (`trajectory.*`, `geometry.*`, `tracked_objects.*`,
`rover_state.*`); old adapter sections remain for compatibility.

Default replay age limits relative to the trajectory timestamp are objects
0.5 s, GridMap 2 s and rover state 0.25 s. An empty object batch retains its
measurement timestamp and expires too. Future measurements cannot complete a
cycle. `/geometry.header.stamp` is the GridMap sensor timestamp;
`source_trajectory_stamp` and `source_trajectory_id` identify its trajectory.
Set the bridge `geometry.max_geometry_age_sec` and runtime
`max_geometry_age_sec` together when changing terrain age limits.

Path, pose and GridMap frames must match `expected_frame_id`; the compatibility
`force_frame_id_map` parameter no longer relabels coordinates. Boxes in other
frames use timestamped TF, retried without blocking up to `tf_timeout_sec`.
Missing/invalid TF or malformed detections do not become an empty observation.

Mask/depth synchronization retains at most 30 messages per stream and 1 s of
source time (`sync_buffer.max_samples`, `sync_buffer.max_age_sec`), pairing the
nearest available timestamps within `sync_tolerance_sec` (50 ms). Each message
is consumed at most once. Rewinding SVO or changing the time source clears
buffers, terrain, tracking, bridge and runtime caches. Bridge trajectory IDs
continue increasing after rewind. Pausing leaves the simulation clock still.
Reset clears presentation without emitting a synthetic empty objects batch;
prediction waits for a new measured observation.

`/prediction/diagnostics` reports missing, future or stale runtime inputs and
callback time. Perception timing and mask/depth pairing counters are on
`/diagnostics`. Prediction visualization clears warnings while runtime reports
unavailable evidence or an expired trajectory.

## Validation and replay measurement

Tests are selected per package to avoid same-name pytest modules across packages:

```bash
colcon test --packages-select prediction_core prediction_ros lr_prediction_bridge \
  lr_segmentation lr_terrain_geometry lr_path_prediction lr_display_rviz2
colcon test-result --verbose
```

The replay observer records 60 SVO seconds after a 10-second warmup and shuts
down its own launch process group:

```bash
python3 scripts/benchmark_prediction.py \
  --output log/prediction-benchmark \
  --svo /workspace/svo/zed_20260710_092420_0001.svo2 \
  --model models/best.pt
```

To verify the original display command with RViz, including pause/resume and
rewind through the ZED services:

```bash
python3 scripts/verify_prediction_replay.py \
  --svo /workspace/svo/zed_20260710_092420_0001.svo2
```

Reports include observed topic counts, completed cycles, simulated-time latency,
per-process CPU/RSS and GPU memory/utilization. The observer subscribes to masks
and boxes, but not debug images/markers. Callback timing and pairing diagnostics
are recorded when published by that version. See
[refactor measurements](docs/prediction-refactor.md) for the measured comparison.

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
