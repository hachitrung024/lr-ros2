# Landfill Rover ROS 2

## Full launch command

The following command plays an SVO, automatically finds the matching MAVLink
session, uses MAVLink instead of ZED dynamic TF, publishes the future
ground-truth path, and starts segmentation, terrain geometry, and RViz:

```bash
ros2 launch lr_display_rviz2 display_zed_cam.launch.py \
  camera_model:=zed2i \
  publish_svo_clock:=true \
  segmentation_model_path:=/path/to/best.pt \
  mavlink:=true \
  future_path:=true \
  svo_path:=/path/to/recording.svo2
```

By default, `mavlink_db_path` is empty and the launch file recursively searches
`mavlink_dir` for `session_mavlink.db`. To select a database explicitly, use:

```bash
mavlink_db_path:=/path/to/session_mavlink.db
```

When `future_path:=true` is enabled together with `mavlink:=true`, the future
path is read directly from the MAVLink database, so `future_path_cache_dir` and
`future_path_rebuild_cache` are not used.

## Building

```bash
cd /path/to/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select \
  lr_future_path \
  lr_segmentation \
  lr_terrain_geometry \
  lr_path_prediction \
  lr_display_rviz2
source install/setup.bash
```

## Common modes

### SVO with ZED pose, without MAVLink

```bash
ros2 launch lr_display_rviz2 display_zed_cam.launch.py \
  camera_model:=zed2i \
  svo_path:=/path/to/recording.svo2 \
  publish_svo_clock:=true \
  mavlink:=false \
  future_path:=false \
  segmentation_model_path:=/path/to/best.pt
```

### Future path from ZED VIO

```bash
ros2 launch lr_display_rviz2 display_zed_cam.launch.py \
  camera_model:=zed2i \
  svo_path:=/path/to/recording.svo2 \
  publish_svo_clock:=true \
  mavlink:=false \
  future_path:=true \
  future_path_cache_dir:=/path/to/future_path_cache
```

The first run creates a rosbag2 pose cache from the SVO. Later runs reuse the
cache.

### Live camera

```bash
ros2 launch lr_display_rviz2 display_zed_cam.launch.py \
  camera_model:=zed2i \
  svo_path:=live \
  mavlink:=false \
  future_path:=false \
  segmentation_model_path:=/path/to/best.pt
```

MAVLink and the future ground-truth path are supported only with SVO playback,
not with a live camera.

## Main outputs

```text
/lr/mavlink/pose
/lr/future_path/ground_truth
/segmentation/overlay
/segmentation/boxes_3d
/terrain_geometry/grid_map
/terrain_geometry/markers
/terrain_geometry/heatmap
/lr/path_prediction/steps
/lr/path_prediction/markers
```

When `future_path:=true`, terrain, and segmentation are enabled, the launch
also starts one self-contained `path_risk_predictor` node. It evaluates the
next 20 path poses (the current pose is excluded), uses the configured oriented
rectangular rover footprint for object clearance/collision, and computes
terrain roll, pitch, and static stability margin wherever a terrain normal is
available. It has no runtime or build dependency on the `prediction-rover/`
reference repository.

`/lr/path_prediction/steps` adds slope, normal, collision IDs, roll/pitch, and
stability evidence. `/lr/path_prediction/markers` shows all 20 path points,
compact slope labels, `!` collision marks, and terrain-normal arrows. Steps
without valid terrain stay gray and have no slope label. Colors change to
orange/red for slope warnings and magenta for a footprint collision.

The rover values in `lr_path_prediction/config/path_prediction.yaml` are
estimates only. Replace them with measured mass, body/support dimensions, and
center-of-mass values before field use. A different parameter file can be used
with:

```bash
prediction_params_file:=/path/to/path_prediction.yaml
```

The default `prediction_profile:=static` does not require velocity. Optional
`prediction_profile:=dynamic` makes the same node estimate map-frame kinematic
acceleration directly from `/lr/mavlink/pose` and adds effective stability
margin evidence. No state adapter is required.

MAVLink publishes the `map -> zed_camera_link` TF when
`camera_name:=zed`. If exactly one MAVLink session matching the SVO timestamp
cannot be found, the launch stops and does not fall back to ZED TF.

`/lr/future_path/ground_truth` is the recorded future trajectory for
visualization and offline evaluation. It is not a planned path and must not be
used to control the rover.
