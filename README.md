# Landfill Rover ROS 2

## Full launch command

The following command plays an SVO, automatically finds the matching MAVLink
session, uses MAVLink instead of ZED dynamic TF, publishes the future
ground-truth path, and starts segmentation, terrain geometry, and RViz:

```bash
ros2 launch lr_display_rviz2 display_zed_cam.launch.py \
  camera_model:=zed2i \
  svo_path:=/path/to/recording.svo2 \
  publish_svo_clock:=true \
  segmentation_model_path:=/path/to/best.pt \
  future_path:=true \
  mavlink:=true
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
```

MAVLink publishes the `map -> zed_camera_link` TF when
`camera_name:=zed`. If exactly one MAVLink session matching the SVO timestamp
cannot be found, the launch stops and does not fall back to ZED TF.

`/lr/future_path/ground_truth` is the recorded future trajectory for
visualization and offline evaluation. It is not a planned path and must not be
used to control the rover.
