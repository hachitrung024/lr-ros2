# Landfill Rover ROS 2 workspace

ROS 2 Humble workspace for the Landfill Rover perception pipeline.

## Source layout

- `src/zed-ros2-wrapper`: ZED camera and SVO/SVO2 playback.
- `src/trajectory_node`: MAVLink pose replay, point-cloud transformation,
  accumulated map, RViz, and the top-level `lr_bringup` launch package.
- `src/simulation_node` and `simulation/`: Gazebo bridges, rover model,
  proving-ground world, and the integrated simulation runner.
- `src/terrain_node`: rolling terrain-plane estimation.
- `src/prediction_node`: typed ROS interfaces and the prediction runtime adapter.

Generated `build/`, `install/`, and `log/` directories are intentionally not
stored in the workspace.

## Build inside the mounted development container

```bash
cd /workspace/lr-ros2
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

`prediction_ros` requires the ROS-independent `prediction_core`, which is
not currently vendored in this workspace. Restore that module and a rover
configuration before enabling prediction.

## Main camera and trajectory flow

SVO/SVO2 playback:

```bash
ros2 launch lr_bringup rover.launch.py \
  camera_model:=zed2i \
  svo_path:=/workspace/dataset/session_1/svo/video_1.svo2 \
  publish_svo_clock:=true
```

Live camera:

```bash
ros2 launch lr_bringup rover.launch.py \
  camera_model:=zed2i \
  svo_path:=live
```

The transformed/accumulated cloud is published on
`/lr/point_cloud/cloud_in_map`.

## Terrain flow

Run this in another shell after sourcing `install/setup.bash`:

```bash
ros2 launch lr_terrain_geometry terrain_geometry.launch.py \
  point_cloud_topic:=/lr/point_cloud/cloud_in_map \
  map_frame:=map \
  use_sim_time:=true
```

Use `use_sim_time:=false` with a live camera that does not publish `/clock`.

## Prediction flow

```bash
ros2 launch prediction_ros prediction.launch.py \
  config_path:=/absolute/path/to/rover.yaml \
  prediction_profile:=static
```

The prediction runtime consumes `safety_perception_msgs` on `/trajectory`,
`/tracked_objects`, `/geometry`, `/rover/state`, and `/external_wrenches`.
The current trajectory and terrain nodes do not yet publish those canonical
message types, so a typed upstream node or bridge is required for an integrated
prediction run. For a standalone smoke test, run:

```bash
ros2 run prediction_ros mock_upstream_node --ros-args -p demo_mode:=static
ros2 topic echo /predict_output
```

## Clean rebuild

Run inside the container so root-owned build products can be removed:

```bash
cd /workspace/lr-ros2
rm -rf -- build install log
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```
