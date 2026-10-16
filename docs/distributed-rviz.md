# Distributed headless pipeline and RViz

`headless_zed_cam.launch.py` runs the ZED wrapper and the complete LR
perception/prediction pipeline without starting RViz. `rviz_zed_cam.launch.py`
runs only RViz and can connect to that pipeline on the same host or another
machine through ROS 2 DDS.

## Same-host test

Build the workspace once, then run the following setup in each terminal:

```bash
cd /zed/lr-ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
```

Terminal 1 runs the headless side:

```bash
ros2 launch lr_display_rviz2 headless_zed_cam.launch.py \
  camera_model:=zed2i \
  publish_svo_clock:=true \
  segmentation_model_path:=models/best.pt \
  mavlink:=true \
  future_path:=true \
  svo_path:=/zed/svo/zed_20260710_092420_0001.svo2
```

Terminal 2 runs only the UI:

```bash
ros2 launch lr_display_rviz2 rviz_zed_cam.launch.py \
  camera_model:=zed2i \
  use_sim_time:=true \
  svo_mode:=true \
  segmentation_enabled:=true
```

There should be one `rviz2` process and one copy of every backend node. The UI
terminal does not need access to the SVO, MAVLink database, segmentation model,
or Jetson GPU.

## Move the headless side to Jetson

Build and source this workspace on both machines. The laptop needs the custom
message packages, `lr_display_rviz2`, RViz and `grid_map_rviz_plugin`; it does
not run any perception node. Use the same ROS distribution on both systems.

Set these values on both machines before launching:

```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
```

If either side runs in Docker, use `--network host`. The Jetson headless
container needs no `DISPLAY` variable or X11 socket mount. The laptop container
still needs its normal display/X11 or Wayland setup because RViz runs there.

Confirm discovery on the laptop:

```bash
ros2 node list
ros2 topic list
ros2 topic info /segmentation/overlay --verbose
ros2 topic echo /clock --once
```

If no remote nodes appear, check that both terminals have the same
`ROS_DOMAIN_ID`, neither has `ROS_LOCALHOST_ONLY=1`, multicast works between the
two LAN interfaces, and the host firewall permits DDS UDP traffic. Matching the
RMW implementation on both machines simplifies diagnosis, although DDS vendors
are designed to interoperate.

Raw images, depth maps and point clouds use substantial bandwidth. A wired LAN
is preferable. For a constrained link, reduce the ZED publication resolution,
image frame rate and point-cloud frequency before changing application QoS.

## A mask appears but its 3D footprint is late or missing

Check the `box_estimator_3d` status on `/diagnostics`: compare `masks_received`,
`matched_pairs`, `dropped_masks`, `invalid_pairs`, and `tf_failures`.
Many received masks but few matched pairs indicate a mask/depth delivery or
timestamp problem. `All instances have insufficient valid depth` means a
2D detection exists but cannot yet support a 3D measurement. `Waiting for
object confirmation` means the tracker needs another matching measurement.

The bundled segmentation configuration uses **Reliable** mask and registered
depth delivery with DDS history 10; ZED publishes registered depth reliably.
Best Effort depth delivery can lose large 1080p samples before the timestamp
matching buffer receives them, even when both nodes run on Jetson. The overlay
remains Best Effort. After updating this configuration, restart the complete
Jetson backend so the mask publisher and estimator use compatible QoS. For an
external Best Effort image source, explicitly set `input.mask_reliability` or
`input.depth_reliability` to `best_effort` in the segmentation parameters file.

The RViz preset displays `/segmentation/box_markers` as vertical footprint
prisms; rectangular OBB markers are hidden by default. Markers expire after
`visualization.hold_sec` (0.5 seconds of ROS time), so sparse measured updates
can make a footprint flash briefly. Check synchronization and valid depth
before increasing this presentation hold or the timestamp tolerance.

## UI modes

For SVO playback, use `use_sim_time:=true svo_mode:=true`. RViz then follows the
remote `/clock`, and the SVO control panel discovers the remote pause/seek
services under the camera namespace.

For a live camera, use:

```bash
ros2 launch lr_display_rviz2 rviz_zed_cam.launch.py \
  camera_model:=zed2i \
  use_sim_time:=false \
  svo_mode:=false \
  segmentation_enabled:=true
```

The bundled RViz preset uses the default `camera_name:=zed`. For another camera
namespace, supply `rviz_config` with matching topic names. Set
`segmentation_enabled:=false` when the backend does not run segmentation; the
RGB dock is then remapped to `/<camera_name>/zed_node/rgb/color/rect/image`.
