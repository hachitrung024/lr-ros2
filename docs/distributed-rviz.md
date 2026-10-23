# Distributed headless pipeline and RViz

## Host source → ZED NET → Jetson → RViz

The SDK 5.4.1 sequential deployment was tested on the current host/Jetson pair;
see the [acceptance results and limits](net-streaming-validation.md).

The host owns the SVO2 file or USB stereo camera and runs `lr_zed_stream`.
Jetson receives the encoded ZED stream, computes depth/tracking and runs LR
segmentation, terrain and prediction. Only Jetson broadcasts the processing
TF tree; SVO MAVLink mode retains the MAVLink TF owner. RViz runs on the host.

Both machines use ROS 2 Humble, ZED SDK 5.4.1, Docker host networking,
`ROS_DOMAIN_ID=42` and `ROS_LOCALHOST_ONLY=0`. The current wired link is
host `10.42.0.1` → Jetson `10.42.0.242`. ZED NET uses even UDP port `30000`;
ROS 2 discovery, services and visualization still use DDS on the same LAN.

After source changes, preview and sync **only** the source described in
`AGENTS.md`, without deleting remote files or copying datasets/build outputs.
Stop the previous ROS launches before building. Inside the host container:

```bash
cd /zed/lr-ros2
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to lr_zed_stream lr_display_rviz2
source install/setup.bash
```

On Jetson, build `--packages-up-to lr_display_rviz2` and source its own install.
The source package needs the SDK Python binding `pyzed.sl` on the host.
Every terminal below runs inside its machine's `zed-dev` container, after:

```bash
cd /zed/lr-ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0
```

**Terminal 1, host source:**

```bash
ros2 launch lr_zed_stream stream_sender.launch.py \
  svo_path:=/zed/svo/zed_20260710_092420_0001.svo2 \
  port:=30000 codec:=H264 bitrate:=20000 gop_size:=1
```

SVO defaults to sequential 1× playback. Keep it playing until the receiver
opens: SDK 5.4.1 needs advancing frames to establish a NET connection.
The source owns `/clock` and publishes SVO status under `/zed_source/zed_node`.

**Terminal 2, Jetson backend:**

```bash
ros2 launch lr_display_rviz2 headless_zed_cam.launch.py \
  camera_model:=zed2i stream_address:=10.42.0.1 stream_port:=30000 \
  stream_svo_mode:=true \
  segmentation_model_path:=/zed/lr-ros2/models/best.pt \
  mavlink:=true mavlink_dir:=/zed/lr-ros2/mavlink future_path:=true
```

The receiver keeps `svo_path:=live` and `publish_svo_clock:=false`. It does not
need the SVO file. `stream_svo_mode:=true` enables simulated time for LR
consumers; the ZED receiver itself grabs on wall time, preserving source
IMAGE timestamps (`debug.use_pub_timestamps:=false`). Model and MAVLink data
remain on Jetson. NET matches the MAVLink session by time coverage so the
backend can join midway through playback; overlapping sessions require an
explicit `mavlink_db_path`. NET future path requires MAVLink, since there is
no local SVO for building a VIO cache.

**Terminal 3, host UI:**

```bash
ros2 launch lr_display_rviz2 rviz_zed_cam.launch.py \
  camera_model:=zed2i use_sim_time:=true svo_mode:=false \
  svo_control_namespace:=/zed_source/zed_node segmentation_enabled:=true
```

The UI keeps the existing images/results under Jetson's `/zed` namespace.
`svo_mode:=false` hides the playback panel; `use_sim_time:=true` still follows
the SVO clock. Sequential NET is the supported SDK 5.4.1 deployment.

### Pause/seek: experimental, not accepted

Hardware testing with SDK 5.4.1 reproduced NET receive timeouts during long
pauses and failure to update the selected frame after a paused seek. The
same failure occurs outside ROS and with GEN1/GEN2 transport. Holding the
source frame keeps its timestamp and `/clock` fixed, but does **not** guarantee
that the SDK transmits that frame or keeps the receiver session alive.

Playback services are therefore disabled by default. For investigation only,
start the source with `enable_svo_controls:=true`, and set RViz
`svo_mode:=true svo_control_namespace:=/zed_source/zed_node`. The source-local
pause/seek/EOF state is tested; NET pause/seek/Replay has **not** passed
acceptance. Keep the direct-file workflow below when those controls are needed.

### Live camera

Use `svo_path:=live` in terminal 1. Optionally select `serial_number` or
`camera_id`, `resolution:=HD720 fps:=60`. In terminal 2 use
`stream_svo_mode:=false`, omit MAVLink/future path and retain the segmentation
model. In terminal 3 use `use_sim_time:=false svo_mode:=false`. Live mode
supports depth, tracking, segmentation and terrain; recorded-data future
path/prediction are not enabled automatically.

### Verify the NET deployment

```bash
ros2 topic info /clock --verbose  # SVO: exactly one publisher, /zed_source/zed_node
ros2 topic echo /zed_source/zed_node/status/svo --once
ros2 topic echo /zed/zed_node/rgb/color/rect/camera_info --field header --once
ros2 topic hz /segmentation/overlay
ros2 topic hz /terrain_geometry/grid_map
python3 src/lr-ros2/lr-zed-stream/tools/verify_net_replay.py --duration 20
```

Compare `SvoStatus.frame_ts` with image/depth/cloud header timestamps for the
same source frame. The verification tool measures received frame rates and
Ethernet throughput and matches RGB/cloud stamps against the source clock.
Its latency metric measures source `/clock` arrival → returned data on the
same host's monotonic clock, without subtracting SVO timestamps from wall time.
`--exercise` requires experimental controls and reproduces the currently
unaccepted pause/seek scenarios.
After a source restart, start the source before restarting the backend if
the SDK exhausted its connection retries.

Keep the current RViz layers when measuring FPS and both Ethernet directions.
NET adds host → Jetson traffic; RViz RGB/overlay/clouds still use DDS in the
opposite direction. H.264 is lossy, and NET does not guarantee every source
frame is processed. The original direct-file launches below remain available.

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
