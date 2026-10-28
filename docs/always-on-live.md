# Always-on Jetson, standalone live simulator

Start the Jetson backend once. Start or stop the host source whenever needed.
The backend uses wall time, has no database dependency and never publishes or
subscribes to `/clock`. NET, MAVLink and the return DDS visualization traffic
keep their existing endpoints. Prediction and database ground truth are disabled.

## Time synchronization

The host runs chrony; Jetson's systemd-timesyncd uses the host over the local
network. Run these commands on the host **before starting the pipeline**:

```bash
sudo bash tools/host_replay/configure_time_sync.sh host 10.42.0.1 10.42.0.242
scp tools/host_replay/configure_time_sync.sh jetson:/tmp/lr-configure-time-sync.sh
ssh -t jetson 'sudo bash /tmp/lr-configure-time-sync.sh jetson 10.42.0.1 10.42.0.242'
timedatectl show -p NTPSynchronized
ssh jetson 'timedatectl show -p NTPSynchronized; timedatectl timesync-status'
```

The script backs up configurations before changing them. It grants NTP access
only to the specified Jetson address. The machine time zones can differ: their
Unix timestamps must agree. The standalone simulator additionally supports
MAVLink TIMESYNC; real camera NET timestamps require synchronized machine clocks.

## Jetson: start once

Build/sync following AGENTS.md, then inside Jetson's `zed-dev` container:

```bash
cd /zed/lr-ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0
ros2 launch lr_display_rviz2 headless_live.launch.py \
  camera_model:=zed2i stream_address:=10.42.0.1 stream_port:=30000 \
  mavros_fcu_url:='udp://0.0.0.0:14540@10.42.0.1:14550' \
  segmentation_model_path:=/zed/lr-ros2/models/best.pt
```

This command can precede the host source. Network arguments are required; no
addresses or ports are built into the code. Segmentation is optional when the
model path is empty. `camera_name`, `map_frame`, `mavlink_body_to_camera` and
`mavros_audit_file` are optional. Replay/DB/clock/prediction arguments are rejected.
There is no boot-time service in this version; keep this launch running.

MAVROS uses live `sys_time` synchronization at 10 Hz (30-sample convergence,
20 ms maximum accepted RTT). A single pose adapter owns `map → zed_camera_link`;
ZED tracking and wrapper map TF are disabled. The first valid, current GPS fix
sets the ENU origin. The adapter converts ellipsoid height back to MSL using
GeographicLib, applies the configured body-camera extrinsics and retains that
origin until the backend process exits. Sensor extrapolation remains causal.
Terrain uses the full configured ROI with trajectory corridor filtering off.

## Host: start or stop the simulated source

Use a Python environment with ZED SDK 5.4.1/pyzed and pymavlink. No ROS setup is
required. The entire `tools/host_replay` directory can be copied elsewhere.

```bash
python3 tools/host_replay/replay.py --time-mode live \
  --svo-path /zed/svo/zed_20260710_092420_0001.svo2 \
  --mavlink-db-path /zed/lr-ros2/mavlink/session_20260710_0924_mavlink/session_mavlink.db \
  --stream-port 30000 \
  --mavlink-bind-address 10.42.0.1 --mavlink-bind-port 14550 \
  --mavlink-destination 10.42.0.242 --mavlink-destination-port 14540 \
  --play-from-frame 4500
```

Only the host reads the DB. `--play-from-frame` defaults to zero. Playback is
sequential 1×, H.264 20000 Kbit/s, GOP 1. EOF closes the source; Ctrl+C/SIGTERM
closes its sockets and SDK camera. There is no automatic loop, pause or seek.
Live mode rejects clock endpoints. Do not run another sender on the same NET
or MAVLink endpoints at the same time.

The simulator maps recording time to the host's monotonic boot clock. Standard
telemetry keeps its units and timestamp resolution, SYSTEM_TIME carries the
current Unix epoch, and TIMESYNC replies report that same boot clock. Only
samples at or before the current SVO frame are sent. Live startup waits for
fresh samples; the replay mode retains its historical-sample priming behavior.
As with the underlying encoder, 32-bit MAVLink boot milliseconds must not
overflow; this version fails explicitly if host uptime exceeds that range.

For actual live equipment, use the existing live camera sender and configure
the FCU to send GPS/attitude/IMU telemetry to the same Jetson MAVROS endpoint.
The Jetson command stays the same. No replay metadata is required for native,
current camera timestamps. Physical camera/FCU acceptance is separate from
the SVO simulator checks.

## Host RViz

Inside the host ROS environment:

```bash
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0
ros2 launch lr_display_rviz2 rviz_zed_cam.launch.py \
  camera_model:=zed2i use_sim_time:=false svo_mode:=false \
  segmentation_enabled:=true prediction_enabled:=false
```

There is no ground-truth future path in this mode. Existing sensor, pose,
terrain and segmentation topics and frame IDs remain unchanged.

## Calibration and recovery

The simulator carries one calibration descriptor in standard MAVLink TUNNEL
messages (`payload_type=32768`, a local experimental type), immediately at
startup and then once per second. The 48-byte little-endian layout is
`<4sB3x16sQQQ`: magic `LRST`, version `1`, zero padding, UUID bytes, sequence,
record IMAGE anchor nanoseconds, boot anchor nanoseconds. Sequence begins at
zero. The anchors are constant throughout a source process. This is not a
frame clock, and it adds no network socket or UDP port.

The MAVROS plugin rejects malformed, duplicate, reordered and retired-session
descriptors. Its `/lr/source/time_state` String topic carries version-one JSON
with connection/readiness, effective MAVROS offset and the source anchors.
The C++ relay converts `record → boot → ROS` using that offset. It caches each
frame's converted timestamp so RGB/depth/cloud/camera info agree even when
TIMESYNC updates between their callbacks. Native wall-time stamps pass through.
Sensor payloads and frame IDs are preserved; camera IMU input is subscribed
only when a consumer requests that output. Raw remapped topics live under
`/<camera>/zed_node/_raw/` and are internal implementation interfaces.

`/lr/net/frame_stamp` is a Header per normalized RGB frame and drives pose;
`/lr/net/raw_frame` monitors transport reception. `/lr/source/generation`
carries version-one JSON (`runtime_id`, `generation`, `ready`, `start_ns`,
`session_id`, `status`). A changed generation clears sensor/TF caches, terrain
and object tracking. In-flight inference results from the old generation are
discarded. Models stay loaded and the ENU origin stays fixed.
Sensor subscriptions rejoin the new camera writer when readiness returns;
the inference model is not reconstructed. The supervisor also retains the
origin in memory so a respawned pose adapter can restore it.

Missing sources are an ordinary waiting state. Without synchronized MAVLink,
the supervisor waits before opening NET. Inputs older than two seconds are
stale. A NET stall of five seconds restarts only its camera subprocess, with a
two-second retry interval. The top-level launch remains alive; child processes
also respawn after unexpected exits. Recovery is not permission to reuse old
sensor values or extrapolate indefinitely.

## Verification and comparison

```bash
python3 src/lr-ros2/lr-mavros-bridge/tools/verify_live.py \
  --duration 25 --output /tmp/live-net.json
ros2 topic info /clock --verbose
ros2 topic echo /lr/source/generation --once
```

The live probe checks outputs, wall timestamps, exact RGB/cloud timestamp
matches, pose-frame matches, one actual DDS publisher of the camera TF, and
absence of clock/ground-truth/prediction publishers. Run in a ROS environment;
use `--without-segmentation` if the model is intentionally disabled.

The original [ROS replay guide](mavros-replay.md) remains unchanged in behavior.
The [standalone replay guide](../tools/host_replay/README.md) uses the default
`--time-mode replay` and the UDP clock bridge. Those two modes retain recorded
time, simulated ROS time and the optional Jetson ground-truth DB.

Measured results, recovery scenarios and the physical-hardware boundary are
recorded in [live acceptance](always-on-live-validation.md).
