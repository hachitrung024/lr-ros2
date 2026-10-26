# Host telemetry replay → Jetson MAVROS

The Jetson backend always receives images over ZED NET. In SVO mode it runs
MAVROS and estimates camera pose using only received sensor samples at or
before each SVO timestamp. A separate DB process provides the fixed ENU origin
and offline future reference path. The backend does not open a local SVO or
publish pose from the DB.
See [measured acceptance results](mavros-replay-validation.md).

## Build and dependencies

Both containers use ROS 2 Humble, ZED SDK 5.4.1, host networking,
`ROS_DOMAIN_ID=42`, `ROS_LOCALHOST_ONLY=0`. Install MAVROS and GeographicLib on
Jetson and on the host development container (needed to build the workspace):

```bash
apt-get update
apt-get install -y ros-humble-mavros libgeographic-dev geographiclib-tools
geographiclib-get-geoids egm96-5
```

The host source additionally needs:

```bash
python3 -m pip install -r /zed/lr-ros2/src/lr-ros2/lr-mavlink-replay/requirements.txt
```

The environment Docker build installs these dependencies too. The accepted
versions are MAVROS **2.16.0**, pymavlink **2.4.50**, and GeographicLib's
**egm96-5** dataset. Changing MAVROS requires rerunning the telemetry timestamp
check because the replay plugin uses its UAS plugin API.

Stop old launches before building. Preview/sync only `src/lr-ros2/` following
`AGENTS.md`; recordings, models and build directories are not transferred.
Inside the host container:

```bash
cd /zed/lr-ros2
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to lr_mavlink_replay lr_display_rviz2
```

On Jetson use `colcon build --symlink-install --packages-up-to lr_display_rviz2`.
Both machines need the matching MAVLink DB: host for telemetry, Jetson for the
fixed origin and future ground truth. The SVO stays on the host.

## Three terminals

In each terminal, enter that machine's `zed-dev` container and run:

```bash
cd /zed/lr-ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0
```

**1. Host source (ZED NET and telemetry together):**

```bash
ros2 launch lr_mavlink_replay host_replay.launch.py \
  svo_path:=/zed/svo/zed_20260710_092420_0001.svo2 stream_port:=30000 \
  mavlink_bind_address:=10.42.0.1 mavlink_bind_port:=14550 \
  mavlink_destination:=10.42.0.242 mavlink_destination_port:=14540 \
  mavlink_db_path:=/zed/lr-ros2/mavlink/session_20260710_0924_mavlink/session_mavlink.db
```

`play_from_frame:=4500` selects the moving segment used for acceptance.
The source remains sequential 1×, H.264 20000 Kbit/s, GOP 1, ZED port 30000.
For custom encoder or network settings, run `lr_zed_stream` separately and
`ros2 run lr_mavlink_replay telemetry_replay` with the parameters below.

**2. Jetson backend:**

```bash
ros2 launch lr_display_rviz2 headless_zed_cam.launch.py \
  camera_model:=zed2i stream_address:=10.42.0.1 stream_port:=30000 \
  stream_svo_mode:=true \
  segmentation_model_path:=/zed/lr-ros2/models/best.pt \
  future_path:=true \
  mavros_fcu_url:='udp://0.0.0.0:14540@10.42.0.1:14550' \
  mavlink_db_path:=/zed/lr-ros2/mavlink/session_20260710_0924_mavlink/session_mavlink.db
```

**3. Host RViz:**

```bash
ros2 launch lr_display_rviz2 rviz_zed_cam.launch.py \
  camera_model:=zed2i use_sim_time:=true svo_mode:=false \
  svo_control_namespace:=/zed_source/zed_node segmentation_enabled:=true
```

`stream_svo_mode:=true` always enables the MAVROS pose adapter and reference
DB node. `future_path:=auto` (the default) enables reference paths for SVO;
`start_path_prediction_node:=auto` starts prediction when segmentation and
terrain are enabled too. Choose `prediction_profile:=static|dynamic` as before.
NET pause/seek remains experimental and disabled.

### Live stereo over NET

On the host, replace the SVO/telemetry source command with:

```bash
ros2 launch lr_zed_stream stream_sender.launch.py svo_path:=live port:=30000
```

On Jetson, use `stream_svo_mode:=false` and omit `mavros_fcu_url` and the DB
argument. Keep `stream_address`, `stream_port`, camera model and segmentation
model. Live uses wall time and ZED tracking, without replay MAVROS, DB ground
truth or prediction. On host RViz use `use_sim_time:=false`, `svo_mode:=false`
and `prediction_enabled:=false`.

## Data and time contracts

Network endpoints have no defaults in the replay node or MAVROS launch. Pass
the host `stream_port`, all four MAVLink endpoint arguments and the Jetson
`stream_address`, `stream_port` and SVO `mavros_fcu_url` explicitly.
The examples use the current wired network; change them for another network.

| Host launch argument | Replay node parameter | Meaning |
| --- | --- | --- |
| `mavlink_bind_address` | `bind_address` | Host local UDP address |
| `mavlink_bind_port` | `bind_port` | Host local UDP port |
| `mavlink_destination` | `destination` | Jetson destination address |
| `mavlink_destination_port` | `port` | Jetson destination port |

In `udp://BIND_ADDRESS:BIND_PORT@REMOTE_ADDRESS:REMOTE_PORT`, MAVROS binds to
the Jetson address/port before `@` and sends replies to the host endpoint after
`@`. `0.0.0.0` listens on every local interface. Both ends must agree on ports.
SVO NET rejects an empty `mavros_fcu_url` before starting processing.
Live NET does not start the replay gateway.

For a separately launched replay node, pass its parameters directly:

```bash
ros2 run lr_mavlink_replay telemetry_replay --ros-args \
  -p bind_address:=10.42.0.1 -p bind_port:=14550 \
  -p destination:=10.42.0.242 -p port:=14540 \
  -p db_path:=/zed/lr-ros2/mavlink/session_20260710_0924_mavlink/session_mavlink.db
```

`db_path` may be empty to match by timestamp within `mavlink_dir`.
The standalone `lr_mavros_bridge mavros_replay.launch.py` requires `fcu_url`.
System/component IDs are 1/1, with heartbeat at 1 Hz on a steady timer.

The DB contains decoded fields, not original MAVLink packets. Replay rebuilds
GPS_RAW_INT, ATTITUDE and HIGHRES_IMU in original timestamp order. For this
recorder, IMU acceleration is milli-g, gyro is mrad/s and temperature is
centidegrees Celsius; HIGHRES_IMU carries SI acceleration/gyro and Celsius.
Zero-filled magnetometer fields are not treated as measurements. Missing
fields remain unknown, and invalid GPS fixes cannot generate a pose. Validate
these unit assumptions before using a different recorder's database.

`/clock` from the ZED source is the sole producer. Replay releases only records
at or before that clock. Mid-session startup and rewind prime at most two
past samples per sensor within 1.5 seconds. A fixed session epoch and rounded
SYSTEM_TIME pair define the boot-to-recording offset exactly. The
`lr_replay_time` MAVROS plugin supplies that offset; ordinary `sys_time` /
TIMESYNC is disabled. GPS/IMU retain microsecond stamps; ATTITUDE is truncated
to milliseconds by MAVLink's field width. These settings are for recorded
telemetry, not a production FCU time-synchronization configuration.

MAVROS publishes `/mavros/global_position/raw/fix`, `/mavros/imu/data`,
`/mavros/imu/data_raw` and `/mavros/state`. The pose adapter waits 100 ms wall
time per SVO frame, considers only sensor timestamps ≤ that frame, and uses:

- The last two valid GPS samples; maximum gap 1.5 s, maximum age 0.75 s and
  maximum inferred speed 5 m/s.
- The latest attitude and angular velocity; maximum age 0.25 s.
- The session's first valid GPS as ENU origin, published by the ground-truth
  process as `/lr/mavlink/origin` (GeoPointStamped, transient-local; altitude
  uses the DB's AMSL convention). MAVROS ellipsoid height is converted back
  with the same GeographicLib geoid before constructing local coordinates.

The adapter alone emits `/lr/mavlink/pose` and dynamic
`map → zed_camera_link`, applying `mavlink_body_to_camera` as before. The
separate `mavlink_ground_truth_node` emits the fixed origin and optional
`/lr/future_path/ground_truth`; it never publishes pose or TF. The adapter
never reads the DB. MAVROS's NED/FRD → ENU/FLU conversion is applied once.

Missing/stale telemetry suppresses pose and TF, with status on `/diagnostics`
under `lr/mavros_pose`. A clock rewind clears pending frames and sensor
buffers. A stopped clock reports `clock_stopped`. Heartbeat connectivity alone
does not imply fresh sensor data. Existing prediction profiles and RViz layers
remain selectable; ground-truth paths remain an offline reference, not a
live navigation planner.

## Verification

```bash
ros2 topic info /clock --verbose
ros2 topic echo /mavros/state --once
ros2 run lr_mavros_bridge verify_tf_owner
python3 src/lr-ros2/lr-mavlink-replay/tools/verify_telemetry.py \
  --db mavlink/session_20260710_0924_mavlink/session_mavlink.db --duration 30
python3 src/lr-ros2/lr-zed-stream/tools/verify_net_replay.py \
  --duration 30 --require-prediction
```

Several nodes can advertise `/tf`; `verify_tf_owner` counts DDS publishers
actually sending the camera transform over eight seconds. The default check
uses `map → zed_camera_link`.

For numerical audit, pass `mavros_audit_file:=/tmp/lr-mavros-pose.jsonl` to the
backend. Copy that audit file to the host and run:

```bash
python3 src/lr-ros2/lr-mavlink-replay/tools/compare_pose_audit.py \
  --audit /tmp/lr-mavros-pose.jsonl \
  --db mavlink/session_20260710_0924_mavlink/session_mavlink.db
```

The audit comparison currently assumes the default identity body–camera
extrinsic. It checks an independent causal calculation and reports the separate
error against offline interpolation. A successful transport test does not
establish localization accuracy or prediction quality.
