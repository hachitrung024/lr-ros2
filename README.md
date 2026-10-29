# Landfill Rover ROS 2

A distributed perception pipeline: the host provides ZED NET video and MAVLink
telemetry, Jetson runs the headless backend, and the host displays results in
RViz. The standalone SVO simulator runs without ROS.

## System architecture

```mermaid
flowchart LR
    subgraph Host
        S[SVO recording or stereo camera]
        D[Recorded telemetry or FCU]
        R[RViz]
    end
    subgraph Jetson
        Z[ZED NET receiver and timestamp relay]
        M[MAVROS and source-session plugins]
        P[Causal pose and map TF]
        T[Terrain geometry]
        G[Segmentation and 3D tracking]
        F[Future-path assembly and ENU conversion]
        B[Prediction bridge and physics]
    end
    S -->|ZED NET| Z
    D -->|MAVLink UDP: telemetry, time calibration, optional path| M
    M --> P
    M --> F
    P --> Z
    Z --> T
    Z --> G
    P --> G
    F --> B
    T --> B
    G --> B
    P --> B
    Z -->|ROS 2 DDS| R
    T -->|ROS 2 DDS| R
    G -->|ROS 2 DDS| R
    B -->|ROS 2 DDS| R
```

The normal deployment keeps Jetson running while the host source starts,
stops or replays a recording. Jetson receives NET input only; it does not open
an SVO or a local camera.

| Mode | Host source | Jetson launch | Time ownership | DB on Jetson |
|---|---|---|---|---|
| Always-on live simulation | Python, `--time-mode live` | `headless_live.launch.py` | System time; no `/clock` | No |
| ROS replay baseline | `host_replay.launch.py` | `headless_zed_cam.launch.py` | Host ZED source publishes `/clock` | Yes, reference path/origin only |
| Python replay baseline | Python, `--time-mode replay` | `headless_zed_cam.launch.py`, UDP clock | Jetson clock bridge publishes `/clock` | Yes, reference path/origin only |

Run one source/backend mode per set of endpoints. The two recorded-time
baselines remain available for comparison.

### Packages and responsibilities

| Location | Responsibility |
|---|---|
| `tools/host_replay/` | Independent Python SVO NET sender, telemetry replay and optional future path |
| `src/zed-ros2-wrapper/` | Upstream ZED ROS 2 dependency, pinned as a submodule |
| `lr_zed_stream`, `lr_mavlink_replay` | Original ROS source packages |
| `lr_mavros_bridge` | MAVROS plugins, timestamp relay, persistent camera supervisor, pose/TF and path reception |
| `lr_future_path` | Coordinate/path processing and DB reference paths for replay baselines |
| `lr_terrain_geometry` | Point-cloud terrain estimation and GridMap output |
| `lr_segmentation` | Image segmentation, depth-based 3D boxes and object tracking |
| `safety_perception_msgs` | Typed prediction interfaces |
| `lr_prediction_bridge` | Path, terrain, object and pose adapters |
| `prediction_core`, `prediction_ros` | Collision/rollover calculations and ROS runtime |
| `lr_path_prediction` | Prediction diagnostics and cycle-matched visualization |
| `lr_display_rviz2` | Backend launch files and the RViz-only UI |

LR packages live under `src/lr-ros2/`. Recordings belong on the host; models
belong on Jetson. Build products and local data are outside source control.

### Time, pose and recovery

- Always-on nodes use `use_sim_time=false`. The simulator maps recorded IMAGE
  time to host monotonic boot time; MAVROS TIMESYNC maps boot time to ROS time.
  The relay preserves sensor payloads/frame IDs and gives matching RGB, depth,
  camera-info and cloud products the same converted timestamp.
- Pose uses received GPS/attitude/IMU samples at or before the image timestamp.
  The first valid GPS fix sets a fixed ENU origin for the backend lifetime.
  MAVROS coordinate conversion, GeographicLib geoid correction and configured
  body-to-camera extrinsics are applied consistently.
- The pose adapter alone owns `map -> zed_camera_link`. Wrapper tracking and
  map TF are disabled in the always-on path. With another `camera_name`, the
  child frame is `<camera_name>_camera_link`.
- MAVROS TIMESYNC runs at 10 Hz, converges over 30 samples and rejects RTT
  samples above 20 ms. Missing/stale telemetry suppresses operational outputs.
- Inputs expire after 2 s. The camera supervisor allows 15 s for SDK startup,
  requires a real raw frame before readiness, and restarts an established NET
  receiver after 5 s without frames, with a 2 s retry interval.
- Session/generation changes clear queues, timestamp/TF caches, terrain,
  trackers and prediction state. In-flight old results and pre-boundary queued
  data are rejected. Models and the ENU origin remain loaded.

### Future path and prediction

The host's `--future-path` option reads recorded look-ahead separately from the
causal telemetry cursor. It sends path snapshots over the existing MAVLink
socket; no additional port is needed. Private `TUNNEL` types are `32768` for
source/time calibration and `32769` for path fragments.

Jetson assembles complete, checksummed snapshots, rejects old sessions and
sequences, converts GPS/MSL coordinates to the fixed ENU frame, applies the
camera extrinsics, and publishes `/lr/future_path/mavlink`. Paths expire after
1 s; missing paths clear prediction while the sensor pipeline continues.

The canonical bridge samples up to 20 future steps. The `static` profile uses
path, terrain and objects; `dynamic` additionally uses pose-derived velocity
and acceleration. Prediction publishes collision/rollover evidence, not rover
control commands. Recorded future GPS is an evaluation reference, not a planner.
A physical FCU does not automatically provide this private path format; a
planner or a separate standard-MAVLink mission adapter must provide its route.

## Environment and build

Commands assume a prepared ROS 2 Humble environment with ZED SDK **5.4.1** and
its matching `pyzed.sl` binding. Run commands directly in that environment from
the workspace root. The tested ROS platform is Ubuntu 22.04 / compatible
JetPack on Jetson.

Required dependencies include MAVROS/GeographicLib (`egm96-5`), GridMap/RViz,
OpenCV, NumPy, Shapely and the ROS message/build dependencies in `package.xml`.
Jetson segmentation needs a CUDA-enabled PyTorch build matching its JetPack,
plus the Python requirements in `lr-segmentation`. The source host needs
pymavlink; the standalone sender itself does not need ROS or colcon.

Initialize the pinned dependency and install ROS dependencies:

```bash
git submodule update --init --recursive
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
sudo apt-get install ros-humble-mavros libgeographic-dev geographiclib-tools \
  ros-humble-grid-map-rviz-plugin
sudo geographiclib-get-geoids egm96-5
```

Install host sender requirements:

```bash
python3 -m pip install -r tools/host_replay/requirements.txt
```

For segmentation, first install the platform-compatible CUDA PyTorch build,
then install `src/lr-ros2/lr-segmentation/requirements.txt` without replacing
that build. Keep the SVO and DB paths explicit when running the sender.

Stop active ROS launches before rebuilding. Build the host source and UI:

```bash
cd ~/Projects/zed/lr-ros2
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to lr_mavlink_replay lr_display_rviz2
source install/setup.bash
```

Build the Jetson backend:

```bash
cd ~/Projects/zed/lr-ros2
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to lr_display_rviz2
source install/setup.bash
```

### Source synchronization

From the host workspace root, preview, then synchronize LR source to Jetson:

```bash
git ls-files -co --exclude-standard -z -- src/lr-ros2/ | \
  rsync -avhn --ignore-missing-args --from0 --files-from=- ./ \
  jetson:/home/hachi/Projects/zed/lr-ros2/
git ls-files -co --exclude-standard -z -- src/lr-ros2/ | \
  rsync -avh --ignore-missing-args --from0 --files-from=- ./ \
  jetson:/home/hachi/Projects/zed/lr-ros2/
```

This transfers tracked and non-ignored new LR source only. It does not delete
remote files or transfer recordings, models, build outputs or `tools/`.
Initialize the upstream submodule on each machine. Rebuild/source the Jetson
workspace after synchronization. Retired ROS entry points and stale installed
copies belong in `.legacy-archive/` with `COLCON_IGNORE` before rebuilding.

### ROS environment

In every ROS terminal on either machine:

```bash
cd ~/Projects/zed/lr-ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0
```

The examples use host `10.42.0.1`, Jetson `10.42.0.242`, NET port `30000` and
MAVLink host `14550` -> Jetson `14540`. Supply your network endpoints explicitly.
NET requires an even port. Permit ZED NET traffic and ROS 2 DDS traffic between
the machines; images/clouds/results return to RViz over DDS.

In `udp://BIND_ADDRESS:BIND_PORT@REMOTE_ADDRESS:REMOTE_PORT`, the address before
`@` is the Jetson bind endpoint and the address after `@` is the source endpoint.

### Machine clock synchronization

Before running the always-on mode, verify both machines are synchronized:

```bash
timedatectl show -p NTPSynchronized
ssh jetson 'timedatectl show -p NTPSynchronized; timedatectl timesync-status'
```

For initial configuration, with chrony installed on the host:

```bash
sudo bash tools/host_replay/configure_time_sync.sh host 10.42.0.1 10.42.0.242
scp tools/host_replay/configure_time_sync.sh jetson:/tmp/lr-configure-time-sync.sh
ssh -t jetson 'sudo bash /tmp/lr-configure-time-sync.sh jetson 10.42.0.1 10.42.0.242'
```

The script backs up changed configuration, serves NTP from the host and points
Jetson's systemd-timesyncd at it. Different display time zones are acceptable;
Unix timestamps must agree.

## Run the always-on pipeline

### 1. Jetson: start the backend once

In a Jetson ROS terminal:

```bash
ros2 launch lr_display_rviz2 headless_live.launch.py \
  camera_model:=zed2i stream_address:=10.42.0.1 stream_port:=30000 \
  mavros_fcu_url:='udp://0.0.0.0:14540@10.42.0.1:14550' \
  segmentation_model_path:="$PWD/models/best.pt" \
  prediction_enabled:=true prediction_profile:=static
```

The backend can start before the source and remain running between sessions.
It accepts no recording DB or clock arguments. Prediction defaults off; the
example enables it and therefore requires a segmentation model. To run without
prediction, set `prediction_enabled:=false` here and in RViz and omit the
host's `--future-path`. Omit `segmentation_model_path` to disable segmentation
as well. No operating-system boot service is installed.

Optional backend parameters include `camera_name`, `map_frame`,
`mavlink_body_to_camera:='[x,y,z,roll,pitch,yaw]'`, `mavros_audit_file` and
`rover_config:=/absolute/path/to/measured-rover.yaml`. Extrinsics use metres and
radians. The bundled rover config contains reference values for evaluation.

### 2. Host: start the independent simulator

Use a Python environment with ZED SDK/pyzed and pymavlink; ROS setup is not
required. Replace the SVO path with the matching local recording:

```bash
cd ~/Projects/zed/lr-ros2
python3 tools/host_replay/replay.py --time-mode live --future-path \
  --svo-path /absolute/path/to/recording.svo2 \
  --mavlink-db-path "$PWD/mavlink/session_20260710_0924_mavlink/session_mavlink.db" \
  --stream-port 30000 \
  --mavlink-bind-address 10.42.0.1 --mavlink-bind-port 14550 \
  --mavlink-destination 10.42.0.242 --mavlink-destination-port 14540 \
  --play-from-frame 4500
```

The entire `tools/host_replay/` directory can be copied elsewhere and run as
`python3 replay.py` with the same arguments. Playback is sequential 1x with
H.264, 20000 Kbit/s and GOP 1. `--play-from-frame` defaults to 0. EOF, Ctrl+C or
SIGTERM closes the source and sockets; there is no loop, pause or seek.
Live mode rejects clock endpoints. Stop/start the sender to begin a new session.

Path options are `--future-path-radius-m` (15 m), `--future-path-step-m` (0.2 m)
and `--future-path-rate-hz` (at most 5 Hz, sampled on frames). Snapshots contain
at most 128 points and 120 s of look-ahead; fragments expire after 0.5 s.
Keep updates comfortably faster than the 1 s path freshness limit.

### 3. Host: open RViz

In a separate host ROS terminal:

```bash
ros2 launch lr_display_rviz2 rviz_zed_cam.launch.py \
  camera_model:=zed2i use_sim_time:=false svo_mode:=false \
  segmentation_enabled:=true prediction_enabled:=true
```

This starts only the UI. Prediction displays use
`/lr/path_prediction/reference_path`, matched to the displayed prediction cycle.
Use `segmentation_enabled:=false` to display RGB without the model.

### Physical camera and FCU

Replace the simulator with the camera NET sender in a host ROS terminal:

```bash
ros2 launch lr_zed_stream stream_sender.launch.py svo_path:=live port:=30000
```

Configure the FCU to send GPS/attitude/IMU to the same Jetson MAVROS endpoint.
Use `headless_live.launch.py` with `prediction_enabled:=false` until a supported
future-route provider is present. Native current camera timestamps do not need
simulator session metadata. Physical camera/FCU acceptance is separate from SVO
simulation. Camera options include `serial_number`, `camera_id`, `resolution`
and `fps`.

## Recorded-time replay baselines

These modes use `use_sim_time=true` for processing/RViz and require the matching
DB on Jetson for reference ground truth. Keep the source playing while NET
connects. SDK 5.4.1 sequential playback is supported; pause/seek is experimental
and disabled by default.

### ROS source and ROS clock

Host, in a ROS terminal:

```bash
ros2 launch lr_mavlink_replay host_replay.launch.py \
  svo_path:=/absolute/path/to/recording.svo2 stream_port:=30000 \
  mavlink_bind_address:=10.42.0.1 mavlink_bind_port:=14550 \
  mavlink_destination:=10.42.0.242 mavlink_destination_port:=14540 \
  mavlink_db_path:="$PWD/mavlink/session_20260710_0924_mavlink/session_mavlink.db" \
  play_from_frame:=4500
```

Jetson, in a ROS terminal:

```bash
ros2 launch lr_display_rviz2 headless_zed_cam.launch.py \
  camera_model:=zed2i stream_address:=10.42.0.1 stream_port:=30000 \
  stream_svo_mode:=true future_path:=true \
  segmentation_model_path:="$PWD/models/best.pt" \
  mavros_fcu_url:='udp://0.0.0.0:14540@10.42.0.1:14550' \
  mavlink_db_path:="$PWD/mavlink/session_20260710_0924_mavlink/session_mavlink.db"
```

Host RViz, in another ROS terminal:

```bash
ros2 launch lr_display_rviz2 rviz_zed_cam.launch.py \
  camera_model:=zed2i use_sim_time:=true svo_mode:=false \
  svo_control_namespace:=/zed_source/zed_node \
  segmentation_enabled:=true prediction_enabled:=true
```

`replay_clock_source:=ros` is the backend default. Only the host node
`/zed_source/zed_node` publishes `/clock`; the Jetson DB node does not publish
pose or TF.

### Independent Python source and UDP clock

Replace the ROS source with:

```bash
python3 tools/host_replay/replay.py --time-mode replay \
  --svo-path /absolute/path/to/recording.svo2 \
  --mavlink-db-path "$PWD/mavlink/session_20260710_0924_mavlink/session_mavlink.db" \
  --stream-port 30000 \
  --mavlink-bind-address 10.42.0.1 --mavlink-bind-port 14550 \
  --mavlink-destination 10.42.0.242 --mavlink-destination-port 14540 \
  --clock-destination 10.42.0.242 --clock-destination-port 30010 \
  --play-from-frame 4500
```

Use the same Jetson replay command, adding these three launch arguments:

```text
replay_clock_source:=udp clock_udp_bind_address:=0.0.0.0 clock_udp_bind_port:=30010
```

Permit UDP `30010` and retain `use_sim_time:=true` in RViz. Only Jetson's
`/replay_clock_bridge` publishes `/clock`; it rejects malformed, duplicate,
reordered and retired-session packets and stops the launch on another clock
publisher. It never advances time without a received timestamp.
`--time-mode replay` remains the Python tool's default; `--future-path` is a
live-mode option only.

## Main interfaces and configuration

| Topic | Meaning |
|---|---|
| `/zed/zed_node/rgb/color/rect/image` | Normalized RGB image |
| `/zed/zed_node/depth/depth_registered` | Registered depth |
| `/zed/zed_node/point_cloud/cloud_registered` | Registered point cloud |
| `/lr/mavlink/pose` | Causal camera pose in `map` |
| `/lr/mavlink/origin` | Fixed geodetic ENU origin, MSL altitude |
| `/lr/future_path/mavlink` | Received path in always-on mode |
| `/lr/future_path/ground_truth` | DB reference path in replay baselines |
| `/segmentation/overlay`, `/segmentation/instance_mask` | Segmentation products |
| `/segmentation/boxes_3d`, `/segmentation/tracked_footprints` | 3D objects and tracked footprints |
| `/terrain_geometry/grid_map`, `/terrain_geometry/markers` | Terrain products |
| `/trajectory`, `/geometry`, `/tracked_objects` | Canonical prediction inputs |
| `/rover/state` | Dynamic-profile rover state |
| `/predict_output` | Raw prediction evidence |
| `/lr/path_prediction/reference_path`, `/lr/path_prediction/markers` | Matched path and warning visualization |
| `/lr/source/generation`, `/lr/future_path/diagnostics` | Live source readiness and path diagnostics |

The bridge consumes `Detection3DArray` from `/segmentation/boxes_3d` by default.
Objects are treated as static within a cycle when velocity is unavailable.
Missing terrain remains unknown; visualization preserves the full source path.

Runtime configuration is in each package's `config/` directory. Important files
are `lr_mavros_bridge/config/live.yaml`, `lr_prediction_bridge/config/bridge.yaml`,
`prediction_ros/config/prediction.yaml`, `prediction_core/config/rover.reference.yaml`
and `lr_path_prediction/config/path_prediction.yaml`. Change vehicle physics
through the rover config; RViz thresholds do not change the physical model.

## Verification

Observe the always-on pipeline with prediction:

```bash
python3 src/lr-ros2/lr-mavros-bridge/tools/verify_live.py \
  --with-prediction --duration 25 --timeout 60 --output /tmp/live-path.json
ros2 run lr_mavros_bridge verify_tf_owner
ros2 topic info /clock --verbose
ros2 topic echo /lr/source/generation --once
```

Expect zero clock publishers in always-on mode and exactly one in either replay
baseline. Without prediction, omit `--with-prediction`; without segmentation,
add `--without-segmentation`.

For recorded-time replay:

```bash
python3 src/lr-ros2/lr-zed-stream/tools/verify_net_replay.py \
  --duration 25 --require-prediction --output /tmp/replay.json
```

Run standalone tests without ROS, and ROS tests in a built/sourced workspace:

```bash
python3 -m pytest -q tools/host_replay/test
export ROS_DOMAIN_ID=93 ROS_LOCALHOST_ONLY=1
colcon test --packages-select lr_mavros_bridge lr_display_rviz2 \
  lr_prediction_bridge prediction_ros lr_path_prediction \
  lr_segmentation lr_terrain_geometry lr_zed_stream lr_mavlink_replay
colcon test-result --verbose
```

Use separate test domains on the two machines when testing concurrently.
Restore `ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0` before running the deployment.
