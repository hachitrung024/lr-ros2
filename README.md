# Landfill Rover ROS 2

A distributed perception pipeline: independent sources provide ZED NET video
and MAVLink telemetry, Jetson runs the headless backend, and the host displays
results in RViz. The host's standalone SVO simulator runs without ROS.

## System architecture

```mermaid
flowchart LR
    subgraph Sources[External sources or host simulator]
        S[SVO recording or stereo camera]
        D[Recorded telemetry or FCU]
    end
    subgraph Host
        R[RViz]
    end
    subgraph Jetson
        Z[ZED NET receiver and timestamp relay]
        M[MAVROS and source-session plugins]
        P[Causal pose and map TF]
        T[Terrain geometry]
        G[Segmentation and 3D tracking]
        F[Mission or replay path and ENU conversion]
        B[Prediction bridge and physics]
    end
    S -->|ZED NET| Z
    D <-->|MAVLink: telemetry, TIMESYNC, mission or replay path| M
    M --> P
    M --> F
    Z -->|Frame timestamp| P
    P -->|Map TF| T
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

The normal deployment keeps Jetson running while either source starts or
stops. The replay profile also supports seeking on the host. Jetson receives
NET input only; it does not open an SVO or a local camera.

| Mode | Host source | Jetson launch | Time ownership | DB on Jetson |
|---|---|---|---|---|
| Native deployment | External ZED NET and FCU | `headless_live.launch.py`, `source_profile:=native` | Verified system time; no `/clock` | No |
| Native source simulation | Python, `--time-mode native` | Same native launch and clock contract as real sources | System time; no `/clock` | No |
| Legacy live simulation | Python, `--time-mode live` | `headless_live.launch.py`, `source_profile:=host_replay` | System time; no `/clock` | No |
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

`headless_live` defaults to `source_profile:=native`. ZED NET and MAVLink
start and reconnect independently. Camera RGB/depth/cloud and 2D segmentation
continue without telemetry; pose, map products and prediction require verified
camera time, fresh synchronized telemetry and the appropriate data inputs.
`source_profile:=host_replay` selects the existing LRST/LRFP simulator contract.

- Always-on nodes use `use_sim_time=false`. In `host_replay`, the simulator maps recorded IMAGE
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
- MAVROS TIMESYNC runs at 10 Hz with a 30-sample convergence window. The
  readiness monitor additionally requires 30 observations agreeing with the
  applied estimate, RTT below 20 ms, and an accepted observation within 2 s.
  Zero and negative offsets are valid. Reboots, clock jumps and expired sync
  invalidate fusion. Missing/stale telemetry suppresses map products, not 2D.
- Native camera time is explicit: `camera_time_mode:=unverified` (default)
  preserves source stamps for camera-only processing; `system` asserts that
  acquisition timestamps already use synchronized Unix time; `offset` adds
  the measured signed `camera_time_offset_ns`. The latter two require prior
  verification of the sender clock and its drift against Jetson. Packet arrival
  time or an epoch-looking timestamp is not evidence of synchronization.
  FCU TIMESYNC alone does not verify the camera clock.
- `camera_time_mode:=fcu_utc` explicitly asserts that camera IMAGE timestamps
  use the same UTC clock as the FCU's standard `SYSTEM_TIME.time_unix_usec`.
  Five consistent SYSTEM_TIME anchors relate UTC to boot time; converged
  TIMESYNC relates boot time to Jetson system time. The relay adds
  `timesync_offset - (fcu_utc - fcu_boot)` to each raw frame stamp. Anchors must
  be from the configured component, fresh within 2 s, and consistent with the
  TIMESYNC boot clock. This uses reported clock pairs and measured RTT, never
  network arrival as evidence of camera synchronization. Missing, stale or
  inconsistent anchors block fusion and retain camera/2D with the last mapping.
  A changed UTC relation flushes the NET receiver and both generations.
  The native simulator defines this shared virtual UTC from SVO IMAGE time;
  real sources must independently verify that their camera shares FCU UTC.
  Jetson requests SYSTEM_TIME at 1 Hz through standard
  `MAV_CMD_REQUEST_MESSAGE` so an FCU need not broadcast it by default. A rejected
  or missing reply is diagnosed and fusion remains blocked; no clock is guessed.
- Inputs expire after 2 s. The camera supervisor allows 15 s for SDK startup,
  requires a real raw frame before readiness, and restarts an established NET
  receiver after 5 s without frames, with a 2 s retry interval.
- Camera and fusion have separate generations. MAVLink loss resets fusion
  queues, TF caches, terrain, trackers and prediction while keeping camera and
  segmentation loaded. NET restarts or camera clock jumps reset both domains.
  Replay session changes restart only the NET receiver to flush old frames.
  In-flight old results and pre-boundary data are rejected; the ENU origin is retained.

### Future path and prediction

Native prediction uses `/lr/future_path/mission` (`nav_msgs/Path`). The Jetson
reads the FCU mission through MAVROS `/mavros/mission/pull` and
`/mavros/mission/waypoints`; MAVLink is bidirectional for time synchronization
and read requests. The backend never uploads, clears or starts a mission.
Successful complete downloads refresh every 10 s and expire after 30 s.
`MISSION_CURRENT` must be received from the configured target within 3 s.
Reconnect or FCU clock reset discards the cached mission and HOME.
Reported inactive/completed or suspended mission states stop prediction; a
changed mission ID forces a fresh download. FCUs omitting these MAVLink 2
extensions use the fresh sequence number and downloaded mission instead.
See the standard [MISSION_CURRENT fields](https://mavlink.io/en/messages/common.html#MISSION_CURRENT).

V1 accepts remaining `MAV_CMD_NAV_WAYPOINT` items in GLOBAL/GLOBAL_INT (MSL)
or GLOBAL_RELATIVE_ALT/GLOBAL_RELATIVE_ALT_INT frames. Relative altitude needs
HOME_POSITION; MAVROS ellipsoid HOME altitude is converted to MSL using
GeographicLib. Other commands/frames, missing HOME, invalid coordinates and
empty/completed missions block prediction with a diagnostic. The local route
runs from current pose through the active and subsequent waypoints using the
same ENU origin/extrinsics as pose, tangent yaw, 15 m radius and 0.2 m spacing.
It publishes up to 4 Hz with at most 128 path points; pose stamps identify the
current evaluation, not future arrival times. This is a waypoint polyline,
not a reconstruction of the FCU's steering controller. Missing route data clears
prediction and markers while valid pose/map processing continues.
Path/trajectory stamps retain the pose observation used for evaluation. Native
consumers require a route update within 1 s and a source stamp within the same
2 s bound as fusion, allowing camera/terrain processing latency without
repeated cache resets. The private replay baseline retains its 1 s source bound.

For `host_replay`, the following recorded-path baseline remains available.

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
The native profile uses the mission adapter above and does not require either private TUNNEL type.

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

The optional playback window needs Ubuntu's `python3-tk` package in the host
runtime. In a root terminal inside `zed-dev`, install it with
`apt-get install -y python3-tk`. The GUI also needs access to the desktop display
(`DISPLAY` and the X11 socket mount in the desktop container). Terminal and
unattended playback do not require Tk.

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

### Native deployment: Jetson receives the external sources

Provide measured rover physics (the schema is in
`prediction-core/config/rover.reference.yaml`), measured XYZ/RPY camera mounting,
a segmentation model and explicit network endpoints. Native prediction rejects
an omitted rover file or the bundled reference filename. File contents must
come from the real rover; software cannot certify the measurements.

Set `ROVER_CONFIG`, `BODY_TO_CAMERA` (a six-number list, metres/radians),
`ZED_STREAM_ADDRESS` and `MAVROS_FCU_URL` to the actual deployment values:

```bash
ros2 launch lr_display_rviz2 headless_live.launch.py \
  source_profile:=native camera_model:=zed2i \
  stream_address:="${ZED_STREAM_ADDRESS:?Set the NET source address}" stream_port:=30000 \
  mavros_fcu_url:="${MAVROS_FCU_URL:?Set the explicit MAVROS connection URL}" \
  target_system_id:=1 target_component_id:=1 \
  segmentation_model_path:="$PWD/models/best.pt" \
  mavlink_body_to_camera:="${BODY_TO_CAMERA:?Set measured XYZ/RPY extrinsics}" \
  rover_config:="${ROVER_CONFIG:?Set the measured rover YAML path}" \
  camera_time_mode:=unverified prediction_profile:=static
```

This starts camera/2D and reports why fusion is blocked. After independently
verifying camera timestamps, set `camera_time_mode:=system`, or use `offset`
and the measured `camera_time_offset_ns`. Do not enable fusion by estimating an
offset from UDP/DDS arrival latency. Native deployment requires heartbeat,
GPS/attitude topics supported by the MAVROS adapters, TIMESYNC responses and a
supported mission; the protocol names alone do not guarantee these inputs.
The current pose adapter uses `/mavros/global_position/raw/fix` and
`/mavros/imu/data`; standard GPS_RAW_INT, ATTITUDE and HIGHRES_IMU are exercised
in integration tests. Target IDs are explicit and configurable (1/1 defaults).

`gps_max_age_s` (0.75), `gps_max_gap_s` (1.5), `attitude_max_age_s` (0.25) and
`max_speed_mps` (5.0) are launch parameters. These limits are applied by the
pose estimator against each image's acquisition time. The supervisor uses a
separate 2 s stream watchdog so a brief per-frame telemetry gap does not reset
the entire map. Native prediction defaults on;
`prediction_enabled:=false` is available for camera/fusion diagnostics and does
not qualify as a complete deployment. The pose adapter alone publishes camera
map TF; always-on nodes use system time and never open a recording DB.

### One native backend for simulation and real sources

The native simulator is the deployment compatibility path. Start Jetson with
the same model, measured rover physics, extrinsics, target IDs and endpoints
that will be used with the real sources. For sources verified to share FCU UTC,
use the native command above with `camera_time_mode:=fcu_utc`. Keep that Jetson
launch running when starting, stopping or seeking the simulator. There is no
simulator profile, DB path, private path plugin or `/clock` on Jetson in this
mode. If the real sender does not meet the shared-UTC contract or does not
provide SYSTEM_TIME, this configuration will block fusion; use a separately
verified `system`/`offset` configuration instead and repeat compatibility tests.
Protocol names alone cannot establish clock equivalence.

After verifying that clock contract, the **same Jetson command** is used with
either source (the required environment variables are defined above):

```bash
ros2 launch lr_display_rviz2 headless_live.launch.py \
  source_profile:=native camera_time_mode:=fcu_utc camera_model:=zed2i \
  stream_address:="${ZED_STREAM_ADDRESS:?Set the NET source address}" stream_port:=30000 \
  mavros_fcu_url:="${MAVROS_FCU_URL:?Set the explicit MAVROS connection URL}" \
  target_system_id:=1 target_component_id:=1 \
  segmentation_model_path:="$PWD/models/best.pt" \
  mavlink_body_to_camera:="${BODY_TO_CAMERA:?Set measured XYZ/RPY extrinsics}" \
  rover_config:="${ROVER_CONFIG:?Set the measured rover YAML path}" \
  prediction_enabled:=true prediction_profile:=static
```

Source addresses, IDs, camera model and measured calibration must match the
actual installation. Within the same geographic map, both producers can use
the configured endpoints without a Jetson restart, profile switch or parameter
update. The ENU origin is fixed for the backend lifetime: start a fresh backend
map when changing GPS sites, such as switching from a recording made in another
country. The native launch command and processing implementation remain the same.

| Input contract | Native simulator and required real-source behavior |
|---|---|
| Camera | ZED SDK NET stereo stream, IMAGE timestamps in the explicitly verified shared FCU UTC |
| Clock | Boot-domain TIMESYNC replies; SYSTEM_TIME UTC/boot pair, broadcast or standard REQUEST_MESSAGE reply |
| GPS | GPS_RAW_INT with valid fix, global coordinates and MSL altitude; tested sensor time is boot microseconds |
| Attitude/acceleration | ATTITUDE boot milliseconds and HIGHRES_IMU boot microseconds with measured SI fields |
| Mission | Retryable standard mission download and fresh MISSION_CURRENT from configured IDs; V1 waypoint/frame restrictions above |
| HOME | HOME_POSITION MSL altitude when relative-altitude waypoints are used |

FCUs using Unix time in 64-bit sensor timestamp fields require verification
that MAVROS-produced sensor headers are already aligned to Jetson system time;
MAVROS treats Unix timestamps differently from boot timestamps. This simulator
does not establish compatibility for every permitted MAVLink dialect, time
convention or vendor message set. Rejecting unavailable/unaligned inputs is
part of the shared contract, not evidence of successful full deployment.

On the host, from the workspace root, run:

```bash
python3 tools/host_replay/replay.py --time-mode native --mission-from-recording --gui \
  --svo-path "$PWD/../svo/zed_20260710_092420_0001.svo2" \
  --mavlink-db-path "$PWD/mavlink/session_20260710_0924_mavlink/session_mavlink.db" \
  --stream-port 30000 \
  --mavlink-bind-address 10.42.0.1 --mavlink-bind-port 14550 \
  --mavlink-destination 10.42.0.242 --mavlink-destination-port 14540 \
  --system-id 1 --component-id 1 --play-from-frame 4500
```

`--gui` can be replaced with `--interactive` or omitted. Native mode emits
HEARTBEAT, SYSTEM_TIME, recorded GPS_RAW_INT/ATTITUDE/HIGHRES_IMU,
MISSION_CURRENT, TIMESYNC replies, mission download replies and requested
HOME_POSITION/AUTOPILOT_VERSION. It never sends private TUNNEL packets.
The FCU runs in a separate process so camera SDK startup, disconnect or seek
cannot stall heartbeat, TIMESYNC or mission requests. Telemetry advances on
its own monotonic schedule at the recorded rates; overdue image frames may be
dropped to retain the shared clock instead of accumulating timing drift.

`--mission-from-recording` explicitly generates a geometric **test mission**
from GPS stations at least 5 m apart, with progress based on recording time.
It does not recover the actual FCU mission or steering controller. To use a
specified mission, replace it with `--mission-path` pointing to JSON:

```json
{"version": 1, "home_msl": [34.2948, -118.3935, 509.0], "waypoints": [
  {"frame": 0, "command": 16, "lat": 34.29485, "lon": -118.3935, "alt": 509.0},
  {"frame": 3, "command": 16, "lat": 34.295, "lon": -118.3935, "alt": 0.0}
]}
```

Frames 0/5 use MSL altitude; 3/6 use HOME-relative altitude. Optional
`param1`..`param4` default to zero. File missions advance within 2 m of the
recorded GPS position. An omitted mission is empty; `home_msl: null` models
missing HOME. Downloads are read-only and retryable in float or integer form.
The simulator rejects mission uploads, clearing and vehicle control commands.

Native seek/resume keeps the FCU process/socket but models an FCU reboot and
announces the new UTC/boot relation through standard messages. Jetson clears
fusion and mission state and restarts only its NET receiver when required;
models and the backend remain loaded. Pause stops NET and sensor playback but
keeps FCU heartbeat/time services with mission mode suspended. The legacy
`--time-mode live --future-path` contract remains a separate regression baseline.

For fault tests, add host-only `--control-file /tmp/native-control.json` and
`--native-status-file /tmp/native-status.json`. Atomically replace the control
JSON with booleans `camera_enabled`, `telemetry_enabled`, `timesync_enabled`, `system_time_enabled`,
`mission_enabled`, `progress_enabled`, `home_enabled`, `mission_empty`; omitted
enable flags default to true. Increment integer `reboot` to reset the FCU boot
clock while NET continues. The status JSON counts actual emitted message types
and private packets. These files are never sent to Jetson.

Evaluate native operation with `verify_live.py --mode deployment --source-profile
native`; check the same backend runtime ID/configuration before and after
MAVLink/NET loss, recovery, reboot and forward/backward seek. Check standard
mission downloads, empty mission, missing HOME and stale progress separately.
MAVLink loss must retain camera/2D and its generation; NET loss must retain the
MAVROS process. Successful interface tests establish software compatibility,
not physical timing accuracy, real mission fidelity or measured vehicle physics.
The 30-minute real-source acceptance remains required before deployment approval.

### Replay baseline: start the Jetson backend once

In a Jetson ROS terminal:

```bash
ros2 launch lr_display_rviz2 headless_live.launch.py \
  source_profile:=host_replay camera_model:=zed2i stream_address:=10.42.0.1 stream_port:=30000 \
  mavros_fcu_url:='udp://0.0.0.0:14540@10.42.0.1:14550' \
  segmentation_model_path:="$PWD/models/best.pt" \
  prediction_enabled:=true prediction_profile:=static
```

The backend can start before the source and remain running between sessions.
It accepts no recording DB or simulated-clock arguments. Prediction defaults on
and therefore requires a segmentation model. To run without
prediction, set `prediction_enabled:=false` here and in RViz and omit the
host's `--future-path`. Omit `segmentation_model_path` to disable segmentation
as well. No operating-system boot service is installed.

Optional backend parameters include `camera_name`, `map_frame`,
`mavlink_body_to_camera:='[x,y,z,roll,pitch,yaw]'`, `mavros_audit_file` and
`rover_config:=/absolute/path/to/measured-rover.yaml`. Extrinsics use metres and
radians. The bundled rover config contains reference values for evaluation.

### 2. Host: start the independent simulator

Use a Python environment with ZED SDK/pyzed and pymavlink; ROS setup is not
required. Run from the workspace root (`/zed/lr-ros2` inside `zed-dev`). The
example uses the July 10 recording in the sibling `svo/` directory and its
matching MAVLink DB. When selecting another recording, change both paths:

```bash
python3 tools/host_replay/replay.py --time-mode live --future-path --gui \
  --svo-path "$PWD/../svo/zed_20260710_092420_0001.svo2" \
  --mavlink-db-path "$PWD/mavlink/session_20260710_0924_mavlink/session_mavlink.db" \
  --stream-port 30000 \
  --mavlink-bind-address 10.42.0.1 --mavlink-bind-port 14550 \
  --mavlink-destination 10.42.0.242 --mavlink-destination-port 14540 \
  --play-from-frame 4500
```

The entire `tools/host_replay/` directory can be copied elsewhere and run as
`python3 replay.py` with the same arguments. Playback is sequential 1x with
H.264, 20000 Kbit/s and GOP 1. `--play-from-frame` defaults to 0. Without
graphical or terminal controls, EOF, Ctrl+C or SIGTERM closes the source and
sockets; playback does not loop.
Live mode rejects clock endpoints. Stop/start the sender to begin a new session.

The command above opens the **Host SVO Playback** window with `--gui`. Drag the
timeline and release to seek once, use the **Pause/Play**, **−10 s**, **+10 s** or
**Replay** buttons, or enter a frame number and click **Go**. The window shows
the current time, duration, frame and playback state. Space toggles playback
when the frame entry is not focused. Seeking while paused selects a position;
press **Play** to transmit from it. SDK work runs on a single worker so the
window remains responsive while opening or reconnecting. Video and perception
results are displayed by RViz.

For terminal playback controls, replace `--gui` with `--interactive`. Type a
command and press Enter in the sender terminal:

| Command | Action |
|---|---|
| `seek 9000` | Jump to the absolute, zero-based SVO frame |
| `skip 10`, `skip -10` | Jump forward/backward by seconds at the recording's FPS |
| `pause` | Stop the source; retain the next frame for resume |
| `resume` | Resume from the selected frame |
| `replay` | Start again from frame 0 |
| `status`, `help`, `quit` | Show position, show commands, or exit |

Each seek while playing, resume or replay closes and reopens the NET source
and MAVLink socket, creates a new live session, resets the causal telemetry
cursor and discards pending future-path fragments. The Jetson supervisor
restarts its NET receiver when the source session changes, clearing buffered
images before resuming processing. Keep the backend running; reconnecting is
not instantaneous. The ENU origin and loaded perception models are retained.

Pause releases the source and sockets rather than repeatedly streaming a
frozen frame. Jetson outputs expire under the normal freshness rules. Seeking
while paused selects a frame without transmitting it; `resume` starts a new
session there. At EOF, the GUI stays open for another seek or replay; terminal
mode waits for `seek`/`resume`, `replay` or `quit`. Closing the window, Ctrl+C or
SIGTERM exits and releases resources; closed standard input also exits terminal
mode. `--gui` and `--interactive` are mutually exclusive and support
`--time-mode live` or `native` with `headless_live.launch.py`. Recorded-time replay baselines
keep sequential playback and their existing clock behavior. The controls require no ROS imports
and remain usable after copying `tools/host_replay/` outside the workspace.

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
  svo_path:="$PWD/../svo/zed_20260710_092420_0001.svo2" stream_port:=30000 \
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
  --svo-path "$PWD/../svo/zed_20260710_092420_0001.svo2" \
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
| `/lr/future_path/mission` | Native FCU mission route |
| `/lr/future_path/mavlink` | Received path in host_replay mode |
| `/lr/future_path/ground_truth` | DB reference path in replay baselines |
| `/segmentation/overlay`, `/segmentation/instance_mask` | Segmentation products |
| `/segmentation/boxes_3d`, `/segmentation/tracked_footprints` | 3D objects and tracked footprints |
| `/terrain_geometry/grid_map`, `/terrain_geometry/markers` | Terrain products |
| `/trajectory`, `/geometry`, `/tracked_objects` | Canonical prediction inputs |
| `/rover/state` | Dynamic-profile rover state |
| `/predict_output` | Raw prediction evidence |
| `/lr/path_prediction/reference_path`, `/lr/path_prediction/markers` | Matched path and warning visualization |
| `/lr/source/generation`, `/lr/source/camera_generation` | Fusion and camera readiness/generations |
| `/diagnostics`, `/lr/future_path/diagnostics` | Input, clock, fusion and route diagnostics |

The bridge consumes `Detection3DArray` from `/segmentation/boxes_3d` by default.
Objects are treated as static within a cycle when velocity is unavailable.
Missing terrain remains unknown; visualization preserves the full source path.

Runtime configuration is in each package's `config/` directory. Important files
are `lr_mavros_bridge/config/live.yaml`, `lr_prediction_bridge/config/bridge.yaml`,
`prediction_ros/config/prediction.yaml`, `prediction_core/config/rover.reference.yaml`
and `lr_path_prediction/config/path_prediction.yaml`. Change vehicle physics
through the rover config; RViz thresholds do not change the physical model.

## Verification

Observe a native deployment with prediction (`--duration 1800` for the
30-minute hardware acceptance run):

```bash
python3 src/lr-ros2/lr-mavros-bridge/tools/verify_live.py \
  --mode deployment --source-profile native --duration 25 --timeout 60 --output /tmp/live-path.json
ros2 run lr_mavros_bridge verify_tf_owner
ros2 topic info /clock --verbose
ros2 topic echo /lr/source/generation --once
```

Use `--source-profile host_replay --with-prediction` for the replay baseline.
Use `--mode camera --expect-degraded` to verify camera/2D continuity and absent
map outputs while telemetry is disconnected or camera time is unverified;
add `--with-prediction` when those processes are enabled. `--camera-name` and
`--map-frame` support renamed deployments. The JSON report distinguishes
camera readiness from fusion readiness and full native prediction operation.
Use `--mode fusion --with-prediction --expect-no-prediction` to verify that
pose/map continue while an empty, stale, unsupported or incomplete mission
blocks prediction.

`/diagnostics` reports camera, telemetry, clock and fusion readiness, input
rates/ages, target IDs, RTT and clock offsets. Mission download/progress age,
state and route rate are on `/lr/future_path/diagnostics`; prediction input age,
output rate and missing-input reasons are on `/prediction/diagnostics`.

Hardware acceptance additionally requires both real sources, measured vehicle
calibration, clock verification and independent disconnect/reconnect trials.
Synthetic input tests and recorded NET streaming do not certify those facts.

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
