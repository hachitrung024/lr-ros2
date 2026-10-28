# Standalone SVO + MAVLink replay

For an always-on Jetson that uses wall time without a DB or `/clock`, use
`--time-mode live` and the [live backend guide](../../docs/always-on-live.md).
The commands below document the compatible `--time-mode replay` default.

This directory runs independently of ROS and the ROS workspace. Copy the whole
directory to another machine if needed. No colcon build or ROS setup is used.
The host needs ZED SDK **5.4.1**, its matching `pyzed.sl` Python binding, an
SDK-compatible NVIDIA driver/GPU, and Python 3.10 or newer.

```bash
python3 -m pip install -r requirements.txt
python3 replay.py --help
```

`pyzed` comes from the ZED SDK installation, not this requirements file. The
existing `lr_zed_stream` and `lr_mavlink_replay` ROS packages remain unchanged
as the comparison baseline. This tool carries its own replay core and never
imports those packages or other files from the repository.

## Host source

The example below runs from the repository root; from a copied directory use
`python3 replay.py` with the same arguments. All network endpoints are explicit.
The clock port `30010` is an example and must match the Jetson bind port.

```bash
python3 tools/host_replay/replay.py \
  --svo-path /zed/svo/zed_20260710_092420_0001.svo2 \
  --mavlink-db-path /zed/lr-ros2/mavlink/session_20260710_0924_mavlink/session_mavlink.db \
  --stream-port 30000 \
  --mavlink-bind-address 10.42.0.1 --mavlink-bind-port 14550 \
  --mavlink-destination 10.42.0.242 --mavlink-destination-port 14540 \
  --clock-destination 10.42.0.242 --clock-destination-port 30010 \
  --play-from-frame 4500
```

Keep the source playing while Jetson establishes its NET connection. NET uses
H.264, 20000 Kbit/s, GOP 1, depth disabled and the recording's native frame rate.
One monotonic scheduler sends each frame's exact IMAGE timestamp over UDP and
then releases MAVLink records at or before that timestamp. GPS/IMU retain
microsecond timestamps; MAVLink ATTITUDE retains its millisecond resolution.
Heartbeat and discovery replies use the same UDP endpoint as the baseline.

The explicit DB is checked against the first selected frame with the same
60-second coverage tolerance as the baseline. The DB is read-only. Startup
and rewind cursor logic retain at most two recent past records per sensor.
Playback starts at frame 0 unless `--play-from-frame` is given. This version has
no pause, seek, rate change or auto-loop. EOF exits normally; Ctrl+C/SIGTERM
close the camera and sockets (exit 130). Setup/runtime errors exit nonzero.

## Jetson backend

Inside Jetson's ROS container, build/source the existing workspace normally:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to lr_display_rviz2
source install/setup.bash
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0
ros2 launch lr_display_rviz2 headless_zed_cam.launch.py \
  camera_model:=zed2i stream_address:=10.42.0.1 stream_port:=30000 \
  stream_svo_mode:=true replay_clock_source:=udp \
  clock_udp_bind_address:=0.0.0.0 clock_udp_bind_port:=30010 \
  segmentation_model_path:=/zed/lr-ros2/models/best.pt future_path:=true \
  mavros_fcu_url:='udp://0.0.0.0:14540@10.42.0.1:14550' \
  mavlink_db_path:=/zed/lr-ros2/mavlink/session_20260710_0924_mavlink/session_mavlink.db
```

The only `/clock` publisher is now **`/replay_clock_bridge` on Jetson**. The ZED
NET receiver still grabs on wall time and preserves IMAGE timestamps; the LR
pipeline and RViz still use simulated time. The bridge exits on another clock
publisher and its exit shuts down the backend launch. Do not run the ROS source
and standalone source at the same time. The legacy launch retains
`replay_clock_source:=ros` by default and starts no UDP bridge.

Open the chosen clock UDP port in addition to ZED NET/MAVLink ports. Host RViz
uses its existing launch with `use_sim_time:=true svo_mode:=false`. The
standalone source does not provide ROS SVO status or experimental controls.
Normal `verify_net_replay.py` observation works; `--exercise` requires the
experimental legacy ROS source and is not supported here.

## Clock protocol

Each frame sends one ASCII JSON UDP datagram (version 1):

```json
{"version":1,"session_id":"fa0f2aef-a206-4e68-a48f-5ee3bdb95906","sequence":0,"stamp_ns":1783700660123456789}
```

`session_id` is a new canonical UUID for each process start. `sequence` starts
at zero and increases once per frame. `stamp_ns` is an integer, not a float or
wall-clock replacement. The receiver can join at any sequence. It rejects
malformed/oversized packets, duplicate/backward sequences and non-increasing
timestamps within a session. A new session can rewind time; retired sessions
remain rejected, including delayed datagrams arriving after a restart.
Loss skips that clock update; source stop/EOF freezes the last ROS time. There
is no extrapolation or wall-time fallback. UDP arrival timing is not a guarantee
of image/telemetry arrival ordering; their recording timestamps remain the
synchronization reference and the pose adapter retains its existing wait.

## Tests

Install pytest in the same Python environment, then run:

```bash
python3 -m pytest -q test
```

The tests use a fake SDK and real loopback UDP. They verify timestamp/unit
encoding, causal telemetry, EOF/start frame/cleanup and a copied directory
whose process rejects all ROS/workspace imports. ROS bridge and backend launch
tests run separately through colcon. Runtime transport results are recorded in
the workspace documentation after hardware acceptance.
