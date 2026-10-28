# Distributed host source, Jetson backend and host RViz

For the new persistent wall-time backend, use the [always-on live guide](always-on-live.md):
`headless_live.launch.py` runs without Jetson DB, ground truth, prediction or
`/clock`; pose owns map TF and the ENU origin comes from GPS. The remaining
commands in this document describe the preserved baseline paths.

Follow the [three-terminal run guide](mavros-replay.md) for the complete SVO
and live commands, dependencies and explicit IP/port configuration.

- Host source: `lr_mavlink_replay host_replay.launch.py` for SVO2 + telemetry;
  `lr_zed_stream stream_sender.launch.py` for a live stereo camera.
- Jetson processing: `lr_display_rviz2 headless_zed_cam.launch.py`, with required
  `camera_model`, `stream_address`, `stream_port`, `stream_svo_mode`.
- Host UI: `lr_display_rviz2 rviz_zed_cam.launch.py`. Image/result subscriptions
  remain under `/zed`; SVO control targets `/zed_source/zed_node`.

The optional [standalone host replay](../tools/host_replay/README.md) requires no
ROS on the source machine. It sends an additional UDP clock channel. Select
`replay_clock_source:=udp` and explicit `clock_udp_bind_address` /
`clock_udp_bind_port` on Jetson; `/replay_clock_bridge` then owns `/clock`.
The default `replay_clock_source:=ros` retains the existing three-terminal run.

Both containers use host networking, ROS 2 Humble, SDK 5.4.1,
`ROS_DOMAIN_ID=42`, `ROS_LOCALHOST_ONLY=0`. Keep the host source playing while
Jetson opens NET; SDK 5.4.1 requires advancing frames for stream establishment.

## Source sync and build

Preview/sync only the files specified in [AGENTS.md](../AGENTS.md). Stop old
launches before building. Host builds up to `lr_mavlink_replay lr_display_rviz2`;
Jetson builds up to `lr_display_rviz2`. Source each machine's own install.
SVO recordings stay on the host. Models and the reference DB stay on Jetson;
the host also needs that DB to reconstruct telemetry.

## Discovery and network checks

```bash
ros2 node list
ros2 topic list
ros2 topic info /clock --verbose
ros2 topic echo /mavros/state --once
```

For SVO, exactly one clock publisher must be `/zed_source/zed_node` with the ROS
source, or `/replay_clock_bridge` with standalone replay. Jetson LR
and host RViz consume simulated time; the ZED NET receiver grabs on wall time
and retains SDK IMAGE timestamps. Live NET uses wall time and ZED tracking.

If discovery fails, verify both containers are running, their domain IDs,
host networking, DDS UDP/multicast, and firewall rules between the machines.
The example network uses host `10.42.0.1` and Jetson `10.42.0.242`. Open the
chosen even ZED NET UDP port (example `30000`) and the explicitly configured
MAVLink endpoints (example host `14550`, Jetson `14540`). DDS visualization
traffic flows back to the host independently of ZED NET and MAVLink.

## Runtime and UI checks

The [verification tools](mavros-replay.md#verification) check timestamp/unit
restoration, received telemetry, full pipeline publication and the actual DDS
publisher of `map → zed_camera_link`. Other nodes can advertise `/tf` without
publishing that edge. In SVO mode the MAVROS pose adapter alone owns the edge;
in live mode ZED tracking owns it.

The existing RViz preset retains segmentation, boxes, terrain, ground-truth
future path and prediction layers. Use `segmentation_enabled:=false` to show
RGB without a segmentation model. Use `prediction_enabled:=false` when
prediction is disabled so the path display subscribes to raw ground truth.

## Sequential replay limits

SDK 5.4.1 sequential NET playback is the accepted mode. Keep `svo_mode:=false`
in RViz to hide unsupported pause/seek controls. The source's experimental
controls remain disabled by default. A replay process restart may rewind
recorded time; LR buffers reset, while the wrapper can log `TF_OLD_DATA`.
See [transport acceptance](mavros-replay-validation.md) and the
[migration report](net-mavros-migration.md).
