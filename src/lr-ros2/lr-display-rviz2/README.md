# LR NET backend and host RViz

## Persistent live backend

Use `headless_live.launch.py` for a continuously running NET + MAVLink receiver
using wall time, first-GPS origin, no Jetson DB and no prediction. The original
launches below remain replay baselines. See the
[live deployment guide](../../../docs/always-on-live.md).

This package provides two separate entry points:

- `headless_zed_cam.launch.py`: Jetson ZED NET receiver and LR processing.
  Required arguments: `camera_model`, `stream_address`, `stream_port`,
  `stream_svo_mode`. SVO mode additionally requires `mavros_fcu_url` and a
  matching reference DB (explicit `mavlink_db_path` or `mavlink_dir`).
- `rviz_zed_cam.launch.py`: host UI only. It never starts a camera, MAVROS,
  segmentation, terrain or prediction process.

SVO NET always uses MAVROS telemetry for pose/TF. The DB process only provides
an ENU origin and reference future path. Live NET uses ZED tracking and wall
time. The backend cannot open local camera devices or SVO files and never
starts RViz. Follow the [three-terminal guide](../../../docs/mavros-replay.md).

The default `replay_clock_source:=ros` consumes the legacy host source's clock.
For standalone host replay, set `replay_clock_source:=udp` and provide explicit
`clock_udp_bind_address` and `clock_udp_bind_port`. The required Jetson
`replay_clock_bridge` process becomes the sole `/clock` publisher; its exit
shuts down the backend. UDP replay clock is rejected in live-camera mode.

A segmentation model path enables segmentation unless explicitly disabled.
`future_path:=auto` enables DB ground truth in SVO mode. Prediction auto-starts
when reference path, segmentation and terrain are enabled. Static and dynamic
prediction profiles use the same shared processing launches.

The host retains the existing stereo RViz layers. The SVO panel defaults to
`/zed_source/zed_node`; topic subscriptions stay under the Jetson camera
namespace. Use `svo_mode:=false` to hide pause/seek during sequential SDK 5.4.1
NET playback. `prediction_enabled:=false` selects the raw future-path display;
`segmentation_enabled:=false` shows the received RGB image.
