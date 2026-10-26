# LR NET backend and host RViz

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

A segmentation model path enables segmentation unless explicitly disabled.
`future_path:=auto` enables DB ground truth in SVO mode. Prediction auto-starts
when reference path, segmentation and terrain are enabled. Static and dynamic
prediction profiles use the same shared processing launches.

The host retains the existing stereo RViz layers. The SVO panel defaults to
`/zed_source/zed_node`; topic subscriptions stay under the Jetson camera
namespace. Use `svo_mode:=false` to hide pause/seek during sequential SDK 5.4.1
NET playback. `prediction_enabled:=false` selects the raw future-path display;
`segmentation_enabled:=false` shows the received RGB image.
