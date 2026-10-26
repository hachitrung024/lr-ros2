# NET/MAVROS runtime migration — 2026-10-26

The LR runtime now has one distributed architecture. The host opens SVO2/live
stereo and sends ZED NET to Jetson. SVO additionally replays DB GPS, attitude
and IMU over MAVLink UDP. Jetson always uses MAVROS received telemetry for
SVO pose/TF, and reads its DB only for the fixed ENU origin and reference path.
The host runs the independent RViz UI. Commands are in [the run guide](mavros-replay.md).
Raw observations are in [net-mavros-migration.json](net-mavros-migration.json).

## Removed runtime paths

- Combined camera/backend/RViz LR launch and local-SVO/camera backend input.
- `mavlink_backend` / `mavlink` switches and the direct DB pose/TF executable.
- SVO VIO preprocessing launch, cache recorder/reader, rosbag helpers and their
  dependencies/tests. Shared DB, quaternion and path processing remain.
- Obsolete installed launch files and console scripts from previous builds.

The installed LR display package contains only `headless_zed_cam.launch.py`
and `rviz_zed_cam.launch.py`. `lr_future_path` exposes only
`mavlink_ground_truth_node`. Actual CLI checks confirmed that the old combined
launch, DB pose node and VIO recorder are unavailable. Passing a removed
backend argument fails before any processing nodes start.

The Jetson backend requires `camera_model`, `stream_address`, `stream_port`
and `stream_svo_mode`. SVO additionally requires an explicit `mavros_fcu_url`;
there is no alternate localization backend. `future_path:=auto` enables SVO
reference paths; prediction auto-starts with path, segmentation and terrain.
Live NET keeps ZED tracking and wall time, with terrain's path corridor disabled
so perception does not wait for a replay-only future path.

## Source migration and installation

Source was previewed and synchronized using the AGENTS.md workflow with
`--ignore-missing-args` to skip deleted tracked host paths. No rsync deletion
was used. On Jetson, the 12 retired source files and the old build/install
folders for `lr_future_path` and `lr_display_rviz2` were moved into
`.legacy-archive/20261026-net-mavros/`. Host old build/install folders were
preserved in the equivalent archive. The archive has `COLCON_IGNORE` and is
excluded from source control. Fresh package installs expose no old entry points.
Recordings, models and telemetry databases were not migrated or deleted.

Active README, Docker and three-terminal documentation now describe only the
NET/MAVROS architecture. The terrain benchmark requires a host NET source and
explicit network endpoints. Older dated measurements remain historical evidence.

## Verification

Both machines built successfully: 14 packages on the host and 12 on Jetson.
Final test-result totals were **187 on host** and **167 on Jetson**, each with
zero errors, zero failures and three skips. The focused pre-build run passed
68 tests. Tests cover SVO/live NET selection, mandatory endpoints, refusal of
old CLI paths, clock/TF ownership, reference-only DB publication, clock rewind,
telemetry encoding and causal extrapolation. XML schema validation used a
local catalog for the same ROS schema; no validation was disabled.

The real SVO test started at frame 4500 and used the new commands with no
`mavlink` or backend selector. A 30-second probe confirmed:

- One `/clock` publisher and 224 shared RGB CameraInfo/cloud timestamps.
- Pose 14.39 Hz; reference path 14.72 Hz; point cloud 7.79 Hz;
  segmentation overlay 1.63 Hz; terrain 4.36 Hz; prediction 0.30 Hz.
- MAVROS received 59 GPS, 148 attitude and 415 raw IMU messages in its
  30-second window, with zero timestamp mismatches against the DB.
- A DDS MessageInfo probe observed 117 camera transforms from exactly one
  publisher. Ground truth advertised neither pose nor TF.
- Independent reconstruction of 1629 pose audit records found zero future
  telemetry violations. Maximum difference using the same received samples
  was `1.78e-15 m` and `2.50e-16` in quaternion coefficients.

Offline ground-truth position differences were median 0.033 m, P95 0.192 m
and maximum 0.383 m. Those describe bounded causal extrapolation against an
interpolated reference; they are not absolute localization accuracy.

Ethernet traffic was about 620 Mbit/s received and 23 Mbit/s transmitted on
the host during the probe. No RViz GUI was open in this measurement, so this
is not a direct comparison with earlier runs that included RViz subscriptions.
The UI launch, Qt plugin and retained display preset passed their tests.

## Remaining acceptance limits

The physical live camera was not exercised; live NET launch/clock/terrain
configuration was tested. SDK `MOTION SENSORS REQUIRED` warnings and initial
TF readiness warnings remain. Jetson system clock skew was not changed.
SDK 5.4.1 sequential NET remains the accepted replay mode; pause/seek is still
experimental. Prediction publication was checked, not prediction quality.

All hardware test launches were stopped after verification; containers remain
running for the documented three-terminal commands.
