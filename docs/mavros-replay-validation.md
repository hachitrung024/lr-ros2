# MAVROS telemetry replay acceptance — 2026-10-23

Implemented and exercised on the current host/Jetson pair with ROS 2 Humble,
ZED SDK 5.4.1, MAVROS 2.16.0, pymavlink 2.4.50 and GeographicLib egm96-5.
The source was `zed_20260710_092420_0001.svo2`, starting at frame 4500,
with its matching `session_20260710_0924_mavlink/session_mavlink.db`.
Both containers used host networking and ROS domain 42. The existing RViz
preset and default static prediction profile were enabled.

Raw observations are in [mavros-replay-validation.json](mavros-replay-validation.json).
Reproduction commands are in [mavros-replay.md](mavros-replay.md).

## Transport and pose checks

- Received GPS, attitude and raw IMU through MAVLink UDP into MAVROS.
  Across a 30-second sample: 60 GPS, 139 attitude and 423 IMU messages,
  with **zero timestamp mismatches** against the DB (attitude rounded down
  to its MAVLink millisecond representation).
- Maximum acceleration conversion error was `4.93e-7 m/s²`, quaternion
  coefficient error `2.92e-16`, and reconstructed DB altitude error
  `1.14e-13 m`. These are transport/conversion checks, not sensor accuracy.
- The receiver produced 412 pose messages while the host observed 450 clock
  frames. There were 410 exact pose/clock timestamp matches within the
  measurement window; the boundary samples account for the different counts.
  Missing pose frames are expected when causal GPS/attitude age limits fail.
- An independent calculation over **1381 pose audit records** found zero
  future-sample violations. Maximum difference from the same received samples
  was `1.78e-15 m` and `2.50e-16` in quaternion coefficients.
- Against offline ground-truth interpolation, position differences were
  median **0.029 m**, P95 **0.184 m**, maximum **0.383 m**. Orientation differences
  were median **0.149°**, P95 **3.105°**, maximum **7.985°**. These quantify the
  chosen causal extrapolation, not absolute localization error.
- A C++ DDS MessageInfo probe observed 106 camera transforms over eight seconds,
  all from **one publisher GID**. Other nodes advertise `/tf` but do not publish
  the `map → zed_camera_link` edge. Exactly one `/clock` publisher was present.

## Full pipeline measurement

| Output | Host received rate | Median source-clock arrival → output arrival |
| --- | ---: | ---: |
| Pose | 14.23 Hz | 112 ms |
| Ground-truth future path | 12.90 Hz | 57 ms |
| RGB CameraInfo | 6.03 Hz | 218 ms |
| Point cloud | 3.80 Hz | 396 ms |
| Segmentation overlay | 0.87 Hz | 1433 ms |
| Terrain grid | 2.43 Hz | — |
| Prediction | 0.13 Hz | Four messages; insufficient for a latency conclusion |

The 30-second full-pipeline check passed with 84 matching RGB/cloud stamps.
Host Ethernet traffic was approximately **848 Mbit/s received** and
**23 Mbit/s transmitted**, including RViz and verification subscriptions.
The pose delay includes the configured 100 ms wait for arriving telemetry.

## Loss and restart

Suspending the host telemetry process for four seconds while the SVO clock
continued advancing caused the adapter to report stale data and stop pose
publication. The settled pose count stayed at 34 through the final two seconds
of the interruption, then increased to 92 after resuming. Readiness recovered
without restarting the Jetson backend.

Stopping and restarting the complete host source at frame 4500 rewound the
clock. The same Jetson backend launch process (PID 2310) remained running.
The adapter cleared its history; the full-pipeline probe passed again, with
69 matching RGB/cloud stamps and all required output topics including
prediction. A second telemetry check found zero timestamp mismatches.

## Automated validation and limits

Both machines built successfully. The host package test run reported
**143 tests, zero errors, zero failures, three skipped**. The Jetson test result
reported **123 tests, zero errors, zero failures, three skipped** after testing
`lr_future_path`, `lr_mavros_bridge` and `lr_display_rviz2`. Its XML linter first
timed out fetching the ROS package schema because Jetson had no direct Internet
access. Repeating the display package tests with an XML catalog pointing to a
temporary copy of that same schema downloaded on the host passed; no validation
was disabled. Tests cover wire
encoding, units, timestamp restoration, past-only startup, rewind, duplicates,
out-of-order samples, missing fixes/rates, stale data, reference-only publisher
ownership, launch compatibility and the existing local DB behavior.

The following limits remain explicit:

- Telemetry reconstructs decoded recorder fields. It is not a raw packet replay
  or an autopilot simulator, and does not implement actuator control or EKF
  localization. Magnetometer validity is not established by this DB.
- GPS at about 2 Hz and gaps in attitude can leave frames without a valid pose.
  Prediction publication was verified; prediction quality was not revalidated.
- The SDK still reports `MOTION SENSORS REQUIRED` and occasional
  `CORRUPTED FRAME` warnings in NET mode. The wall-clock ZED wrapper also logs
  `TF_OLD_DATA` after rewinding the source; LR pose/processing outputs recovered.
- Latency uses one host's monotonic clock, from `/clock` arrival to output
  arrival. It is not exposure-to-display latency. The existing DDS display
  traffic remains close to the 1 Gbit/s link capacity.
- Jetson system time remains about nine days behind the host and produces build
  clock-skew warnings. Its clock and SDK were not changed.
- Physical FCU/live camera and NET pause/seek remain outside this acceptance.

The hardware test launches were stopped after verification.
