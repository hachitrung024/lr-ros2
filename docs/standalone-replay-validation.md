# Standalone host replay validation — 2026-10-28

The standalone tool was copied to `/tmp/lr-host-replay-independent` inside the
host ZED SDK environment and launched from `/tmp`. ROS/Python workspace setup
variables were removed from its environment. It used the real
`zed_20260710_092420_0001.svo2` and matching MAVLink DB, starting at frame 4500.
The source sent H.264 NET at native 15 FPS, 20000 Kbit/s, GOP 1, with depth
disabled. Jetson used the existing model, processing parameters and age limits.
No RViz GUI was opened for these measurements.

## Automated and baseline checks

- All 20 standalone tests passed, including a copied directory with ROS/workspace
  imports explicitly denied, causal sensor batches, integer nanosecond clock
  transport, EOF/start frame and cleanup after interruption/runtime failure.
- Host tests for both legacy source packages, MAVROS bridge and display passed.
  Jetson bridge/display tests passed too. `colcon test-result` summaries were
  212 on host and 192 on Jetson, each with zero errors/failures and three skips;
  these summaries include unchanged existing package results.
- Both machines built their documented targets successfully. Jetson reported
  existing clock-skew warnings; system clocks and SDK versions were not changed.
- Using the real DB, standalone and legacy cursor/encoding logic produced
  identical MAVLink packets for 48,809 records across startup, forward progress
  and rewind. The original two source packages have no source changes.

## Real NET and telemetry

The final standalone probe and the probe after restarting only its source both
passed the existing `verify_net_replay.py --require-prediction` check. Each
25-second window observed 191 exact shared RGB CameraInfo/cloud timestamps and
exact timestamp matches with received `/clock` values. The sole clock publisher
was `/replay_clock_bridge` on Jetson.

| Output | Standalone | After source restart | Legacy ROS source |
|---|---:|---:|---:|
| RGB CameraInfo | 7.87 Hz | 8.20 Hz | 8.15 Hz |
| Point cloud | 7.79 Hz | 7.96 Hz | 7.95 Hz |
| Pose | 13.63 Hz | 13.60 Hz | 13.60 Hz |
| Reference path | 15.03 Hz | 15.04 Hz | 15.05 Hz |
| Terrain | 4.20 Hz | 4.48 Hz | 4.20 Hz |
| Segmentation overlay | 1.72 Hz | 1.64 Hz | 1.80 Hz |
| Prediction | 0.08 Hz | 0.16 Hz | 0.15 Hz |

The legacy measurement lasted 20 seconds and observed 150 shared RGB/cloud
timestamps. These short sequential runs establish publication and transport;
they are not a controlled latency, throughput or prediction-quality comparison.

The standalone telemetry probe received 276 IMU, 40 GPS and 157 attitude samples
over 20 seconds, with zero timestamp mismatches. Maximum conversion differences
were `5.02e-7 m/s²` acceleration, `1.42e-14` degrees GPS, `5.69e-14 m` geoid-corrected
altitude and `3.79e-16` quaternion coefficient norm. A post-restart probe also
passed with zero timestamp mismatches and 202 pose/clock matches.

An independent reconstruction of 3,516 pose audit entries found no future-sample
violations. Maximum reconstruction differences were `1.78e-15 m` position and
`2.55e-16` quaternion coefficients. A DDS TF probe observed 109 camera transforms
from exactly one publisher. These checks do not establish absolute pose accuracy.

## Restart, delayed packets and conflicting publishers

Stopping the standalone source froze ROS time. Restarting it created a new
session UUID and rewound `/clock` to the selected SVO frame without restarting
the Jetson backend. Image, perception, pose and prediction publication recovered.
An actual UDP datagram from the retired session, with a higher sequence and old
future timestamp, was ignored while the new session continued advancing.

Creating another ROS `/clock` publisher caused the bridge to exit nonzero and
the required-process launch handler shut down the whole backend. The injected
publisher did not publish clock values during this check.

## Limits and artifacts

Two initial probes timed out waiting for prediction while diagnostics reported
stale tracked objects. The final probes above passed after a fresh backend run,
without changing model, age thresholds or processing settings. Prediction can
remain unavailable when perception evidence misses those existing deadlines;
the short publication checks are not a prediction-availability guarantee.

Physical live-camera capture, pause/seek, auto-loop and alternate SDK versions
were not exercised. SDK `MOTION SENSORS REQUIRED` warnings remained in NET mode,
as in the previous acceptance runs.

Raw JSON measurements and host logs are in
`log/standalone-replay-20261028/`; Jetson backend logs use the same workspace
relative directory. Key files are `udp-final-net.json`,
`udp-final-restarted-net.json`, `legacy-net.json`, `restart-final-clock.json`,
`udp-first-telemetry.json`, `udp-pose-comparison.json` and `duplicate-clock.json`.
All test sources and backend launches were stopped after validation. The
containers remain available for the [documented commands](../tools/host_replay/README.md).
