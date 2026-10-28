# Always-on live acceptance

Date: 2026-10-28 (Asia/Saigon). Run commands and protocol: [live guide](always-on-live.md).

## Configuration

- Host and Jetson `zed-dev`, ROS 2 Humble, ZED SDK 5.4.1, domain 42, host networking.
- SVO `zed_20260710_092420_0001.svo2`: 28154 frames, 15 FPS; tests start at frame 4500.
- H.264, 20000 Kbit/s, GOP 1; host Python source copied to `/tmp/lr-live-independent-20261028` with ROS setup variables removed.
- Jetson uses `best.pt`, no DB, no clock, no ground truth or prediction. Origin comes from received GPS.
- Host chrony serves Jetson systemd-timesyncd. Both report synchronized; a later Jetson sample reports +177 us offset from host.

## Source and failure recovery

| Scenario | Result | First complete measured outputs |
|---|---|---|
| Initial source | PASS | 17.322 s |
| Restart after one minute | PASS | 10.432 s |
| NET receiver process group suspended | PASS | 16.092 s |
| MAVLink UDP dropped for 10 seconds | PASS | 2.647 s |
| Pose adapter killed and respawned | PASS | 4.141 s |
| Retired-session TUNNEL injected | PASS | 0.601 s |
| Restart after ten minutes | PASS | 9.931 s |

The source was actually stopped for 63.013 s and 601.649 s. These are source-process outages, not host power-off tests. Backend PID **7095** remained alive throughout. Segmentation, box-estimator and supervisor PIDs stayed unchanged; the pose adapter was intentionally restarted in its dedicated test.

All three source sessions had different UUIDs, retained the same ENU origin and produced current wall timestamps with exact shared RGB/cloud and pose/frame stamps. Each live probe observed zero `/clock` publishers, no ground-truth/prediction publishers and one actual DDS publisher of `map → zed_camera_link`.

Recovery times are measured from probe startup, immediately following source start or fault restoration; they include ROS discovery. Empty/reset GridMap snapshots are excluded from terrain readiness in the accepted run. All measured recoveries are below 30 seconds.

The UDP proxy on Jetson port 14541 was a test fixture only. Production traffic and the final backend use port 14540; the proxy and drop controls were stopped.

An additional supervisor SIGKILL test also passes: all outputs recover within 6.870 s after supervisor respawn, source UUID and origin are preserved, model PIDs stay fixed and the old camera launcher exits. Linux parent-death signaling prevents an orphan camera launch. The final backend remains running in the waiting state with the host source stopped.

## Sequential measurements

| Mode | Cloud Hz | Pose Hz | Terrain Hz | Overlay Hz | Prediction Hz |
|---|---:|---:|---:|---:|---:|
| Live after ten-minute outage | 9.757 | 12.636 | 3.279 | 1.719 | disabled |
| Original ROS replay | 8.032 | 14.105 | 4.515 | 1.718 | 0.360 |
| Standalone UDP-clock replay | 8.074 | 14.550 | 3.997 | 1.679 | 0.719 |

Each window is 20–25 seconds after readiness. The replay baselines also passed their complete prediction/ground-truth checks with exactly one clock publisher. Live uses full-ROI terrain and disables prediction, whereas replay uses the future-path corridor; these rates are not a controlled comparison of relay CPU cost.

The live C-window CameraInfo raw/public arrival comparison has 344 matches, median 57.462 ms and p95 157.247 ms. This measures the additional observed ROS/DDS hop at the host, including scheduling and network arrival differences, rather than isolated C++ execution time.

## Verification coverage

- Standalone Python: 23 tests pass, including both modes copied outside the workspace with ROS/workspace imports denied, real UDP encoding, TIMESYNC, causal samples and cleanup.
- Host colcon checks pass for bridge, display, segmentation, terrain and both original source packages. Jetson checks pass for bridge, display, segmentation and terrain.
- The native FCU test drives actual host MAVROS with standard GPS/TIMESYNC and no TUNNEL metadata. Jetson skips that optional Python fixture because pymavlink is not a Jetson runtime dependency.
- 48701 encoded sensor packets match the original encoder byte for byte after zeroing only their timestamp fields. Boot-clock mapping is tested separately.
- The complete pose audit checks 4928 real published poses: zero future GPS/attitude samples; maximum quaternion norm error 2.22e-16.
- The reader-resume test verifies that reconnecting recreates the image subscription, preserves the same model object and suppresses inference results from a previous generation.

Jetson cannot reach the ROS XML schema server over HTTP. Its XML linter was rerun against the unchanged official `http://download.ros.org/schema/package_format3.xsd`, downloaded on the host and selected using `XML_CATALOG_FILES=/tmp/lr-ros-catalog.xml`. This is a test-only local schema cache; no schema checks were disabled.

An early receiver/source restart left the existing segmentation image reader inactive while fresh observers still received images. Rejoining the sensor subscriptions on readiness return fixed this; the accepted full cycle above retains the model processes throughout.

## Artifacts and limits

- Host artifacts: `log/live-net-20261028/`, including `accept-A/B/C.json`, transport/respawn/rejected-session probes, baseline ROS/UDP reports and `wire-comparison.json`.
- Jetson artifacts: the same relative log directory, with backend logs and `pose-accepted-readers.jsonl`.
- The initial failed probes remain in the logs for diagnosis; accepted results are the completed cycle report.
- Actual camera/FCU hardware has not been accepted in this run. SVO NET and a protocol-level FCU fixture are not physical-hardware evidence.
- These short operational windows do not establish long-duration throughput, model quality or a latency SLA.
