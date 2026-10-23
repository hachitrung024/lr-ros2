# ZED NET sequential acceptance — 2026-10-23

The accepted first deployment retains SDK 5.4.1 and sequential SVO2 playback.
Host capture/encoding and RViz run in the host container; the complete headless
LR backend runs on Jetson. Pause/seek NET is excluded, experimental and disabled
by default. The original direct-file workflow is retained.

Both machines built successfully. The final host `colcon test` run reported
126 tests, zero errors, zero failures and three skipped tests across
`lr_zed_stream`, `lr_display_rviz2` and `lr_future_path`. The targeted checks
included 55 Python tests and two SVO-panel C++ tests; the applicable MAVLink and
launch tests also passed on Jetson (40 tests), followed by the SVO-panel C++
target after the final rebuild. Raw measurements and limitations are preserved in
[net-streaming-validation.json](net-streaming-validation.json).

## Hardware run

The source was `zed_20260710_092420_0001.svo2`, native stereo 1920×1080 at 15 FPS,
starting from frame 4500 in a moving segment. Encoding used H.264, 20000 Kbit/s,
GOP 1 and port 30000. The current RViz preset remained enabled, with its SVO
control panel hidden. Both containers used host networking and ROS domain 42
over the 1 Gbit/s Ethernet link.

The 20-second measurement observed all required outputs: RGB CameraInfo, point
cloud, segmentation overlay, terrain, MAVLink pose, future path and prediction.
Exactly one `/clock` publisher existed. Sixty-nine RGB/cloud timestamps matched;
an additional Jetson-local check confirmed three exact timestamps shared by
`/clock`, RGB CameraInfo, registered depth and cloud.

| Observation on host | Received rate | Median clock-arrival → data-arrival |
| --- | ---: | ---: |
| RGB CameraInfo | 5.39 Hz | 215 ms |
| Point cloud | 4.64 Hz | 389 ms |
| Segmentation overlay | 1.10 Hz | 1430 ms |
| Terrain grid | 3.04 Hz | — |
| MAVLink pose / future path | 14.97 Hz | 4 ms / 46 ms |
| Prediction output | 0.10 Hz | Two samples; insufficient for a latency conclusion |

Host Ethernet received approximately **846 Mbit/s** and transmitted **23 Mbit/s**.
These measurements include RViz and observer subscriptions. They show the load
of the current display configuration; they are not an isolated codec benchmark.
The existing display/processing rates were retained, without bandwidth tuning.

Stopping and restarting the host source at frame 4500 recovered all required
outputs without restarting the Jetson backend launch process. A subsequent
10-second measurement passed with 38 matching RGB/cloud timestamps. The source
was also checked with experimental controls disabled: its parameter was false
and its node advertised no pause/seek service servers.

## Limits of this acceptance

- NET pause/seek failed: a long pause froze ROS time but the SDK receiver timed
  out, and a paused seek did not deliver the target frame. SDK-only GEN1/GEN2
  tests reproduced the issue. Source-local state-machine tests do not establish
  successful NET delivery. Replay and long-pause connection continuity are not
  accepted features.
- No physical camera was connected. Live source configuration has automated
  coverage; live capture/tracking still needs a hardware run.
- The SDK receiver logged `MOTION SENSORS REQUIRED` when polling sensors in
  NET/MAVLink mode. The accepted run uses recorded MAVLink pose, with ZED VIO
  disabled; RGB/depth/cloud processing and MAVLink outputs were verified.
- Prediction publication was verified, including messages with missing geometry
  evidence. Prediction quality and terrain observability were not revalidated.
- The Jetson clock was about nine days behind the host, producing build clock
  skew warnings. No SDK upgrade or system-clock change was made. Latency uses
  one host's monotonic clock and excludes capture work before `/clock` emission.

The test launches were stopped after verification. Reproduce the workflow with
the [distributed guide](distributed-rviz.md). For a full-pipeline measurement,
run `verify_net_replay.py --duration 20 --require-prediction` in the host ROS
environment while the source is in a moving segment.
