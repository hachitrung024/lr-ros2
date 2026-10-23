# Host ZED NET source

`lr_zed_stream` opens a host-attached stereo ZED camera or SVO/SVO2 using
`pyzed.sl` from ZED SDK 5.4.1. The source disables depth and does not enable
tracking, publish images over ROS, or broadcast TF. Jetson receives the ZED
NET stream and runs the normal ZED/LR processing pipeline.

```bash
ros2 launch lr_zed_stream stream_sender.launch.py \
  svo_path:=/zed/svo/zed_20260710_092420_0001.svo2
```

For live capture, omit `svo_path` or pass `svo_path:=live`. Select the camera
with `serial_number` or `camera_id`. Live defaults are `resolution:=HD720`
and `fps:=60`; SVO uses its recorded resolution and frame rate.

Encoder defaults: `port:=30000 codec:=H264 bitrate:=20000 gop_size:=1`.
The port must be even and bitrate is Kbit/s. `camera_name:=zed_source`
selects the ROS namespace, not a physical camera model.

SVO-only endpoints under `/zed_source/zed_node`:

| Endpoint | Type |
| --- | --- |
| `status/svo` | `zed_msgs/msg/SvoStatus`, 2 Hz using a steady timer |
| `toggle_svo_pause` | `std_srvs/srv/Trigger`, experimental opt-in |
| `set_svo_frame` | `zed_msgs/srv/SetSvoFrame`, experimental opt-in |

Only SVO mode publishes `/clock`. All capture and control callbacks share a
single executor thread. Playback is paced at the recorded frame rate with
`replay_rate:=1.0`, independently of ROS time. `start_paused:=true` sends the
first frame and holds it; `play_from_frame` selects an initial frame.
Resume a source started paused before connecting a new receiver: SDK 5.4.1
needs advancing frames to finish the NET handshake.

**Sequential playback is the accepted mode for SDK 5.4.1.** Pause/seek services
are disabled unless `enable_svo_controls:=true`; `start_paused` also requires
that experimental opt-in. Source-local controls hold IMAGE timestamps and
freeze `/clock`, but the SDK deduplicates held frames and the NET receiver can
time out. Paused seek and NET Replay have not passed hardware acceptance.
The source-local state machine is unit/ROS-tested; those tests do not establish
NET delivery. A live source exposes no SVO controls and publishes no `/clock`.

NET is a compressed real-time transport: it does not guarantee that Jetson
processes every source frame. Results need not be numerically identical to
processing the original SVO locally. RViz images, point clouds and results
still travel from Jetson to the host over ROS 2 DDS.

See the workspace [distributed guide](../../../docs/distributed-rviz.md)
for the three-terminal workflow, build and verification commands.
