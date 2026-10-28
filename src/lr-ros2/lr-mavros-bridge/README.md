# lr_mavros_bridge

## Always-on live mode

`headless_live.launch.py` uses the `live.yaml` MAVROS profile, source-session
plugin, C++ sensor timestamp relay and `live_runtime` supervisor. Pose is driven
by normalized frame stamps with a first-GPS ENU origin; no DB or `/clock` is
used. See the [live deployment guide](../../../docs/always-on-live.md).

Jetson-side MAVROS configuration, deterministic replay time plugin, and causal
GPS/attitude adapter for `/lr/mavlink/pose` and `map → zed_camera_link`.

See [the deployment guide](../../../docs/mavros-replay.md) for configuration,
dependencies, coordinate conventions, limits and acceptance commands.

`mavros_replay.launch.py` starts the gateway. The LR display launch adds the
pose adapter and separate DB ground-truth process for every SVO NET source
(`stream_svo_mode:=true`). Live NET uses ZED tracking instead.
Pass `fcu_url` to the standalone gateway, or `mavros_fcu_url` to the LR launch;
both require an explicit transport URL with the desired addresses and ports.

The adapter publishes nothing until the replay time offset, fixed origin and
valid past sensor samples are available. Data older than configured bounds
cannot be relabeled as a fresh pose. `/diagnostics` reports readiness/staleness.
`verify_tf_owner` checks the DDS publisher identity of the actual camera TF.

`replay_clock_bridge` optionally converts the standalone host's version-one
JSON clock UDP packets to `/clock`. It requires explicit `bind_address` and
`bind_port`, runs on wall time, rejects stale sessions/packets and fails on
another ROS clock publisher. The headless launch selects it with
`replay_clock_source:=udp`. See the [standalone source guide](../../../tools/host_replay/README.md).
