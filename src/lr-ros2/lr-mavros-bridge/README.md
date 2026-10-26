# lr_mavros_bridge

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
