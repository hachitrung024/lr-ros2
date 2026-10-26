# lr_mavlink_replay

Host-side causal replay of decoded GPS, attitude and IMU from a MAVLink SQLite
recording into MAVLink 2 UDP, paced by the existing SVO `/clock`.

See [the deployment guide](../../../docs/mavros-replay.md) for dependencies,
three-terminal commands, timestamp/unit contracts and verification tools.

`host_replay.launch.py` starts the ZED source and telemetry replay together.
`telemetry_replay` also runs independently; parameters select the DB/directory
and local/remote UDP endpoints. Both addresses and ports are required on the
command line; network endpoints have no defaults. The replay does not produce `/clock`, TF or
ROS sensor messages. Heartbeat and version discovery are supported; command
requests unrelated to version discovery receive `UNSUPPORTED`.
