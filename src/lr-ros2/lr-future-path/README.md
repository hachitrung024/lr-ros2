# lr_future_path

`mavlink_ground_truth_node` supplies the session's fixed ENU origin and offline
reference future path from `session_mavlink.db`. It never publishes camera
pose, `/tf` or `/clock`; pose estimation belongs to `lr_mavros_bridge` and uses
received telemetry only. This package also provides shared DB decoding,
coordinate conversion, quaternion and path processing utilities.

The reference process selects the unique DB covering the first received host
clock, or validates `mavlink_db_path` explicitly. It publishes the first valid
GPS origin as transient-local `/lr/mavlink/origin`. `future_path_enabled` enables
transient-local `/lr/future_path/ground_truth`. Body–camera extrinsics and ENU
height conventions match the MAVROS pose adapter.

Future paths are sampled within the configured XY radius with GPS gap/speed
limits, spatial smoothing, bounded resampling and ROS-time display retention.
Rewinding the clock clears retained display state. These recorded paths are
for offline reference and prediction evaluation, not vehicle control.

See the [deployment guide](../../../docs/mavros-replay.md). SVO VIO caching,
rosbag pose extraction and DB pose/TF publication are no longer part of this
package.
