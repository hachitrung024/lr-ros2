# LR Future Path

This package builds and replays an offline future ground-truth path for ZED
SVO datasets. It is an evaluation/visualization oracle, not a route planner
and must not be consumed by prediction or rover control.

The first enabled launch performs a headless, non-real-time SVO pass and
records ZED map poses into a rosbag2 directory. Later launches load that cache
and publish a 15 m look-ahead nav_msgs/msg/Path on
/lr/future_path/ground_truth.

By default caches live beside the SVO:

    <svo-directory>/.lr_future_path_cache/<svo-stem>-<fingerprint>/

Each cache contains metadata.yaml, a SQLite .db3 bag, and manifest.json. The
fingerprint includes the SVO file identity, camera model, ZED SDK/wrapper
versions, and cache schema.

The package also provides `mavlink_pose_node` for matching a custom rover
`session_mavlink.db` to SVO `/clock`. It reads SQLite with immutable read-only
access, publishes the camera pose and dynamic TF from GPS/attitude, and can
publish the future ground-truth Path directly from MAVLink without creating a
rosbag cache. Invalid GPS fixes and gaps larger than the configured threshold
do not produce TF or Path data.
