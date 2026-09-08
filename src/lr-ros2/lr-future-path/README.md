# LR Future Path

This package builds and replays an offline future ground-truth path for ZED
SVO datasets. It is an evaluation/visualization oracle, not a route planner
and must not be consumed by prediction or rover control.

The first enabled launch performs a headless, non-real-time SVO pass and
records ZED map poses into a rosbag2 directory. Later launches load that cache
and publish a bounded look-ahead nav_msgs/msg/Path on
/lr/future_path/ground_truth.

The default path budget is 15 m of cumulative XY travel and 20 seconds. Points
are emitted only after moving 0.2 m from the last emitted point, rather than by
accumulating every sub-threshold position fluctuation. A route that turns back
or loops therefore still consumes the 15 m budget instead of remaining inside
an unbounded 15 m radius.

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
do not produce TF or Path data. MAVLink path sampling additionally suppresses
reported stationary samples at or below 0.1 m/s and rejects reported or
position-derived speeds above 5 m/s by default. These thresholds are
configurable from the display launch.
