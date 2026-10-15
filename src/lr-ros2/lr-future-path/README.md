# LR Future Path

This package builds and replays an offline future ground-truth path for ZED
SVO datasets. It supplies the recorded future trajectory to the offline
prediction/evaluation pipeline. It is not a route planner and must not be
used for live rover control.

The first enabled launch performs a headless, non-real-time SVO pass and
records ZED map poses into a rosbag2 directory. Later launches load that cache
and publish a bounded look-ahead nav_msgs/msg/Path on
/lr/future_path/ground_truth.

The default display extent is a 15 m XY radius around the current pose. The
path stops at its first exit, with an interpolated boundary endpoint; it does
not re-enter the circle from later in the recording. There is no time cutoff
by default (`horizon_s: 0`), so slow motion and long stops do not shorten the
look-ahead. An explicit positive `horizon_s` restores an optional time cap.
For loops contained inside the circle, a secondary spatial limit of twice the
radius bounds the displayed route. Real gaps and the end of a recording can
still shorten it; no trajectory is fabricated across missing data.

Both GPS and VIO sources sample new route sections at approximately 0.2 m,
with at most 256 output poses. Bounded local linear smoothing uses a 0.8 m
spatial window and moves each sample by at most 0.2 m. It preserves the current
anchor and endpoint, recorded/interpolated timestamps and attitudes.
Interior landmarks keep their map coordinates and source timestamps while the
view advances. New landmarks enter at the front; existing ones are not
redistributed when the visible length changes. Retention is validated against
the current raw route within `smoothing_max_deviation_m`; larger route
corrections, rewinds and frame changes discard old landmarks. This is a processed reference trajectory,
not the original unsmoothed samples. The same processed path feeds terrain and
prediction; pose/TF localization outputs are unchanged. Set
`smoothing_window_m: 0` to disable smoothing.

Output uses reliable transient-local QoS with depth one, so RViz can attach
while replay is paused. It retains the last path for `display_hold_s: 1.5`
seconds of ROS time through input gaps or transient one-point paths, without
republishing it as fresh data.
Longer gaps publish an empty path; backward ROS time changes clear retained
data immediately. A paused ROS clock does not expire the displayed path.

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
