# LR Prediction Bridge

`prediction_bridge_node` converts the four LR inputs in one ROS node:

| Input | Output |
|---|---|
| `/lr/future_path/ground_truth` | `/trajectory`, at most 20 poses per cycle |
| `/terrain_geometry/grid_map` | `/geometry`, sampled directly at those poses |
| `/segmentation/boxes_3d` | `/tracked_objects`, oriented XY footprints |
| `/lr/mavlink/pose` | `/rover/state`, dynamic profile only |

The standalone and display prediction launches use this consolidated node.
`trajectory_adapter_node`, `geometry_adapter_node`,
`tracked_objects_adapter_node` and `rover_state_adapter_node` remain thin
compatibility executables using the same converters.

`config/bridge.yaml` has one consolidated section with role-prefixed parameters,
and unprefixed sections for the compatibility executables. Topic overrides,
profile, expected frame and `use_sim_time` come from launch arguments. No ROS
message types or canonical topic defaults have changed.

Trajectory sampling retains the 1 m minimum start distance, 0.25 s minimum
cycle interval and preference for 0.25 s pose spacing. GridMap history selects
the newest map at or before each trajectory, at most 2 s old by default.
`GeometryArray.header.stamp` retains the GridMap timestamp; its separate source
ID/stamp reference the trajectory. Unknown terrain stays unknown.

All source frames must be explicit. Path/pose/GridMap must match the configured
frame; boxes can be transformed using timestamped TF. A TF miss is retried up
to `tracked_objects.tf_timeout_sec` without blocking other bridge inputs.
`force_frame_id_map` is accepted in compatibility YAMLs but no longer relabels
coordinates. Missing TF and malformed detections are reported, not published
as a valid empty object batch.

Numeric box IDs from the Kalman tracker are preserved until the tracker resets.
Fallback IDs for upstream sources without numeric IDs remain frame-local.
Object velocity is unavailable (`velocity_valid=false`). The state converter
keeps the existing three-pose finite-difference acceleration model, excluding
gravity; fewer than three valid samples cannot supply acceleration.

Clock rewind/source changes clear all history and pending TF work. Trajectory
IDs continue increasing within the process. Readiness warnings are published
on `/prediction/diagnostics`.
