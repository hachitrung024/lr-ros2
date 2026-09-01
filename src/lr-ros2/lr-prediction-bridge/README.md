# LR Prediction Bridge

This package preserves the canonical Prediction topic contract while adapting
the existing LR pipeline:

| Executable | Conversion |
|---|---|
| `trajectory_adapter_node` | `nav_msgs/Path` to `/trajectory` |
| `geometry_adapter_node` | GridMap elevation/normals to `/geometry` |
| `tracked_objects_adapter_node` | `vision_msgs/Detection3DArray` to `/tracked_objects` |
| `rover_state_adapter_node` | stamped pose to `/rover/state` with finite-difference velocity and acceleration |

The default adapter configuration is `config/bridge.yaml`. Trajectories contain
at most 20 poses and exclude the current rover position using the configured
minimum start distance. Geometry coverage may be partial; missing terrain is
kept unknown rather than replaced by a flat plane.

Detection IDs are frame-local and object velocity is unavailable, so objects
are treated as static by the current Prediction cycle. The rover-state adapter
is launched only for the dynamic profile.
