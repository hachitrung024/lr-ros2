# Prediction ROS

`prediction_ros` is the typed ROS 2 wrapper around `prediction_core`. Its main
executable is:

```bash
ros2 run prediction_ros prediction_node --ros-args \
  -p config_path:=/path/to/rover.yaml \
  -p prediction_profile:=static
```

## Canonical topics

| Topic | Type | Direction |
|---|---|---|
| `/trajectory` | `safety_perception_msgs/Trajectory` | input |
| `/tracked_objects` | `safety_perception_msgs/TrackedObjectArray` | input |
| `/geometry` | `safety_perception_msgs/GeometryArray` | input |
| `/rover/state` | `safety_perception_msgs/RoverState` | dynamic input |
| `/external_wrenches` | `safety_perception_msgs/ExternalWrenchArray` | optional input |
| `/predict_output` | `safety_perception_msgs/PredictionOutput` | output |

`/predict_output` is raw safety evidence. RViz and diagnostics are produced
separately by `lr_path_prediction`; this package has no runtime RViz UI and
does not make Stop/Go decisions.


Runtime replay age limits are in `config/prediction.yaml`: objects 0.5 s,
geometry 2 s and state 0.25 s. Histories select the newest same-frame observation
at/before a trajectory timestamp, including valid empty object batches.
Future inputs are never reused for earlier cycles. Geometry may arrive before
its trajectory; it is buffered by frame, source ID and source stamp.

`/prediction/diagnostics` reports readiness, callback duration and output count.
Clock rewinds clear histories and the core runtime; the ROS-independent
`PredictionRuntime.reset()` API also exposes that operation. The existing
`on_objects` API now accepts optional batch `timestamp`, required by age
validation for empty batches. The core physics algorithms are unchanged.
