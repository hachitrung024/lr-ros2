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
