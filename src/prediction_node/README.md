# ROS 2 prediction runtime

The runtime uses generated `safety_perception_msgs` types. JSON-over-
`std_msgs/String` remains only in `JsonAdapters` as a development migration
helper; it is not subscribed to by `prediction_node`.

## Packages

- `safety_perception_msgs`: shared system-wide canonical interfaces.
- `prediction_ros`: typed adapter, thread-safe cache, validator, coordinator,
  and thin `rclpy` node.

`prediction_core` remains ROS-independent.

## Build and run

```bash
cd /workspace/lr-ros2
source /opt/ros/humble/setup.bash
colcon build --packages-select safety_perception_msgs prediction_ros
source install/setup.bash
ros2 launch prediction_ros prediction.launch.py \
  config_path:=/absolute/path/to/rover.yaml
```

The launch configuration provides parameterized `/trajectory`,
`/tracked_objects`, `/geometry`, `/rover/state`, `/external_wrenches`, and
`/predict_output` topics. Collision and rollover evidence run once per
`trajectory_id`; matching `GeometryArray.source_trajectory_id` is mandatory.

Read the upstream-facing interface contract at
[`../documents/ROS_UPSTREAM_INTERFACE_CONTRACT.md`](../documents/ROS_UPSTREAM_INTERFACE_CONTRACT.md).
