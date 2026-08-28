# Landfill Rover simulation

Gazebo Harmonic assets and the one-command simulation runner for the main
`lr-ros2` workspace.

## Build

Run inside the development container:

```bash
cd /workspace/lr-ros2
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

The container image must provide Gazebo Harmonic, `python3-gz-transport13`,
and `ros-humble-depth-image-proc`. They are declared in the root Dockerfile.

## Run the pipeline

```bash
cd /workspace/lr-ros2
BAG_PROFILE=none bash simulation/run_pipeline.sh
```

Use `BAG_PROFILE=physics` for state/contact recording or
`BAG_PROFILE=full` to include camera streams. Press `Q` in teleop to stop
Gazebo, ROS nodes, and rosbag cleanly.

The integrated ROS launch starts:

- Gazebo-to-ROS command, state, contact, camera, and clock bridges.
- Depth-image to point-cloud conversion.
- Point-cloud transformation and accumulation in `map`.
- Terrain geometry estimation.
- RViz visualization.

Prediction consumes `/rover/state` and `/external_wrenches` directly, but
still requires typed trajectory, geometry, and tracked-object adapters.
