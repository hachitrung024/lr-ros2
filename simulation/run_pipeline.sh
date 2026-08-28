#!/usr/bin/env bash
set -eo pipefail

ROS_WS="/workspace/lr-ros2"
SIM_ROOT="$ROS_WS/simulation"
LOG_DIR="$SIM_ROOT/logs"
BAG_DIR="$SIM_ROOT/rosbags"
BAG_PROFILE="${BAG_PROFILE:-physics}"

source /opt/ros/humble/setup.bash
source "$ROS_WS/install/setup.bash"
set -u

export GZ_PARTITION="${GZ_PARTITION:-landfill_rover}"
export GZ_SIM_RESOURCE_PATH="$SIM_ROOT/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
mkdir -p "$LOG_DIR" "$BAG_DIR"
python3 "$SIM_ROOT/scripts/generate_proving_ground.py"

gazebo_pid=""
ros_pid=""
bag_pid=""

cleanup() {
  trap - INT TERM EXIT
  echo
  echo "Stopping teleop, rosbag, ROS nodes and Gazebo..."
  ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}' >/dev/null 2>&1 || true
  for process_id in "$bag_pid" "$ros_pid" "$gazebo_pid"; do
    if [[ -n "$process_id" ]] && kill -0 "$process_id" 2>/dev/null; then
      kill -INT -- "-$process_id" 2>/dev/null || true
    fi
  done
  sleep 2
  for process_id in "$bag_pid" "$ros_pid" "$gazebo_pid"; do
    if [[ -n "$process_id" ]] && kill -0 "$process_id" 2>/dev/null; then
      kill -TERM -- "-$process_id" 2>/dev/null || true
    fi
  done
}
trap cleanup INT TERM EXIT

echo "Starting Gazebo (log: $LOG_DIR/gazebo.log)..."
setsid gz sim -r "$SIM_ROOT/worlds/rollover_proving_ground.sdf" >"$LOG_DIR/gazebo.log" 2>&1 &
gazebo_pid=$!

echo "Waiting for the Gazebo world..."
for _ in $(seq 1 60); do
  if ! kill -0 "$gazebo_pid" 2>/dev/null; then
    echo "Gazebo exited. See $LOG_DIR/gazebo.log"
    exit 1
  fi
  if gz topic -l 2>/dev/null | grep -Eq '^/world/.*/clock$'; then
    break
  fi
  sleep 1
done

echo "Starting the ROS 2 simulation pipeline (log: $LOG_DIR/ros.log)..."
setsid ros2 launch rover_sim_bringup simulation_pipeline.launch.py >"$LOG_DIR/ros.log" 2>&1 &
ros_pid=$!

for _ in $(seq 1 30); do
  if ros2 topic list 2>/dev/null | grep -q '^/rover/state$'; then
    break
  fi
  sleep 1
done

timestamp=$(date '+%Y%m%d_%H%M%S')
case "$BAG_PROFILE" in
  none)
    echo "Rosbag recording disabled."
    ;;
  physics|full)
    bag_path="$BAG_DIR/rollover_v2_${BAG_PROFILE}_${timestamp}"
    bag_topics=(
      /clock /cmd_vel /odom /imu/data /tf /tf_static
      /contacts/left_track/wrench /contacts/right_track/wrench
      /contacts/chassis/wrench /external_wrenches /rover/state /diagnostics
      /zed2i/point_cloud /lr/point_cloud/cloud_in_map
      /terrain_geometry/grid_map /terrain_geometry/heatmap
    )
    if [[ "$BAG_PROFILE" == "full" ]]; then
      bag_topics+=(
        /zed2i/left/image_raw /zed2i/left/camera_info
        /zed2i/right/image_raw /zed2i/right/camera_info
        /zed2i/depth/image_raw /zed2i/depth/camera_info
      )
    fi
    setsid ros2 bag record -o "$bag_path" "${bag_topics[@]}" >"$LOG_DIR/rosbag.log" 2>&1 &
    bag_pid=$!
    echo "Recording $BAG_PROFILE rosbag: $bag_path"
    ;;
  *)
    echo "Invalid BAG_PROFILE='$BAG_PROFILE' (use physics, full, or none)."
    exit 2
    ;;
esac

echo
echo "Gazebo and ROS 2 are ready. Keyboard control starts below."
echo "Press Q in teleop to stop everything cleanly."
echo
ros2 run rover_sim_bringup teleop
