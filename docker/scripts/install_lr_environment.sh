#!/usr/bin/env bash
# Install the build/runtime dependencies needed by the mounted LR workspace.
# LR source code and build products deliberately remain outside the image.
set -Eeuo pipefail

REQUIREMENTS_FILE="${1:-/tmp/requirements-lr.txt}"
LR_PLATFORM="${2:-desktop}"
TORCH_INDEX_URL="${3:-}"
TORCH_SPEC="${4:-torch>=2.1,<3}"
TORCHVISION_SPEC="${5:-torchvision}"
[ -f "${REQUIREMENTS_FILE}" ] || {
  echo "ERROR: Python requirements file not found: ${REQUIREMENTS_FILE}" >&2
  exit 1
}
case "${LR_PLATFORM}" in
  desktop|jetson) ;;
  *)
    echo "ERROR: Unsupported LR platform: ${LR_PLATFORM}" >&2
    exit 2
    ;;
esac
if [ "${LR_PLATFORM}" = "jetson" ] && [ -z "${TORCH_INDEX_URL}" ]; then
  echo "ERROR: Jetson builds require a JetPack-compatible PyTorch index URL." >&2
  exit 2
fi

# The upstream image writes /opt/ros_env for both its APT and source-based ROS
# installation modes. ROS-generated setup scripts are not compatible with
# Bash's nounset option: they probe variables such as AMENT_TRACE_SETUP_FILES
# before those variables have been defined. Temporarily disable nounset while
# sourcing them, then restore the strict setting for this script.
set +u
# shellcheck disable=SC1091
source /opt/ros_env
# Source only the ZED underlay inherited from the base image. No LR workspace
# exists in the environment image until the user mounts and builds one.
# shellcheck disable=SC1091
source /root/ros2_ws/install/local_setup.bash
set -u

export DEBIAN_FRONTEND=noninteractive

echo "============================================================"
echo " Installing Landfill Rover dependencies for ROS ${ROS_DISTRO}"
echo "============================================================"

apt-get update

# Keep the external dependencies explicit so the environment does not depend
# on having the LR repository available while the image is built.
apt-get install -y --no-install-recommends \
  qtbase5-dev \
  libopenblas-dev \
  libopenmpi-dev \
  libomp-dev \
  python3-numpy \
  python3-opencv \
  python3-pytest \
  python3-shapely \
  python3-yaml \
  "ros-${ROS_DISTRO}-ament-cmake-auto" \
  "ros-${ROS_DISTRO}-ament-cmake-clang-format" \
  "ros-${ROS_DISTRO}-ament-cmake-pytest" \
  "ros-${ROS_DISTRO}-ament-copyright" \
  "ros-${ROS_DISTRO}-ament-flake8" \
  "ros-${ROS_DISTRO}-ament-index-python" \
  "ros-${ROS_DISTRO}-ament-lint-auto" \
  "ros-${ROS_DISTRO}-ament-lint-common" \
  "ros-${ROS_DISTRO}-ament-pep257" \
  "ros-${ROS_DISTRO}-builtin-interfaces" \
  "ros-${ROS_DISTRO}-diagnostic-msgs" \
  "ros-${ROS_DISTRO}-geometry-msgs" \
  "ros-${ROS_DISTRO}-grid-map-msgs" \
  "ros-${ROS_DISTRO}-grid-map-rviz-plugin" \
  "ros-${ROS_DISTRO}-launch" \
  "ros-${ROS_DISTRO}-launch-ros" \
  "ros-${ROS_DISTRO}-nav-msgs" \
  "ros-${ROS_DISTRO}-pluginlib" \
  "ros-${ROS_DISTRO}-rcl-interfaces" \
  "ros-${ROS_DISTRO}-rclcpp" \
  "ros-${ROS_DISTRO}-rclpy" \
  "ros-${ROS_DISTRO}-robot-state-publisher" \
  "ros-${ROS_DISTRO}-rosbag2-py" \
  "ros-${ROS_DISTRO}-rosgraph-msgs" \
  "ros-${ROS_DISTRO}-rosidl-default-generators" \
  "ros-${ROS_DISTRO}-rosidl-default-runtime" \
  "ros-${ROS_DISTRO}-rviz-common" \
  "ros-${ROS_DISTRO}-rviz2" \
  "ros-${ROS_DISTRO}-sensor-msgs" \
  "ros-${ROS_DISTRO}-sensor-msgs-py" \
  "ros-${ROS_DISTRO}-std-msgs" \
  "ros-${ROS_DISTRO}-std-srvs" \
  "ros-${ROS_DISTRO}-tf2-msgs" \
  "ros-${ROS_DISTRO}-tf2-ros" \
  "ros-${ROS_DISTRO}-vision-msgs" \
  "ros-${ROS_DISTRO}-visualization-msgs"

rosdep update --rosdistro "${ROS_DISTRO}"

# Ubuntu 24.04 marks its system Python as externally managed. Use pip's
# supported override there, while remaining compatible with Humble/22.04.
PIP_ARGS=(--no-cache-dir)
if python3 -m pip install --help | grep -q -- '--break-system-packages'; then
  PIP_ARGS+=(--break-system-packages)
fi
if [ "${LR_PLATFORM}" = "jetson" ]; then
  echo ">>> Installing Jetson GPU-enabled PyTorch from ${TORCH_INDEX_URL}"
  python3 -m pip install "${PIP_ARGS[@]}" \
    --index-url "${TORCH_INDEX_URL}" \
    "${TORCH_SPEC}" \
    "${TORCHVISION_SPEC}"
fi
python3 -m pip install "${PIP_ARGS[@]}" \
  -r "${REQUIREMENTS_FILE}"

echo ">>> Verifying LR build/runtime dependencies"
python3 - "${LR_PLATFORM}" <<'PY'
import sys

import grid_map_msgs
import shapely
import torch
import ultralytics
import vision_msgs

platform = sys.argv[1]
if platform == "jetson" and torch.version.cuda is None:
    raise RuntimeError("The installed Jetson PyTorch build has no CUDA support")
print(f"LR Python imports: OK (torch={torch.__version__}, CUDA={torch.version.cuda})")
PY
ros2 pkg prefix grid_map_rviz_plugin >/dev/null
ros2 pkg prefix zed_wrapper >/dev/null
command -v colcon >/dev/null

rm -rf /var/lib/apt/lists/* /root/.cache/pip
echo ">>> Landfill Rover environment is ready for a mounted workspace."
