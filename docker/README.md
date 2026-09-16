# Landfill Rover Docker environments

These images extend the desktop or Jetson image produced by the corresponding
builder in `src/zed-ros2-wrapper/docker`. They contain the ZED SDK, CUDA, ROS 2,
the ZED ROS 2 wrapper, and the external dependencies needed to build and run
Landfill Rover.

It deliberately does **not** contain the LR repository, colcon build products,
models, SVO recordings, or MAVLink databases. Mount those at runtime and build
the mounted workspace inside the container.

## Build the desktop environment

From the repository root:

```bash
./docker/build_desktop.sh \
  --ros-distro humble \
  --os ubuntu-22.04 \
  --sdk 5.4.1 \
  --cuda 12.8
```

The resulting tag is:

```text
lr_zed_ros2_desktop_humble_u22.04_sdk5.4.1_cuda12.8
```

The script first builds or reuses the ZED wrapper image, then adds only the LR
development and runtime dependencies. To reuse an existing ZED image directly:

```bash
./docker/build_desktop.sh \
  --base-image zed_ros2_desktop_humble_u22.04_sdk5.4.1_cuda12.8 \
  --tag lr-ros2:humble
```

## Build the Jetson environment

On a Jetson Orin running JetPack 6.2.2:

```bash
./docker/build_jetson.sh \
  --ros-distro humble \
  --os jp6.2.2 \
  --sdk latest \
  --tag lr-ros2:jetson-humble
```

JetPack selects CUDA, so the Jetson builder has no separate `--cuda` option.
For JetPack 6.1/6.2 it installs the CUDA-enabled PyTorch 2.8 and TorchVision
0.23 pair from the Jetson CUDA 12.6 package index.

On Jetson Thor running JetPack 7.1:

```bash
./docker/build_jetson.sh \
  --ros-distro jazzy \
  --os jp7.1.0 \
  --sdk latest \
  --tag lr-ros2:jetson-jazzy
```

JetPack 7.1 uses the upstream PyTorch CUDA 13.2 index. To override the selected
package source for a custom JetPack image, pass
`--torch-index-url <compatible-index>`.

Native builds on the target Jetson are recommended. Before cross-building an
ARM64 image on an x86_64 host, register QEMU:

```bash
docker run --rm --privileged \
  multiarch/qemu-user-static --reset -p yes
```

## Run the desktop image with mounted sources and data

Run this from the LR repository root. Adjust the host paths for SVO and MAVLink
data as needed:

```bash
xhost +si:localuser:root

docker run --rm -it \
  --name lr-ros2-humble \
  --gpus all \
  --privileged \
  --network host \
  --ipc host \
  --env DISPLAY="${DISPLAY}" \
  --env NVIDIA_DRIVER_CAPABILITIES=all \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --volume "$(pwd):/workspace/lr-ros2:rw" \
  --volume /zed/svo:/data/svo:ro \
  --volume /zed/mavlink:/data/mavlink:ro \
  --workdir /workspace/lr-ros2 \
  lr_zed_ros2_desktop_humble_u22.04_sdk5.4.1_cuda12.8 \
  bash
```

On Jetson, use the NVIDIA runtime and the Jetson tag instead of `--gpus all`:

```bash
docker run --rm -it \
  --name lr-ros2-jetson \
  --runtime nvidia \
  --privileged \
  --network host \
  --ipc host \
  --env DISPLAY="${DISPLAY}" \
  --env NVIDIA_DRIVER_CAPABILITIES=all \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --volume "$(pwd):/workspace/lr-ros2:rw" \
  --volume /zed/svo:/data/svo:ro \
  --volume /zed/mavlink:/data/mavlink:ro \
  --workdir /workspace/lr-ros2 \
  lr-ros2:jetson-humble \
  bash
```

The inherited ZED entrypoint sources the ROS and ZED underlay automatically.
Inside the container, verify dependencies against the mounted checkout and
build LR locally:

```bash
rosdep install \
  --from-paths src/lr-ros2 \
  --ignore-src \
  --rosdistro humble \
  -r -y

colcon build --symlink-install \
  --packages-select \
  safety_perception_msgs \
  prediction_core \
  prediction_ros \
  lr_prediction_bridge \
  lr_future_path \
  lr_segmentation \
  lr_terrain_geometry \
  lr_path_prediction \
  lr_display_rviz2 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

source install/setup.bash
```

Because `build/`, `install/`, and `log/` are created in the mounted repository,
the results persist on the host and can be rebuilt without rebuilding the
Docker image.

## Start the pipeline

After building and sourcing the mounted workspace:

```bash
ros2 launch lr_display_rviz2 display_zed_cam.launch.py \
  camera_model:=zed2i \
  publish_svo_clock:=true \
  segmentation_model_path:=/workspace/lr-ros2/models/best.pt \
  mavlink:=true \
  mavlink_db_path:=/data/mavlink/session_mavlink.db \
  future_path:=true \
  svo_path:=/data/svo/recording.svo2
```

After leaving the container, revoke the temporary X11 permission:

```bash
xhost -si:localuser:root
```

## What is installed in the image

- ROS 2 and the ZED wrapper inherited from the base image
- Grid Map messages and RViz plugin
- Vision messages, RViz, robot state publisher, rosbag Python support
- ROS build, lint, and test dependencies used by the LR packages
- NumPy, OpenCV, Shapely, PyTorch, and Ultralytics
- `rosdep`, `colcon`, CMake, Qt development files, and compiler tools

The image build smoke test verifies these dependencies and the ZED underlay. It
does not build or launch LR packages because LR sources are runtime mounts.

## Cleanup

Remove stopped containers, unused networks, dangling images, and build cache:

```bash
./docker/cleanup.sh
```

Add `--all-images` to also remove non-dangling images that no container uses.
Docker volumes are always preserved.
