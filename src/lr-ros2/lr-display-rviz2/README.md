# Stereolabs ZED Camera - ROS 2 Display package

This package lets you visualize in the [ROS 2 RViz application](https://github.com/ros2/rviz/tree/foxy) all the
possible information that can be acquired using a Stereolabs camera.
The package provides the launch files for ZED, ZED Mini and ZED 2 camera models.

**Note:** The main package [zed-ros2-wrapper](https://github.com/stereolabs/zed-ros2-wrapper)
is required to correctly execute the ROS node to acquire data from a Stereolabs 3D camera.

## Getting started

- First, be sure to have installed the main ROS package to integrate the ZED cameras in the ROS framework: [zed-ros2-wrapper](https://github.com/stereolabs/zed-ros2-wrapper/#build-the-package)
- [Install](#Installation) the package
- Read the online documentation for [More information](https://www.stereolabs.com/docs/ros2/)

### Prerequisites

- ROS 2 Foxy Fitzroy (deprecated), ROS 2 Humble Hawksbill, or ROS 2 Jazzy Jalisco:
  - [Foxy on Ubuntu 20.04](https://docs.ros.org/en/foxy/Installation/Linux-Install-Debians.html) - [**Not recommended. EOL reached**]
  - [Humble on Ubuntu 22.04](https://docs.ros.org/en/humble/Installation/Linux-Install-Debians.html) - [EOL May 2027]
  - [Jazzy Jalisco on Ubuntu 24.04](https://docs.ros.org/en/jazzy/Installation/Linux-Install-Debians.html) - [EOL May 2029]

### Installation

The *lr_display_rviz2* is a colcon package. 

Install the [zed-ros2-wrapper](https://www.stereolabs.com/documentation/guides/using-zed-with-ros/introduction.html) package
following the [installation guide](https://github.com/stereolabs/zed-ros2-wrapper#build-the-package)

Install the [zed-ros2-examples](https://github.com/stereolabs/zed-ros2-examples) package following the [installation guide](https://github.com/stereolabs/zed-ros2-examples#build-the-package)

### Execution

Use the following launch command to start the ZED ROS2 Wrapper node and RVIZ2 with the default setting for the camera that you are using:

```bash
$ ros2 launch lr_display_rviz2 display_zed_cam.launch.py camera_model:=<camera_model>
```

To deploy the processing pipeline without a local GUI, use
`headless_zed_cam.launch.py` with the same backend arguments. Run the UI on the
debug machine with `rviz_zed_cam.launch.py`; for SVO playback pass
`use_sim_time:=true svo_mode:=true`. Both processes can be tested on one host
and use ROS 2 DDS unchanged when moved to two hosts on the same LAN. The full
commands and network checklist are in the workspace
`docs/distributed-rviz.md` guide.

Replace `<camera_model>` with the model of the camera that you are using: `'zed'`, `'zedm'`, `'zed2'`, `'zed2i'`, `'zedx'`, `'zedxm'`, `'virtual'`.

To play an SVO file in real time and publish its clock, use:

```bash
$ ros2 launch lr_display_rviz2 display_zed_cam.launch.py \
    camera_model:=<camera_model> \
    svo_path:=/path/to/recording.svo2 \
    svo_realtime:=true \
    publish_svo_clock:=true
```

Set `svo_realtime:=false` to process every frame instead of preserving the
recorded timing. In that mode, playback speed can be customized through the
ZED wrapper configuration.

### SVO future ground truth

Enable future_path to visualize an offline VIO look-ahead path:

    ros2 launch lr_display_rviz2 display_zed_cam.launch.py \
        camera_model:=zed2i \
        svo_path:=/path/to/recording.svo2 \
        publish_svo_clock:=true \
        future_path:=true

On the first run, the launch performs a headless non-real-time SVO pass and
stores a rosbag2 cache under the hidden .lr_future_path_cache directory beside
the SVO. Once the cache is complete, the normal SVO/RViz pipeline starts
automatically. Later runs reuse the cache immediately. Set
future_path_rebuild_cache to true to force preprocessing again, or set
future_path_cache_dir to choose another cache root.

The output is a nav_msgs/msg/Path on /lr/future_path/ground_truth. By default
it is sampled every 0.2 m and bounded by both 15 m of cumulative XY travel and
20 seconds. Small sub-step fluctuations are not accumulated into fake travel,
so loops and stationary jitter cannot create an unbounded path. It is future
ground-truth for offline evaluation only, not a route planner or a control
input.

### MAVLink localization for SVO

An SVO can use the matching rover MAVLink SQLite session instead of ZED
positional tracking:

    ros2 launch lr_display_rviz2 display_zed_cam.launch.py \
        camera_model:=zed2i \
        svo_path:=/workspace/svo/zed_20260710_092420_0001.svo2 \
        publish_svo_clock:=true \
        mavlink:=true \
        future_path:=true

The launch recursively searches `mavlink_dir` (default: `mavlink` relative to
the directory where the launch command runs) for `session_mavlink.db`. It
selects the unique session matching the first SVO clock timestamp. Use
`mavlink_db_path` to select a database explicitly.

In this mode ZED dynamic TF and positional tracking are disabled. The MAVLink
node converts valid GPS to a local ENU `map`, converts attitude from MAVLink
NED/FRD to ROS ENU/FLU, and publishes `map -> zed_camera_link` plus
`/lr/mavlink/pose`. The ZED URDF continues to publish the camera's static
frames. The default camera mounting transform is identity; override
`mavlink_body_to_camera:='[x,y,z,roll,pitch,yaw]'` when measured extrinsics
are available.

When `future_path` is also enabled, the MAVLink database directly supplies
`/lr/future_path/ground_truth`; no VIO preprocessing pass or rosbag cache is
created. Pose, TF, and Path are suppressed across valid-GPS gaps larger than
`mavlink_max_gps_gap_s` (default 1.5 s), then resume automatically. This is
recorded ground truth for offline visualization/evaluation, not a planned
route or a control input.

MAVLink path sampling also excludes reported stationary samples at or below
0.1 m/s and rejects reported or position-derived speeds above 5 m/s. Configure
the window and gates with `future_path_radius_m`, `future_path_horizon_s`,
`future_path_step_m`, `future_path_stationary_speed_mps`, and
`future_path_max_speed_mps`.

For stereo cameras the launch file also starts `lr_terrain_geometry` by
default. Use `start_terrain_node:=false` to disable it, or override
`terrain_params_file` and `map_frame` for a different terrain setup. The
terrain heatmap is shown as an Image dock inside the main RViz window, in the
former Depth Map position. The ZED Depth Map remains available but is disabled
by default. The central 3D Terrain group remains disabled by default and can be
enabled for GridMap and marker debugging.

Instance segmentation starts automatically when an external checkpoint path
is provided:

```bash
$ ros2 launch lr_display_rviz2 display_zed_cam.launch.py \
    camera_model:=zed2i \
    svo_path:=/path/to/recording.svo2 \
    segmentation_model_path:=/path/to/best.pt
```

The `start_segmentation_node` argument defaults to `auto`; use
`start_segmentation_node:=false` to explicitly disable segmentation.

The existing RGB dock then shows `/segmentation/overlay`. Segmentation also
publishes light-weight tracked 3D boxes on `/segmentation/boxes_3d` as
`vision_msgs/msg/Detection3DArray`; downstream nodes can consume those boxes
without subscribing to a dense segmentation point cloud. RViz visualizes the
same result from `/segmentation/box_markers`. When no model path is provided,
segmentation and box estimation are disabled and the RGB dock is remapped to
the original ZED RGB image. Terrain processing remains independent and does
not consume segmentation output.

![ZED rendering on Rviz](images/depthcloud-RGB.jpg)
![ZED rendering on Rviz](images/ZEDM-Rviz.jpg)
![ZED rendering on Rviz](images/ZED-Rviz.jpg)

[Detailed information](https://www.stereolabs.com/docs/ros2/rviz2/)
