# Landfill Rover ROS 2

Quick instructions for running the ZED camera, segmentation, and terrain
geometry nodes in this workspace.

## 1. Prepare the workspace

Source the ROS 2 environment in every terminal that runs ROS 2 commands:

```bash
cd /workspace/testros2
source /opt/ros/humble/setup.bash
source install/setup.bash
```

If you have just modified the code or cloned the workspace, build the main packages:

```bash
cd /workspace/testros2
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select lr_segmentation lr_terrain_geometry lr_display_rviz2
source install/setup.bash
```

The segmentation model must be an instance-segmentation YOLO model and must
exist on the filesystem, for example:

```text
/workspace/testros2/landfill-rover/segmentation/best.pt
```

## 2. Run the full SVO pipeline: ZED + segmentation + terrain + RViz

This is the recommended way to test an SVO file:

```bash
ros2 launch lr_display_rviz2 display_zed_cam.launch.py \
  camera_model:=zed2i \
  svo_path:=/workspace/svo/zed_20260608_105844_0001.svo2 \
  publish_svo_clock:=true \
  segmentation_model_path:=/workspace/testros2/landfill-rover/segmentation/best.pt
```

When `segmentation_model_path` is set, segmentation is enabled automatically.
Therefore, you do not need to add `start_segmentation_node:=true`.

The pipeline will:

1. Read the image and point cloud from the ZED camera.
2. Run instance segmentation.
3. Project the point cloud into the 2D masks.
4. Create an object-oriented 3D bounding box.
5. Remove points inside the box before terrain fitting.
6. Display the 3D box and terrain markers in RViz.

## 3. Run an SVO without segmentation

Terrain processing will still run, but objects will not be filtered:

```bash
ros2 launch lr_display_rviz2 display_zed_cam.launch.py \
  camera_model:=zed2i \
  svo_path:=/workspace/svo/zed_20260608_105844_0001.svo2 \
  publish_svo_clock:=true \
  start_segmentation_node:=false
```

You may also omit `start_segmentation_node:=false` when
`segmentation_model_path` is not provided, because the default mode is `auto`
and segmentation will not start without a model path.

## 4. Run with a live camera

Use `svo_path:=live` or omit the `svo_path` argument:

```bash
ros2 launch lr_display_rviz2 display_zed_cam.launch.py \
  camera_model:=zed2i \
  svo_path:=live \
  segmentation_model_path:=/workspace/testros2/landfill-rover/segmentation/best.pt
```

To run without segmentation:

```bash
ros2 launch lr_display_rviz2 display_zed_cam.launch.py \
  camera_model:=zed2i \
  svo_path:=live \
  start_segmentation_node:=false
```

## 5. Run each node separately

Use three terminals. Source ROS 2 and the workspace in all three terminals.

Terminal 1 — ZED:

```bash
ros2 launch zed_wrapper zed_camera.launch.py \
  camera_model:=zed2i \
  svo_path:=/workspace/svo/zed_20260608_105844_0001.svo2 \
  publish_svo_clock:=true
```

Terminal 2 — segmentation:

```bash
ros2 launch lr_segmentation segmentation.launch.py \
  camera_name:=zed \
  model_path:=/workspace/testros2/landfill-rover/segmentation/best.pt \
  use_sim_time:=true
```

Terminal 3 — terrain with object filtering:

```bash
ros2 launch lr_terrain_geometry terrain_geometry.launch.py \
  camera_name:=zed \
  object_filter_enabled:=true \
  use_sim_time:=true
```

If you only want terrain processing, set `object_filter_enabled:=false`.

## 6. Common launch arguments

| Argument | Meaning |
|---|---|
| `camera_model:=zed2i` | ZED camera model |
| `svo_path:=live` | Use the live camera |
| `svo_path:=/path/file.svo2` | Read an SVO file |
| `publish_svo_clock:=true` | Synchronize the clock when playing an SVO |
| `segmentation_model_path:=/path/best.pt` | Automatically enable segmentation with this model |
| `start_segmentation_node:=false` | Explicitly disable segmentation |
| `start_terrain_node:=false` | Do not start terrain geometry |
| `svo_realtime:=false` | Play the SVO as fast as possible |

## 7. Main output topics

Segmentation:

```text
/segmentation/overlay
/segmentation/detections_2d
/segmentation/instance_masks
```

Terrain:

```text
/terrain_geometry/grid_map
/terrain_geometry/markers
/terrain_geometry/heatmap
/terrain_geometry/object_boxes_3d
/terrain_geometry/object_box_markers
```

In RViz, the `Detected Object Boxes` display is inside the `Terrain` group and
uses the `/terrain_geometry/object_box_markers` topic.

## 8. Quick troubleshooting

Check the active topics:

```bash
ros2 topic list | grep -E 'segmentation|terrain_geometry|point_cloud'
```

Check diagnostics:

```bash
ros2 topic echo /diagnostics --once
```

If the model does not run, verify the path:

```bash
test -f /workspace/testros2/landfill-rover/segmentation/best.pt && echo OK
```

When playing an SVO, use `publish_svo_clock:=true` so that all nodes use the
same SVO timestamps and TF extrapolation errors are less likely.
