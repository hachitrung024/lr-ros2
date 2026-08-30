# LR Segmentation

ROS 2 Humble nodes that run an Ultralytics segmentation model on ZED RGB,
publish a rendered overlay and instance-label image, then project that mask
through registered depth into a labeled 3D point cloud.

## Install

```bash
python3 -m pip install -r src/lr-ros2/lr-segmentation/requirements.txt
```

## Run

```bash
ros2 launch lr_segmentation segmentation.launch.py \
  camera_name:=zed \
  model_path:=/workspace/testros2/models/best.pt
```

The launch derives registered ZED inputs from `camera_name`:

- RGB: `/<camera>/zed_node/rgb/color/rect/image`
- Depth: `/<camera>/zed_node/depth/depth_registered`
- Intrinsics: `/<camera>/zed_node/rgb/color/rect/camera_info`

They can be overridden with `image_topic`, `depth_topic`, and
`camera_info_topic`. Set `start_mask_projector_3d:=false` to run only the
image-domain segmentation node.

## Outputs

- `~/overlay`: rendered `sensor_msgs/msg/Image` with `rgb8` encoding.
- `~/instance_mask`: `sensor_msgs/msg/Image` with `mono16` encoding. Zero is
  background; positive values are per-frame instance IDs.
- `/segmentation/mask_cloud`: `sensor_msgs/msg/PointCloud2` in the registered
  depth frame with `x`, `y`, `z`, `rgb`, and `instance_id` fields.

The projector synchronizes mask and depth within 50 ms. Invalid depth is
discarded, output is capped at 200,000 points, and RViz colors each instance
with a stable per-frame palette.
