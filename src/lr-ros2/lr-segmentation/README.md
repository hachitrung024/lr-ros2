# LR Segmentation

ROS 2 Humble nodes that run an Ultralytics segmentation model on ZED RGB,
publish a rendered overlay and instance-label image, then estimate tracked,
light-weight oriented 3D boxes from the masks and registered depth.

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
`camera_info_topic`. Set `start_box_estimator_3d:=false` to run only the
image-domain segmentation node.

## Outputs

- `~/overlay`: rendered `sensor_msgs/msg/Image` with `rgb8` encoding.
- `~/instance_mask`: `sensor_msgs/msg/Image` with `mono16` encoding. Zero is
  background; positive values are per-frame instance IDs.
- `/segmentation/boxes_3d`: `vision_msgs/msg/Detection3DArray`. Every detection
  contains an oriented metric `BoundingBox3D` and a stable tracker ID.
- `/segmentation/box_markers`: `visualization_msgs/msg/MarkerArray` for RViz
  only; it contains transparent box cubes and their tracker IDs.

The estimator synchronizes mask and depth within 50 ms. It filters depth
outliers, fits an oriented box with a gravity-aligned vertical axis, and applies
a constant-velocity Kalman filter. The default output frame is `map`; set
`box_frame:=<camera_optical_frame>` and configure `geometry.up_axis` if no
world-frame TF is available.
