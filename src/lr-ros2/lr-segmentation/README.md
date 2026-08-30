# LR Segmentation

ROS 2 Humble node that runs an Ultralytics segmentation model on ZED RGB,
renders the RGB overlay, and estimates simple 3D boxes from the model's 2D
boxes plus the registered depth image.

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

The launch derives these registered ZED inputs from `camera_name`:

- RGB: `/<camera>/zed_node/rgb/color/rect/image`
- Depth: `/<camera>/zed_node/depth/depth_registered`
- Intrinsics: `/<camera>/zed_node/rgb/color/rect/camera_info`

They can be overridden with `image_topic`, `depth_topic`, and
`camera_info_topic`.

## Outputs

- `~/overlay`: rendered `sensor_msgs/msg/Image` with `rgb8` encoding.
- `~/boxes_3d`: `visualization_msgs/msg/MarkerArray` wireframe boxes in the
  depth optical frame.

For each 2D detection, the node estimates the robust 1st-to-99th percentile
depth span in the center of the box, keeps pixels in the full box near that
span, back-projects them with `CameraInfo`, rejects the outer 5% as outliers,
and fits an axis-aligned box. This is deliberately a simple approximation: a
segmentation mask or point-cloud clustering will be more accurate when a 2D
box contains much background.
