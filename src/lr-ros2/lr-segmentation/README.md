# LR Segmentation

ROS 2 Humble node that runs an Ultralytics segmentation model on ZED RGB and
publishes the rendered RGB overlay.

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

The launch derives the registered ZED RGB input from `camera_name`:

- RGB: `/<camera>/zed_node/rgb/color/rect/image`

It can be overridden with `image_topic`.

## Outputs

- `~/overlay`: rendered `sensor_msgs/msg/Image` with `rgb8` encoding.
