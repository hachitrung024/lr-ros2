# LR Segmentation

Minimal ROS 2 Humble node that reads a ZED color image, runs an Ultralytics
segmentation model, and publishes the rendered RGB overlay.

## Install

```bash
python3 -m pip install -r src/lr-ros2/lr-segmentation/requirements.txt
```

The YOLO segmentation checkpoint stays external and must exist when the node
starts.

## Run

```bash
ros2 launch lr_segmentation segmentation.launch.py \
  camera_name:=zed \
  model_path:=/workspace/testros2/models/best.pt
```

## ROS interface

- Input parameter: `input.image_topic` (`sensor_msgs/msg/Image`).
- Only output: `~/overlay` (`sensor_msgs/msg/Image`, encoding `rgb8`).

The remaining model parameters are in `config/segmentation.yaml`. The node has
no detections, masks, diagnostics, reset service, point-cloud input, merging,
or 3D processing.
