# LR Segmentation

ROS 2 Humble Python node for image-only ZED instance segmentation. It does not
import `landfill-rover`, `pyzed`, or its checkpoint.

The node consumes the rectified RGB image, runs Ultralytics YOLO, and merges
compatible same-class mask fragments using only image-space IoU, gap, and
dilation. Only the newest pending frame is retained when inference is slower
than the camera.

## Dependencies

Install ROS dependencies with `rosdep`, then install the Python inference
runtime:

```bash
python3 -m pip install -r src/lr-ros2/lr-segmentation/requirements.txt
```

The checkpoint is deliberately external. An invalid path or missing
Torch/Ultralytics dependency makes the node fail at startup with a clear error.

## Run

Against an already-running ZED wrapper:

```bash
ros2 launch lr_segmentation segmentation.launch.py \
  camera_name:=zed \
  model_path:=/workspace/models/best.pt
```

With the combined ZED, terrain, and RViz launch:

```bash
ros2 launch lr_display_rviz2 display_zed_cam.launch.py \
  camera_model:=zed2i \
  svo_path:=/workspace/svo/recording.svo2 \
  start_segmentation_node:=true \
  segmentation_model_path:=/workspace/models/best.pt
```

Segmentation is disabled by default in the combined launch. When enabled with
a model path, the existing RGB dock displays `/segmentation/overlay`. When
disabled, launch remaps that dock directly to the original ZED RGB topic.

## ROS interface

- Input: `input.image_topic`.
- Outputs: `~/overlay`, `~/detections_2d`, `~/instance_masks`, and
  `/diagnostics`.
- Service: `~/reset` clears the pending image and current detections.

`~/instance_masks` is a `sensor_msgs/Image` with `mono16` encoding. Label `0`
is background and label `i + 1` corresponds to detection `i` in the
same-stamped `Detection2DArray`.

All algorithm parameters are read-only after startup and live in
`config/segmentation.yaml`. The segmentation node remains image-only and has
no point-cloud subscription or 3D output. In the combined launch, terrain can
consume its stamped detections and instance masks, infer cloud-frame oriented
3D boxes, and remove those object volumes before fitting terrain.
