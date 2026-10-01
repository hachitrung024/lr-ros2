# LR Segmentation

ROS 2 Humble nodes that run an Ultralytics segmentation model on ZED RGB,
publish a rendered overlay and instance-label image, then estimate tracked 3D
boxes and map-frame XY footprints from the masks and registered depth.

## Install

```bash
python3 -m pip install -r src/lr-ros2/lr-segmentation/requirements.txt
```

## Run

```bash
ros2 launch lr_segmentation segmentation.launch.py \
  camera_name:=zed \
  model_path:=/path/to/best.pt
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
- `/segmentation/tracked_footprints`:
  `safety_perception_msgs/msg/TrackedObjectArray`. Each object contains the
  mask point cloud's concave XY occupancy boundary and the same tracker ID as
  its 3D box.
- `/segmentation/box_markers`: `visualization_msgs/msg/MarkerArray` for RViz
  only; it contains translucent vertical polygon prisms, prism wireframes, and
  tracker IDs. Their bottom and top use robust Z limits from the mask points.

The default wireframe is 0.07 m thick and uses Reliable QoS. Its width, fill
alpha, and label size are configurable with the `visualization.*` parameters in
`config/segmentation.yaml`.

The estimator synchronizes mask and depth within 50 ms. It filters depth
outliers, fits an oriented box, projects the mask points onto a 2 cm XY grid,
closes small depth gaps, keeps the largest connected component, and simplifies
its concave boundary. A box footprint is used if the projected visible surface
is degenerate. Both shapes follow a constant-velocity Kalman track. Rectangular
box markers are hidden by default with `visualization.show_boxes: false`; the
`/segmentation/boxes_3d` compatibility topic remains available. The default
output frame is `map`; set
`box_frame:=<camera_optical_frame>` and configure `geometry.up_axis` if no
world-frame TF is available.

Mask labels are mapped back to the original image after removing YOLO letterbox
padding. Instance overlap is resolved at inference resolution before resizing
the single label image. Depth validity and intrinsics are evaluated once per
frame, then valid pixels are grouped by instance for projection.

A nonempty mask whose instances all lack sufficient depth does not produce an
empty objects observation. The same applies while every measured object is
still awaiting tracker confirmation. A mask with no instances can produce a
valid empty observation. If only some instances can be reconstructed, their
confirmed boxes are retained and diagnostics report a partial observation;
the resulting object list does not imply complete obstacle coverage.

Timestamped TF lookup is nonblocking. A matched pair waits up to
`tf_timeout_sec` (0.2 s by default), with one pending pair and bounded input
buffers. Later pairs cannot overtake it. Expiry and clock rewind discard the
pending pair without publishing synthetic free-space evidence.

Both nodes report unavailable/stale output on `/diagnostics`.
`diagnostics.stale_after_sec` defaults to 2 s and uses ROS time, so pausing an
SVO does not age a successful observation. Box diagnostics also report
`invalid_pairs`, `unobservable_instances`, and `tf_failures`.

The implementation separates ROS orchestration (`box_estimator_3d`) from
geometry (`box_geometry`), tracking (`box_tracking`), image decoding/projection
(`depth_sampling`), and output rendering (`box_outputs`). Historical public
box/fitting imports from `box_estimator_3d` remain available.
