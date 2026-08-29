# LR Terrain Geometry

ROS 2 Humble Python node that accumulates ZED point clouds into world-aligned
cells and estimates local terrain planes. The package uses REP-103 coordinates
(`X` forward, `Y` left, `Z` up) and has no dependency on the ZED SDK Python API.

```bash
ros2 launch lr_terrain_geometry terrain_geometry.launch.py camera_name:=zed
```

The node publishes `~/grid_map`, `~/markers`, `~/heatmap`, `/diagnostics`, and
exposes the `~/reset` service. The heatmap is an `rgb8` image intended for an
RViz Image dock. Processing parameters are read-only after startup and are
configured in `config/terrain_geometry.yaml`.

## Detection-aware object filtering

When `object_filter.enabled` is true, terrain synchronizes each point cloud
with a `vision_msgs/Detection2DArray` and a `mono16` instance-mask image by
source timestamp. XYZ points are projected into the rectified RGB image using
`CameraInfo` and TF. Only points carrying the corresponding instance label are
used to select the nearest supported depth cluster. A robust yaw-oriented 3D
box is then fit to that cluster, and points in its volume are
removed before terrain accumulation and fitting.

The inferred cloud-frame boxes are published on `~/object_boxes_3d` as a
`vision_msgs/Detection3DArray`. RViz wireframes and class-confidence labels are
published on `~/object_box_markers`; the combined display launch shows them in
the Terrain group by default. The combined launch enables filtering
automatically whenever `start_segmentation_node:=true`. For a standalone node:

```bash
ros2 launch lr_terrain_geometry terrain_geometry.launch.py \
  camera_name:=zed \
  object_filter_enabled:=true
```

An empty `object_filter.classes` list filters every detected class. Set a list
of class names in `config/terrain_geometry.yaml` to restrict removal. If no
timestamp-matched detection is available, the cloud is bounded in the sync
cache and is not sent unfiltered into the terrain estimator.
