# Landfill Rover ROS 2

## Lệnh chạy đầy đủ

Lệnh dưới đây chạy SVO, tự tìm session MAVLink tương ứng, dùng MAVLink thay TF
động của ZED, phát future path, chạy segmentation, terrain và RViz:

```bash
ros2 launch lr_display_rviz2 display_zed_cam.launch.py \
  start_zed_node:=true \
  camera_name:=zed \
  camera_model:=zed2i \
  svo_path:=/path/to/recording.svo2 \
  svo_realtime:=true \
  publish_svo_clock:=true \
  mavlink:=true \
  mavlink_dir:=/path/to/mavlink \
  mavlink_db_path:='' \
  mavlink_pose_topic:=/lr/mavlink/pose \
  mavlink_match_tolerance_s:=60.0 \
  mavlink_max_gps_gap_s:=1.5 \
  mavlink_body_to_camera:='[0,0,0,0,0,0]' \
  future_path:=true \
  future_path_topic:=/lr/future_path/ground_truth \
  future_path_radius_m:=15.0 \
  future_path_step_m:=0.2 \
  future_path_cache_dir:=/path/to/future_path_cache \
  future_path_rebuild_cache:=false \
  start_segmentation_node:=auto \
  segmentation_model_path:=/path/to/best.pt \
  segmentation_params_file:=/path/to/segmentation.yaml \
  start_terrain_node:=true \
  terrain_params_file:=/path/to/terrain_geometry.yaml \
  map_frame:=map
```

`mavlink_db_path:=''` nghĩa là tự tìm đệ quy các file `session_mavlink.db`
trong `mavlink_dir`. Để chọn trực tiếp một database, dùng:

```bash
mavlink_db_path:=/path/to/session_mavlink.db
```

Khi `mavlink:=true`, future path được đọc trực tiếp từ database MAVLink nên
`future_path_cache_dir` và `future_path_rebuild_cache` không được sử dụng.

## Build

```bash
cd /path/to/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select \
  lr_future_path \
  lr_segmentation \
  lr_terrain_geometry \
  lr_display_rviz2
source install/setup.bash
```

## Các chế độ thường dùng

### SVO dùng ZED pose, không dùng MAVLink

```bash
ros2 launch lr_display_rviz2 display_zed_cam.launch.py \
  camera_model:=zed2i \
  svo_path:=/path/to/recording.svo2 \
  publish_svo_clock:=true \
  mavlink:=false \
  future_path:=false \
  segmentation_model_path:=/path/to/best.pt
```

### Future path từ ZED VIO

```bash
ros2 launch lr_display_rviz2 display_zed_cam.launch.py \
  camera_model:=zed2i \
  svo_path:=/path/to/recording.svo2 \
  publish_svo_clock:=true \
  mavlink:=false \
  future_path:=true \
  future_path_cache_dir:=/path/to/future_path_cache
```

Lần chạy đầu sẽ tạo rosbag2 cache pose từ SVO. Các lần sau dùng lại cache.

### Camera live

```bash
ros2 launch lr_display_rviz2 display_zed_cam.launch.py \
  camera_model:=zed2i \
  svo_path:=live \
  mavlink:=false \
  future_path:=false \
  segmentation_model_path:=/path/to/best.pt
```

MAVLink và future ground-truth path chỉ hỗ trợ SVO, không hỗ trợ camera live.

## Output chính

```text
/lr/mavlink/pose
/lr/future_path/ground_truth
/segmentation/overlay
/segmentation/boxes_3d
/terrain_geometry/grid_map
/terrain_geometry/markers
/terrain_geometry/heatmap
```

MAVLink phát TF `map -> zed_camera_link` khi `camera_name:=zed`. Nếu không tìm
được đúng một session MAVLink khớp timestamp SVO, toàn bộ launch sẽ dừng và
không tự fallback sang ZED TF.

`/lr/future_path/ground_truth` là quỹ đạo thực tế tương lai để hiển thị và
đánh giá offline, không phải planned path và không được dùng cho điều khiển
rover.
