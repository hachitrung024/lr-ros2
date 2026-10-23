# Chạy ROS 2 trên Jetson và host

- Hai máy dùng ROS 2 Humble, container tên `zed-dev`, repo trong container ở `/zed/lr-ros2`. SSH tới Jetson bằng `ssh jetson`.
- Host chạy nguồn SVO2/live bằng `lr_zed_stream` và RViz; Jetson nhận ZED Streaming NET và chạy backend headless. Cả hai container dùng mạng host, `ROS_DOMAIN_ID=42`, `ROS_LOCALHOST_ONLY=0`.

## Dev: đồng bộ source từ host sang Jetson

Chạy tại gốc repo **trên host** sau khi sửa code. Git liệt kê file đã theo dõi và file mới không bị `.gitignore` loại trừ, chỉ trong `src/lr-ros2`. Xem trước rồi mới đồng bộ; không gửi cache/build, SVO, model và không xóa file trên Jetson.

```bash
git ls-files -co --exclude-standard -z -- src/lr-ros2/ | \
  rsync -avhn --from0 --files-from=- ./ jetson:/home/hachi/Projects/zed/lr-ros2/
git ls-files -co --exclude-standard -z -- src/lr-ros2/ | \
  rsync -avh --from0 --files-from=- ./ jetson:/home/hachi/Projects/zed/lr-ros2/
```

Sau khi đồng bộ, vào `zed-dev` trên Jetson, chạy `cd /zed/lr-ros2`, `source /opt/ros/humble/setup.bash`, rồi `colcon build --symlink-install --packages-up-to lr_display_rviz2` và `source install/setup.bash` trước khi chạy lại backend. Nếu đang chạy ROS, dừng launch cũ trước khi build. Container host mount trực tiếp source local nên không cần rsync ngược; build lại trên host khi sửa package UI hoặc message.

Trên host, build cả nguồn phát và UI bằng `colcon build --symlink-install --packages-up-to lr_zed_stream lr_display_rviz2`. Nguồn phát dùng `pyzed.sl` của ZED SDK 5.4.1. SVO2 nằm trên host; model và MAVLink DB nằm trên Jetson.

## Host: terminal nguồn phát

```bash
docker start zed-dev 2>/dev/null || true
docker exec -it zed-dev bash
cd /zed/lr-ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0
ros2 launch lr_zed_stream stream_sender.launch.py \
  svo_path:=/zed/svo/zed_20260710_092420_0001.svo2 \
  port:=30000 codec:=H264 bitrate:=20000 gop_size:=1
```

Giữ nguồn đang phát khi Jetson mở kết nối NET; SDK 5.4.1 cần frame tiến lên để thiết lập kết nối. Bản NET hiện nghiệm thu phát tuần tự; pause/seek chưa đạt trên SDK 5.4.1 và mặc định tắt. Chỉ bật `enable_svo_controls:=true` khi thử nghiệm. `/clock` và trạng thái SVO thuộc `/zed_source/zed_node`. Live camera dùng `svo_path:=live`; có thể chọn `serial_number`, `camera_id`, `resolution`, `fps`.

## Jetson: terminal backend

```bash
ssh jetson
docker start zed-dev 2>/dev/null || true
docker exec -it zed-dev bash
cd /zed/lr-ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0
ros2 launch lr_display_rviz2 headless_zed_cam.launch.py \
  camera_model:=zed2i stream_address:=10.42.0.1 stream_port:=30000 \
  stream_svo_mode:=true \
  segmentation_model_path:=/zed/lr-ros2/models/best.pt \
  mavlink:=true mavlink_dir:=/zed/lr-ros2/mavlink future_path:=true
```

## Host hiện tại: terminal RViz

```bash
docker start zed-dev 2>/dev/null || true
docker exec -it zed-dev bash
cd /zed/lr-ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0
ros2 topic info /clock --verbose  # chỉ một publisher: /zed_source/zed_node
ros2 launch lr_display_rviz2 rviz_zed_cam.launch.py \
  camera_model:=zed2i use_sim_time:=true svo_mode:=false \
  svo_control_namespace:=/zed_source/zed_node \
  segmentation_enabled:=true
```

Jetson nhận NET giữ `svo_path:=live`, `publish_svo_clock:=false`; không cần file SVO trên Jetson. Với live camera, backend dùng `stream_svo_mode:=false`, bỏ MAVLink/future path; UI dùng `use_sim_time:=false svo_mode:=false`.

Nếu không thấy `/clock`, kiểm tra nguồn SVO trên host. Nếu Jetson không thấy dữ liệu, kiểm tra domain, mạng host, DDS UDP/multicast và ZED NET UDP cổng chẵn `30000`. Không chạy `display_zed_cam.launch.py` trên host vì lệnh đó khởi động thêm backend. NET đưa ảnh vào Jetson; ảnh/point cloud/kết quả RViz chiều ngược lại vẫn đi DDS. Cách chạy SVO trực tiếp cũ vẫn có trong [README.md](README.md) và [docs/distributed-rviz.md](docs/distributed-rviz.md).
