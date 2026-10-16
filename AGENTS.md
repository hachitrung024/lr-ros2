# Chạy ROS 2 trên Jetson và host

- Hai máy dùng ROS 2 Humble, container tên `zed-dev`, repo trong container ở `/zed/lr-ros2`. SSH tới Jetson bằng `ssh jetson`.
- Chỉ chạy backend headless trên Jetson và RViz UI trên host; cả hai container dùng mạng host, `ROS_DOMAIN_ID=42`, `ROS_LOCALHOST_ONLY=0`.

## Dev: đồng bộ source từ host sang Jetson

Chạy tại gốc repo **trên host** sau khi sửa code. Git liệt kê file đã theo dõi và file mới không bị `.gitignore` loại trừ, chỉ trong `src/lr-ros2`. Xem trước rồi mới đồng bộ; không gửi cache/build, SVO, model và không xóa file trên Jetson.

```bash
git ls-files -co --exclude-standard -z -- src/lr-ros2/ | \
  rsync -avhn --from0 --files-from=- ./ jetson:/home/hachi/Projects/zed/lr-ros2/
git ls-files -co --exclude-standard -z -- src/lr-ros2/ | \
  rsync -avh --from0 --files-from=- ./ jetson:/home/hachi/Projects/zed/lr-ros2/
```

Sau khi đồng bộ, vào `zed-dev` trên Jetson, chạy `cd /zed/lr-ros2`, `source /opt/ros/humble/setup.bash`, rồi `colcon build --symlink-install --packages-up-to lr_display_rviz2` và `source install/setup.bash` trước khi chạy lại backend. Nếu đang chạy ROS, dừng launch cũ trước khi build. Container host mount trực tiếp source local nên không cần rsync ngược; build lại trên host khi sửa package UI hoặc message.

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
  camera_model:=zed2i publish_svo_clock:=true \
  segmentation_model_path:=/zed/lr-ros2/models/best.pt \
  mavlink:=true mavlink_dir:=/zed/lr-ros2/mavlink future_path:=true \
  svo_path:=/zed/svo/zed_20260710_092420_0001.svo2
```

## Host hiện tại: terminal RViz

```bash
docker start zed-dev 2>/dev/null || true
docker exec -it zed-dev bash
cd /zed/lr-ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0
ros2 topic echo /clock --once  # kiểm tra đã thấy Jetson
ros2 launch lr_display_rviz2 rviz_zed_cam.launch.py \
  camera_model:=zed2i use_sim_time:=true svo_mode:=true \
  segmentation_enabled:=true
```

Nếu không thấy `/clock`, kiểm tra hai container đang chạy, cùng domain, mạng host và DDS UDP/multicast giữa hai máy. Không chạy `display_zed_cam.launch.py` trên host vì lệnh đó khởi động thêm backend. Chi tiết: [README.md](README.md) và [docs/distributed-rviz.md](docs/distributed-rviz.md).
