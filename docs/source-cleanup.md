# Dọn source và kiểm chứng — 2026-10-01

Đợt này sửa mapping mask/depth, tránh phát batch objects rỗng khi tất cả vật
thể thiếu depth hoặc chưa được xác nhận, chuyển chờ TF sang retry không chặn,
và bổ sung timeout 3 giây cho request của panel SVO. Diagnostics báo lỗi,
quan sát một phần và output quá hạn; tuổi output dùng ROS time để không tăng
khi pause SVO.

Khi một phần vật thể có depth, pipeline vẫn xuất những box đã dựng và xác nhận
được, đồng thời báo WARN. Schema objects hiện chưa mang cờ độ bao phủ của cả
batch; một danh sách không rỗng không chứng minh đã quan sát đủ mọi vật cản.
Đoạn SVO kiểm tra thực tế có instance không có bất kỳ pixel depth hữu hạn nào.
Vì vậy có thể có khoảng thời gian không phát prediction, thay vì phát một
prediction dựa trên batch rỗng sai nghĩa. Đây là thay đổi hành vi có chủ đích.

Phần geometry, tracking, giải mã/project depth và dựng output được tách khỏi
box node; file node giảm từ 1.813 dòng xuống khoảng 670 dòng. Terrain được tách
thành quản lý cell, kiểu cấu hình/snapshot và plane fitting. Thuật toán physics
trong prediction_core không thay đổi. Các public import cũ của box/terrain,
JsonAdapters và các executable adapter tương thích vẫn được giữ.

Display dùng chung launch của segmentation và terrain. JSON development
adapter nằm trong legacy_json.py. Python requirements của Docker và package
segmentation dùng một nguồn; build context Docker là repository root với danh
sách file được phép trong Dockerfile.dockerignore. README future-path được sửa
để mô tả đúng vai trò làm đầu vào đánh giá prediction offline.

Đã build thành công prediction_ros, lr_segmentation, lr_terrain_geometry và
lr_display_rviz2. Test Python chạy từng package đạt:

| Package | Passed |
|---|---:|
| prediction_core | 15 |
| prediction_ros | 4 |
| lr_prediction_bridge | 12 |
| lr_segmentation | 39 |
| lr_terrain_geometry | 36 |
| lr_future_path | 27 |
| lr_path_prediction | 12 |
| lr_display_rviz2 | 14 |
| Tổng | 159 |

Lượt `colcon test` cuối trên cả 8 package hoàn tất thành công;
`colcon test-result --verbose` báo 202 tests, 0 errors, 0 failures, 3 skipped
(bao gồm các mục test và lint do ament tổng hợp).

12 mục CTest của display đều đạt, bao gồm hai test hành vi Qt chạy offscreen
trong một mục gtest, test launch và các kiểm tra lint. Các ca mới bao phủ
padding YOLO theo hai chiều ảnh, overlap, depth thiếu toàn bộ/một phần,
confirmation, TF đến muộn/hết hạn/reset và diagnostics lỗi/phục hồi/stale.
Các script Docker qua kiểm tra cú pháp, phân giải requirements include và
kiểm tra tham số với Docker stub; chưa build lại image Docker/Jetson.

Replay headless trên zed_20260710_092420_0001.svo2 với model models/best.pt đã
xác nhận đủ 7 node LR, pause dừng clock/prediction, resume phục hồi, rewind
xóa gia tốc/markers, trajectory ID tiếp tục tăng và prediction phục hồi.
Kết quả lưu ở [cleanup-replay-results.json](cleanup-replay-results.json).
Log đầy đủ nằm trong log/cleanup-replay-nonrealtime-long/ của workspace.

Lượt realtime tạo được 102 prediction nhưng không kiểm tra được pause vì
wrapper đang cài không advertise toggle_svo_pause trong chế độ này. Lượt
không realtime qua pause/resume; lần đầu vượt giới hạn 30 giây chờ phục hồi
sau rewind. Lượt sau dùng giới hạn 60 giây và qua toàn bộ kiểm tra. Chưa xác
nhận thao tác trên GUI RViz thật; panel được kiểm tra bằng Qt offscreen.

Chạy lại kiểm tra replay bằng script có thêm lựa chọn clock và thời gian chờ:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=83 python3 scripts/verify_prediction_replay.py \
  --headless --svo-realtime false --recovery-timeout 60 \
  --svo /zed/svo/zed_20260710_092420_0001.svo2 \
  --model models/best.pt --output log/cleanup-replay
```

Tối ưu CPU được đo riêng với đầu vào tổng hợp, median 25 lượt sau warmup,
không chạy replay/build/test đồng thời. Baseline là commit
b93db398f0d2fffb86ea9c53209a90d78a3774e4. Terrain grouping giữ nguyên chính xác
array mẫu của từng cell; depth grouping giữ nguyên chính xác array điểm của
từng instance, bao gồm thứ tự mẫu và giới hạn số điểm.

| Phần xử lý | Dữ liệu | Trước | Sau |
|---|---|---:|---:|
| Tích lũy/gom terrain | 12.000 điểm, 324 cell | 18,32 ms | 11,51 ms |
| Gom/project depth | 1920×1080, 12 instance | 36,71 ms | 15,63 ms |

Số liệu từ [cleanup-kernel-results.json](cleanup-kernel-results.json), tương
ứng giảm khoảng 37% và 57% thời gian của hai đoạn xử lý này. Đây không phải
mức tăng FPS toàn pipeline. Môi trường: ROS Humble, Python 3.10, NumPy 2.2.6,
Torch 2.14.0, Ultralytics 8.4.153, OpenCV 5.0.0.93; không đổi phiên bản thư viện
trong đợt dọn này. Chạy lại từ workspace đã source:

```bash
python3 scripts/benchmark_perception_kernels.py \
  --baseline-ref b93db398f0d2fffb86ea9c53209a90d78a3774e4 \
  --output log/cleanup-kernel-results.json
```
