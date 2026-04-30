# Báo cáo kỹ thuật hiện trạng hệ thống robot hút bụi tự hành ROS 2

**Tác giả:** Cuong  
**Workspace:** `robot_ros2_ws`  
**Gói chính:** `vacuum_driver`  
**Ngày khảo sát mã nguồn:** 30/04/2026  

## Tóm tắt

Báo cáo này trình bày hiện trạng mã nguồn của hệ thống robot hút bụi tự hành xây dựng trên ROS 2. Hệ thống bao gồm firmware Arduino Mega cho điều khiển động cơ, encoder, servo và IMU; driver ROS 2 cho phần cứng thật; driver mô phỏng Webots; pipeline tiền xử lý LiDAR; SLAM bằng `slam_toolbox`; và node tự hành thực hiện khám phá biên bản đồ kết hợp phủ vùng đã đi qua. Mục tiêu hiện tại của hệ thống là tạo bản đồ môi trường bằng LiDAR, ước lượng quỹ đạo bằng encoder/IMU, điều khiển robot vi sai bốn bánh và tự động di chuyển để khám phá, sau đó phủ các vùng có thể tiếp cận.

**Từ khóa:** ROS 2, robot hút bụi, Arduino Mega, SLAM, LiDAR, frontier exploration, coverage path planning, Webots.

## 1. Giới thiệu

Robot hút bụi tự hành cần đồng thời giải quyết các bài toán nhận thức môi trường, định vị, lập bản đồ, lập kế hoạch đường đi, tránh vật cản và điều khiển truyền động. Mã nguồn hiện tại tổ chức hệ thống theo kiến trúc ROS 2, trong đó từng chức năng được tách thành các node độc lập và giao tiếp qua topic, TF và service.

Phạm vi báo cáo này là phân tích hiện trạng code trong workspace, không đánh giá kết quả thực nghiệm ngoài đời nếu chưa có log, rosbag hoặc số liệu đo đi kèm.

## 2. Cấu trúc mã nguồn

Workspace hiện có các thành phần chính:

- `firmware/arduino_mega_base/arduino_mega_base.ino`: firmware Arduino Mega, khoảng 362 dòng.
- `src/vacuum_driver/vacuum_driver/real_driver.py`: driver phần cứng thật, khoảng 768 dòng.
- `src/vacuum_driver/vacuum_driver/pure_driver.py`: driver Webots, khoảng 907 dòng.
- `src/vacuum_driver/vacuum_driver/scan_filter_node.py`: node lọc dữ liệu LiDAR thật, khoảng 173 dòng.
- `src/vacuum_driver/vacuum_driver/slam_session_manager_node.py`: node reset session `slam_toolbox`.
- `src/vacuum_driver/vacuum_driver/autonomous_cleaning_node.py`: node tự hành, khoảng 1479 dòng.
- `src/vacuum_driver/launch/mapping.launch.py`: launch mapping phần cứng thật.
- `src/vacuum_driver/launch/autonomous_mapping.launch.py`: launch mapping kết hợp tự hành.
- `src/vacuum_driver/config/slam.yaml`: cấu hình `slam_toolbox`.
- `src/vacuum_driver/config/real_hardware.yaml`: cấu hình driver phần cứng và lọc scan.
- `src/vacuum_driver/urdf/vacuum_robot.urdf`: mô hình hình học robot cho TF/RViz.

Package ROS 2 được khai báo bằng `ament_python`, có các executable: `pure_driver`, `real_driver`, `scan_filter_node`, `autonomous_cleaning_node`, và `slam_session_manager_node`.

## 3. Kiến trúc hệ thống

Kiến trúc tổng thể có thể mô tả theo chuỗi xử lý sau:

```text
Arduino Mega + encoder + MPU6500
        |
        | Serial STAT / CMD_VEL / PWM
        v
real_driver.py  ---->  /odom, /odom/raw, /imu/data, /imu/data_raw
        |
        v
robot_state_publisher + TF: odom -> base_link -> laser

SLLIDAR ----> /scan/raw ----> scan_filter_node ----> /scan
                                                |
                                                v
                                      slam_toolbox ----> /map, map->odom
                                                |
                                                v
                              autonomous_cleaning_node ----> /cmd_vel
```

Trong mô phỏng, `pure_driver.py` thay thế phần cứng thật bằng Webots, xuất bản `/scan`, `/odom`, `/odom/raw`, `/imu/data` và nhận `/cmd_vel`.

## 4. Phần cứng và firmware Arduino Mega

Firmware Arduino Mega chịu trách nhiệm điều khiển tầng thấp:

- Điều khiển hai cụm motor trái/phải bằng PWM qua các chân `LEFT_RPWM_PIN`, `LEFT_LPWM_PIN`, `RIGHT_RPWM_PIN`, `RIGHT_LPWM_PIN`.
- Đọc encoder trái/phải bằng interrupt trên các chân encoder.
- Đọc IMU MPU6500 qua I2C, gồm gia tốc kế và con quay hồi chuyển.
- Điều khiển 5 servo, trong đó khớp servo số 2 dùng hai servo đối xứng.
- Nhận lệnh từ USB Serial và Bluetooth Serial2.
- Gửi telemetry định kỳ dạng:

```text
STAT,mode,estop,motor_enabled,left_ticks,right_ticks,accelX,accelY,accelZ,gyroX,gyroY,gyroZ,left_pwm,right_pwm
```

Firmware hỗ trợ các lệnh chính:

- `CMD_VEL,v,w`: nhận vận tốc tuyến tính và góc, chuyển sang PWM trái/phải.
- `PWM,left,right`: đặt PWM trực tiếp.
- `ESTOP,0/1`: bật/tắt dừng khẩn cấp.
- `MOTOR,0/1`: bật/tắt motor.
- `SERVO,id,deg`: điều khiển servo.
- `PING`: kiểm tra kết nối.

Cơ chế an toàn tầng thấp hiện có là timeout lệnh sau 500 ms. Nếu không có lệnh mới, firmware tự dừng motor.

## 5. Driver phần cứng thật

`real_driver.py` là cầu nối giữa ROS 2 và Arduino Mega. Node này tự động dò cổng serial, ưu tiên `/dev/rfcomm*`, `/dev/serial/by-id/*`, `/dev/ttyACM*`, `/dev/ttyUSB*`, đồng thời giảm điểm các thiết bị có khả năng là LiDAR.

Các chức năng chính:

- Subscribe `/cmd_vel`, giới hạn vận tốc theo tham số `max_linear_speed_mps=0.16` và `max_angular_speed_radps=0.85`.
- Gửi `CMD_VEL` và tùy chọn gửi thêm `PWM` thô xuống Arduino.
- Parse telemetry `STAT` từ Arduino.
- Publish:
  - `/odom`: odometry dùng cho hệ ROS.
  - `/odom/raw`: odometry encoder thô.
  - `/imu/data` và `/imu/data_raw`: dữ liệu IMU từ MPU6500.
  - `/hardware/mega/telemetry`: dòng telemetry gốc.
  - `/hardware/encoder_ticks`: ticks encoder.
  - `/hardware/motor_pwm`: PWM motor.
- Publish TF `odom -> base_link` nếu `publish_odom_tf=true`.
- Hỗ trợ điều khiển `ESTOP`, `MOTOR`, `SERVO` qua ROS topic.

Odometry được tính theo mô hình vi sai:

```text
dist_left  = delta_left_ticks  * meters_per_tick
dist_right = delta_right_ticks * meters_per_tick
delta_s    = (dist_left + dist_right) / 2
delta_yaw  = (dist_right - dist_left) / wheel_separation
```

Các tham số hình học mặc định:

- Bán kính bánh: `0.0425 m`.
- Khoảng cách hai bên bánh: `0.42 m`.
- Encoder: `1320 ticks/rev`.
- Tốc độ publish odom: `30 Hz`.
- Tốc độ đọc serial: `50 Hz`.
- Tốc độ ghi lệnh: `20 Hz`.

Driver có xử lý phát hiện chiều encoder tự động khi robot nhận lệnh đi thẳng và có delta tick đủ lớn. Ngoài ra, node có cơ chế loại bỏ bước nhảy encoder bất thường bằng `max_tick_delta`.

## 6. LiDAR và tiền xử lý scan

Với phần cứng thật, `mapping.launch.py` chạy `sllidar_ros2` để đọc LiDAR và remap scan thô về `/scan/raw`. Sau đó `scan_filter_node.py` xuất bản dữ liệu đã xử lý sang `/scan`.

Pipeline lọc scan gồm:

- Chuẩn hóa giá trị ngoài dải đo thành `inf`.
- Tùy chọn đảo chiều scan.
- Median filter theo cửa sổ lân cận.
- Temporal filter dùng hệ số trộn `scan_filter_temporal_alpha=0.60`.
- Bỏ qua jump lớn hơn `scan_filter_temporal_jump_threshold_m=0.20`.

Mục tiêu của khâu này là giảm nhiễu điểm đơn lẻ và làm ổn định dữ liệu đưa vào SLAM và node tự hành.

## 7. SLAM và cấu hình mapping

Hệ thống dùng `slam_toolbox` ở chế độ mapping. Cấu hình hiện tại trong `slam.yaml` có các điểm chính:

- Frame:
  - `map_frame: map`
  - `odom_frame: odom`
  - `base_frame: base_link`
  - `scan_topic: /scan`
- Độ phân giải bản đồ: `0.05 m/cell`.
- Khoảng đo laser: `0.05 m` đến `8.0 m`.
- Chu kỳ cập nhật map: `1.0 s`.
- Chu kỳ publish transform: `0.05 s`.
- Scan matching bật.
- Loop closing hiện đang tắt bằng `do_loop_closing: false`.

Node `slam_session_manager_node.py` gọi service reset của `slam_toolbox` sau khi launch, giúp bắt đầu một phiên mapping mới thay vì dùng trạng thái cũ.

## 8. Node tự hành

`autonomous_cleaning_node.py` là thành phần điều phối hành vi di chuyển. Node này nhận:

- `/map`: occupancy grid từ SLAM.
- `/scan`: scan đã lọc.
- TF giữa `map` và `base_link`.

Node xuất:

- `/cmd_vel`: lệnh vận tốc cho driver.
- `/autonomy/path`: đường đi hiện tại.
- `/autonomy/markers`: marker RViz gồm robot, footprint, target, frontier, visited cells và trạng thái.

Chiến lược tự hành gồm hai pha:

1. `EXPLORE`: tìm frontier, tức vùng biên giữa ô đã biết và ô chưa biết.
2. `COVER`: khi bản đồ ổn định hoặc frontier không còn hiệu quả, chọn mục tiêu phủ vùng chưa được đánh dấu visited.

Node xây dựng passability map bằng cách:

- Đọc occupancy grid.
- Xem ô có giá trị lớn hơn `obstacle_threshold` là vật cản.
- Lọc cụm vật cản nhỏ.
- Inflate vật cản theo footprint robot.
- Chỉ lập kế hoạch qua các ô passable.

Lập kế hoạch đường đi dùng A* trên grid 8 hướng, có kiểm tra chuyển động chéo để tránh đi xuyên góc vật cản. Sau khi có path, node có bước làm mượt bằng kiểm tra line-of-sight trên grid.

Điều khiển bám đường dùng waypoint lookahead:

- Xác định waypoint cách robot tối thiểu `path_lookahead_m`.
- Tính heading error.
- Điều khiển góc theo `heading_kp`.
- Giảm tốc khi lệch hướng lớn hoặc scan phía trước gần vật cản.
- Bổ sung hiệu chỉnh side clearance để tránh sát tường.

Node cũng có recovery khi bị kẹt:

- Backup.
- Quay tại chỗ theo phía thông thoáng hơn.
- Drive out ngắn.
- Blacklist target tạm thời nếu không thoát được.

## 9. Mô phỏng Webots

`pure_driver.py` cung cấp driver mô phỏng cho Webots. Node này:

- Kết nối Webots qua `WEBOTS_CONTROLLER_URL`.
- Đọc LiDAR, IMU, encoder mô phỏng.
- Điều khiển motor Webots theo `/cmd_vel`.
- Publish `/scan`, `/scan/raw`, `/odom`, `/odom/raw`, `/imu/data`, `/imu/data_raw`.
- Có pipeline lọc scan tương tự `scan_filter_node`.
- Hỗ trợ odometry bằng encoder, open-loop hoặc static tùy tham số.

Mã có xử lý riêng cho WSL bằng cách suy ra IP Windows host từ default route nếu chưa có `WEBOTS_CONTROLLER_URL`.

## 10. Mô hình robot và TF

URDF mô tả:

- `base_link`: thân robot dạng hộp kích thước `0.35 x 0.42 x 0.06 m`.
- `laser`: LiDAR dạng cylinder, đặt tại `x=-0.10 m`, `z=0.081 m`, yaw `pi`.
- Bốn bánh đặt tại các vị trí:
  - trước trái: `(0.10, 0.21, 0.0)`
  - trước phải: `(0.10, -0.21, 0.0)`
  - sau trái: `(-0.10, 0.21, 0.0)`
  - sau phải: `(-0.10, -0.21, 0.0)`

`robot_state_publisher` đọc URDF để publish transform tĩnh `base_link -> laser` và các link bánh. `real_driver` publish `odom -> base_link`.

## 11. Quy trình chạy hiện tại

Build package:

```bash
cd ~/robot_ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Mapping phần cứng thật:

```bash
ros2 launch vacuum_driver mapping.launch.py
```

Mapping kết hợp tự hành:

```bash
ros2 launch vacuum_driver autonomous_mapping.launch.py
```

Chạy với Arduino USB cụ thể:

```bash
ros2 launch vacuum_driver autonomous_mapping.launch.py arduino_port:=/dev/ttyACM0 arduino_baudrate:=115200
```

Mở RViz:

```bash
rviz2 -d src/vacuum_driver/rviz/mapping.rviz
```

## 12. Đánh giá hiện trạng

Điểm mạnh hiện tại:

- Kiến trúc ROS 2 tách rõ firmware, driver, scan filter, SLAM và autonomy.
- Có song song driver phần cứng thật và driver mô phỏng Webots.
- Có cả dữ liệu raw và dữ liệu đã xử lý cho odom, IMU, scan.
- Driver phần cứng có timeout, reconnect serial, E-stop, motor enable và kiểm tra telemetry.
- Node tự hành đã có frontier exploration, coverage, A*, smoothing, obstacle inflation, stuck detection và recovery.
- Launch file đã gom được pipeline mapping và autonomous mapping.

Hạn chế/rủi ro hiện tại:

- Chưa thấy test tự động cho các thuật toán odometry, scan filter, A* và target selection.
- `README.md` gốc đang là ghi chú lệnh thao tác, chưa phải tài liệu hệ thống hoàn chỉnh.
- `package.xml` và `setup.py` còn `license TODO`, version `0.0.0`.
- `slam_toolbox` đang tắt loop closure, phù hợp chạy đơn giản nhưng có thể tích lũy sai số ở môi trường lớn.
- Một số comment tiếng Việt trong code có dấu bị lỗi encoding hiển thị, ví dụ trong `pure_driver.py`.
- Chưa có dữ liệu benchmark về sai số odometry, chất lượng bản đồ, tỉ lệ phủ vùng hoặc thời gian hoàn thành.
- Tham số hardware README có chỗ nhắc `/dev/rfcomm0` baud `9600`, trong khi launch/config hiện dùng Arduino `115200`; cần thống nhất tài liệu theo cách kết nối thực tế.

## 13. Đề xuất phát triển tiếp

Các bước nên ưu tiên:

1. Thêm test đơn vị cho các hàm thuần: chuẩn hóa góc, chuyển grid/world, A*, scan filter và odometry tick-to-pose.
2. Ghi rosbag thực nghiệm gồm `/scan`, `/odom`, `/map`, `/cmd_vel`, `/tf`, telemetry để đánh giá lại sau mỗi lần chỉnh tham số.
3. Bổ sung bảng thông số phần cứng chuẩn: loại LiDAR, loại encoder, tỉ số truyền, ticks/rev thực, đường kính bánh đo thực tế.
4. Chuẩn hóa README thành tài liệu cài đặt, build, upload firmware, launch và debug.
5. Đánh giá lại cấu hình `do_loop_closing` của `slam_toolbox` theo môi trường chạy thật.
6. Thêm cơ chế lưu map sau khi coverage đạt ngưỡng.
7. Tách bớt `autonomous_cleaning_node.py` nếu tiếp tục mở rộng, ví dụ thành module planner, coverage, recovery và visualization.

## 14. Kết luận

Mã nguồn hiện tại đã hình thành một hệ thống robot hút bụi tự hành tương đối đầy đủ ở mức prototype kỹ thuật: có firmware điều khiển tầng thấp, driver ROS 2 cho phần cứng, mô phỏng Webots, tiền xử lý LiDAR, SLAM và node tự hành để khám phá/phủ vùng. Phần cần củng cố tiếp theo là kiểm thử, tài liệu hóa chuẩn, đo đạc thực nghiệm và chuẩn hóa tham số phần cứng để hệ thống ổn định hơn khi chạy ngoài thực tế.

## Tài liệu tham chiếu nội bộ

- `firmware/arduino_mega_base/arduino_mega_base.ino`
- `src/vacuum_driver/vacuum_driver/real_driver.py`
- `src/vacuum_driver/vacuum_driver/pure_driver.py`
- `src/vacuum_driver/vacuum_driver/scan_filter_node.py`
- `src/vacuum_driver/vacuum_driver/slam_session_manager_node.py`
- `src/vacuum_driver/vacuum_driver/autonomous_cleaning_node.py`
- `src/vacuum_driver/launch/mapping.launch.py`
- `src/vacuum_driver/launch/autonomous_mapping.launch.py`
- `src/vacuum_driver/config/slam.yaml`
- `src/vacuum_driver/config/real_hardware.yaml`
- `src/vacuum_driver/urdf/vacuum_robot.urdf`
