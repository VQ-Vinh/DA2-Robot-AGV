# RasPi – ROS 2 cho AGV kho hàng

ROS 2 **Jazzy** (Ubuntu 24.04) + **Gazebo Harmonic**. Chạy mô phỏng trên PC (WSL), sau này chạy node thật trên Raspberry Pi 4.

| Package | Nội dung |
|---|---|
| `agv_description` | URDF/xacro xe skid-steer 4 bánh, lidar, IMU; cấu hình RViz |
| `agv_gazebo` | World kho hàng, bridge Gazebo ↔ ROS 2, launch mô phỏng |

## Tạo workspace (một lần)

Code nằm trong repo; workspace trong WSL chỉ symlink tới đây, nên sửa file ở Windows là WSL thấy ngay.

```bash
mkdir -p ~/agv_ws/src
ln -sfn /mnt/d/MyBK/HK261/DA2/Src/RasPi/src/agv_description ~/agv_ws/src/agv_description
ln -sfn /mnt/d/MyBK/HK261/DA2/Src/RasPi/src/agv_gazebo      ~/agv_ws/src/agv_gazebo
cd ~/agv_ws && colcon build --symlink-install
source ~/agv_ws/install/setup.bash
```

Build lại chỉ khi thêm file mới. Sửa file có sẵn (`.xacro`, `.sdf`, `.yaml`, `.py`) thì không cần build lại vì đã dùng `--symlink-install`.

## Chạy

```bash
ros2 launch agv_gazebo sim.launch.py    # Gazebo (có bảng Teleop) + RViz + rqt_robot_steering
```

Lái xe (chọn một):

| Cách | Dùng cho | Ghi chú |
|---|---|---|
| Bảng **Teleop** bên phải Gazebo | Chỉ mô phỏng | Nút mũi tên, thanh trượt, hoặc bật *Keyboard* rồi dùng W/A/S/D |
| Cửa sổ **rqt_robot_steering** | Mô phỏng + xe thật | 2 thanh trượt. Lần đầu **bỏ tick ô "stamped"** (bridge nhận `Twist`), rqt sẽ nhớ |
| `ros2 run teleop_twist_keyboard teleop_twist_keyboard` | Mô phỏng + xe thật | Bàn phím trong terminal |

Tham số: `rviz:=false`, `steering:=false`, `headless:=true` (không mở cửa sổ nào), `x:= y:= yaw:=` (vị trí xuất phát), `rpm_min:=0` (tắt vùng chết động cơ, xem mục dưới).

Chỉ xem model, không cần Gazebo: `ros2 launch agv_description display.launch.py`

## Mô hình động cơ (`motor_model`)

DiffDrive của Gazebo quay bánh đúng tốc độ được yêu cầu, kể cả rất chậm. Xe thật thì không: firmware STM32 ép mọi lệnh khác 0 vào khoảng **100–270 RPM** (`SPD_RPM_MIN/MAX` trong `STM32/App/Inc/speed_ctrl.h`), vì dưới ~30 % duty động cơ không quay nổi. Node `motor_model` đứng giữa `/cmd_vel` và Gazebo, làm y hệt firmware:

```
teleop / Nav2 -> /cmd_vel -> motor_model -> /cmd_vel_limited -> DiffDrive (gz /model/agv/cmd_vel)
```

1. Bánh vượt 270 RPM: thu nhỏ cả hai bên cùng tỉ lệ (giữ độ cong đường đi).
2. Bánh khác 0 mà dưới 100 RPM: đẩy lên 100 RPM (= 0.34 m/s).

Hệ quả cần nhớ khi chỉnh Nav2:

| Xin | Xe thật chạy |
|---|---|
| tiến 0.10 m/s | tiến **0.34 m/s** |
| quay tại chỗ 0.5 rad/s | quay **1.51 rad/s** |
| v = 0.3, ω = 0.6 (bán kính 0.5 m) | v = 0.39, ω = 0.21 (bán kính **1.9 m**) |
| tiến 1.5 m/s | tiến 0.92 m/s (tối đa) |

Bảng Teleop trong Gazebo cũng đi qua `motor_model` (bridge chuyển gz `/cmd_vel` sang ROS `/cmd_vel`).

## Đo sai số odom

`/ground_truth` là vị trí thật của xe trong world (chỉ có trong mô phỏng). So sánh với odom:

```bash
ros2 run agv_gazebo odom_drift.py                                         # /odom bánh xe
ros2 run agv_gazebo odom_drift.py --ros-args -p odom_topic:=/odometry/filtered
```

Lái xe một lúc rồi Ctrl+C, node in sai số vị trí, sai số góc và % quãng đường. Lái thử ~5.5 m có quay tại chỗ và chạy vòng: odom bánh xe lệch **0.30 m (5.4 %)** và **góc lệch tới 21°**. Nguyên nhân chính là bánh trượt ngang khi skid-steer quay, nên cần IMU để sửa góc.

## Topic

| Topic | Kiểu | Chiều |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | ROS → `motor_model` (và Teleop Gazebo → ROS) |
| `/cmd_vel_limited` | `geometry_msgs/Twist` | `motor_model` → Gazebo DiffDrive |
| `/ground_truth` | `nav_msgs/Odometry` | Gazebo → ROS, frame `world`, chỉ để đánh giá |
| `/odom`, `/tf` (odom → base_footprint) | `nav_msgs/Odometry`, `tf2_msgs/TFMessage` | Gazebo → ROS |
| `/scan` (7 Hz, 720 tia, 0.12–10 m) | `sensor_msgs/LaserScan` | Gazebo → ROS |
| `/imu` (100 Hz) | `sensor_msgs/Imu` | Gazebo → ROS |
| `/joint_states`, `/clock` | | Gazebo → ROS |

## Kích thước xe

Đặt ở đầu `agv_description/urdf/agv.urdf.xacro`: thân 2 tầng mica 30 × 30 cm cách nhau 10 cm, 4 trụ ốc ở góc, linh kiện ở tầng 1, lidar ở tầng 2 (mặt quét cao ~17 cm), bánh 65 mm. Giá trị đánh dấu `(*)` là ước lượng (track 0.34 m, wheelbase 0.20 m, độ dày mica...), cần đo xe thật rồi sửa.

Lidar quét ở ~17 cm nên **không thấy vật thấp hơn** (pallet 15 cm trong world là ví dụ) – cần camera/cảm biến gần sàn hoặc vùng cấm trên bản đồ.

`effective_track` (0.45 m) là track hiệu dụng cho skid-steer: bánh trượt ngang khi quay nên xe quay ít hơn tính theo `track_width`. Hiệu chỉnh bằng cách cho xe quay tại chỗ rồi đặt `effective_track` mới = `effective_track` hiện tại × `góc_odom / góc_thật`.
