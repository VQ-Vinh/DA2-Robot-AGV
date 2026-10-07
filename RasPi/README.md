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

Tham số: `rviz:=false`, `steering:=false`, `headless:=true` (không mở cửa sổ nào), `x:= y:= yaw:=` (vị trí xuất phát).

Chỉ xem model, không cần Gazebo: `ros2 launch agv_description display.launch.py`

## Topic

| Topic | Kiểu | Chiều |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | ROS → Gazebo |
| `/odom`, `/tf` (odom → base_footprint) | `nav_msgs/Odometry`, `tf2_msgs/TFMessage` | Gazebo → ROS |
| `/scan` (7 Hz, 720 tia, 0.12–10 m) | `sensor_msgs/LaserScan` | Gazebo → ROS |
| `/imu` (100 Hz) | `sensor_msgs/Imu` | Gazebo → ROS |
| `/joint_states`, `/clock` | | Gazebo → ROS |

## Kích thước xe

Đặt ở đầu `agv_description/urdf/agv.urdf.xacro`: thân 2 tầng mica 30 × 30 cm cách nhau 10 cm, 4 trụ ốc ở góc, linh kiện ở tầng 1, lidar ở tầng 2 (mặt quét cao ~17 cm), bánh 65 mm. Giá trị đánh dấu `(*)` là ước lượng (track 0.34 m, wheelbase 0.20 m, độ dày mica...), cần đo xe thật rồi sửa.

Lidar quét ở ~17 cm nên **không thấy vật thấp hơn** (pallet 15 cm trong world là ví dụ) – cần camera/cảm biến gần sàn hoặc vùng cấm trên bản đồ.

`effective_track` (0.45 m) là track hiệu dụng cho skid-steer: bánh trượt ngang khi quay nên xe quay ít hơn tính theo `track_width`. Hiệu chỉnh bằng cách cho xe quay tại chỗ rồi đặt `effective_track` mới = `effective_track` hiện tại × `góc_odom / góc_thật`.
