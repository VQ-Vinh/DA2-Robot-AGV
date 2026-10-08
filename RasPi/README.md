# RasPi – ROS 2 cho AGV kho hàng

ROS 2 **Jazzy** (Ubuntu 24.04) + **Gazebo Harmonic**. Chạy mô phỏng trên PC (WSL), sau này chạy node thật trên Raspberry Pi 4.

| Package | Nội dung |
|---|---|
| `agv_description` | URDF/xacro xe skid-steer 4 bánh, lidar, IMU; cấu hình RViz |
| `agv_gazebo` | World kho hàng kiểu Kiva (sinh từ `shelves.yaml`), bridge Gazebo ↔ ROS 2, launch mô phỏng, node riêng cho mô phỏng (động cơ, cơ cấu nâng, bài thử) |
| `agv_localization` | EKF gộp odom bánh xe + IMU → `/odom`; dùng chung cho mô phỏng và xe thật |
| `agv_navigation` | SLAM (slam_toolbox), bản đồ kho `maps/warehouse`, dẫn đường Nav2 + vùng cấm + chui gầm kệ (docking), `payload_manager` (lọc scan, footprint theo tải); dùng chung cho mô phỏng và xe thật |
| `agv_mission` | Bố trí kho (`shelves.yaml`: ô kệ, kệ), vị trí có tên (trạm sạc, trạm lấy hàng...), nhiệm vụ định sẵn, gõ lệnh từ terminal hoặc web dashboard; dùng chung cho mô phỏng và xe thật |

## Tạo workspace (một lần)

Code nằm trong repo; workspace trong WSL chỉ symlink tới đây, nên sửa file ở Windows là WSL thấy ngay.

```bash
mkdir -p ~/agv_ws/src
ln -sfn /mnt/d/MyBK/HK261/DA2/Src/RasPi/src/agv_description ~/agv_ws/src/agv_description
ln -sfn /mnt/d/MyBK/HK261/DA2/Src/RasPi/src/agv_gazebo      ~/agv_ws/src/agv_gazebo
ln -sfn /mnt/d/MyBK/HK261/DA2/Src/RasPi/src/agv_localization ~/agv_ws/src/agv_localization
ln -sfn /mnt/d/MyBK/HK261/DA2/Src/RasPi/src/agv_navigation   ~/agv_ws/src/agv_navigation
ln -sfn /mnt/d/MyBK/HK261/DA2/Src/RasPi/src/agv_mission      ~/agv_ws/src/agv_mission
cd ~/agv_ws && colcon build --symlink-install
source ~/agv_ws/install/setup.bash
```

Build lại chỉ khi thêm file mới. Sửa file có sẵn (`.xacro`, `.sdf`, `.yaml`, `.py`) thì không cần build lại vì đã dùng `--symlink-install`.

## Xem mô phỏng (từng bước)

1. **Mở terminal Ubuntu**: trong Windows Terminal chọn tab *Ubuntu-24.04*, hoặc gõ `wsl -d Ubuntu-24.04` trong PowerShell. `~/.bashrc` đã nạp sẵn ROS, workspace và biến GPU.
2. **Build** (chỉ cần khi có file mới, ví dụ sau khi `git pull`):
   ```bash
   cd ~/agv_ws && colcon build --symlink-install && source install/setup.bash
   ```
3. **Chạy mô phỏng kèm lập bản đồ**:
   ```bash
   ros2 launch agv_gazebo sim.launch.py slam:=true
   ```
   Sau ~15 giây sẽ hiện 3 cửa sổ:
   - **Gazebo**: kho 3D, xe ở đầu phía tây kho (x = −4.5). Bảng *Teleop* bên phải để lái, tia lidar được vẽ quanh xe.
   - **RViz** (nhìn từ trên xuống): bản đồ đang được vẽ (xám = chưa biết, trắng = trống, đen = vật cản), chấm đỏ = tia lidar, mũi tên vàng = vị trí EKF.
   - **rqt_robot_steering**: 2 thanh trượt tốc độ tiến / quay.
4. **Cho xe chạy**, chọn một trong hai cách:
   - Lái tay bằng bảng Teleop hoặc rqt_robot_steering. Lái **chậm, quay chậm**, đi qua hết các lối đi.
   - Cho xe tự đi hết lộ trình qua các lối đi (~2.5 phút). Mở terminal thứ hai:
     ```bash
     ros2 run agv_gazebo drive_route.py --ros-args -p rot_start:=0.6 -p rot_stop:=0.45
     ```
     Hai tham số `rot_*` dùng khi vùng chết động cơ đang bật (mặc định). Chạy với `rpm_min:=0` thì bỏ hai tham số này.
5. **Lưu bản đồ** khi đã đi hết kho (terminal thứ hai):
   ```bash
   ros2 run nav2_map_server map_saver_cli -f /mnt/d/MyBK/HK261/DA2/Src/RasPi/src/agv_navigation/maps/warehouse --ros-args -p save_map_timeout:=30.0
   ```
6. **Tắt**: Ctrl+C ở terminal đang chạy `ros2 launch`.

**Cho xe tự đi (Nav2)** trên bản đồ đã lưu:
```bash
ros2 launch agv_gazebo sim.launch.py nav:=true
```
- RViz mở `agv_navigation/rviz/nav.rviz` (từ cấu hình mặc định của Nav2, gọn hơn): bản đồ trắng/đen, các ô kệ mini là vùng cấm, chấm đỏ = lidar, đường xanh = đường Nav2 lập, ô mờ quanh xe = local costmap. Global costmap (lớp tím/xanh phủ cả kho) tắt sẵn, muốn xem thì tick lại trong *Displays → Global Planner*.
- Xe đã được đặt sẵn vị trí ban đầu. Nếu đặt xe chỗ khác (`x:= y:=`), bấm **2D Pose Estimate** rồi kéo chuột tại chỗ xe đứng.
- Bấm **Nav2 Goal**, click vào điểm đích trên bản đồ rồi kéo chuột để chọn hướng. Xe tự lập đường (đường xanh) và đi tới.

**Chạy nhiệm vụ kho** (bước 5). Tự chạy khi khởi động:
```bash
ros2 launch agv_gazebo sim.launch.py nav:=true mission:=tuan_tra
```
Hoặc chạy `nav:=true` rồi gõ lệnh ở terminal thứ hai:
```bash
ros2 run agv_mission agv_cmd.py list                      # vị trí và nhiệm vụ
ros2 run agv_mission agv_cmd.py goto tram_lay_hang        # đi tới một vị trí
ros2 run agv_mission agv_cmd.py run tuan_tra --follow     # chạy nhiệm vụ, in tiến độ tới khi xong
ros2 run agv_mission agv_cmd.py cancel                    # dừng
```
Lệnh mới: `seq khu_nhap_hang:pick tram_lay_hang:drop:2 tram_sac:charge` (chuỗi tự tạo, mỗi bước `vị_trí[:task[:giây chờ]]`), `goto_xy 1.5 1.0 0` (đi tới điểm bất kỳ, frame `map`).

**Web dashboard** (chạy sẵn cùng `mission.launch.py` / `sim.launch.py nav:=true`): mở `http://localhost:8080` trên PC, hoặc `http://<IP Tailscale của máy chạy>:8080` từ điện thoại (trên xe thật: `http://sms-pi:8080`). Trên trang có bản đồ + vị trí xe + đường Nav2, nút **DỪNG**, chạy nhiệm vụ định sẵn, tạo chuỗi nhiệm vụ, ô kệ (vùng cấm) tô đỏ, bấm một trạm trên bản đồ để đi tới, chế độ "Bấm để đi" tới điểm bất kỳ, và nhật ký. Tắt bằng `dashboard:=false`, đổi cổng bằng `port:=...`. **Chưa có đăng nhập**, chỉ mở trong LAN / Tailscale.

| API | |
|---|---|
| `GET /api/config` | vị trí và nhiệm vụ (từ `stations.yaml`, `missions.yaml`) |
| `GET /api/map` | bản đồ `/map` (ô base64, 255 = chưa biết) |
| `GET /api/keepout` | mặt nạ vùng cấm `/keepout_filter_mask` (các ô kệ mini), cùng dạng |
| `GET /api/events` | Server-Sent Events 5 Hz: `pose`, `path`, `state`, `log`, `link` |
| `POST /api/command` | `{"cmd": "run tuan_tra"}`: các lệnh `goto`, `goto_xy`, `run`, `seq`, `cancel`, `status`, `list` |

Ví dụ ra lệnh từ phần mềm khác (WMS): `curl -X POST localhost:8080/api/command -d '{"cmd":"run tuan_tra"}'`.

Trên RViz, các vị trí hiện thành đĩa màu có tên (xanh lá = trạm sạc, cam = trạm lấy hàng, xanh dương = khu nhập hàng, xám = điểm trên lối). Trong Gazebo, các trạm và ô kệ là vạch sơn trên sàn.

Sự cố thường gặp:

| Hiện tượng | Cách xử lý |
|---|---|
| Không cửa sổ nào hiện, hoặc tiêu đề cửa sổ có `[WARN:COPY MODE]` | WSLg bị treo: chạy `wsl --shutdown` trong PowerShell rồi mở lại Ubuntu |
| Cửa sổ Gazebo tắt (driver GPU của WSL thỉnh thoảng crash khi khởi tạo OpenGL) | Mô phỏng, RViz, SLAM vẫn chạy vì server và cửa sổ Gazebo là 2 tiến trình riêng. Mở lại cửa sổ ở terminal khác: `gz sim -g --gui-config ~/agv_ws/install/agv_gazebo/share/agv_gazebo/config/gui.config` |
| RViz báo lỗi `indexed_8bit_image ... GLSL link result` | Lỗi đã biết của rviz2 ([ros2/rviz#463](https://github.com/ros2/rviz/issues/463)), bản đồ vẫn hiện bình thường, bỏ qua |
| `ros2 topic echo/list` không ra gì | `ros2 daemon stop; ros2 daemon start`, hoặc thêm `--no-daemon` |

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

DiffDrive của Gazebo quay bánh đúng tốc độ được yêu cầu, kể cả rất chậm. Xe thật thì không: firmware STM32 ép mọi lệnh khác 0 vào khoảng **100–270 RPM** (`SPD_RPM_MIN/MAX` trong `STM32/App/Inc/speed_ctrl.h`), vì dưới ~30 % duty động cơ không quay nổi. Node `motor_model` đứng giữa `/cmd_vel` và Gazebo, làm y hệt những gì node cầu nối Pi ↔ STM32 (bước 1–2) và firmware (bước 3) sẽ làm:

```
teleop / Nav2 -> /cmd_vel -> motor_model -> /cmd_vel_limited -> DiffDrive (gz /model/agv/cmd_vel)
```

1. **Giữ bán kính cua** (`preserve_turning_radius`, giống `diff_drive_controller` của Clearpath Husky): bánh khác 0 mà dưới 100 RPM thì nhân **cả hai bánh** cùng tỉ lệ cho tới khi bánh chậm nhất đạt 100 RPM. Xe đi nhanh hơn nhưng đúng đường cong.
2. Bánh vượt 270 RPM: thu nhỏ cả hai bên cùng tỉ lệ (cũng giữ bán kính cua).
3. Firmware: bánh nào còn khác 0 mà dưới 100 RPM thì đẩy lên 100 RPM (= 0.34 m/s).

| Xin | Xe chạy | Ghi chú |
|---|---|---|
| tiến 0.10 m/s | tiến **0.34 m/s** | không đi chậm hơn được |
| quay tại chỗ 0.5 rad/s | quay **1.51 rad/s** | |
| v = 0.25, ω = 0.33 (bán kính 0.76 m) | v = 0.48, ω = 0.64 (bán kính 0.76 m) | nhanh hơn, **đúng đường cong** |
| tiến 1.5 m/s | tiến 0.92 m/s (tối đa) | |

> **Node cầu nối Pi ↔ STM32 trên xe thật phải làm bước 1.** Nếu chỉ có bước 3 (firmware), lệnh `v = 0.25, ω = 0.33` thành hai bánh cùng 100 RPM: xe đi thẳng, **mất lái**. Trong mô phỏng, Nav2 khi đó trôi vào kệ ở lối hẹp 1 m và hỏng 3/4 điểm đích. Tắt bước 1 để thử: `--ros-args -p preserve_turning_radius:=false`.

Bảng Teleop trong Gazebo cũng đi qua `motor_model` (bridge chuyển gz `/cmd_vel` sang ROS `/cmd_vel`).

## Định vị: EKF gộp odom bánh xe + IMU

```
Gazebo DiffDrive -> /sim/wheel_odom -> wheel_odom (thêm covariance) -> /wheel/odom --+
                                                                                     +--> EKF -> /odom + TF odom -> base_footprint
Gazebo IMU ------------------------------------------------------------> /imu --------+
```

- Odom bánh xe tính góc rất kém vì skid-steer trượt ngang khi quay. EKF lấy **vx** từ bánh xe và **tốc độ quay wz từ gyro** (cấu hình và giải thích trong `agv_localization/config/ekf.yaml`).
- Không dùng yaw tuyệt đối của BNO055: nó dựa vào từ kế, trong kho nhiều sắt thép nên không tin được.
- Gyro trong mô phỏng có bias nhỏ (~0.03 °/s) giống IMU thật, nên góc vẫn trôi chậm theo thời gian. Phần trôi này để SLAM/AMCL sửa ở bước sau.
- Trên xe thật, node cầu nối STM32 phải gửi `/wheel/odom` **có covariance** (giống `wheel_odom.py`) và BNO055 gửi `/imu`; khi đó chạy `ros2 launch agv_localization ekf.launch.py` là xong.

## Đo sai số odom

`/ground_truth` là vị trí thật của xe trong world (chỉ có trong mô phỏng). So sánh với odom:

```bash
ros2 run agv_gazebo odom_drift.py                                          # /odom (EKF)
ros2 run agv_gazebo odom_drift.py --ros-args -p odom_topic:=/wheel/odom    # odom bánh xe
```

Lái xe một lúc rồi Ctrl+C, node in sai số vị trí, sai số góc và % quãng đường. Muốn đo hai nguồn cùng lúc thì đặt tên node khác nhau: thêm `-r __node:=drift_wheel`.

Kết quả (xuất phát `x:=-4.4 y:=-2.5 yaw:=1.5708`, ~7.5 m gồm đi thẳng, quay tại chỗ, chạy vòng; 2 lần chạy):

| | Lệch vị trí cuối | Lệch góc lớn nhất |
|---|---|---|
| Odom bánh xe `/wheel/odom` | 0.20–0.24 m (2.7–3.0 %) | 9.0–9.1° |
| EKF `/odom` | **0.09–0.12 m (1.2–1.5 %)** | **2.4–3.4°** |

## Lập bản đồ: slam_toolbox

```bash
ros2 launch agv_gazebo sim.launch.py slam:=true      # mô phỏng
ros2 launch agv_navigation slam.launch.py            # xe thật (cần /scan, /odom, TF)
```

Cấu hình `agv_navigation/config/slam.yaml` là **nguyên file mặc định** `mapper_params_online_async.yaml` của slam_toolbox, chỉ đổi `max_laser_range` thành 10 m (YDLidar X4). Cách làm này giống [linorobot2](https://github.com/linorobot/linorobot2) (dự án AGV DIY dùng Jazzy + Gazebo Harmonic + YDLidar). Kiến trúc DiffDrive → EKF (odom vx, vy, wz + gyro wz) → slam_toolbox cũng giống linorobot2 và robot mẫu `sam_bot` của [Nav2](https://github.com/ros-navigation/navigation2_tutorials).

> **Đừng chỉnh tham số SLAM khi chưa đo.** Lần đầu mình hạ `minimum_travel_distance/heading` từ 0.5 xuống 0.2 và `map_update_interval` từ 5 s xuống 2 s. Kết quả là SLAM quá tải ("Message Filter dropping message... queue is full"), TF map → odom trễ tới 1.7 s, bản đồ đầy vệt chéo, chỉ 25–36 % ô vật cản nằm đúng chỗ. Trả về mặc định thì hết.

Kết quả (lộ trình `drive_route.py`, ~52 m qua mọi lối đi, chấm bằng cách so ô vật cản với tường/kệ thật trong world):

| | Ô vật cản cách vật thật ≤ 10 cm | Sai lệch trung vị |
|---|---|---|
| Không vùng chết (`rpm_min:=0`), quay 0.5 rad/s | 98.4 % | 2.8 cm |
| Có vùng chết (mặc định, quay ≥ 1.5 rad/s) | **99.4 %** | **2.7 cm** |

Bảng trên là của kho cũ. Bản đồ trong repo (`maps/warehouse.pgm/.yaml`) là của **kho kiểu Kiva** (mục dưới), lập lại sau khi hạ lidar xuống 0.12 m: 100 % ô vật cản cách vật thật ≤ 10 cm (91.6 % ≤ 5 cm, trung vị 1.8 cm), thấy đủ 32/32 chân kệ mini. Gốc frame `map` là chỗ xe xuất phát (world x = −4.5, y = 0).

## Dẫn đường: Nav2

```bash
ros2 launch agv_gazebo sim.launch.py nav:=true       # mô phỏng
ros2 launch agv_navigation navigation.launch.py      # xe thật (cần /scan, /odom, TF)
```

Làm theo [linorobot2](https://github.com/linorobot/linorobot2):
- `agv_navigation/config/nav2.yaml` là **file mặc định của Nav2 Jazzy**, chỉ thay khối bộ điều khiển bằng khối của linorobot2. Khối đó dùng **RotationShim** (quay tại chỗ về hướng đường đi) kết hợp **Regulated Pure Pursuit** (bám đường, ~0.4 m/s). Kiểu chạy này hợp với xe có vùng chết.
- Đổi thêm: bán kính xe 0.25 m, lidar 10 m, AMCL đặt sẵn vị trí ban đầu ở gốc bản đồ, AMCL `alpha1–4: 0.05` và `z_hit/z_rand: 0.95/0.05` (xem mục dưới).
- `navigation.launch.py` gọi thẳng `bringup_launch.py` của Nav2 (map_server + AMCL + navigation).

**Vùng cấm cho pallet (kho cũ).** Lidar quét cao 17 cm không thấy pallet 15 cm. Lần chạy đầu, Nav2 lập đường sát pallet_2 và xe húc vào. Bánh quay trượt nên odom vẫn tăng, AMCL bị kéo lệch 3.8 m. Đã thêm **keepout filter** của Nav2, cấu hình lấy từ [nav2_costmap_filters_demo](https://github.com/ros-navigation/navigation2_tutorials):
- `maps/keepout_mask.pgm/.yaml`: mặt nạ cùng kích thước với bản đồ, pixel đen là vùng cấm.
- Tạo lại khi đổi bản đồ: `python3 scripts/make_keepout_mask.py`. Kho Kiva: lidar đã thấy pallet, vùng cấm giờ là **các ô kệ mini** (đọc từ `shelves.yaml`) và **chỉ ở global costmap** (xem mục Kiva).

Kết quả: 4 điểm đích qua các lối đi (lối dưới, phía đông giữa 2 pallet, lối hẹp giữa hàng A–B, về chỗ xuất phát), có vùng chết động cơ:

| Cấu hình | Thành công | Thời gian mỗi điểm | Lệch thật tại đích |
|---|---|---|---|
| Nav2 mặc định + linorobot2, chưa vùng cấm | 1/4 | — | xe húc pallet, AMCL lệch 3.8 m |
| + vùng cấm pallet | 3/4 | — | mất lái trong lối hẹp (xem mục mô hình động cơ) |
| + giữ bán kính cua, lần 1–3 | **4/4, 4/4, 4/4** | 13–44 s | 0.10–0.30 m |
| + giữ bán kính cua, lần 4 | 1/4 | — | AMCL lệch ở khu phía đông (xem dưới) |
| + chỉnh AMCL theo số đo (3 lần) | **4/4, 4/4, 4/4** | 13–30 s | 0.12–0.26 m |

Dung sai đích của Nav2 là 0.25 m, tính theo vị trí AMCL. Lệch thật có thể lớn hơn một chút vì AMCL tự nó lệch khoảng 0.1–0.2 m.

**AMCL lệch ở khu phía đông (đã sửa).** Với tham số AMCL mặc định của Nav2, khi xe chạy sang khu phía đông, AMCL lệch dần 0.3–1.1 m (2/3 lần đo bị lạc hẳn tới 6 m). Xe thật đi quá, dừng sát tường và kẹt. Các phép đo để tìm nguyên nhân:
- Đặt scan lên bản đồ theo vị trí thật: khớp 97–100 %, nên **bản đồ đúng**.
- Scan đều có TF, AMCL vẫn cập nhật đều, thời gian của lidar không lệch (τ ≈ 0–0.1 s).
- Đám hạt AMCL **tản rộng tới 1.8 m**. Tự chấm điểm từng hạt theo mô hình của AMCL: hạt điểm cao nhất cách vị trí thật 1–6 cm, nhưng chỉ hơn hạt trung bình khoảng 2.3 lần. Vị trí AMCL báo ra là trung bình cả đám nên bị kéo lệch.
- Nguyên nhân: `alpha1–4 = 0.2` giả định odom sai khoảng 45 %, trong khi odom EKF đo được chỉ sai 2–3 %. Mỗi lần xe quay tại chỗ (rất nhiều, do vùng chết), đám hạt phình ra. Thêm vào đó `z_hit/z_rand = 0.5/0.5` làm điểm của các hạt ít khác nhau.
- Cách sửa: `alpha1–4: 0.05`, `z_hit: 0.95, z_rand: 0.05` (mặc định của AMCL ROS 1). Kết quả: sai số định vị lớn nhất còn 0.14–0.16 m, 12/12 điểm đích, không còn "collision ahead".

## Kho kiểu Kiva: chui gầm, nâng kệ, chở tới trạm

Xe chui vào gầm **kệ mini 4 chân**, nâng kệ lên ~1.7 cm, chở cả kệ tới trạm lấy hàng (mô hình "hàng tới người" của Amazon Kiva), rồi trả kệ về ô.

**Bố trí** (`agv_mission/config/shelves.yaml`, nguồn duy nhất): 10 ô (2 dãy × 5, cách nhau 1.1 m) hai bên lối giữa rộng 2.45 m, 8 kệ, ô N5 và S5 để trống. Kệ 0.75 × 0.75 m, gầm cao 0.18 m, nặng 3 kg (kệ 1 kg + hàng 2 kg). Sửa file này rồi chạy:
```bash
python3 agv_gazebo/scripts/make_warehouse.py
python3 agv_navigation/scripts/make_keepout_mask.py
```
(`make_warehouse.py` ghi world và danh sách dock trong `nav2.yaml`; đổi vị trí ô thì lập lại bản đồ trước khi tạo vùng cấm.)

**Xe:** lidar đặt **giữa 2 tầng** (mặt quét 0.12 m): trên nóc thì bị đáy kệ che khi chui gầm; ở đây lidar thấy cả pallet 15 cm. Tầng 2 có **mặt nâng** (khớp `lift_joint`, hành trình 3 cm, 2 cm/s như vít me) và cảm biến "có kệ".

| Node | Việc | Xe thật |
|---|---|---|
| `lift_sim.py` (agv_gazebo) | `/lift/command` (`up`/`down`) → khớp nâng; phát `/lift/state` (`down`/`moving_up`/`up`/`moving_down`/`error`), `/lift/has_load` | cầu nối STM32 phát đúng các topic này |
| `shelf_detector.py` (agv_navigation) | `/scan_raw` → `/detected_dock_pose`: tâm kệ gần xe nhất (frame lidar, stamp của scan) | dùng nguyên |
| `payload_manager.py` (agv_navigation) | `/scan_raw` → `/scan`: bỏ điểm 4 trụ ốc của xe, khi chở thì bỏ cả 4 chân kệ; đổi footprint costmap (xe 0.32 × 0.38 m ↔ kệ 0.84 × 0.84 m); giới hạn 0.35 m/s khi chở; xoá costmap khi mặt nâng lên hết | dùng nguyên |

**Chui gầm** dùng `docking_server` của Nav2 (`SimpleNonChargingDock`), mỗi ô là một dock `slot_N1`…`slot_S5`. Xe tới **staging** cách tâm ô 1.0 m, rồi tiến thẳng vào. Hai loại dock:
- `shelf_dock` (**lấy kệ**): chui theo **kệ thật**. Node `shelf_detector.py` (agv_navigation, dùng chung xe thật) tìm 4 chân kệ trong `/scan_raw` (4 cụm điểm nhỏ tạo hình vuông cạnh 0.72 m), phát tâm kệ lên `/detected_dock_pose`. Kệ trả về mỗi lần lệch vài cm, lâu dần sẽ lệch nhiều; AMCL cũng lệch ~5 cm, nên không tin vị trí ô trên bản đồ.
- `slot_dock` (**trả kệ** vào ô trống, không có chân nào để nhận diện): theo vị trí ô trên bản đồ. Gửi `DockRobot` với `use_dock_id: false`, `dock_pose` = tâm ô, `dock_type: slot_dock`.
- Tới staging qua một điểm cách thêm 1.2 m trên cùng trục ô (`NavigateThroughPoses`), để đoạn cuối là đường thẳng: quay tại chỗ với vùng chết thì vọt lố 30–50°.
- Vùng cấm (ô kệ) **chỉ ở global costmap**: để ở local costmap thì bộ điều khiển docking coi mép vùng cấm là vật cản.
- `docking_server` của Jazzy phát thẳng `cmd_vel` (không qua collision monitor), nên **giữ** kiểm tra va chạm của nó.

Thử cả chu trình (lấy kệ → trạm lấy hàng → trả về ô → trạm sạc), đo bằng vị trí thật:
```bash
ros2 launch agv_gazebo sim.launch.py nav:=true
ros2 run agv_gazebo shelf_transport_test.py --ros-args -p shelf:=ke_03 -p station:=tram_lay_hang
# kệ đặt lệch khỏi tâm ô (dx m, dy m, dyaw độ); detect:=false để so với cách chui theo bản đồ
ros2 run agv_gazebo shelf_transport_test.py --ros-args -p shelf:=ke_03 -p shelf_offset:="[0.08, -0.06, 4.0]" -p detect:=false
```

Kết quả (6 lần mỗi cấu hình, 3 kệ ở 2 dãy: `ke_01`, `ke_03`, `ke_06`, mỗi lần 10 bước):

| | Vùng chết 100 RPM (firmware hiện tại) | 50 RPM (nếu cải thiện tốc độ thấp) |
|---|---|---|
| Chu trình trọn vẹn | **6/6** | **6/6** |
| Lệch góc tại staging (TB / lớn nhất) | 17.8° / 42.6° | 5.2° / 13.0° |
| Lệch góc tại staging khi chở kệ | 33.3° / 66.3° | 8.5° / 13.6° |
| Sau khi chui gầm lấy kệ (ngang / góc, lớn nhất) | 5.5 cm / 3.3° | 4.1 cm / 0.7° |
| Kệ trả về lệch tâm ô | 3.1–8.7 cm | 0.6–5.6 cm |
| Thời gian một chu trình | 74–107 s | 98–119 s |

Bộ điều khiển docking nắn được cả góc lệch 66° ở staging nhờ khe 15 cm giữa xe và chân kệ. Hạ vùng chết xuống 50 RPM thì xe tới staging thẳng hơn 4 lần và đặt kệ chính xác hơn, đổi lại chạy chậm hơn ~20 s.

## Nhiệm vụ kho: `agv_mission`

```
agv_cmd.py ──/mission/command──► mission_server ──NavigateToPose──► Nav2
                                     │
                                     ├─/mission/status   (tiến độ, giữ tin cuối)
                                     └─/mission/stations (MarkerArray cho RViz)
```

- `config/stations.yaml`: vị trí có tên trong frame `map`: `tram_sac` (chỗ xe xuất phát), `tram_lay_hang` (tây nam), `khu_nhap_hang` (tây bắc), `loi_tay`, `loi_dong` (hai đầu lối giữa). Ô kệ nằm ở `shelves.yaml`.
- `config/missions.yaml`: lộ trình đơn giản (`tuan_tra`). Chở kệ theo đơn hàng sẽ do `order_manager` lo (giai đoạn sau); hiện thử bằng `shelf_transport_test.py`.
- `mission_server.py` chờ `bt_navigator` "active" rồi mới gửi đích. Chặng thất bại thì xoá costmap, đợi 3 s và thử lại 1 lần; vẫn thất bại thì bỏ qua và đi tiếp. Hết nhiệm vụ thì in dòng `TONG KET` kèm thời gian từng chặng. Tham số launch: `mission:=<tên>`, `repeat:=<số lần>`.
- Trên xe thật: chạy `navigation.launch.py` + `ros2 launch agv_mission mission.launch.py`. Toạ độ trong `stations.yaml` phải đo lại trên bản đồ của kho thật.

## Topic

| Topic | Kiểu | Chiều |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | ROS → `motor_model` (và Teleop Gazebo → ROS) |
| `/cmd_vel_limited` | `geometry_msgs/Twist` | `motor_model` → Gazebo DiffDrive |
| `/ground_truth` | `nav_msgs/Odometry` | Gazebo → ROS, frame `world`, chỉ để đánh giá |
| `/sim/wheel_odom` | `nav_msgs/Odometry` | Gazebo → `wheel_odom`, covariance = 0 |
| `/wheel/odom` | `nav_msgs/Odometry` | `wheel_odom` → EKF (xe thật: node cầu nối STM32) |
| `/odom`, `/tf` (odom → base_footprint) | `nav_msgs/Odometry`, `tf2_msgs/TFMessage` | EKF → SLAM, Nav2 |
| `/map`, `/tf` (map → odom) | `nav_msgs/OccupancyGrid` | slam_toolbox → RViz, Nav2 |
| `/scan_raw` (7 Hz, 720 tia, 0.12–10 m) | `sensor_msgs/LaserScan` | Gazebo → `payload_manager` (xe thật: driver YDLidar) |
| `/scan` | `sensor_msgs/LaserScan` | `payload_manager` → SLAM, AMCL, Nav2 |
| `/lift/command`, `/lift/state`, `/lift/has_load` | `std_msgs/String`, `String`, `Bool` | cơ cấu nâng (`lift_sim` / cầu nối STM32) |
| `/lift/cmd_pos`, `/lift/contact` | `std_msgs/Float64`, `ros_gz_interfaces/Contacts` | `lift_sim` ↔ Gazebo |
| `/imu` (100 Hz) | `sensor_msgs/Imu` | Gazebo → ROS |
| `/joint_states`, `/clock` | | Gazebo → ROS |

## Kích thước xe

Đặt ở đầu `agv_description/urdf/agv.urdf.xacro`: thân 2 tầng mica 30 × 30 cm cách nhau 10 cm, 4 trụ ốc ở góc, linh kiện ở tầng 1, lidar đặt trên khối linh kiện **giữa 2 tầng** (mặt quét cao ~0.12 m), mặt nâng trên tầng 2, bánh 65 mm. Giá trị đánh dấu `(*)` là ước lượng (track 0.34 m, wheelbase 0.20 m, độ dày mica...), cần đo xe thật rồi sửa.

Lidar quét ở ~0.12 m: thấy pallet 15 cm và chân kệ mini, không thấy vật thấp hơn 12 cm. 4 trụ ốc giữa 2 tầng che lidar vài độ; `payload_manager` lọc bỏ.

`effective_track` (0.45 m) là track hiệu dụng cho skid-steer: bánh trượt ngang khi quay nên xe quay ít hơn tính theo `track_width`. Hiệu chỉnh bằng cách cho xe quay tại chỗ rồi đặt `effective_track` mới = `effective_track` hiện tại × `góc_odom / góc_thật`.
