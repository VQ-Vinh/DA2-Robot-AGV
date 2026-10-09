# Nhật ký quá trình làm đồ án DA2 – Robot AGV kho hàng

Tài liệu này ghi lại **quá trình** làm đồ án để làm báo cáo cho giảng viên: làm gì, dùng công cụ gì, **sai ở đâu, vì sao sai, sửa thế nào và rút ra gì**. Nó không phải hướng dẫn sử dụng (xem `RasPi/README.md`). Đồ án thật không đi thẳng từ đầu tới đích, nên các lần thử hỏng được ghi lại đầy đủ như các lần thành công.

Quy ước mỗi mục:
- **Mục tiêu**: định làm gì.
- **Cách làm**: công cụ, thiết kế.
- **Sai lầm / sự cố**: chuyện gì xảy ra, số liệu.
- **Nguyên nhân**: tìm ra bằng cách nào.
- **Cách sửa** và **kết quả** sau khi sửa.
- **Bài học**.

---

## 1. Tổng quan

**Đề tài:** AGV (xe tự hành) chạy trong kho hàng: lập bản đồ, tự định vị, tự đi tới các vị trí kệ.

**Phần cứng dự kiến:**

| Khối | Linh kiện |
|---|---|
| Máy tính trên xe | Raspberry Pi 4 (4 GB), Ubuntu Server 24.04, ROS 2 Jazzy |
| Vi điều khiển | STM32F4 Discovery (STM32F407), FreeRTOS, nối với Pi qua UART (USART2) |
| Động cơ | 4 × GA25-370 có encoder, kiểu skid-steer (4 bánh thường), 2 × driver L298N |
| Cảm biến | YDLidar X2/X4 (USB), IMU BNO055, đo pin INA226, PiCamera |
| Khung | 2 tầng mica 30 × 30 cm cách nhau 10 cm, bánh 65 mm |

**Kiến trúc phần mềm:**
```
Pi (ROS 2):  Nav2 ── SLAM / AMCL ── EKF (odom bánh + IMU) ── node cầu nối UART ──┐
STM32:                                       điều khiển tốc độ 4 bánh (PID) ◄────┘
```

**Phương pháp:** làm firmware điều khiển động cơ trước, sau đó **làm toàn bộ phần ROS trong mô phỏng Gazebo trước khi lắp xe**. Mục tiêu là các file cấu hình ROS (EKF, SLAM, Nav2) dùng lại nguyên khi chạy trên Pi.

**Công cụ:**

| Mục đích | Công cụ |
|---|---|
| Firmware | STM32CubeMX (sinh code HAL, FreeRTOS), CMake + arm-none-eabi-gcc, ST-Link |
| Công cụ PC | Python (vẽ log tốc độ, chỉnh PID trực tiếp qua UART) |
| ROS 2 / mô phỏng | ROS 2 Jazzy, Gazebo Harmonic, RViz2, robot_localization, slam_toolbox, Nav2 |
| Môi trường PC | Windows 11 + WSL2 Ubuntu 24.04 (GPU RTX 3050 qua driver D3D12) |
| Quản lý mã | Git + GitHub, mỗi phần việc một nhánh và một pull request (PR) |
| Hỗ trợ | Trợ lý AI (Claude Code) viết code, chạy test, phân tích log; người làm đồ án quyết định hướng đi và kiểm tra kết quả |

**Mốc thời gian:**

| Ngày | Việc | PR |
|---|---|---|
| 19/09/2026 | Dựng project STM32, nháy LED | #1 |
| 25/09/2026 | Điều khiển tốc độ 1 động cơ (PID + feedforward), FreeRTOS, UART CLI | #2 |
| 25–26/09/2026 | Chọn môi trường ROS 2, dựng mô hình xe và kho trong Gazebo, cài Pi | #3 |
| 07/10/2026 | Bước 1 mô phỏng: giới hạn động cơ thật | #4 |
| 07/10/2026 | Bước 2: EKF gộp odom bánh xe + IMU | #5 |
| 07/10/2026 | Bước 3: SLAM lập bản đồ kho | #6 |
| 07/10/2026 | Bước 4: Nav2 tự dẫn đường + vùng cấm | #7 |
| 07/10/2026 | Nhật ký quá trình (tài liệu này) + skill ghi báo cáo | #8 |
| 07/10/2026 | Bước 5: nhiệm vụ kho (trạm sạc, khu nhận hàng, kịch bản + lệnh), video demo | #9 |
| 07–08/10/2026 | Sửa vấn đề còn mở: AMCL lệch ở khu phía đông, "collision ahead" trong lối hẹp | #10 |
| 08/10/2026 | Web dashboard giao nhiệm vụ qua trình duyệt | #11 |
| 08/10/2026 | Giảm tải mô phỏng: bước vật lý 1 ms → 3 ms | #12 |
| 08/10/2026 | Kho kiểu Kiva, giai đoạn 1: chui gầm, nâng, chở kệ | #13 |
| 08/10/2026 | Kho Kiva, giai đoạn 2: nhận diện kệ bằng chân kệ | #14 |
| 08/10/2026 | Kho Kiva, giai đoạn 3: quản lý đơn hàng | #15 |

---

## 2. Firmware STM32: điều khiển tốc độ động cơ

### 2.1. Dựng project
- **Cách làm:** cấu hình ngoại vi trong STM32CubeMX (file `.ioc`), build bằng CMake. Code tự viết đặt riêng trong `STM32/Modules` (driver) và `STM32/App` (logic), không sửa vào `Core/` do CubeMX sinh ra.
- **Lý do:** CubeMX sinh lại các file trong `Core/` mỗi lần đổi cấu hình, code tự viết để lẫn trong đó có nguy cơ bị ghi đè. Quy tắc "CubeMX trước, code sau, code riêng ở Modules/App" được ghi thành skill của project (`.claude/skills/stm32-cubemx-first`) để luôn làm đúng.

### 2.2. Driver động cơ, encoder, PID
- **Cách làm:**
  - TIM1 phát PWM 1 kHz cho L298N, TIM3 ở chế độ đọc encoder.
  - Đo đặc tuyến thật duty → RPM của động cơ: 30 % → 96.5 RPM, … , 100 % → 278.6 RPM.
  - Bộ điều khiển = **feedforward nội suy từ bảng đo** + PID rời rạc có anti-windup, chu kỳ 50 ms.
- **Sự cố:** dưới khoảng 30 % duty động cơ không quay (ma sát tĩnh, gọi là **vùng chết**). Firmware vì thế giới hạn tốc độ đặt khác 0 trong khoảng **100–270 RPM**. Hệ quả của quyết định này hiện ra rất rõ ở phần mô phỏng (mục 4.1 và 4.4).
- **Chỉnh PID:** đo tự động bằng bài thử 0 → 120 → 240 → 120 RPM (không tải):

  | Kp / Ki / Kd | Vọt lố 0→120 | Ổn định TB | Sai số xác lập |
  |---|---|---|---|
  | 0.12 / 0.50 / 0.01 | 11.3 % | 2.60 s | 1.6 RPM |
  | 0.20 / 0.50 / 0.01 | 9.1 % | 2.35 s | 1.3 RPM |
  | **0.20 / 0.90 / 0.01** (chọn) | 14.7 % | **1.63 s** | **0.1 RPM** |
  | 0.20 / 0.90 / 0.03 | 18.0 % | 1.57 s | 0.2 RPM |

  Kd gần như vô tác dụng vì feedforward đã gánh phần lớn; tăng Kd chỉ làm vọt lố tệ hơn.

### 2.3. FreeRTOS và UART
- **Sự cố:** ban đầu task log in ra UART kiểu polling (`printf` → `HAL_UART_Transmit`). CPU đứng chờ từng byte, khoảng 3.3 ms mỗi dòng, chiếm **~7 % CPU**.
- **Cách sửa:** chuyển sang **UART + DMA**. Task giao buffer cho DMA rồi chờ semaphore, CPU rảnh trong lúc truyền. Tải CPU của task log còn khoảng 0.1 %.
- **Kết quả:** task điều khiển chạy đúng chu kỳ 50 ms (`osDelayUntil`), log không làm lệch nhịp điều khiển. Sơ đồ thời gian ở `STM32/docs/rtos-task-timing.md`.

**Còn lại cho phần cứng:** mới điều khiển 1 động cơ / 1 encoder; cần mở rộng lên 4 bánh, giao thức UART với Pi, driver BNO055 / INA226.

---

## 3. Chọn môi trường ROS 2 và dựng mô phỏng (PR #3)

### 3.1. Sai lầm chọn môi trường: ROS 2 Humble trên Ubuntu 22.04
- **Sự cố:** máy đã có WSL Ubuntu 22.04 + ROS 2 Humble. Gazebo bị crash ngay khi mở (lỗi Ogre2 `GL3PlusTextureGpu::copyTo`).
- **Nguyên nhân:** Mesa trên Ubuntu 22.04 chỉ cho OpenGL 4.2 qua D3D12, trong khi Gazebo mới cần ≥ 4.3. Kho PPA Mesa mới không còn hỗ trợ bản 22.04.
- **Cách sửa:** tạo WSL **Ubuntu 24.04** mới + **ROS 2 Jazzy + Gazebo Harmonic**, giữ nguyên bản 22.04 cũ. Pi cũng cài Ubuntu 24.04 + Jazzy để hai bên cùng phiên bản.
- **Sự cố tiếp theo:** mặc định WSL dùng card AMD tích hợp (OpenGL 4.2) hoặc render bằng CPU. Phải đặt `GALLIUM_DRIVER=d3d12` và `MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA` thì mới dùng RTX 3050 với OpenGL 4.6.

### 3.2. Mạng giữa PC và Pi
- **Sự cố:** các node ROS 2 trên PC và Pi không thấy nhau. Router chặn multicast giữa các thiết bị Wi-Fi. Trong WSL, gói discovery lớn hơn 1500 byte bị mất trên loopback.
- **Cách sửa:** cấu hình Fast DDS bằng file XML: chỉ định card mạng, khai báo sẵn địa chỉ peer qua Tailscale, giới hạn kích thước message.

### 3.3. Mô hình xe và kho
- **Cách làm:** URDF xe theo kích thước thật (các số chưa đo được đánh dấu là ước lượng), lidar và IMU giả lập, kho 12 × 8 m có 8 kệ và 2 pallet.
- **Sai lầm:** odom của xe skid-steer báo góc quay sai (odom 163° trong khi xe thật quay 124°). Bánh trượt ngang khi quay nên xe quay ít hơn công thức của xe 2 bánh.
- **Cách sửa:** dùng "track hiệu dụng" `effective_track = 0.45 m` (track thật khoảng 0.34 m), hiệu chỉnh bằng cách cho quay tại chỗ rồi so góc. Husky của Clearpath (xe skid-steer thương mại) cũng làm cách này, với `wheel_separation_multiplier = 1.875`.

---

## 4. Mô phỏng từng bước

### 4.1. Bước 1 – Làm mô phỏng "khó" như xe thật (PR #4)
- **Mục tiêu:** mô phỏng không được dễ hơn thực tế, nếu không cấu hình chỉnh trong mô phỏng sẽ hỏng khi lên xe.
- **Cách làm:**
  - Node `motor_model` đứng giữa `/cmd_vel` và Gazebo, áp đúng giới hạn 100–270 RPM của firmware.
  - Thêm vị trí thật `/ground_truth` và công cụ `odom_drift.py` để đo sai số.
- **Phát hiện:** vùng chết có hệ quả lớn:

  | Lệnh xin | Xe thật chạy |
  |---|---|
  | tiến 0.10 m/s | 0.34 m/s (không đi chậm được) |
  | quay tại chỗ 0.5 rad/s | 1.51 rad/s |
  | cua bán kính 0.5 m | cua bán kính 1.9 m |

- **Sự cố khi test:** xe lao vào tường hoặc kệ trong bài test vì vòng cua bị nới rộng. **Sửa:** cho các bài test xuất phát ở vùng trống phía tây kho.
- **Kết quả:** odom bánh xe lệch 0.30 m (5.4 %) và góc lệch tới 21° sau khoảng 5.5 m.

### 4.2. Bước 2 – EKF gộp odom bánh xe và IMU (PR #5)
- **Cách làm:** `robot_localization` lấy vận tốc tiến từ bánh xe và tốc độ quay từ gyro. Không dùng góc tuyệt đối của BNO055, vì nó dựa vào từ kế, mà trong kho nhiều sắt thép.
- **Sự cố:** odom của Gazebo gửi covariance (độ tin cậy) bằng 0, tức "chắc chắn tuyệt đối", khiến EKF bỏ qua IMU. **Sửa:** node `wheel_odom` gán covariance hợp lý, đúng việc node cầu nối STM32 sau này phải làm.
- **Thêm cho sát thực tế:** gyro có bias nhỏ (~0.03 °/s). Không có bias thì góc không bao giờ trôi, và EKF trong mô phỏng sẽ đẹp hơn thực tế.
- **Kết quả:** lệch góc lớn nhất giảm từ **9° xuống 2.4–3.4°**, lệch vị trí từ 0.24 m xuống 0.12 m.

### 4.3. Bước 3 – SLAM lập bản đồ (PR #6): sai lầm lớn nhất của đồ án
- **Mục tiêu:** dùng `slam_toolbox` lập bản đồ kho.
- **Lần 1 – tự chỉnh tham số:** đổi `minimum_travel_distance/heading` từ 0.5 xuống 0.2 và cập nhật bản đồ mỗi 2 s thay vì 5 s, "cho bản đồ dày hơn". Kết quả bản đồ đầy **vệt chéo**, tường bị thủng. Chỉ **25–36 %** ô vật cản nằm đúng chỗ (lệch ≤ 10 cm). Điểm chấm do script tự viết: so từng ô trên bản đồ với tường và kệ thật trong world.
- **Bốn lần đoán sai liên tiếp:**
  1. Xe quay nhanh do vùng chết → viết `scan_gate` bỏ scan khi quay. **Không cải thiện.**
  2. Xe bị chúi hoặc nảy làm lidar nghiêng → đo roll/pitch: **bằng 0**, sai.
  3. Lidar trả về 10 m thay vì "vô cực" khi không chạm gì → kiểm tra: trả về `inf` đúng chuẩn, sai.
  4. Điểm "ma" ở mép kệ (mixed pixel) → viết bộ lọc shadow. **Tệ hơn**: lọc mất 7.9 % điểm, gồm cả tường thật, SLAM mất dấu.
- **Bước ngoặt:** người làm đồ án yêu cầu dừng đoán, đi xem các dự án mã nguồn mở làm thế nào. Đối chiếu với **linorobot2** (AGV tự chế, cùng Jazzy + Gazebo Harmonic + YDLidar), robot mẫu **sam_bot** trong hướng dẫn Nav2, và **Clearpath Husky**:
  - Kiến trúc DiffDrive → EKF → slam_toolbox giống hệt của đồ án.
  - Họ dùng **nguyên file tham số mặc định** của slam_toolbox, không có bộ lọc scan tự viết.
- **Cách sửa:** trả cấu hình về mặc định, chỉ đổi tầm lidar 10 m, bỏ mọi bộ lọc tự chế.
- **Kết quả:** **99.4 %** ô vật cản đúng chỗ, lệch trung vị 2.7 cm, ngay lần chạy đầu tiên.
- **Nguyên nhân thật:** cấu hình tự chỉnh làm SLAM xử lý dày gấp đôi, bị quá tải. Log có "Message Filter dropping message… queue is full" và TF trễ tới 1.7 s. Vùng chết, thứ bị nghi đầu tiên, **không phải** nguyên nhân: chạy với vùng chết bản đồ vẫn sạch.
- **Sự cố phụ:**
  - Lưu bản đồ bị hết thời gian chờ (mặc định 2 s, DDS trong WSL kết nối chậm) → đặt `save_map_timeout:=30`.
  - Cửa sổ Gazebo thỉnh thoảng crash trong driver GPU của WSL, kéo theo cả hệ thống tắt → tách server và cửa sổ Gazebo thành 2 tiến trình như linorobot2. Cửa sổ chết thì mô phỏng vẫn chạy.
  - RViz báo lỗi shader khi vẽ bản đồ → tra ra là lỗi đã biết của rviz2 (issue ros2/rviz#463), vô hại.
- **Bài học:**
  - Gặp kết quả xấu thì **đối chiếu với dự án tham khảo và cấu hình mặc định trước**, đừng đoán rồi vá.
  - Đổi từng thứ một và đo bằng số.
  - "Chỉnh cho tốt hơn" mà không đo có thể làm hỏng hệ thống.

### 4.4. Bước 4 – Nav2 tự dẫn đường (PR #7)
- **Cách làm (theo linorobot2):** lấy file tham số mặc định của Nav2 Jazzy, chỉ thay bộ điều khiển bằng **RotationShim + Regulated Pure Pursuit**: quay tại chỗ về hướng đường rồi đi đều khoảng 0.4 m/s. Kiểu chạy này hợp với xe có vùng chết.
- **Đánh giá:** script gửi 4 điểm đích qua các lối đi, đo kết quả và độ lệch so với vị trí thật.

Các sự cố gặp phải, theo thứ tự:

| # | Sự cố | Nguyên nhân (tìm bằng cách nào) | Cách sửa |
|---|---|---|---|
| 1 | Launch lỗi ngay: `name 'false' is not defined` | Launch con kế thừa tham số `slam=false` của launch cha; Nav2 tính `not false` trong Python | Truyền rõ `slam: 'False'` cho Nav2 |
| 2 | Chỉ 1/4 điểm thành công, AMCL lệch 3.8 m | Nhật ký vị trí thật cho thấy xe **đứng yên** trong khi odom vẫn tăng: xe húc pallet cao 15 cm mà lidar (quét ở 17 cm) không thấy | **Vùng cấm (keepout)** cho pallet, theo demo chính thức của Nav2 |
| 3 | 3/4: xe trôi chéo vào kệ trong lối hẹp 1 m | Nhật ký lệnh: Nav2 xin `v=0.25, w=0.33`, ra bánh thành `v=0.34, w=0` → **mất lái**: cả 2 bánh dưới 100 RPM nên bị firmware đẩy lên bằng nhau | **Giữ bán kính cua**: nhân cả 2 bánh cùng tỉ lệ (giống `preserve_turning_radius` của Husky). Node cầu nối trên xe thật phải làm y vậy |
| 4 | Sau khi sửa, các lần chạy về sau lại hỏng; đã nghi WSL quá tải, rác bộ nhớ chia sẻ của DDS, rồi khởi động lại cả WSL | **Lỗi ở script test**: hàm chờ Nav2 của thư viện gửi lại vị trí ban đầu mỗi lần có message tới, các subscription ghi log làm nó gửi hàng chục lần/giây, AMCL reset liên tục | Script test tự chờ AMCL, không gửi vị trí ban đầu nữa |
| 5 | Nạp node vùng cấm vào chung container Nav2 → "xe không chạy", nên đã **bỏ cách này** | Thực ra lỗi do script ở dòng 4. Kết luận sai vì đo bằng công cụ đo đang hỏng | Sau khi sửa script, dùng lại cách container: đạt 4/4 |
| 6 | Chạy có cửa sổ thì vùng cấm không hiện | Bộ khởi động vùng cấm (tiến trình riêng) kẹt khi máy tải nặng | Nạp vào chung container Nav2, đúng mặc định của demo Nav2 |
| 7 | RViz mở trống trơn | Lúc đầu đoán do dòng chú thích trong file cấu hình (**đoán sai**). Thật ra file RViz mới chưa được cài vì chưa build lại workspace | Build lại; tự chụp cửa sổ RViz để kiểm tra trước khi báo |
| 8 | Hình xe trong RViz báo đỏ | Cấu hình gốc của Nav2 đăng ký `/robot_description` kiểu volatile nên bỏ lỡ message phát một lần lúc khởi động | Dùng cấu hình "transient local" như file RViz đã chạy tốt |

**Kết quả:** sau các sửa đổi, **4/4 điểm đích ở 4 lần chạy riêng** (13–45 s mỗi điểm, lệch thật 0.10–0.30 m). Có **1 lần 1/4** (mục 6).

**Bài học:**
- Khi một phép đo cho kết quả lạ, **kiểm tra công cụ đo trước** (sự cố 4–5). Một script test lỗi đã dẫn tới hai kết luận sai và mất nhiều lần chạy thử.
- Nhật ký "vị trí thật – vị trí ước lượng – lệnh xin – lệnh thực" theo thời gian tìm ra nguyên nhân nhanh hơn nhiều so với đoán (sự cố 2–3).
- Báo "đã sửa" chỉ khi đã tự kiểm tra kết quả (sự cố 7).

### 4.5. Bước 5 – Nhiệm vụ kho (PR #9)
- **Mục tiêu:** biến khả năng "đi tới một điểm" thành nhiệm vụ AGV. Kho có trạm sạc và khu nhận hàng. Ra lệnh được theo 2 cách: chạy kịch bản tự động, và gõ lệnh từ máy tính.
- **Cách làm:** package `agv_mission`, dùng chung cho mô phỏng và xe thật:
  - `stations.yaml`: 10 vị trí có tên (trạm sạc ở chỗ xe xuất phát, khu nhận hàng ở vùng trống tây nam, 8 điểm lấy hàng ở giữa lối đi phía nam mỗi kệ).
  - `missions.yaml`: 3 nhiệm vụ: `giao_hang` (3 chặng), `giao_hang_2` (4 chặng), `tuan_tra` (9 chặng).
  - `mission_server.py`: gửi từng đích cho Nav2 qua action `NavigateToPose`. Chặng thất bại thì xoá costmap, đợi 3 s và thử lại 1 lần. In dòng tổng kết kèm thời gian từng chặng.
  - `agv_cmd.py`: lệnh `list / goto / run / cancel / status`.
  - Trạm sạc và khu nhận hàng vẽ thành ô sơn trên sàn Gazebo (chỉ để nhìn, không va chạm, lidar không thấy) và thành đĩa màu có tên trên RViz.
  - Node điều phối **không dùng** `BasicNavigator`, vì lỗi gửi dồn vị trí ban đầu ở bước 4.

| # | Sự cố | Nguyên nhân (tìm bằng cách nào) | Cách sửa |
|---|---|---|---|
| 1 | Lần chạy đầu: 0/3 chặng, cả 3 bị "Nav2 từ chối đích" trong 0.6 s | Node coi Nav2 sẵn sàng khi action server có mặt và có vị trí AMCL, nhưng lúc đó `bt_navigator` chưa "active" nên từ chối mọi đích. Thử lại ngay cũng bị từ chối (log thời điểm từng dòng) | Hỏi trạng thái vòng đời `bt_navigator/get_state`, chờ "active", giống thư viện của Nav2; khi bị từ chối thì đợi 3 s rồi mới thử lại |
| 2 | Lệnh `goto ke_A1 --follow` thoát sau 6 s, khi xe chưa tới | Kênh `/mission/status` giữ lại 10 tin cũ cho node vào sau, trong đó có dòng "TONG KET" của nhiệm vụ trước. Công cụ chỉ bỏ qua tin cũ trong 0.5 s đầu, nhưng kết nối DDS trong WSL lâu hơn thế | Bỏ qua mọi tin nhận được trước khi gửi lệnh |
| 3 | Thời gian tính lại cho lần thử lại không được đặt lại, nên lần thử lại sẽ bị huỷ ngay (phát hiện khi đọc lại code, chưa xảy ra lúc chạy) | Dùng chung một mốc thời gian cho cả chặng và từng lần thử | Tách 2 mốc: bắt đầu chặng (tính tổng) và bắt đầu lần thử (tính timeout) |
| 4 | Công cụ quay video rò bộ nhớ, rồi đọc vùng nhớ đã giải phóng (phát hiện khi đọc lại code trước khi chạy) | Ảnh lấy bằng `XGetImage` không được giải phóng; sau khi thêm `XFree` lại đọc kích thước ảnh sau khi giải phóng | Lưu kích thước trước, rồi `XFree` |
| 5 | Trên RViz không thấy đĩa/tên vị trí, dù node phát đủ 20 marker (kiểm bằng subscriber riêng) | Lớp hiển thị nằm cuối danh sách. Giả thuyết: lớp vùng cấm (phủ cả bản đồ, mờ 50 %) vẽ đè lên; **chưa kiểm chứng riêng** | Đưa lớp Stations lên ngay dưới lớp Map thì hiện; đổi chữ tên sang màu tối cho dễ đọc trên nền trắng |

**Kết quả** (mô phỏng không cửa sổ, có vùng chết động cơ):

| Nhiệm vụ | Cách ra lệnh | Kết quả |
|---|---|---|
| `giao_hang` (3 chặng) | tự động, 4 lần (1 lần riêng + 3 lần lặp liên tiếp) | **4/4 lần đủ 3/3 chặng**: 62.0, 61.8, 60.6, 74.4 s |
| `tuan_tra` (9 chặng) | tự động | **9/9 chặng**, 152.4 s, không cần thử lại |
| `giao_hang_2` (4 chặng) | gõ lệnh `run` | **4/4 chặng**, 81.5 s, có 2 lần thử lại tự động (1 do bị lệnh trước chen ngang, 1 do lỗi TF thoáng qua lúc nhận đích mới) |
| `giao_hang` có cửa sổ (2 lần quay video) | tự động | 3/3 chặng, 62.8 và 61.4 s |

- Lần lặp thứ 3 của `giao_hang` chậm hơn (74.4 s so với khoảng 61 s): chặng tới khu nhận hàng mất 35.7 s thay vì khoảng 24 s. Log có 24 lần "collision ahead" và 3 lần "Controller patience exceeded"; bước tự gỡ bên trong Nav2 xử lý được nên xe vẫn tới đích.
- Video demo [`RasPi/docs/demo_giao_hang.mp4`](RasPi/docs/demo_giao_hang.mp4): nhiệm vụ `giao_hang`, ghép cửa sổ Gazebo + RViz + dòng trạng thái, 45 s (tua nhanh ×2). Chỉ chụp cửa sổ của WSL, không chụp màn hình Windows.

**Bài học:**
- "Có action server" chưa có nghĩa là "nhận lệnh được"; phải kiểm tra đúng trạng thái mà thư viện chuẩn kiểm tra.
- Kênh có giữ tin cũ (transient local) cần phân biệt tin cũ với tin mới.
- Đọc lại code trước khi chạy bắt được 2 lỗi (sự cố 3–4) mà test có thể không lộ ra.

### 4.6. Sửa 2 vấn đề còn mở: AMCL lệch và "collision ahead" (PR #10)
- **Mục tiêu:** sửa 2 vấn đề để lại từ bước 4 và 5:
  - AMCL lệch tới 1.1 m ở khu phía đông kho, có lần làm xe kẹt sát tường.
  - Bộ điều khiển báo "collision ahead" trong lối hẹp, làm chặng chậm hơn khoảng 50 %.
- **Cách làm:** theo bài học ở bước 3, đo để biết chuyện gì xảy ra, và đối chiếu tài liệu chính thức (Nav2), dự án tham khảo (Clearpath, linorobot2), mã nguồn gốc (AMCL ROS 1) trước khi sửa. Mỗi giả thuyết được kiểm bằng một phép đo riêng:

| # | Giả thuyết / câu hỏi | Phép đo | Kết quả |
|---|---|---|---|
| 1 | Hai vấn đề có liên quan nhau? | 3 lần chạy 4 điểm đích, ghi sai số TF `map → base_footprint` so với vị trí thật mỗi 0.5 s | Sai số lớn **chỉ** ở khu phía đông. 2/3 lần AMCL lạc tới 6 m. "Collision ahead" chỉ xuất hiện **sau** khi lạc, tức là hệ quả |
| 2 | Collision monitor gây kẹt? | Đọc issue của Nav2 ([navigation2#3313](https://github.com/ros-navigation/navigation2/issues/3313)) | Chế độ "approach" (đang dùng) vẫn cho lùi; xe kẹt vì đã vào sát tường do AMCL lệch. **Không phải nguyên nhân gốc** |
| 3 | Bản đồ khu phía đông sai? | Chụp scan ở khu phía đông, đặt lên bản đồ theo vị trí thật và theo vị trí AMCL | Theo vị trí thật khớp 97–100 %, theo AMCL 60 % → 5 %. **Bản đồ đúng** |
| 4 | AMCL bỏ scan vì TF odom trễ? | Với mỗi scan, kiểm tra TF `odom → laser_frame` tại đúng thời điểm của scan; đếm số lần AMCL cập nhật | 100 % scan có TF (scan chỉ mới hơn TF khoảng 3 ms); AMCL cập nhật 157 lần / 27.5 m. **Loại** |
| 5 | AMCL không sửa gì, chỉ chạy theo odom? | Theo dõi TF `map → odom` (phần hiệu chỉnh của AMCL) | Có hiệu chỉnh nhưng quá ít và sai hướng (cần khoảng (−0.23, −0.37) m, AMCL cho (−0.15, −0.04) m) |
| 6 | Lidar gắn lệch hoặc lật? | Đọc URDF / TF | `rpy = 0 0 0`, đúng. **Loại** |
| 7 | AMCL quá tự tin sai? | Chụp đám hạt (`/particle_cloud`) và covariance | Ngược lại: đám hạt **tản rộng tới 1.8 m** theo x (covariance x tới 3.1 m²) |
| 8 | Thời gian (stamp) của scan lệch? | Đặt scan theo vị trí thật tại `stamp + τ`, τ ∈ [−1, 1] s, tìm τ khớp nhất | τ tốt nhất 0 (55 %) hoặc −0.1 s (36 %), tương đương 2–3 cm. **Loại** |
| 9 | Mô hình đo của AMCL có phân biệt được hạt đúng? | Tự tính lại đúng công thức likelihood field của AMCL cho từng hạt | Hạt điểm cao nhất cách vị trí thật 1–6 cm (có đủ hạt đúng), nhưng chỉ hơn hạt trung vị khoảng **2.3 lần**. AMCL báo ra trung bình cả đám tản rộng, nên bị kéo lệch |

- **Nguyên nhân** (từ phép đo 7 và 9):
  - `alpha1–4 = 0.2` (mặc định Nav2) nghĩa là giả định odom sai khoảng 45 %, trong khi odom EKF đo được chỉ sai **2–3 %** quãng đường, và góc lấy từ gyro. Riêng `alpha4` làm đám hạt phình ra mỗi lần quay tại chỗ, mà xe quay tại chỗ rất nhiều vì vùng chết động cơ.
  - `z_hit/z_rand = 0.5/0.5` (mặc định Nav2) coi một nửa số đo là nhiễu, nên điểm của các hạt ít khác nhau.
  - Linorobot2 và Clearpath dùng nguyên mặc định này mà vẫn chạy được. Khác biệt ở xe này là quay tại chỗ nhiều (vùng chết) và kho có nhiều lối dài giống nhau.
- **Cách sửa** (`agv_navigation/config/nav2.yaml`, có ghi chú lý do):
  - `alpha1–4: 0.05` (tương đương sai số 22 %, vẫn dư nhiều so với 2–3 % đo được).
  - `z_hit: 0.95, z_rand: 0.05`: mặc định của AMCL ROS 1 (`AMCL.cfg`). Tính trên dữ liệu đã chụp, độ chênh hạt đúng / hạt trung vị tăng từ 2.3 lên 2.9 lần.
- **Kết quả** (cùng bài test 4 điểm đích, 3 lần mỗi cấu hình):

  | | Mặc định Nav2 | Sau khi sửa |
  |---|---|---|
  | Điểm đích thành công | 4/4, 2/4, 2/4 | **4/4, 4/4, 4/4** |
  | Sai số định vị: 90 % thời gian dưới | 0.24; 4.72; 4.65 m | **0.07; 0.07; 0.07 m** |
  | Sai số định vị lớn nhất | 0.54; 5.97; 6.08 m | **0.16; 0.15; 0.14 m** |
  | "Collision ahead" | 0; 0; 26 | **0; 0; 0** |

  Bài nhiệm vụ: `tuan_tra` 9/9 chặng (145.0 s, trước là 152.4 s, còn 4 lần "collision ahead"). `giao_hang` 3 lần đều 3/3 chặng (60.8, 69.2, 63.2 s), 0 lần "collision ahead"; trước khi sửa có một chặng mất 35.7 s với 24 lần "collision ahead". Lần 69.2 s có 1 lần tự thử lại do hệ thống khựng khoảng 1 s (vòng điều khiển tụt 9.6 Hz, TF cũ), không liên quan tới định vị.
- **Sai lầm trong quá trình:** ban đầu nghi collision monitor (giả thuyết 2), rồi nghi AMCL bỏ scan (4), rồi nghi lệch thời gian (8). Cả ba bị loại bằng đo đạc chứ không phải bằng thử sửa. Cũng đã định đổi `robot_radius` sang khung chữ nhật như Clearpath để giảm "collision ahead", nhưng số liệu cho thấy "collision ahead" là hệ quả của AMCL lệch, nên không đổi, để mỗi lần chỉ đổi một thứ.
- **Bài học:**
  - Hai triệu chứng có thể cùng một gốc; đo xem chúng xảy ra theo thứ tự nào trước khi sửa từng cái.
  - Mặc định của thư viện là điểm xuất phát tốt, nhưng khi số đo cho thấy giả định của nó (ở đây: độ nhiễu odom) sai lệch xa thực tế thì chỉnh theo số đo, và ghi lý do ngay trong file cấu hình.

### 4.7. Web dashboard giao nhiệm vụ (PR #11)
- **Mục tiêu:** người vận hành mở trình duyệt (điện thoại, laptop) qua LAN hoặc Tailscale để xem bản đồ, vị trí xe và giao nhiệm vụ, không phải cài ROS 2 hay gõ lệnh terminal.
- **Chọn web hay app Qt:** chọn web. Các AMR thương mại (MiR) và Open-RMF cho người vận hành dùng giao diện web; Qt chủ yếu dùng cho công cụ kỹ thuật (RViz, rqt). Lý do quyết định: trình duyệt chỉ cần HTTP tới Pi. Một app Qt phải tham gia DDS, mà mạng PC ↔ Pi ở mục 3.2 đã rất khó cấu hình.
- **Chọn mã nguồn mở hay tự viết:**

  | Dự án | Lý do không dùng làm giao diện chính |
  |---|---|
  | rosbridge_suite + roslibjs | Bản ROS 2 nặng: tác giả Vizanti ghi nhận nó không chạy nổi trên Pi 4 khi tải thường. Trình duyệt cũng phải biết cấu trúc topic ROS |
  | Vizanti (có bản Jazzy) | Nhắm robot ngoài trời (GPS, tàu, robot hiện trường), không có khái niệm kệ, trạm sạc, nhiệm vụ kho |
  | foxglove_bridge + Lichtblick | Công cụ debug kiểu RViz trên trình duyệt, hợp cho kỹ sư, không hợp cho người vận hành |
  | Open-RMF rmf-web | Dành cho cả đội robot, quá nặng cho 1 xe |

  Kết luận: tự viết một server nhỏ bằng thư viện chuẩn Python (không cài thêm gì lên Pi) và HTML/JS thuần. API theo nghiệp vụ (`run giao_hang`), giống REST API của MiR, và dùng lại nguyên `mission_server` của bước 5.
- **Cách làm:**
  - Node `web_dashboard.py` có các API: `GET /api/config`, `GET /api/map`, `GET /api/events` (Server-Sent Events 5 Hz: vị trí xe từ TF, đường đi `/plan`, trạng thái, nhật ký) và `POST /api/command`. Lệnh POST chỉ nhận các động từ cho phép và bị giới hạn độ dài.
  - `mission_server` có thêm topic `/mission/state` (JSON) cho chương trình đọc, và 2 lệnh mới: `seq` (chuỗi bước tự tạo) và `goto_xy` (đi tới điểm bấm trên bản đồ).
  - Giao diện gồm: bản đồ (canvas), nút DỪNG, nhiệm vụ định sẵn, trình tạo chuỗi nhiệm vụ, nút đi nhanh, nhật ký. Có bố cục cho điện thoại và giao diện sáng/tối.
- **Sai lầm / sự cố:**

  | # | Sự cố | Nguyên nhân (tìm bằng cách nào) | Cách sửa |
  |---|---|---|---|
  | 1 | Trợ lý AI bắt tay vào sửa code khi người làm đồ án mới hỏi "có nên làm không", bị dừng lại | Hiểu nhầm câu hỏi ý kiến thành yêu cầu làm | Hoàn tác, xoá nhánh; lập kế hoạch và so sánh phương án trước, được duyệt mới làm |
  | 2 | Bản đồ chỉ là một chấm nhỏ ở góc màn hình | `fit()` chạy khi canvas chưa có kích thước (W = 0, đọc biến `view` trong trình duyệt thấy tỉ lệ = −0.198) | Chưa có kích thước thì chưa fit, để `ResizeObserver` gọi lại |
  | 3 | Đọc mã thấy file web sẽ bị trả 404 khi build `--symlink-install` | `realpath` đi theo symlink ra ngoài thư mục web, nên phép kiểm tra chống `..` chặn luôn file hợp lệ | Dùng `abspath`: vẫn chặn `..`, giữ symlink; kiểm lại bằng `curl` đường dẫn `../../etc/passwd` → 404 |
  | 4 | Node dashboard tốn **82 % CPU** | So sánh: `mission_server` cũng tốn 77 %; một node rclpy **rỗng** bật `use_sim_time` tốn 53 %. Nguyên nhân là `/clock` của Gazebo (bước vật lý 1 ms, khoảng 1000 tin/giây), rclpy phải xử lý từng tin | Dashboard chỉ cần TF mới nhất nên bỏ `use_sim_time` → **8 %** (2 trình duyệt mở). Trên Pi không có `/clock` |
  | 5 | Khởi động lại node dashboard thì nhật ký trên trang bị lặp; bản sửa đầu tiên lại làm nhật ký trống | Id dòng nhật ký đếm lại từ đầu. Bản sửa đầu so `after > log_id`, nhưng node mới nhận đúng 10 dòng (giữ lại trên topic) nên `after = 10` không lớn hơn → không gửi gì | Server có mã phiên (`boot`), trình duyệt gửi kèm khi kết nối lại; khác phiên thì xoá và nhận lại toàn bộ |
  | 6 | Người làm đồ án thấy bản đồ trên web thiếu 2 pallet mà RViz có | Pallet cao 0.15 m, thấp hơn mặt quét lidar (0.17 m), nên không có trong `/map`; RViz vẽ chúng từ mặt nạ vùng cấm `/keepout_filter_mask` (bước 4), còn dashboard chỉ đọc `/map` | Dashboard đọc thêm mặt nạ, vẽ thành lớp đỏ trong suốt theo gốc tọa độ riêng của mặt nạ, thêm chú giải |

- **Kết quả** (mô phỏng headless, bấm nút trên dashboard, xem trên browser pane):
  - `giao_hang` 3/3 chặng, 66.0 s.
  - Chuỗi tự tạo (Kệ A1 lấy hàng → Khu nhận hàng trả hàng, chờ 2 s) 2/2 chặng, 36.4 s.
  - `giao_hang_2` (chạy bằng `curl` POST) 4/4 chặng, 72.4 s.
  - Bấm bản đồ → xe đi tới điểm đó; DỪNG giữa chặng → hai lần đọc vị trí cách nhau 3 s lệch nhau 0 mm.
  - Lệnh sai (`rm`, JSON sai, `cmd` không phải chuỗi, lệnh 1.8 kB) bị từ chối, path traversal → 404. `agv_cmd.py` vẫn dùng song song được.
  - Khởi động lại node giữa chừng → trang tự kết nối lại sau khoảng 2 s.
  - Chưa thử từ điện thoại thật qua Tailscale, chưa chạy trên Pi.
- **Bài học:**
  - Khi số đo của phần mới xấu bất thường, đo luôn phần cũ và một trường hợp rỗng để so; ở đây lỗi không nằm ở code mới.
  - Câu hỏi "nên làm A hay B" là xin ý kiến, không phải yêu cầu làm.

### 4.8. Giảm tải mô phỏng: bước vật lý 1 ms → 3 ms (PR #12)
- **Mục tiêu:** kiểm phát hiện ở mục 4.7: mỗi node Python dùng thời gian mô phỏng tốn 50–80 % một nhân CPU vì `/clock`. Xem đây có phải nguyên nhân của việc thỉnh thoảng khựng khoảng 1 s (vấn đề còn mở 1) không.
- **Cách làm:**
  - Theo bài học ở bước 3, xem các world mẫu chính thức trước khi tự chỉnh. Cả 3 world mẫu của Nav2 cài sẵn trong Jazzy (`nav2_minimal_tb3_sim/tb3_sandbox`, `nav2_minimal_tb4_sim/depot` và `warehouse`) đều dùng `max_step_size` **0.003 s**.
  - World của đồ án dùng 0.001 s, chỉ là giá trị có sẵn từ lúc dựng world, không có lý do ghi lại.
  - Gazebo phát `/clock` mỗi bước vật lý: 1000 tin/s với bước 1 ms, 333 tin/s với bước 3 ms. Các cảm biến (lidar 7 Hz, IMU 100 Hz, odom 30 Hz) đều chậm hơn nhiều so với 333 Hz.
- **Đo:**
  - Bài thử: nhiệm vụ `giao_hang` chạy tự động 2 lượt, headless.
  - Lấy mẫu `top` 10 s và RTF từ `/stats` của Gazebo.
  - Một node riêng so `/odom` (EKF) và `/amcl_pose` với `/ground_truth` theo thời gian của từng tin.
  - Hai cấu hình chạy xen kẽ trong cùng buổi (1 ms → 3 ms → 1 ms), để loại trừ khả năng máy tình cờ rảnh hơn ở một cấu hình.

  | | Bước 1 ms (5 lần, 30 chặng) | Bước 3 ms (4 lần, 24 chặng) |
  |---|---|---|
  | Chặng thành công | **9/30** (các lần: 2+0, 1+0, 0+0, 3+3, 0+0 trên 3+3) | **24/24** |
  | Một lượt `giao_hang` (khi thành công) | 61.6 – 69.8 s | 60.2 – 61.4 s |
  | RTF | 0.14 – 1.4, dao động mạnh | 0.99 – 1.01 (một lần có 2 mẫu 0.56, 0.65) |
  | Tổng CPU cả hệ thống mô phỏng | 670 – 808 % một nhân | 380 – 475 % |
  | Container Nav2 / EKF | 300 – 895 % / 17 – 111 % | 178 – 200 % / 14 – 16 % |
  | Mỗi node Python (`use_sim_time`) | 38 – 75 % | 30 – 48 % |
  | Cảnh báo "Control loop missed" | 11 – 152 mỗi lần | 0 – 1 |
  | Sai số odom EKF (cuối / quãng đường) | — | 0.20 m / 36.5 m (0.6 %) |
  | Sai số AMCL (90 % thời gian / lớn nhất) | — (PR #10: lớn nhất 0.14–0.16 m) | 0.062 / 0.080 m |

  Khi thất bại ở bước 1 ms, log báo TF `odom → base_footprint` trễ 0.5–4 s ("Lookup would require extrapolation into the future", "Transform data too old") và Nav2 huỷ đích. Đây chính là hiện tượng "khựng", nhưng nặng hơn nhiều so với hôm trước.
- **Sai lầm trong quá trình đo:**

  | # | Sai lầm | Phát hiện thế nào | Sửa |
  |---|---|---|---|
  | 1 | Kết luận lần đo 1 ms đầu tiên bị nhiễu vì "unattended-upgrades ăn 98 % CPU" | Đo riêng tiến trình đó: tổng cộng chỉ 0.09 s CPU từ lúc bật máy. Lệnh `awk` tách cột của `top` bị lệch | Rút lại kết luận; in nguyên dòng `top`, không tách cột |
  | 2 | Node đo báo odom sai 1.06 m, AMCL sai 0.91 m ở bước 3 ms, trong khi thời gian chặng vẫn y như cũ | Sai số gần như không đổi suốt lần chạy, giống độ lệch gốc toạ độ: node lấy mẫu `/ground_truth` đầu tiên làm gốc, mà node chỉ bật sau khi xe đã bắt đầu chạy | Lấy gốc là vị trí xuất phát đã biết (−4.5, 0), chạy lại cả hai cấu hình |
  | 3 | Một lần chạy 3 ms Nav2 không bao giờ sẵn sàng | Log: container nạp xong `map_server`, `amcl`, `controller_server` nhưng "failed to send response to /nav2_container/_container/load_node (timeout)", launch chờ mãi | Không liên quan tới bước vật lý; không tính vào bảng, ghi thành vấn đề còn mở |

- **Cách sửa:** `agv_gazebo/worlds/warehouse.sdf`: `max_step_size` 0.001 → 0.003, ghi lý do trong file.
- **Chưa kiểm lại:** các số đo của bước 1 (độ trôi odom bánh xe so với vị trí thật) chưa đo lại với bước 3 ms. Các số trên là odom sau EKF.
- **Bài học:**
  - Mặc định trong file mẫu (world 1 ms) cũng phải đối chiếu với dự án tham khảo như tham số thuật toán; một con số không ai chọn có thể quyết định cả hệ thống có quá tải hay không.
  - Kết quả lạ của công cụ đo (độ lệch không đổi, tiến trình "ăn" CPU) phải kiểm lại công cụ trước khi tin.

### 4.9. Kho kiểu Kiva, giai đoạn 1: chui gầm, nâng và chở kệ (PR #13)
- **Mục tiêu:** biến xe từ "đi tới điểm rồi đứng chờ" thành robot kho thật. Người làm đồ án chọn kiểu **Amazon Kiva**: xe chui gầm kệ mini, nâng kệ, chở cả kệ tới trạm lấy hàng (hàng tới người), tải 1–3 kg. Làm hết trong mô phỏng; khung Solid và PCB (chưa làm) sẽ theo kích thước của mô phỏng.
- **Thiết kế:**
  - **Lidar xuống giữa 2 tầng** (mặt quét 0.17 m → 0.12 m): trên nóc thì đáy kệ che lidar khi chui gầm. Hệ quả tốt: lidar thấy luôn pallet 15 cm, bỏ được vùng cấm vẽ tay ở mục 4.4.
  - **Mặt nâng** trên tầng 2: khớp trượt 0–3 cm, 2 cm/s (mô phỏng vít me), cảm biến tiếp xúc "có kệ". Giao diện topic `/lift/command`, `/lift/state`, `/lift/has_load` giữ nguyên cho cầu nối STM32 sau này.
  - **Kệ chở bằng vật lý thật** (kệ nằm trên mặt nâng nhờ ma sát), không dùng plugin `DetachableJoint`: plugin này gắn sẵn lúc khởi động (đọc world mẫu của Gazebo), mà kệ thì rải khắp kho.
  - **Kho:** 10 ô kệ (2 dãy × 5), 8 kệ, kệ cố định sát tường làm mốc cho AMCL. Một file `shelves.yaml` sinh ra world, danh sách dock của Nav2 và vùng cấm, để 3 nơi luôn khớp nhau.
  - **Chui gầm** bằng `docking_server` có sẵn của Nav2 Jazzy (loại `SimpleNonChargingDock`), mỗi ô là một dock.
  - Node `payload_manager` (dùng chung với xe thật): lọc scan (trụ ốc của xe; chân kệ khi chở), đổi footprint costmap theo tải, giới hạn tốc độ khi chở.
- **Bản đồ** lập lại 3 lần (mỗi lần đổi cỡ kệ): 100 % ô vật cản cách vật thật ≤ 10 cm, thấy 32/32 chân kệ. Lần cuối kém sắc hơn một chút (91.6 % ≤ 5 cm, trước đó 99.6–99.9 %), chưa rõ vì sao, vẫn dùng được.
- **Sự cố và cách tìm ra:**

  | # | Sự cố | Nguyên nhân (tìm bằng cách nào) | Cách sửa |
  |---|---|---|---|
  | 1 | Ra lệnh nâng, mặt nâng đứng yên ở 0 | Đẩy lệnh thẳng bằng `gz topic` vẫn không chạy → lỗi ở chế độ `use_velocity_commands` của `JointPositionController`; chế độ PID lực thì chạy | Dùng PID lực; `lift_sim` gửi quỹ đạo vị trí tăng dần 2 cm/s (vít me chạy đều bất kể tải), PID cứng + khâu I giữ được 3 kg |
  | 2 | Có PID lực nhưng nâng rất chậm (0.019 m sau 4 s) | Mặt nâng chạm đáy kệ sau 1.3 cm, lực P chỉ nhỉnh hơn trọng lượng kệ 29 N | Như trên |
  | 3 | `has_load` luôn False | Soi topic của Gazebo: cảm biến có báo chạm `deck_bottom` nhưng phát ở topic mặc định; với cảm biến contact, `<topic>` phải nằm **trong** `<contact>` | Chuyển thẻ `<topic>` |
  | 4 | Tên collision cho cảm biến sai | URDF → SDF thêm đuôi `_collision` (đọc SDF sinh ra bằng `gz sdf -p`) | Đặt tên URDF `lift_plate` |
  | 5 | Collision monitor sẽ bỏ hết điểm lidar | Đọc cấu hình: `min_height: 0.15` (mặc định Nav2), lidar mới ở 0.12 m | `min_height: 0.05` |
  | 6 | Chui gầm luôn báo "Collision detected" | Đọc mã nguồn `controller.cpp` của Jazzy: chỉ bỏ qua va chạm gần **đích** (= tâm dock lùi thêm 0.25 m); thứ "đụng" là vùng cấm của ô trong local costmap | Vùng cấm chỉ ở global costmap (để planner không xuyên gầm kệ); local costmap chỉ có vật cản lidar thấy |
  | 7 | Kệ 0.55 m: va chạm thật | Khe xe–chân kệ 5.5 cm; staging lệch ~8 cm ngang | Kệ 0.65 m rồi **0.75 m** (khe 15 cm; Kiva thật: kệ 0.9–1.2 m, xe 0.76 m), staging 1.0 m |
  | 8 | Tới staging lệch góc 24–51° | Quay tại chỗ với vùng chết (≥ 1.5 rad/s) vọt lố ~0.5 rad sau khi goal checker báo tới (14°) | Đi tới staging qua điểm xa hơn 1.2 m cùng trục ô → đoạn cuối thẳng (0.7 m chưa đủ: vẫn lệch 37°) |
  | 9 | Nâng kệ xong, lùi ra báo va chạm | Lúc còn hạ mặt nâng, 4 chân kệ đã vào costmap; sau khi nâng, điểm chân kệ bị lọc thành NaN, không tia nào xoá được → "chân kệ ma" trong footprint mới | `payload_manager` xoá costmap |
  | 10 | Vẫn lỗi như 9, lúc có lúc không | Xoá ngay khi **bắt đầu** nâng thì costmap vẫn giữ scan cuối chưa lọc và đánh dấu lại ngay | Xoá khi mặt nâng đã lên hết (`up`) |
  | 11 | Lùi ra khi chở: chân kệ lọt bộ lọc | Kệ nằm lệch trên mặt nâng ~7 cm (AMCL lệch lúc chui gầm), chân kệ ra tới 0.45 m, hộp lọc 0.41 m | Hộp lọc 0.48 m, footprint khi chở 0.84 m |
  | 12 | Ghi nhầm "docking tắt kiểm tra va chạm được" | Đọc launch của Nav2: `docking_server` phát thẳng `cmd_vel`, **không** qua collision monitor | Giữ kiểm tra va chạm của docking |

  Sai lầm của công cụ thử (cũng ghi lại vì đã làm mất thời gian): script chờ `amcl_pose` bằng QoS thường nên chờ mãi (AMCL phát kiểu giữ tin, chỉ khi xe chạy); khai báo tham số trong hàm gọi 2 lần nên crash; coi "có kết quả" là "thành công" nên một lần staging lệch 1.25 m vẫn báo OK; script không nạp profile Fast DDS nên node thử không thấy node khác; DiffDrive giữ lệnh cuối nên xe chạy mãi khi script quên gửi lệnh dừng.
- **Kết quả** (chu trình 10 bước: tới staging → chui gầm → nâng → lùi ra → chở tới trạm lấy hàng → tới staging → trả kệ → hạ → lùi ra → về trạm sạc; đo bằng vị trí thật; 6 lần mỗi cấu hình trên 3 kệ ở 2 dãy):

  | | Vùng chết 100 RPM (firmware hiện tại) | 50 RPM |
  |---|---|---|
  | Chu trình trọn vẹn | **6/6** | **6/6** |
  | Lệch góc tại staging (TB / lớn nhất) | 17.8° / 42.6° | 5.2° / 13.0° |
  | Lệch góc tại staging khi chở kệ | 33.3° / 66.3° | 8.5° / 13.6° |
  | Sau khi chui gầm lấy kệ (ngang / góc, lớn nhất) | 5.5 cm / 3.3° | 4.1 cm / 0.7° |
  | Kệ trả về lệch tâm ô | 3.1–8.7 cm | 0.6–5.6 cm |
  | Thời gian một chu trình | 74–107 s | 98–119 s |

  Trước các bản sửa cuối (kệ 0.75 m, xoá costmap khi `up`, hộp lọc 0.48 m), các lần chạy với kệ 0.55–0.65 m chỉ 1/3–2/3 chu trình xong.
- **Yêu cầu rút ra cho phần cứng:** khung theo kệ 0.75 × 0.75 m, gầm 0.18 m, xe cao ≤ 0.167 m khi hạ mặt nâng, lidar giữa 2 tầng; hạ `SPD_RPM_MIN` của firmware (đo ở trên: 50 RPM cho góc staging tốt hơn ~4 lần).
- **Bài học:**
  - Khi một thư viện báo "va chạm", đọc mã nguồn để biết nó coi cái gì là va chạm, đừng đoán.
  - Kích thước cơ khí phải chừa đủ cho sai số thật của định vị và điều khiển; thử bằng nhiều lần chạy, không tin một lần thành công.

### 4.10. Kho Kiva, giai đoạn 2: nhận diện kệ bằng chân kệ (PR #14)
- **Mục tiêu:** ở giai đoạn 1 xe chui gầm theo vị trí ô trên bản đồ (qua AMCL). Kệ trả về mỗi lần lệch 1–9 cm, lâu dần sẽ lệch nhiều; AMCL cũng lệch ~5 cm. Kiva thật dùng camera đọc mã dưới kệ; ở đây dùng **lidar nhìn 4 chân kệ**, không thêm phần cứng.
- **Cách làm:**
  - Node `shelf_detector.py`: trong `/scan_raw` (chưa lọc), gom điểm liên tiếp thành cụm, giữ cụm nhỏ (< 8 cm, chân kệ 3 cm), lùi tâm cụm thêm nửa bề rộng chân theo tia (lidar chỉ thấy mặt trước); tìm 4 cụm tạo hình vuông cạnh 0.72 m (2 điểm đối diện qua đường chéo → tính 2 đỉnh còn lại → tìm cụm gần); chọn kệ gần xe nhất phía trước. Hướng = trung bình 4 cạnh (nhân 4 để các cạnh lệch 90° trùng nhau).
  - Phát `/detected_dock_pose` với stamp **của scan**: `docking_server` so stamp này với giờ mô phỏng để tính timeout.
  - Đọc mã nguồn `SimpleNonChargingDock` của Jazzy trước khi cấu hình: mặc định `external_detection_translation_x = -0.2` và roll/pitch ±1.57 là cho camera đọc AprilTag, phải đặt về 0.
  - 2 loại dock: `shelf_dock` (lấy kệ, theo nhận diện) và `slot_dock` (trả kệ vào ô trống, không có chân để nhìn, theo bản đồ).
  - Bài thử thêm tham số `shelf_offset` (dời kệ bằng dịch vụ `set_pose` của Gazebo) và `detect` (chọn cách chui), đo sai số so với **kệ thật** chứ không so với ô.
- **Kết quả** (kệ dời lệch (+8, −6 cm, 4°), (−7, +5 cm, −5°), (+10, 0 cm, 0°), mỗi kiểu thử `ke_03` và `ke_06`, vùng chết 100 RPM):

  | | Chui theo chân kệ (`shelf_dock`) | Chui theo ô trên bản đồ (`slot_dock`) |
  |---|---|---|
  | Chui gầm lấy kệ thành công | **6/6** | 2/6 |
  | Chu trình trọn vẹn (10 bước) | **6/6** | 2/6 |
  | Lệch ngang so với kệ thật | **0.0–0.9 cm** | 1.6–10.9 cm |
  | Lệch góc so với kệ thật | 0.4–1.9° | 1.6–34.6° |
  | Lệch dọc so với tâm kệ (xe dừng quá) | 3.8–6.1 cm | 3.1–3.9 cm (2 lần chui được) |

  Theo bản đồ: 4/6 lần hỏng ở bước chui gầm, log `docking_server` báo "Collision detected" (xe vào đúng ô nhưng lệch kệ thật 9–11 cm, quỹ đạo quẹt chân kệ). Theo chân kệ: lệch ngang ≤ 0.9 cm dù kệ lệch ô tới 10.9 cm.
- **Sai lầm / sự cố:**

  | # | Sự cố | Nguyên nhân | Cách sửa |
  |---|---|---|---|
  | 1 | Loạt so sánh đầu mất hết kết quả (5/12 lần) | Log để ở `/tmp` của WSL; WSL tự khởi động lại (uptime về 0) và xoá `/tmp` | Log vào `~/agv_tests` (ext4), loạt thử chạy tách khỏi app (`nohup setsid`) |
  | 2 | Lần thử cmp10 hỏng khi khởi động | Trong lúc loạt thử chạy, đã sửa `mission.launch.py` để làm giai đoạn 3; workspace dùng `--symlink-install` nên launch đọc ngay file đang sửa, mà node mới chưa build | Cất phần giai đoạn 3 (`git stash`), chạy lại 3 lần cuối. Bài học: không sửa workspace đang được loạt thử dùng |
- **Còn lại:**
  - Xe dừng quá tâm kệ 4–6 cm theo chiều dọc ở mọi lần (đi ≥ 0.34 m/s do vùng chết, phanh muộn) → kệ nằm lệch tâm mặt nâng chừng đó. Có thể bù bằng `external_detection_translation_x`, hoặc hạ vùng chết (mục 4.9), chưa làm.
  - Trả kệ vẫn theo bản đồ (ô trống không có gì để nhận diện).
  - Bộ nhận diện thỉnh thoảng thấy "hình vuông" giả ở xa (ghép chân của các kệ / góc kệ cố định) nhưng luôn chọn kệ gần nhất phía trước nên chưa gây lỗi trong 12 lần thử.
- **Bài học:** đo sai số so với đúng đối tượng cần chạm tới (kệ thật), không so với vị trí "lẽ ra" của nó (ô trên bản đồ).

### 4.11. Kho Kiva, giai đoạn 3: quản lý đơn hàng (PR #15)
- **Mục tiêu:** thay "nhiệm vụ cố định" bằng **đơn hàng** như kho thật: tạo đơn "đưa kệ X tới trạm Y", hàng đợi có ưu tiên, người ở trạm xác nhận đã lấy hàng, lịch sử và thống kê.
- **Cách làm:**
  - Node `order_manager.py`: chạy đơn trong một luồng riêng, gọi các action của Nav2 (`NavigateThroughPoses` tới trước ô, `DockRobot`/`UndockRobot`, `NavigateToPose` tới trạm) và topic cơ cấu nâng; ROS spin bằng `MultiThreadedExecutor` ở luồng khác để luồng đơn hàng chờ được kết quả action.
  - Dữ liệu SQLite (thư viện chuẩn Python): bảng `orders` và `shelves` (kệ đang ở ô nào). Đơn đang chạy lúc tắt máy → đánh dấu lỗi khi khởi động lại.
  - Mỗi chặng thử lại 1 lần sau khi xoá costmap. Lỗi khi đang chở kệ → xe dừng, chờ người bấm "Đã xử lý" (xử lý tự động để giai đoạn 5).
  - Dashboard: thẻ *Đơn hàng* (tạo đơn, hàng đợi, nút xác nhận lấy hàng, lịch sử, thời gian TB); bản đồ vẽ từng kệ ở ô của nó, kệ đang chở vẽ theo xe. Lệnh đi qua `POST /api/command` với tiền tố `order`.
- **Kết quả** (mô phỏng, 2 lần, mỗi lần: 3 đơn `ke_03`, `ke_06` thường, `ke_01` gấp, thêm `ke_07` rồi huỷ; bài thử tự bấm xác nhận sau 3 s):

  | | Lần 1 | Lần 2 |
  |---|---|---|
  | Đơn xong / huỷ / lỗi | 3 / 1 / 0 | 3 / 1 / 0 |
  | Thứ tự chạy | ke_03 → **ke_01 (gấp)** → ke_06 | ke_03 → **ke_01 (gấp)** → ke_06 |
  | Thời gian mỗi đơn | 59–98 s | 76–111 s |
  | Kệ về ô lệch | 1.7–3.9 cm | 2.8–8.1 cm |
  | Chặng phải thử lại | 0 | 2 (chui gầm, lùi ra: "Collision detected"), thử lại đều qua |

  `ke_03` chạy đầu vì đơn của nó tạo trước khi các đơn kia vào hàng đợi.
- **Sai lầm / sự cố:**

  | # | Sự cố | Nguyên nhân | Cách sửa |
  |---|---|---|---|
  | 1 | Dashboard xếp hàng đợi theo số đơn, không theo thứ tự xe sẽ làm | `/order/state` lọc đơn chờ theo id | Sắp theo (ưu tiên giảm dần, id tăng dần) giống cách chọn đơn |
  | 2 | Lần chạy thử đầu không có log nào | Lệnh `wsl` thoát ngay sau `nohup setsid ... &`, tiến trình nền chưa kịp tách ra đã bị kết thúc cùng phiên | Đợi vài giây trước khi thoát `wsl` |
- **Còn lại:** chưa dừng khẩn / xử lý sự cố tự động (giai đoạn 5); chưa có pin và tự về sạc (giai đoạn 4); `order_manager` và `mission_server` chưa loại trừ nhau (dùng cùng lúc sẽ tranh điều khiển xe).
- **Bài học:** thử lại một lần sau khi xoá costmap gỡ được lỗi va chạm "ảo" thỉnh thoảng gặp; nhưng phải phân biệt lỗi trước và sau khi nâng kệ, vì khi đang chở thì không được tự bỏ đơn.

---

## 5. Tổng hợp sai lầm và bài học

| Giai đoạn | Sai lầm | Bài học |
|---|---|---|
| Firmware | Gửi UART kiểu polling tốn ~7 % CPU | Dùng DMA cho truyền dữ liệu nền |
| Môi trường | Cố dùng Ubuntu 22.04 / Humble có sẵn | Kiểm tra yêu cầu phần cứng (OpenGL) trước khi chọn bản phân phối |
| Mô hình | Dùng track thật cho xe skid-steer | Skid-steer cần track hiệu dụng, phải hiệu chỉnh |
| SLAM | Tự chỉnh tham số "cho đẹp" rồi đoán nguyên nhân 4 lần | Đối chiếu dự án tham khảo và cấu hình mặc định trước |
| Nav2 | Quên vật cản thấp hơn mặt quét lidar | Vật lidar không thấy phải đưa vào bản đồ dưới dạng vùng cấm |
| Nav2 | Firmware chỉ chặn tốc độ từng bánh | Phải giữ bán kính cua khi giới hạn tốc độ |
| Kiểm thử | Tin kết quả của script test có lỗi | Kiểm tra công cụ đo trước khi kết luận về hệ thống |
| Kiểm thử | Báo đã sửa khi chưa nhìn kết quả | Tự xác nhận (chụp màn hình, đo) trước khi báo |
| Nhiệm vụ | Gửi đích khi Nav2 chưa "active" | Kiểm tra trạng thái vòng đời như thư viện chuẩn |
| Nhiệm vụ | Đọc nhầm tin cũ của kênh giữ tin | Bỏ qua tin nhận trước khi gửi lệnh |
| Định vị | Để nguyên tham số AMCL mặc định dù xe quay tại chỗ nhiều và odom tốt hơn giả định | Đo độ nhiễu odom thật, chỉnh `alpha` theo số đo; kiểm giả thuyết bằng phép đo trước khi sửa |
| Dashboard | Bắt tay vào code khi mới được hỏi ý kiến | Hỏi "nên làm gì" thì trả lời và lập kế hoạch, chưa sửa code |
| Dashboard | Tưởng node mới tốn CPU do code của nó | Đo cả node cũ và node rỗng để tìm phần chung (`/clock` 1 kHz) |
| Mô phỏng | Để nguyên bước vật lý 1 ms có sẵn trong world | Đối chiếu world mẫu (Nav2 dùng 3 ms); bước nhỏ không có nghĩa là tốt hơn nếu máy không theo kịp |
| Kiểm thử | Đổ lỗi cho tiến trình nền và tin số đo lệch gốc toạ độ | Kiểm lại công cụ đo khi số liệu lạ |
| Kiva | Đoán ý nghĩa ngưỡng va chạm của docking, đặt vùng cấm ở local costmap | Đọc mã nguồn thư viện trước khi chỉnh tham số |
| Kiva | Kệ 0.55 m chừa khe 5.5 cm cho xe | Kích thước cơ khí phải tính theo sai số định vị / điều khiển đã đo |
| Kiva | Xoá costmap sai thời điểm (lúc bắt đầu nâng) | Nghĩ tới dữ liệu cũ còn trong bộ đệm khi đổi chế độ lọc |
| Kiểm thử | Để log loạt thử dài ở `/tmp` của WSL | Log vào thư mục bền, chạy tách khỏi phiên làm việc |
| Kiểm thử | Sửa workspace trong lúc loạt thử đang dùng nó | Làm việc khác trên nhánh / thư mục khác, hoặc chờ loạt thử xong |

## 6. Vấn đề còn mở
Đã giải quyết (xem 4.6): AMCL lệch ở khu phía đông; "collision ahead" trong lối hẹp (từ 24 lần một chặng xuống 0–4 lần một nhiệm vụ).

1. **Hệ thống đôi khi khựng khoảng 1 s** khi máy tải nặng (vòng điều khiển tụt xuống 9.6 Hz, TF cũ). **Trong mô phỏng đã giảm hẳn** nhờ bước vật lý 3 ms (mục 4.8: "Control loop missed" 0–1 mỗi lần so với 11–152). Vẫn cần theo dõi khi chạy trên Pi 4 (yếu hơn PC).
2. **Vùng chết của động cơ:** xe không đi chậm hơn 0.34 m/s và không quay chậm hơn 1.5 rad/s. Nên cải thiện điều khiển tốc độ thấp trên STM32 để hạ `SPD_RPM_MIN`. Đo ở mục 4.9: 50 RPM làm xe tới staging lệch góc ít hơn ~4 lần và đặt kệ chính xác hơn.
3. **Phần cứng chưa làm:** mở rộng 4 bánh, node cầu nối UART Pi ↔ STM32 (phải có giữ bán kính cua + covariance cho odom), YDLidar, BNO055, INA226.
4. **Kích thước xe** (track, wheelbase) vẫn là ước lượng, cần đo xe thật.
5. **Node Python trong mô phỏng tốn CPU vì `/clock`:** đã giảm từ 38–75 % xuống 30–48 % mỗi node nhờ bước 3 ms (mục 4.8); vẫn còn vì `/clock` 333 tin/s. Chỉ có trong mô phỏng.
6. **Dashboard chưa có đăng nhập:** chỉ mở trong LAN / Tailscale.
7. **Nav2 thỉnh thoảng không khởi động xong:** container nạp node nhưng phản hồi dịch vụ `load_node` cho launch bị timeout, launch chờ mãi (1/10 lần chạy ngày 08/10). Chưa tìm nguyên nhân; khi gặp thì chạy lại.
8. **Kiva:** lấy kệ đã chui theo chân kệ (lệch ngang ≤ 0.9 cm, mục 4.10) nhưng xe dừng quá tâm kệ 4–6 cm; trả kệ vẫn theo bản đồ. Đã có đơn hàng (mục 4.11); chưa có pin, xử lý sự cố (giai đoạn 4–5).

## 7. Cách ghi tiếp tài liệu này
Mỗi khi xong một phần việc hoặc sửa xong một sự cố, thêm vào mục tương ứng theo quy ước ở đầu file: mục tiêu, cách làm, sai lầm, nguyên nhân, cách sửa, kết quả có số liệu, bài học. Quy tắc này nằm trong skill `.claude/skills/report-log`.
