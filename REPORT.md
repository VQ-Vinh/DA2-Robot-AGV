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
| 08/10/2026 | Kho Kiva, giai đoạn 4: pin và tự về sạc | #16 |
| 08/10/2026 | Kho Kiva, giai đoạn 5: xử lý sự cố | #17 |
| 09/10/2026 | Xe vòng vòng / trả kệ lệch: vùng chết động cơ, kiểm tra tư thế, khoá kệ | #17 |
| 09/10/2026 | Lidar lệch bản đồ (bias con quay), cửa sổ Gazebo crash (driver WSL) | #17 |
| 09/10/2026 | Gộp #14–#17 vào `main`; README gốc thành trang giới thiệu có sơ đồ (kiến trúc, tiến độ, cấu trúc repo) | #18 |
| 09/10/2026 | Mô hình cơ khí FreeCAD: khung, động cơ + bánh, lidar, Raspberry Pi, PCB (giữ chỗ), pin, cơ cấu nâng | #20 |

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

### 4.12. Kho Kiva, giai đoạn 4: pin và tự về sạc (PR #16)
- **Mục tiêu:** xe tự quản lý pin như AMR thật: rảnh thì vào sạc, pin yếu thì không nhận đơn mới, sạc đủ thì làm tiếp.
- **Cách làm:**
  - `battery_sim.py` phát `/battery_state` đúng kiểu ROS (`sensor_msgs/BatteryState`, dòng dương = đang sạc), cùng topic với INA226 trên xe thật. Sạc khi xe (vị trí thật) cách điểm dock ≤ 6 cm, lệch góc ≤ 0.2 rad.
  - Trạm sạc là dock `SimpleChargingDock` có sẵn của Nav2, sinh trong danh sách dock từ `stations.yaml`. Đọc mã nguồn plugin trước: "đang sạc" = dòng > `charging_threshold`.
  - `order_manager`: rảnh → vào sạc; ngưỡng nhận đơn = `battery_low` + `order_reserve`; có đơn khi đang ở trạm và pin đủ → rời trạm rồi làm.
- **Sai lầm / sự cố** (mỗi lần sửa đều chạy lại bài thử pin):

  | # | Sự cố | Nguyên nhân (tìm bằng cách nào) | Cách sửa |
  |---|---|---|---|
  | 1 | `simple_charging_dock` trong file mẫu bật `use_external_detection_pose` | Đọc cấu hình: nó sẽ đọc `/detected_dock_pose` của `shelf_detector` → xe chui vào kệ thay vì vào trạm sạc | Tắt, trạm sạc theo bản đồ |
  | 2 | Cả 3 đơn hỏng "Failed initial dock detection"; bài thử vận chuyển cũ cũng hỏng, staging lệch ngang **5 m** | Khối tiếp điểm (cao 8 cm, cố ý thấp hơn lidar để bản đồ khỏi đổi) đặt ngay trước chỗ xe xuất phát: Nav2 lập đường xuyên qua nó, xe húc vào, bánh trượt nhưng odom vẫn tăng, AMCL tưởng xe đã tới staging. Giống hệt sự cố pallet ở mục 4.4 | Dời trạm sạc ra sau chỗ xuất phát 0.45 m (xe tiến về phía tây vào sạc), khối cao 0.15 m để lidar thấy, lập lại bản đồ |
  | 3 | Nhận đơn lúc pin 34 % (trên ngưỡng 30 %) rồi cạn 0 % giữa đơn | Chỉ so với ngưỡng, không tính phần pin cho chính đơn đó | Thêm `order_reserve` (10 %) |
  | 4 | 2 đơn hỏng ngay khi khởi động | Chỉ chờ action server của docking xuất hiện, chưa chờ Nav2 "active"; và còn 2 đơn "đang chờ" của lần thử trước trong SQLite | Chờ `bt_navigator` + `docking_server` active (như `mission_server`); mô phỏng (`reset_shelves`) huỷ đơn cũ |
  | 5 | Rời trạm thất bại mãi: "Robot is not in the dock" | Xe bật khỏi tiếp điểm > `docking_threshold` 5 cm | Nếu `UndockRobot` lỗi thì lùi thẳng 0.5 m bằng behavior `BackUp` (quay tại chỗ sát khối sẽ quét vào nó) |
  | 6 | Vào trạm sạc báo va chạm, xe chui chéo | Xe trong vòng 0.5 m quanh staging thì `docking_server` bỏ qua bước tới staging; lúc khởi động xe đứng ngay đó, quay lưng về trạm | Tự đi thẳng vào staging dọc trục trạm rồi mới `DockRobot` (như với kệ) |
  | 7 | Vẫn báo va chạm khi xe chỉ còn cách dock 3–7 cm, trên trục | Bản thân khối tiếp điểm (lidar thấy) nằm ngay rìa vùng bỏ qua va chạm 0.3 m quanh đích | `dock_collision_threshold: 0.45`; kiểm lại: các lần chui gầm kệ vẫn qua |
- **Kết quả** (pin đầu 45 %, hao / sạc nhanh gấp 8, 3 đơn, bài thử tự bấm xác nhận):
  - 3/3 đơn xong (77, 70, 57 s).
  - Sau đơn 1 pin 30.3 % < 40 % → dừng nhận đơn, vào trạm ngay lần đầu, sạc 28.3 → 80.4 % trong 187 s (mô phỏng), rời trạm làm đơn 2, 3; hết đơn thì về sạc.
  - Chỉ có 1 lần chạy đạt sau các bản sửa; trước đó 5 lần thử lộ 7 sự cố ở bảng trên.
- **Còn lại:** chưa xử lý pin cạn giữa đơn (pin mô phỏng xuống 0 % vẫn chạy); vị trí tiếp điểm dựa vào AMCL (±5 cm), xe thật cần tiếp điểm lò xo đủ rộng hoặc nhận diện trạm sạc.
- **Bài học:** vật cản thật mà cảm biến không thấy là nguy hiểm, kể cả khi "cố ý" làm vậy để bản đồ khỏi đổi; sửa bản đồ rẻ hơn nhiều so với tìm ra vì sao xe "đi" 5 m mà vẫn đứng yên.

### 4.13. Kho Kiva, giai đoạn 5: xử lý sự cố (PR #17)
- **Mục tiêu:** xe không được "chết lặng" hay tự làm điều nguy hiểm khi có sự cố: dừng đúng lúc, báo cho người, và làm tiếp đúng chỗ sau khi người xử lý xong.
- **Cách làm** (trong `order_manager`):
  - Đơn hàng là danh sách bước có chỉ số; sự cố "tiếp tục được" (dừng khẩn, đường bị chặn, lỗi một bước khi đang chở) → chờ người bấm "Đã xử lý" (`ack`) rồi **làm lại đúng bước đang dở**. Sự cố "không tiếp tục được" (rơi kệ, cơ cấu nâng lỗi) → đơn lỗi, kệ "chưa rõ vị trí" cho tới khi người cập nhật (`shelf <kệ> <ô>`).
  - Giám sát 5 Hz: mất tín hiệu "có kệ" > 1 s khi đang chở, cơ cấu nâng báo lỗi. Khi có sự cố: huỷ mọi goal Nav2 / docking đang chạy, gửi vận tốc 0.
  - Đường bị chặn: theo dõi feedback của Nav2 (`number_of_recoveries`, `distance_remaining`): Nav2 bắt đầu tự hồi phục hoặc không tiến về đích > 15 s → cảnh báo; > 60 s (hoặc chặng lỗi hẳn sau khi thử lại) → sự cố chờ người dọn đường.
  - Lỗi trước khi nâng kệ → trả đơn về hàng đợi (tối đa 2 lần).
  - Dashboard: nút **DỪNG KHẨN**, dải cảnh báo vàng, dải sự cố đỏ + nút "Đã xử lý".
  - Bài thử `fault_test.py` gây sự cố có chủ ý trong lúc xe đang chở kệ: dựng tường chắn ngang kho bằng dịch vụ `create` của Gazebo (rồi gỡ), gửi dừng khẩn, dịch kệ ra khỏi xe bằng `set_pose`.
- **Sai lầm / sự cố khi làm** (10 lần chạy bài thử sự cố + 4 lần thử đơn hàng bình thường):

  | # | Sự cố | Nguyên nhân (tìm bằng cách nào) | Cách sửa |
  |---|---|---|---|
  | 1 | Không đơn nào chạy | Log: luồng xử lý đơn chết vì `unhashable type: 'ClientGoalHandle'` (giữ goal handle trong `set`) | Dùng `list` |
  | 2 | Tường chắn 25 s mà không có cảnh báo nào, đơn vẫn xong (chặng dài 20 → 48 s) | Log Nav2: không có lỗi hay hồi phục; lidar chỉ thấy từng đoạn tường, planner liên tục vạch đường vòng qua phần chưa thấy → Nav2 "đang đi" chứ không lỗi | Theo dõi tiến độ `distance_remaining` (không tiến > 15 s → cảnh báo, > 60 s → sự cố), cộng số lần hồi phục |
  | 3 | Bước lùi ra (đang đội kệ) lỗi thì đơn bị **trả về hàng đợi và chạy lại từ đầu** — xe đội kệ đi tới staging, Nav2 hồi phục 31 lần | Phân loại sai: coi mọi `OrderError` là "lỗi trước khi nâng kệ" | Lỗi bất kỳ khi đang chở → sự cố chờ người, làm lại đúng bước đó; thêm chờ 1.5 s sau khi nâng cho costmap cập nhật |
  | 4 | Một số chỗ ném sự cố mà không ghi trạng thái → "Đã xử lý" không được nhận, xe chờ mãi | Đọc lại code: `ack` chỉ nhận khi `self.fault` có giá trị | Mọi sự cố đi qua `trigger_fault` |
  | 5 | Kết quả bài thử lộn xộn (đơn lạ, dừng khẩn sai lúc) | Bài thử cũ vẫn chạy và gửi lệnh vào mô phỏng mới (cùng domain); script dọn dẹp quên giết `fault_test.py` | Dọn dẹp giết mọi script thử |
  | 6 | Bài thử "đường bị chặn" chờ nhau với xe | Sự cố tới sau 75 s (Nav2 hồi phục + thử lại + chờ 60 s) mà kịch bản chỉ bấm "Đã xử lý" đúng một lần lúc gỡ tường | Kịch bản thấy sự cố lúc nào thì bấm lúc đó |
  | 7 | Chạy lại bài thử đơn hàng bình thường: 1 đơn trả kệ vào ô S2 lỗi 2 lần, xe dừng chờ người | Log docking: va chạm khi xe (đang chở) lệch ~0.3 m vào ô — footprint vuông 0.84 m chạm chân kệ ô S1 bên cạnh (khe chỉ ~17 cm mỗi bên). Footprint vuông đặt từ giai đoạn 1 cho kệ lệch 7 cm mọi hướng; từ giai đoạn 2 kệ chỉ lệch theo chiều dọc | Footprint khi chở hình chữ nhật: dọc ±0.44 m, ngang ±0.39 m |
  | 8 | Xe kẹt sát khối trạm sạc 60 s ngay đơn đầu | Vào trạm sạc lỗi rồi nhận đơn đi luôn từ chỗ sát khối, quay tại chỗ quẹt vào khối (bộ phát hiện kẹt báo đúng) | Vào trạm lỗi thì lùi thẳng 0.5 m trước |
  | 9 | Báo "không tiến về đích 15 s" cả khi xe chạy bình thường | `distance_remaining` của `NavigateThroughPoses` không giảm đều (có lúc 0) | Tự tính khoảng cách thẳng từ `current_pose` (feedback) tới đích cuối |
  | 10 | Lùi ra sau khi **đã hạ kệ** lỗi thì đơn bị trả về hàng đợi, xe quay lại nâng chính kệ đó | Quy tắc "chỉ trả đơn khi chưa chở" sai: sau khi hạ, xe không chở nhưng vẫn đang ở gầm kệ | Từ bước nâng kệ trở đi mọi lỗi thành sự cố chờ người; lùi ra lỗi thì lùi thẳng 1 m bằng `BackUp` |
  | 11 | Người dùng mở mô phỏng có GUI: xe **tự chạy lòng vòng quanh trạm sạc** ~100 s ngay khi mở | Log `docking_server`: "Timed out approaching dock", hết lượt thử. Xe xuất phát ở gốc bản đồ quay lưng về trạm sạc; `order_manager` lúc khởi động gọi về sạc, mỗi lần vào trạm lỗi lại lùi ra rồi đi vòng vào lại. Các bài thử headless không thấy vì chỉ nhìn kết quả đơn | Xe xuất phát ngay ở trạm sạc (launch `x/y/yaw:=auto`, AMCL `initial_pose` tại trạm); đang sạc sẵn thì bỏ bước đi tới staging. Log sau sửa: "Robot is already docked and/or charging, no need to dock" |
- **Kết quả** (tất cả sự cố gây ra trong lúc xe đang chở kệ tới trạm). Hai lần chạy cuối:

  | Tình huống | Kết quả |
  |---|---|
  | Tường chắn ngang kho 75 s | lần f7: cảnh báo → sự cố `duong_bi_chan` → gỡ tường + "Đã xử lý" → đơn **xong** (155.7 s). Lần f10 (sau khi đổi cách đo tiến độ): cảnh báo, nhưng **chưa thành sự cố** trước khi gỡ tường (xe men theo tường về phía đích nên vẫn "tiến"), đơn xong — bài thử tính là **SAI** theo tiêu chí đặt trước |
  | Dừng khẩn | xe đi thêm **0.0 cm** trong 3 s sau khi dừng; "Đã xử lý" → làm tiếp bước đang dở, đơn **xong** |
  | Dịch kệ ra khỏi xe | phát hiện sau **1.7 s**, xe dừng, đơn lỗi, kệ "chưa rõ vị trí"; `shelf ke_02 N5` cập nhật đúng |
  | Chạy đơn bình thường sau khi thêm xử lý sự cố | lần o8: **3/3** đơn xong (70–119 s), đơn gấp trước, đơn huỷ không chạy, kệ về ô lệch 5.0–9.7 cm |
  | Chạy đơn bình thường, xe xuất phát ở trạm sạc (sau sửa #11) | lần o9: **3/3** đơn xong (81–105 s), đơn huỷ không chạy, kệ về ô lệch 5.4–13.3 cm |
- **Chưa làm:** mất định vị (AMCL lạc) chưa phát hiện tự động; mất liên lạc với STM32 (sẽ do cầu nối UART + watchdog của firmware lo); pin cạn giữa đơn.
- **Bài học:** "Nav2 không báo lỗi" không có nghĩa là xe đang tiến tới đích — phải đo tiến độ; và mọi quyết định khi xe đang mang hàng phải tính tới trạng thái vật lý (đang đội kệ), không chỉ trạng thái phần mềm.

### 4.14. Xe "vòng vòng", trả kệ lệch 50°: vùng chết động cơ (PR #17)
- **Phát hiện:** người dùng chạy mô phỏng có GUI và thấy hai chuyện. (1) Một đơn dừng với sự cố "lùi ra thất bại (xe còn ở gầm ke_05)": theo Gazebo, kệ nằm trong ô S1 nhưng **xoay 37°**, còn xe lệch hướng 50° so với trục ô và cách kệ 0.53 m. (2) Nhiều lúc xe "quyết định không dứt khoát", quay vòng vòng.
- **Cách tìm nguyên nhân:**
  - Log `docking_server` của lần (1): khi trả kệ, docking báo "Collision detected" 2 lần, mỗi lần quay về staging rồi vào lại; lần 3 thì "Docking was successful". Bộ docking chỉ kiểm **khoảng cách** tới tâm ô (`docking_threshold`), không kiểm hướng. Vì vậy xe tới tâm ô khi đang lệch 50° vẫn được tính là xong, `order_manager` hạ kệ luôn. Sau đó xe lùi ra theo hướng lệch, đi chéo về phía đông bắc và chạm chân kệ ô S2 ("Collision Ahead").
  - Viết bài thử mới `agv_gazebo/scripts/motion_test.py`. Bài thử chạy 8 đơn liên tiếp, ghi vị trí thật (`/ground_truth`) 20 Hz theo từng bước của đơn. Mỗi bước tính: tổng góc quay, **góc quay thừa** (tổng trừ góc thực sự cần quay), số lần đảo chiều quay, và tư thế xe ngay trước khi hạ kệ.
  - Chạy cùng một code hai lần: có vùng chết động cơ (như xe thật) và tắt vùng chết (`rpm_min:=0`).

  | Góc quay thừa (TB / lớn nhất) | Có vùng chết (m1) | Tắt vùng chết (m2) |
  |---|---|---|
  | Chui gầm lấy kệ | 627° / **4261°** | 4° / 8° |
  | Trả kệ vào ô | 762° / **4198°** | 0° / 1° |
  | Tới staging | 293° / 454° | 227° / 246° |
  | Từ trạm về staging (đang chở) | 519° / 821° | 188° / 201° |

  Vùng chết của động cơ là nguyên nhân chính. Firmware ép mọi lệnh khác 0 lên ≥ 100 RPM, nên xe không đi chậm hơn 0.34 m/s và không quay chậm hơn 1.5 rad/s (mục 4.9). Đọc quỹ đạo từng bước thì có **ba cơ chế** riêng:
  1. **Quay tròn tại staging khi docking thử lại.** Mỗi lần thử lại, docking lùi xe về staging. Nó chỉ dừng khi xe vừa nằm trong 5 cm vừa đúng hướng trong 6° (`undock_linear/angular_tolerance` 0.05 / 0.1). Với bước quay tối thiểu 1.5 rad/s thì gần như không đạt được: xe quay hết vòng này tới vòng khác 8–15 s mỗi lần (đơn #2 lần m1: 60 s, quay 4270°, 29 lần đảo chiều).
  2. **Vọt lố sau mỗi lần quay tại chỗ.** Lệnh quay đi qua `velocity_smoother`, bộ này hạ lệnh dừng từ từ (3.2 rad/s²). Mọi giá trị khác 0 trên đường hạ đều bị vùng chết đẩy lại lên 1.5 rad/s, nên xe quay hết tốc cho tới khi lệnh về đúng 0. Đo: mỗi lệnh Spin quay thừa khoảng 30–80°.
  3. **Đi vòng tròn quanh đường gần đích.** Regulated Pure Pursuit giảm tốc khi gần đích, xuống tới 0.05 m/s (`min_approach_linear_velocity`). Khâu giữ bán kính cua (`motor_model`, mục 4.9) đẩy cả hai bánh lên cùng tỉ lệ cho tới khi bánh chậm nhất đạt 100 RPM. Vì vậy lệnh "0.05 m/s + bẻ lái nhẹ" thành 0.34 m/s và tốc độ bẻ lái gấp 7 lần: vòng điều khiển (có trễ) vọt lố, xe đi vòng tròn quanh điểm đích.
  - **Đoán sai trong lúc tìm:**
    - (a) Nghĩ kệ trượt trên mặt nâng khi xe quay nhanh. Lần đo đầu cho "kệ lệch so với xe 40 cm, 42°", nhưng đó là do lệnh `gz model` mất khoảng 1 s nên lấy vị trí kệ và xe ở hai thời điểm khác nhau. Đo lại lúc xe đứng yên ở trạm: kệ lệch 4–10 cm, xoay tới 11–23° so với xe. Phần lớn là do lúc chui gầm xe đã lệch, nhưng có ít nhất một lần kệ bị coi là rơi khi đang lùi (lần m3).
    - (b) Nghĩ cách "dừng, quay tại chỗ cho thẳng trục ô, rồi đi thẳng" (kiểu Kiva) sẽ dứt khoát hơn. Thực tế bộ quay tại chỗ không thể chỉnh góc nhỏ: xin quay 3° thì xe quay 40°, xin 11° thì quay 36° (lần s2, sau khi đã sửa smoother). Xe lắc qua lắc lại 8–12 lần rồi báo lỗi. Bỏ phương án này và phương án đi thẳng vào ô bằng `DriveOnHeading` (cần quay chỉnh chính xác trước).
    - (c) Đặt tốc độ docking bằng tốc độ thật (0.35 m/s) để bộ điều khiển "biết" xe chạy nhanh thế nào. Kết quả: quỹ đạo dự báo cua rộng hơn và quét chân kệ, docking báo va chạm 4/4 lần, xe kẹt sát chân kệ, planner không lập được đường (lần m4). Trả về 0.15.
- **Cách sửa (giữ):**
  - `nav2.yaml`:
    - `undock_linear/angular_tolerance` 0.10 m / 0.3 rad, để docking khi thử lại không phải quay tròn tìm đúng hướng.
    - `velocity_smoother` gần như bỏ giới hạn gia tốc (5 m/s², 20 rad/s²). Giới hạn thật đã do động cơ (DiffDrive 1 m/s², 3 rad/s²) đảm nhận.
    - RPP không bao giờ xin chậm hơn mức bánh chạy được: `min_approach_linear_velocity` và `regulated_linear_scaling_min_speed` 0.35, `rotate_to_heading_angular_vel` 1.5.
    - `behavior_server` 20 Hz.
  - `order_manager.checked_dock()`: sau khi chui gầm (trước khi nâng) và sau khi trả kệ (trước khi hạ), lấy tư thế xe từ TF `map → base_link` và so với trục ô. Nếu lệch hướng > 8°, hoặc khi trả kệ lệch vị trí > 10 cm, thì lùi thẳng ra staging rồi vào lại, tối đa 3 lần. Lệch > 40° thì quay tại chỗ bớt trước khi lùi (xin bớt 30° vì phần quay thừa). Ở giữa ô quay vẫn an toàn: nửa đường chéo kệ 0.53 m, chân kệ ô bên cạnh cách tâm ô ≥ 0.82 m.
  - Mô phỏng: khi nâng hết thì khoá kệ vào mặt nâng bằng khớp cố định (plugin `DetachableJoint` sinh trong mỗi kệ của world), hạ kệ thì mở khoá trước. Việc này tương ứng với chốt định vị trên mặt nâng của xe thật. Khi chỉ nhờ ma sát, chân kệ có thể lọt ra ngoài vùng lọc scan và thành "vật cản" sát xe. Plugin tự nối khớp mọi kệ với xe lúc mô phỏng bắt đầu, nên `lift_sim` mở khoá hết khi thấy xe. Lần đầu mở khoá lần lượt 8 kệ × 2 lần mất 16 s: xe đã nhận đơn và chạy khi vài kệ còn dính vào xe (lần s1), nên đổi sang gọi song song.
- **Sai lầm của công cụ thử:**
  - Một lần chạy bị hỏng vì `motion_test.py` của lần trước vẫn chạy: script dọn dẹp không giết nó, nên mỗi đơn được thêm 2 lần. Đã thêm vào script dọn dẹp.
  - Một lần sửa script bằng chuỗi có `\n` sinh ra lỗi cú pháp làm hỏng một lần chạy.
- **Kết quả** (8 đơn liên tiếp, có vùng chết; m1 = trước, m7 = sau khi sửa):

  | | m1 | m7 |
  |---|---|---|
  | Đơn xong | 8/8 | 8/8 |
  | Thời gian TB / đơn | 105 s (87–146) | 88 s (63–103) |
  | Nav2 tự hồi phục | 6 lần ở đơn #2, 4 lần ở #8… | 0 |
  | Quay thừa khi chui gầm (TB / max) | 627° / 4261° | 168° / 1251° |
  | Quay thừa khi trả kệ (TB / max) | 762° / 4198° | 28° / 75° |
  | Quay thừa trạm → staging (TB / max) | 519° / 821° | 228° / 390° |
  | Lệch hướng xe khi trả kệ (max, vị trí thật) | 3.2° | 4.1° |
  | Kệ sau đơn: lệch tâm / xoay (max) | 8.6 cm / 10.1° | 8.7 cm / 5.5° |

  - Kiểm tra tư thế đã bắt được một lần chui gầm lệch (ô N1) và vào lại đạt ở lần 2. Lần m5 (trước khi bỏ phương án (b)) tái hiện đúng lỗi của người dùng: docking trả kệ báo xong khi xe lệch +49°, và bước kiểm tra đã chặn không cho hạ kệ.
  - Bài thử sự cố chạy lại sau khi sửa (f11): **3/3 đạt**. Kịch bản "rơi kệ" giờ phải mở khoá trước, như khi chốt định vị bị gãy.
- **Còn lại:** tới staging vẫn quay thừa khoảng 300° (TB) vì đường đi qua điểm trước staging có góc gấp. Một lần chui gầm vẫn quay thừa 1251° (lần thử lại của docking). Gốc rễ là vùng chết, cách sửa thật nằm ở firmware (mục 6).
- **Bài học:** khi bộ chấp hành có vùng chết, mọi khâu "lệnh nhỏ dần về 0" (giảm tốc gần đích, làm mượt vận tốc, dung sai hẹp) đều biến thành vọt lố. Phải đo quỹ đạo thật theo từng bước thay vì chỉ nhìn đơn có xong không, vì 8/8 đơn xong vẫn che được một bước quay 4000°.

### 4.15. Lidar lệch bản đồ khi xe đứng yên; cửa sổ Gazebo không mở (PR #17)
- **Phát hiện:** người dùng chụp RViz: các chấm lidar lệch khỏi tường trên bản đồ. Lỗi cũ thứ hai: chạy `sim.launch.py nav:=true` thì cửa sổ Gazebo đôi khi không mở.
- **Lidar lệch: tìm nguyên nhân.**
  - So vị trí AMCL (TF `map → base_link`) với vị trí thật trên chính mô phỏng đang chạy: xe đứng yên ở trạm sạc nhưng localization lệch **30.8 cm, −11.2°**. Góc odom vẫn tăng đều (−18.7° → −19.0° trong 16 s) dù xe không chạy.
  - Đo vận tốc quay khi đứng yên: odom bánh = 0, IMU −0.000464 rad/s, EKF −0.000409 rad/s. IMU mô phỏng được cấu hình có bias con quay (`bias_mean` 0.0005 rad/s, cố ý giống IMU thật), và EKF cộng bias này vào góc yaw. AMCL chỉ cập nhật khi odom đổi quá `update_min_a`/`update_min_d`, nên khi xe đứng yên bản đồ và lidar lệch dần.
  - **Đoán sai và các lần sửa bỏ đi:**
    1. Khi hai bánh đứng yên, đặt phương sai tốc độ quay của odom bánh rất nhỏ (1e-6, rồi 1e-9), với lập luận xe skid-steer không thể quay khi bánh không lăn. Không có tác dụng: vận tốc quay của EKF chỉ giảm một nửa (0.000207 so với IMU 0.000474), góc vẫn trôi ~1.3°/phút. Nguyên nhân: giữa hai tin odom bánh (20 Hz), nhiễu quá trình của EKF làm phương sai tăng lại, và các tin IMU (71 Hz) lại kéo trạng thái theo.
    2. Hạ `update_min_a` của AMCL từ 0.2 xuống 0.05 rad, để lidar sửa lại mỗi khi odom trôi 3°. Không có tác dụng: đứng yên 10 phút thì odom trôi −24.4° và AMCL lệch −24.7°, không sửa chút nào. AMCL tin odom rất cao (`alpha` 0.05, mục 4.6), nên các hạt quay theo odom và lidar không kéo lại được. Đã trả về 0.2.
- **Lidar lệch: cách sửa.** Node mới `agv_localization/imu_bias.py`, dùng chung cho mô phỏng và xe thật: `/imu/raw → /imu`. Khi `/wheel/odom` báo bánh đứng yên quá 1 s, mọi tốc độ quay con quay đo được đều là bias. Node lấy trung bình trượt (hằng số thời gian 5 s) và trừ vào mọi mẫu. Bridge mô phỏng đổi `/imu` thành `/imu/raw`; cầu nối IMU của xe thật cũng phải phát `/imu/raw`.

  | Xe đứng yên ở trạm sạc | Odom yaw trôi | Localization lệch |
  |---|---|---|
  | Trước (i3, 300 s) | −2.39° (bias mỗi lần chạy khác nhau) | −2.5° |
  | Trước (i4, 600 s) | **−24.4°** | **−24.7°** |
  | Sau (i5, 300 s) | **+0.05°** | −0.2° |

  Chạy lại 8 đơn (m9): 8/8 xong, TB 88 s. Localization (TF) khi xe chạy lệch lớn nhất 13 cm, phần lớn ≤ 10 cm.
- **Sai lầm của công cụ đo:** ở mục 4.14, "AMCL sai max" ~0.3 m được coi là do `/amcl_pose` phát tin cũ, nhưng khi đó chưa kiểm chứng. Đã đổi `motion_test.py` sang đo bằng TF; khi xe chạy, sai số thật ≤ 13 cm.
- **Sự cố phát sinh khi sửa (khoá kệ của mục 4.14):**
  - Một lần chạy (m8) xe hoàn toàn không đi được. Vị trí thật đứng yên, còn odom tưởng xe đã lùi 1.5 m: bánh quay trượt vì xe vẫn còn dính vào 8 kệ. Lệnh mở khoá gửi một lần bằng `gz topic -p` có thể bị mất khi chưa kịp kết nối với plugin; lần m7 chạy được là do may.
  - Sửa: `lift_sim` dùng publisher gz-transport thường trực (Python `gz.transport13`), đọc trạng thái thật của khớp `/shelf/<kệ>/state`, và mỗi giây gửi lại cho tới khi khớp. Lệnh khoá chỉ được gửi trong 3 s sau khi nâng, để không tự khoá lại một kệ đã bị gỡ (bài thử rơi kệ).
  - Một lần (s3) xe vẫn không rời được trạm sạc dù các khớp đã báo `detached`: docking và BackUp báo vật cản ngay sau xe. Chạy lại không gặp lại. **Chưa tìm ra nguyên nhân.**
  - Lần s4: hạ kệ xong mặt nâng dừng ở 2.0 mm, đúng bằng dung sai 0.002 m, nên phép so sánh không đạt và sau 5 s báo "cơ cấu nâng lỗi". Đã nới dung sai lên 4 mm (hành trình 30 mm).
- **Cửa sổ Gazebo:**
  - Tái hiện 4 lần chạy đầy đủ: cửa sổ chết **1/4 lần** (exit 139). Backtrace: segfault trong driver NVIDIA D3D12 của WSL (`driCreateNewScreen` → `libnvwgf2umx.so`) lúc tạo màn hình OpenGL, khi RViz cũng đang khởi tạo. Mở tay sau đó thì chạy bình thường.
  - Đây là lỗi driver, không sửa được trong dự án. Launch giờ chạy thẳng `gz sim -g` trong một vòng lặp: chết vì segfault thì mở lại sau 3 s, tối đa 4 lần; người dùng tự đóng cửa sổ thì không mở lại. Thử 3 lần: một lần cửa sổ crash rồi tự mở lại được.
- **Bài thử sự cố sau khi sửa (f12): 2/3.** Dừng khẩn và rơi kệ đạt. Tường chắn: có cảnh báo nhưng chưa thành sự cố trước khi gỡ tường (giống f10, vì xe men theo tường nên vẫn được tính là "đang tiến").
- **Bài học:** mô phỏng có nhiễu và bias giống thật thì phải có bước bù giống thật (hiệu chỉnh bias con quay khi đứng yên). Và trước khi chỉnh tham số theo một giả thuyết, đo xem đại lượng trung gian (ở đây là vận tốc quay của EKF) có đổi đúng như dự đoán không.

### 4.16. Mô hình cơ khí trong FreeCAD (PR #20)

- **Mục tiêu:** có bản vẽ cơ khí thật của xe để lắp và cắt mica, thay cho các hộp/trụ trong URDF. Đợt 1 gồm khung, 4 cụm động cơ + bánh, lidar, Raspberry Pi. Mạch PCB, các mạch khác và cơ cấu nâng để sau.
- **Cách làm:** FreeCAD 1.1 điều khiển qua MCP `freecad-live` (dựng trực tiếp trong cửa sổ đang mở). File `CAD/agv.FCStd`, gốc toạ độ và trục giống `base_footprint` của URDF. Mọi kích thước nằm trong bảng `Params`, hình học tham chiếu bằng expression. Hai tấm mica là PartDesign Body (sketch ràng buộc đủ, 0 bậc tự do); 3 cụm bánh còn lại là Link tới cụm trước trái. Xem `CAD/README.md`.
- **Lần 1 – khối đơn giản:** động cơ, lidar, Pi là hộp/trụ theo số tra trên trang bán. Bánh nằm ngoài mica như URDF (track 340 mm).
- **Lần 2 – model thật và đổi bố trí bánh:** thay bằng model GrabCAD (GA25 + encoder và Pi 4B dạng STEP, YDLidar X4 dạng STL); bánh 65 mm tự vẽ. Theo yêu cầu, bánh không được lồi ra ngoài khung: khoét 4 chỗ trên tầng 1, bánh thụt vào trong, track còn 270 mm.
- **Sự cố và khác biệt so với URDF:**

| # | Vấn đề | Nguyên nhân (tìm bằng cách nào) | Xử lý |
|---|---|---|---|
| 1 | URDF đặt mặt dưới tầng 1 cao hơn trục bánh 12.5 mm, đúng bằng bán kính hộp số GA25 | Khi vẽ gá chữ L: đế gá phải nằm giữa động cơ và mica, không còn chỗ | CAD dùng 17.5 mm (khoảng từ đế gá tới tâm trục, ước lượng). Thân xe cao hơn URDF 5 mm: mặt trên tầng 2 ở 160 mm thay vì 155 mm |
| 2 | URDF giả định lidar cao 35 mm, đặt trên khối linh kiện 50 mm | Đo lưới STL: X4 cao 58.4 mm (khớp một trong hai số của trang bán; số 63 mm dùng ở lần 1 là sai) | Lidar đặt trên 4 trụ 26.6 mm đúng vị trí 4 chân đế; đỉnh 140 mm, cách mặt dưới tầng 2 15 mm. Vị trí mặt quét trong thân lidar (thấp hơn đỉnh 20 mm) vẫn là ước lượng |
| 3 | Bánh thụt vào khung thì lỗ trụ ốc ở góc (135, 135) chỉ còn cách mép chỗ khoét 0.25 mm | Tính khi đặt chỗ khoét 64 × 31.5 mm quanh bánh | Dời 4 trụ ốc vào (±135, ±105) mm. Góc mica ngoài chỗ khoét còn một tai rộng 18 mm |
| 4 | Mạch encoder trong model thò ra 22.2 mm khỏi tâm trục, hơn khoảng 17.5 mm tới mica | Đo hộp bao model ở các góc xoay 0–315° | Xoay động cơ để mạch encoder quay xuống; điểm thấp nhất cách đất 10.3 mm |
| 5 | Gá động cơ tải về chỉ có file DWG | FreeCAD chưa cài bộ chuyển DWG | Vẽ lại gá theo ảnh chụp kèm file, kích thước ước lượng |
| 6 | Lệnh dựng cụm động cơ báo "Request timed out" | Kiểm tra lại tài liệu: mọi đối tượng đã tạo đủ, chỉ phản hồi bị trễ | Chia lệnh nhỏ hơn, kiểm tra trạng thái trước khi chạy lại |
| 7 | Sketch báo "Failed to parse expression 'Constraints.h'" | `h` là đơn vị giờ trong bộ đọc biểu thức của FreeCAD | Đổi tên ràng buộc thành `notch_ext` |
| 8 | Không tìm thấy lỗ bắt trên model Pi sau khi xoay | `transformGeometry()` biến mặt trụ thành NURBS | Dùng `transformed()` (phép dời cứng); 4 lỗ ở (3.5, 3.5), (61.5, 3.5), (3.5, 52.5), (61.5, 52.5) đúng bản vẽ hãng |
| 9 | 4 trụ đỡ và 4 lỗ lidar trên tầng 1 lệch khỏi chân đế lidar (đối xứng ngược qua tâm), người làm đồ án nhìn mô hình phát hiện | `Mesh.Points` của FreeCAD trả toạ độ đã tính cả phép xoay 180° của lưới; lúc đặt trụ lại đổi dấu thêm một lần. Lần kiểm tra trước chỉ so hộp bao nên không lộ | Lấy lại vị trí chân đế trực tiếp theo toạ độ xe: (22.5, 31.4), (22.5, −30.6), (−34.5, 25.4), (−34.5, −24.6) mm; xác nhận bằng hình chiếu trên và chiếu đứng |
| 10 | Dời PCB về sau (`pcb_x` = −82 mm) thì PCB không nhúc nhích, FreeCAD báo "Failed to convert to Quantity" | Ô bảng tính nhận `-82 mm` thành chuỗi chữ chứ không phải số (xem nội dung ô: `'-82 mm`) | Gõ dạng công thức `=-82 mm`; quét lại cả bảng, không còn ô nào là chuỗi |
| 11 | Sketch lỗ bắt PCB hỏng khi toạ độ lỗ đổi sang âm | Sketch ràng buộc bằng khoảng cách dương tính từ gốc toạ độ | Vẽ 4 lỗ đối xứng quanh tâm PCB rồi dời cả sketch theo `pcb_x`, `pcb_y` |
| 12 | Pin đặt xong lại nằm xuyên qua PCB | `BoundBox` của model pin báo cao 157 mm trong khi pin thật cao 21 mm (hộp bao không chặt), nên phép căn đáy pin sai | Dùng `optimalBoundingBox()`; phát hiện nhờ nhìn hình chiếu sau khi đặt |

- **Lần 3 – bố trí lại tầng 1 (theo yêu cầu):** thêm 4 lỗ chữ nhật 16 × 8 mm luồn dây động cơ và hình giữ chỗ cho PCB 120 × 120 mm (chờ file 3D thật). Thứ tự từ trước ra sau: lidar (tâm x = 108 mm), Pi xoay ngang ở giữa, PCB phía sau. PCB không đặt được cạnh lidar khi lidar ở tâm xe: từ chân lidar tới mép mica chỉ có 114 mm. Sau khi bố trí lại: 37 khối không cặp nào giao nhau; khe giữa lidar–Pi 2 mm, Pi–PCB 4 mm, PCB–mép sau 8 mm. URDF vẫn để lidar ở tâm xe, chưa sửa.
- **Lần 4 – nới bố trí:** lần 3 chật theo chiều dọc xe (lidar + Pi + PCB dài 278 mm trên tấm 300 mm, khe 2 và 4 mm, cổng USB-C của Pi quay vào đuôi lidar) và không còn chỗ cho pin. Xoay lidar 90° cho đuôi quay sang trái (chiều dài theo trục xe 102 → 71 mm), khe thành: lidar–Pi 15.4 mm, Pi–PCB 9 mm, PCB–mép sau 20 mm. 4 lỗ luồn dây dời ra y = 64–72 mm (ngoài mép PCB, phía trên thân động cơ). Đặt thử khối pin 3S 105 × 35 × 25 mm (ước lượng) sát mép phải, giữa hai chỗ khoét bánh. 41 khối, không cặp nào giao nhau. Vị trí 4 chân lidar sau khi xoay được đo lại trên lưới đã đặt, có kiểm tra mỗi cụm đủ 46 đỉnh và rộng dưới 8 mm (rút kinh nghiệm sự cố 9).
- **Lần 5 – xoay lidar lại, pin xuống dưới PCB (theo yêu cầu):** lidar xoay 90° bị chê về thẩm mỹ nên trả đuôi về phía sau. Để vẫn đủ chỗ, mép trước Pi chui dưới đuôi lidar 8 mm (đuôi lidar thấp nhất 88.1 mm, đỉnh Pi 82.8 mm, hở 5.3 mm). Pin đổi từ khối ước lượng sang model pack 3S 18650 + BMS (54 × 65 × 21.3 mm), đặt ngay dưới PCB; trụ PCB nâng từ 10 lên 30 mm, pin cách đáy PCB 8.7 mm. Khe: Pi–PCB 6 mm, PCB–mép sau 16 mm. Mặt PCB ở 86.6 mm, linh kiện trên PCB cao quá 33 mm sẽ chạm mặt quét lidar (120 mm). 41 khối, không cặp nào giao nhau. Chưa có chi tiết giữ pin.
- **Lần 6 – dựng thử cơ cấu nâng (4 vít me + đai):** chọn kiểu này vì tự giữ tải khi mất điện, nâng phẳng và né được lidar; các kiểu khác (1 vít ở tâm, xi lanh điện, 4 servo, nêm trượt) vướng lidar, không lọt khoang 100 mm, hoặc không tự giữ. 4 vít T8 tại (±110, ±86) mm quay trong ổ bi ở tầng 1; mặt nâng mang 4 ống đai ốc dài 46 mm thả xuyên tầng 2, để vít không nhô lên khỏi mặt nâng. Đai GT2 kín dài 872 mm chạy sát mặt tầng 1, động cơ N20 treo dưới tầng 1 ở (0, −116) với 2 con lăn ép đai. Kết quả: mặt nâng cao 172 mm khi hạ (gầm kệ 180 mm, dư 8 mm) và 202 mm khi nâng hết; nâng hết đai ốc vẫn ăn vít 15 mm. Kiểm tra ở cả 2 vị trí: không va chạm với chi tiết khác (chỉ có đai chồng lên puly, do puly vẽ là trụ đặc). Vấn đề lộ ra: (a) khi hạ, ống đai ốc xuống tới 116 mm, cắt mặt quét lidar 120 mm; hai ống trước chỉ cách lidar 86 mm, che khoảng 10.6° ở mỗi bên trái/phải; (b) vít chỉ có một ổ đỡ ở tầng 1; (c) chiều dài đai 872 mm không phải cỡ chuẩn, cần rãnh chỉnh con lăn để căng đai; (d) động cơ nâng treo dưới gầm, cách đất 16 mm. Kích thước puly, ổ bi, động cơ N20 là ước lượng.
- **Lần 7 – thử dời 2 vít trước ra xa lidar:** dời từ (110, ±86) về (50, ±120) mm, động cơ nâng chuyển ra nhánh đai phía sau tại (−140, 0), đai còn 827 mm. Không va chạm ở cả 2 vị trí. Góc che của mỗi ống khi hạ giảm từ 10.6° xuống 6.9° (vít: 5.3° → 3.4°), hướng che chuyển từ ±89° sang ±116°. Cái giá: mặt nâng bị hẫng 80 mm phía trước 2 vít trước; ống chỉ cách mép mặt nâng 2 mm; nhánh đai trước chạy ngang trước cổng USB-C/HDMI của Pi, cách 6.3 mm; nhánh đai chéo cách đầu bu lông gá động cơ sau khoảng 3 mm (ước tính, bu lông chưa dựng). Đánh giá: lợi ít so với cái giá. Hướng khác chưa thử: để đầu vít chui vào bề dày mặt nâng, khi đó đáy ống ở 126 mm, cao hơn mặt quét 120 mm, và chỉ còn vít 8 mm che lidar.
- **Lần 8 – trả về hình chữ nhật, đầu vít chui vào mặt nâng:** 4 vít về lại (±110, ±86) mm, động cơ nâng về (0, −116), đai 872 mm. Mặt nâng khoan thủng 4 lỗ Ø9; vít dài thêm 10 mm, đỉnh vít thấp hơn mặt trên của mặt nâng 1 mm khi hạ hết; ống đai ốc ngắn từ 46 xuống 36 mm. Kết quả: đáy ống khi hạ ở 126 mm, cao hơn mặt quét lidar (120 mm) 6 mm, nên ống không còn chắn lidar; chỉ còn 4 vít 8 mm (mỗi vít trước che 5.3°, mỗi vít sau 2.0°). Nâng hết đai ốc vẫn ăn vít 15 mm. 66 khối, không va chạm ở cả 2 vị trí (trừ đai chồng lên puly do cách vẽ). Còn lại: khe 6 mm giữa đáy ống và mặt quét phụ thuộc vào chiều cao lắp lidar thật và vị trí mặt quét trong thân lidar (đang là ước lượng).
- **Lần 9 – đối chiếu linh kiện nâng với hàng bán thật:** các lần trước tự đặt kích thước mà chưa tra hàng thật; người làm đồ án hỏi mới tra. Kết quả: vít me T8 bước 2 mm + đai ốc đồng (vành 22 mm, dày 3.5 mm, cao 15 mm, 4 lỗ 3.5 mm trên vòng 16 mm), ổ bi F688ZZ (8 × 16 × 5, vành 18 × 1.1 mm), puly GT2 20 răng lỗ 5 hoặc 8 mm (đường kính ngoài 16, cao 16 mm) đều có bán; đai GT2 kín có cỡ 852, 860, 900 mm, không thấy 872 hay 880 mm. Sai so với hàng thật: (a) "ống đai ốc đồng" Ø16 × 36 mm là chi tiết tự chế, không có bán; (b) đai 872 mm không có cỡ; (c) puly lỗ 3 mm cho trục động cơ N20 chưa tìm thấy loại 20 răng. Đã sửa: thay ống bằng đai ốc T8 thật treo dưới mặt nâng bằng 2 trụ đồng M3 × 20, lỗ tầng 2 nới lên Ø24; dời vít về x = ±107 mm để vòng đai dài 859.9 mm, khớp cỡ 860 mm. Đáy đai ốc khi hạ ở 127 mm (cao hơn mặt quét lidar 7 mm); nâng hết đai ốc ăn vít 14 mm. 74 khối, không va chạm ở cả 2 vị trí. Còn mở: chọn động cơ nâng có trục khớp puly bán sẵn; lỗ Ø24 ở tầng 2 không còn dẫn hướng nên vít chỉ có ổ đỡ ở tầng 1; chưa kiểm tra hàng có sẵn ở cửa hàng Việt Nam.
- **Lần 10 – động cơ nâng NEMA17:** thay N20 bằng động cơ bước NEMA17 (thân 42.3 mm, dài 34 mm, trục 5 mm khớp puly GT2 20 răng lỗ 5 mm bán sẵn), treo dưới tầng 1. Đặt ở (0, −116) như N20 thì ốc M3 bắt động cơ nằm ngay dưới 2 con lăn ép đai, nên lùi động cơ ra (0, −124); vòng đai dài thêm 16 mm, bù bằng cách dời vít từ x = ±107 về ±103 mm để giữ đai 859.9 mm (cỡ 860). Tầng 1 thêm lỗ gờ định tâm Ø22.5 và 4 lỗ M3 cách nhau 31 mm. Kết quả: 75 khối, không va chạm ở cả 2 vị trí; động cơ cách mép phải 4.8 mm, cách đất 16 mm; đầu ốc M3 cách con lăn 2.2 mm. Kích thước NEMA17 lấy theo chuẩn chung, chưa đối chiếu datasheet của mã động cơ cụ thể.
- **Lần 11 – model NEMA17 thật:** thay khối hộp bằng model STEP của động cơ đang có (kèm bản vẽ: thân 42.3 × 42.3 × 40 mm, gờ Ø22 × 2, trục Ø5 thò 22 mm, 4 lỗ M3 cách 31 mm). Lỗ bắt và gờ định tâm trên model khớp đúng các lỗ đã khoan trên tầng 1. Khác với lần 10: thân dài 40 mm chứ không phải 34 mm như đã chọn, nên động cơ chỉ còn cách đất 10 mm (trước 16 mm). Đầu nối quay về phía trước xe để không chạm mép mica và đai ốc trục con lăn. 73 khối, không va chạm ở cả 2 vị trí.
- **Lần 12 – đưa động cơ nâng lên trên tầng 1:** treo dưới gầm chỉ cách đất 10 mm nên đổi: động cơ úp ngược (trục hướng xuống) trên 4 trụ đồng M3 × 20 bắt vào đúng 4 lỗ M3 đã có; puly nằm giữa mặt động cơ và mica, đai giữ nguyên 860 mm. Lỗ gờ định tâm Ø22.5 trên tầng 1 thu lại còn Ø8 cho đầu trục (đầu trục ở 51 mm, trong bề dày mica). Thân động cơ cao 75–115 mm: thấp hơn mặt quét lidar 5 mm, cách tầng 2 40 mm. Dưới gầm không còn chi tiết nào của cơ cấu nâng; điểm thấp nhất của xe lại là mạch encoder (10.3 mm). 77 khối, không va chạm ở cả 2 vị trí. Phải dùng trụ 20 mm: trụ 25 mm thì đỉnh động cơ lên đúng 120 mm, chắn lidar.
- **Lần 13 – model bánh xe thật:** thay bánh tự vẽ bằng model STEP (lốp, mâm, khớp nối lục giác). Model cho thấy bánh "65 mm" có đường kính ngoài thực 68 mm, nên trục bánh lên 34 mm và cả thân xe cao thêm 1.5 mm: mặt trên tầng 2 ở 161.5 mm, mặt nâng khi hạ 173.5 mm (gầm kệ 180 mm, dư 6.5 mm). Khớp nối lục giác thò ra 4.75 mm phía trong bánh và đè lên gá động cơ, nên nới `hub_gap` từ 5 lên 8.5 mm (khớp nối cách gá 1.25 mm, trục vào khớp 9.75 mm); động cơ và lỗ bắt gá lùi vào trong 3.5 mm. Nhân dịp này đổi lidar sang trụ chuẩn M3 × 25, mặt quét tính ra 119.9 mm thay vì đặt cứng 120 mm. Kiểm tra (lốp thay bằng trụ đặc để tính nhanh, vì lần chạy với hình lốp thật bị hết thời gian chờ): không va chạm ngoài 2 chỗ biết trước là trục động cơ trong lòng bánh và trục Ø4 trong lỗ khớp nối Ø3.3 (model khớp nối là loại lỗ 3 mm, cần mua loại lỗ 4 mm). Hệ quả: đỉnh động cơ nâng (116.5 mm) chỉ còn thấp hơn mặt quét lidar 3.4 mm; ổ bi vít nâng nằm ngay trên đế gá động cơ bánh (chạm mặt, không cắt nhau); encoder cách đất 11.8 mm.
- **Kiểm tra (lần 2):** 35 khối đều hợp lệ, không cặp nào giao nhau. Hộp bao cả xe đúng ±150 mm theo cả x và y (bánh không lồi), cao 160 mm. Tâm bánh (±100, ±135, 32.5) mm. Lưới lidar không phải khối đặc nên chỉ so hộp bao. Đổi `wheelbase` 200 → 180 → 200: bánh và chỗ khoét dời theo rồi về đúng chỗ cũ. Đóng và mở lại file không lỗi.
- **Giới hạn:** model GrabCAD do người dùng tự vẽ, chưa đối chiếu với linh kiện thật (trừ lỗ bắt Pi). Kích thước gá động cơ và vị trí lỗ bắt gá là ước lượng. URDF và các tham số Nav2 (track 0.34 m, `effective_track` 0.45) chưa cập nhật theo track 270 mm.
- **Bài học:** vẽ chi tiết lắp ghép thật làm lộ những chỗ mô hình khối bỏ qua (bề dày gá, chiều cao lidar, mạch encoder); nên lấy model/số đo linh kiện trước rồi mới chốt kích thước khung.

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
| Kiva | Làm trạm sạc thấp hơn lidar "để bản đồ khỏi đổi" | Vật cản thật phải thấy được bằng cảm biến; lập lại bản đồ |
| Kiva | Chính sách pin chỉ so với ngưỡng | Tính cả phần pin cần cho việc sắp làm |
| Kiva | Coi "Nav2 chưa báo lỗi" là xe đang tiến | Đo tiến độ về đích |
| Kiva | Trả đơn về hàng đợi khi xe đang đội kệ | Quyết định theo trạng thái vật lý của xe |
| Kiva | Tin "Docking was successful" là xe đã vào đúng ô | Tự kiểm tra tư thế trước khi nâng / hạ |
| Kiva | Chỉ nhìn số đơn xong, không đo quỹ đạo | Đo góc quay thừa từng bước (8/8 đơn xong vẫn có bước quay 4000°) |
| Kiva | Giải quyết vùng chết bằng quay tại chỗ chính xác | Đo trước: một lần Spin quay thừa ~30 deg, chỉ chỉnh được khi đang đi |
| Định vị | Chỉnh covariance / ngưỡng AMCL để chữa trôi do bias con quay | Đo vận tốc quay của EKF trước; sửa tận gốc: ước lượng bias khi bánh đứng yên |
| Mô phỏng | Gửi lệnh gz một lần rồi coi như xong | Đọc trạng thái thật và gửi lại tới khi khớp |

## 6. Vấn đề còn mở
Đã giải quyết (xem 4.6): AMCL lệch ở khu phía đông; "collision ahead" trong lối hẹp (từ 24 lần một chặng xuống 0–4 lần một nhiệm vụ).

1. **Hệ thống đôi khi khựng khoảng 1 s** khi máy tải nặng (vòng điều khiển tụt xuống 9.6 Hz, TF cũ). **Trong mô phỏng đã giảm hẳn** nhờ bước vật lý 3 ms (mục 4.8: "Control loop missed" 0–1 mỗi lần so với 11–152). Vẫn cần theo dõi khi chạy trên Pi 4 (yếu hơn PC).
2. **Vùng chết của động cơ:** xe không đi chậm hơn 0.34 m/s và không quay chậm hơn 1.5 rad/s. Nên cải thiện điều khiển tốc độ thấp trên STM32 để hạ `SPD_RPM_MIN`. Đo ở mục 4.9: 50 RPM làm xe tới staging lệch góc ít hơn ~4 lần và đặt kệ chính xác hơn. Mục 4.14: đây là nguyên nhân chính khiến xe "vòng vòng". Tắt vùng chết thì góc quay thừa khi chui gầm giảm từ 627° xuống 4°; phần mềm mới chỉ giảm được một phần. Hướng thử: ở tốc độ thấp, firmware "đá" duty cao lúc khởi động rồi để PID giữ tốc độ, vì ma sát động nhỏ hơn ma sát tĩnh (chưa kiểm chứng trên xe thật).
3. **Phần cứng chưa làm:** cầu nối IMU phát `/imu/raw` (node `imu_bias` trừ bias con quay rồi mới tới EKF, mục 4.15), chốt định vị trên mặt nâng (kệ không trượt khi xe quay; mô phỏng đã khoá kệ, mục 4.14), mở rộng 4 bánh, node cầu nối UART Pi ↔ STM32 (phải có giữ bán kính cua + covariance cho odom), YDLidar, BNO055, INA226.
4. **Kích thước xe** (track, wheelbase) vẫn là ước lượng, cần đo xe thật. Mô hình FreeCAD (mục 4.16) đổi track còn 270 mm (bánh nằm trong khung), thân xe cao hơn URDF 5 mm, lidar thật cao 58.4 mm; URDF, `effective_track` và các tham số Nav2 chưa cập nhật theo. Cần đo GA25-370, gá động cơ và YDLidar thật.
5. **Node Python trong mô phỏng tốn CPU vì `/clock`:** đã giảm từ 38–75 % xuống 30–48 % mỗi node nhờ bước 3 ms (mục 4.8); vẫn còn vì `/clock` 333 tin/s. Chỉ có trong mô phỏng.
6. **Dashboard chưa có đăng nhập:** chỉ mở trong LAN / Tailscale.
7. **Cửa sổ Gazebo crash lúc khởi động** (driver NVIDIA D3D12 của WSL, 1/4 lần chạy): launch tự mở lại (mục 4.15). Có 1 lần xe không rời được trạm sạc vì "vật cản" ngay sau xe, chưa rõ nguyên nhân, chạy lại thì hết.
8. **Nav2 thỉnh thoảng không khởi động xong:** container nạp node nhưng phản hồi dịch vụ `load_node` cho launch bị timeout, launch chờ mãi (1/10 lần chạy ngày 08/10). Chưa tìm nguyên nhân; khi gặp thì chạy lại.
9. **Kiva:** lấy kệ đã chui theo chân kệ (lệch ngang ≤ 0.9 cm, mục 4.10) nhưng xe dừng quá tâm kệ 4–6 cm; trả kệ vẫn theo bản đồ. Đã có đơn hàng (mục 4.11), pin và tự sạc (mục 4.12), xử lý sự cố (mục 4.13); chưa phát hiện mất định vị.

## 7. Cách ghi tiếp tài liệu này
Mỗi khi xong một phần việc hoặc sửa xong một sự cố, thêm vào mục tương ứng theo quy ước ở đầu file: mục tiêu, cách làm, sai lầm, nguyên nhân, cách sửa, kết quả có số liệu, bài học. Quy tắc này nằm trong skill `.claude/skills/report-log`.
