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

## 6. Vấn đề còn mở
1. **AMCL lệch ở khu phía đông kho:** ở 1 trong 5 lần chạy, AMCL lệch tới 1.1 m, xe dừng sát tường và collision monitor chặn cả bước lùi, nên xe kẹt. Cần tìm cách các dự án tham khảo xử lý.
2. **Vùng chết của động cơ:** xe không đi chậm hơn 0.34 m/s và không quay chậm hơn 1.5 rad/s. Nên cải thiện điều khiển tốc độ thấp trên STM32 để hạ `SPD_RPM_MIN`.
3. **Phần cứng chưa làm:** mở rộng 4 bánh, node cầu nối UART Pi ↔ STM32 (phải có giữ bán kính cua + covariance cho odom), YDLidar, BNO055, INA226.
4. **Kích thước xe** (track, wheelbase) vẫn là ước lượng, cần đo xe thật.

## 7. Cách ghi tiếp tài liệu này
Mỗi khi xong một phần việc hoặc sửa xong một sự cố, thêm vào mục tương ứng theo quy ước ở đầu file: mục tiêu, cách làm, sai lầm, nguyên nhân, cách sửa, kết quả có số liệu, bài học. Quy tắc này nằm trong skill `.claude/skills/report-log`.
