<div align="center">

# 🤖 DA2 · Robot AGV kho hàng kiểu Kiva

**Xe tự hành chui gầm, nâng kệ hàng, chở tới trạm lấy hàng rồi trả về ô và tự đi sạc.**

ROS 2 Jazzy · Nav2 · Gazebo Harmonic · Raspberry Pi 4 · STM32F4 · FreeRTOS

[Mô phỏng & ROS 2](RasPi/README.md) · [Nhật ký quá trình](REPORT.md) · [Firmware STM32](STM32/)

</div>

---

## ✨ Xe làm được gì

| 📦 Nhận đơn | 🛞 Chui gầm & nâng kệ | 🚚 Chở tới trạm | 🔋 Tự sạc | 🚨 Xử lý sự cố |
|:---:|:---:|:---:|:---:|:---:|
| Hàng đợi, ưu tiên, huỷ đơn trên web | Lidar tìm 4 chân kệ, căn giữa ≤ 1 cm | Nav2 + vùng cấm, footprint đổi theo tải | Pin thấp thì về trạm, sạc đủ mới nhận đơn | Dừng khẩn, đường bị chặn, rơi kệ |

---

## 🗺️ Kho hàng

```mermaid
flowchart LR
    subgraph W["Tây kho"]
        IN(["🔵 Khu nhập hàng"])
        CH(["🟢 Trạm sạc"])
        PK(["🟠 Trạm lấy hàng"])
    end
    subgraph N["Dãy bắc · xe vào hướng +y"]
        N1[N1] --- N2[N2] --- N3[N3] --- N4[N4] --- N5[N5]
    end
    AISLE{{"Lối giữa rộng 2.45 m"}}
    subgraph S["Dãy nam · xe vào hướng −y"]
        S1[S1] --- S2[S2] --- S3[S3] --- S4[S4] --- S5[S5]
    end
    W <--> AISLE
    AISLE <--> N
    AISLE <--> S
```

8 kệ 0.75 × 0.75 m nằm trong 10 ô, cách nhau 1.1 m. Bố trí lấy từ một file duy nhất, [`shelves.yaml`](RasPi/src/agv_mission/config/shelves.yaml), từ đó sinh ra world Gazebo, danh sách dock của Nav2 và mặt nạ vùng cấm.

---

## 🧩 Kiến trúc

```mermaid
flowchart TB
    subgraph USER["👤 Người vận hành"]
        WEB["🌐 Web dashboard<br/>điện thoại / laptop"]
    end

    subgraph PI["🍓 Raspberry Pi 4 · ROS 2 Jazzy"]
        DASH["web_dashboard<br/>REST + SSE"]
        OM["order_manager<br/>hàng đợi · SQLite · sự cố"]
        NAV["Nav2<br/>planner · RPP · docking · behaviors"]
        LOC["AMCL + EKF<br/>imu_bias"]
        PM["payload_manager<br/>lọc scan · footprint"]
        SD["shelf_detector<br/>tìm 4 chân kệ"]
    end

    subgraph MCU["⚙️ STM32F407 · FreeRTOS"]
        SPD["PID tốc độ + feedforward"]
        ENC["Encoder"]
        DRV["L298N × 2"]
    end

    LIDAR(["📡 YDLidar"])
    IMU(["🧭 BNO055"])
    BAT(["🔋 INA226"])
    MOT(["4 × GA25-370"])

    WEB <-->|HTTP| DASH
    DASH <--> OM
    OM -->|goal, dock| NAV
    NAV -->|cmd_vel| MCU
    LIDAR --> PM --> NAV
    LIDAR --> SD --> NAV
    LOC --> NAV
    MCU -->|odom, IMU, pin| LOC
    IMU --> MCU
    BAT --> MCU
    SPD --> DRV --> MOT
    MOT --> ENC --> SPD
```

---

## 🔁 Một đơn hàng

```mermaid
sequenceDiagram
    autonumber
    actor U as 👤 Người vận hành
    participant OM as order_manager
    participant N as Nav2 / docking
    participant L as Cơ cấu nâng

    U->>OM: add ke_03 tram_lay_hang
    OM->>N: tới staging trước ô (đoạn cuối thẳng)
    OM->>N: chui gầm theo chân kệ (lidar)
    OM->>OM: kiểm tra hướng ≤ 8°
    OM->>L: nâng
    OM->>N: lùi ra · chở tới trạm
    OM-->>U: đã tới, chờ lấy hàng
    U->>OM: confirm
    OM->>N: về ô · vào ô theo bản đồ
    OM->>OM: kiểm tra hướng ≤ 8°, vị trí ≤ 10 cm
    OM->>L: hạ
    OM->>N: lùi ra · về trạm sạc
```

---

## 🚨 Khi có sự cố

```mermaid
flowchart LR
    E{"Sự cố"} -->|Dừng khẩn<br/>Đường bị chặn > 60 s| R["⏸️ Xe dừng<br/>chờ người"]
    E -->|Rơi kệ<br/>Cơ cấu nâng lỗi| F["⛔ Đơn lỗi<br/>kệ chưa rõ vị trí"]
    E -->|Lỗi trước khi nâng kệ| Q["↩️ Trả đơn về hàng đợi"]
    R -->|"Đã xử lý"| C["▶️ Làm lại đúng bước đang dở"]
    F -->|"shelf ke_xx Ny"| C2["📍 Cập nhật vị trí kệ"]
```

```mermaid
flowchart LR
    B["🔋 Pin < 30 % + 10 % dự trữ cho đơn"] --> G["Về trạm sạc"] --> H["Sạc tới 80 %"] --> J["Nhận đơn tiếp"]
```

---

## 🧭 Định vị

```mermaid
flowchart LR
    ENC["Encoder bánh"] --> WO["/wheel/odom<br/>vx tin được · wz kém"]
    GYRO["Gyro"] --> RAW["/imu/raw"] --> IB["imu_bias<br/>học bias khi bánh đứng yên"] --> IMU["/imu"]
    WO --> EKF["EKF"]
    IMU --> EKF
    EKF --> ODOM["/odom"]
    ODOM --> AMCL["AMCL + bản đồ"]
    SCAN["Lidar"] --> AMCL
    AMCL --> MAP["vị trí trên bản đồ"]
```

---

## 📊 Kết quả trong mô phỏng

8 đơn liên tiếp, có mô phỏng vùng chết động cơ giống firmware (≥ 100 RPM):

| | Trước khi chỉnh | Sau khi chỉnh |
|---|:---:|:---:|
| Đơn xong | 8/8 | **8/8** |
| Thời gian TB / đơn | 105 s | **88 s** |
| Quay thừa khi trả kệ (lớn nhất) | 4198° | **75°** |
| Nav2 phải tự hồi phục | nhiều lần | **0** |
| Góc trôi khi đứng yên 5 phút | 2.4°–12° | **0.05°** |

| Bài thử | Kết quả |
|---|---|
| Chui gầm theo chân kệ (kệ đặt lệch) | 6/6, lệch ngang ≤ 0.9 cm · theo bản đồ chỉ 2/6 |
| Đơn hàng + ưu tiên + huỷ | 3/3 |
| Sạc giữa chừng | 3/3 đơn, tự về sạc rồi làm tiếp |
| Sự cố (dừng khẩn, rơi kệ, tường chắn) | dừng khẩn & rơi kệ: 4/4 lần · tường chắn: 2/4 lần |

Cách đo, các lần sai và cách sửa: [REPORT.md](REPORT.md).

---

## 🛣️ Tiến độ

```mermaid
timeline
    title Các mốc chính
    09/2026 : Firmware STM32 · PID tốc độ
            : Chọn ROS 2 Jazzy + Gazebo Harmonic
    10/2026 : Mô phỏng · EKF · SLAM · Nav2
            : Web dashboard
            : Kiva 1 · nâng kệ · Kiva 2 · tìm chân kệ
            : Kiva 3 · đơn hàng · Kiva 4 · pin & sạc · Kiva 5 · sự cố
    Tiếp theo : Cầu nối UART Pi ↔ STM32
              : PCB · khung xe · chốt định vị mặt nâng
              : Chạy trên xe thật
```

---

## 🚀 Chạy thử

```bash
# WSL Ubuntu 24.04 + ROS 2 Jazzy (xem RasPi/README.md để cài)
cd ~/agv_ws && colcon build --symlink-install && source install/setup.bash
ros2 launch agv_gazebo sim.launch.py nav:=true
```

Sau đó mở **http://localhost:8080**, chọn kệ và trạm rồi bấm **Tạo đơn**.

---

## 📁 Cấu trúc repo

```mermaid
flowchart LR
    R["📂 Src"] --> RP["📂 RasPi/src<br/>ROS 2"]
    R --> ST["📂 STM32<br/>firmware"]
    R --> RE["📄 REPORT.md<br/>nhật ký"]
    R --> CA["📂 CAD<br/>mô hình FreeCAD"]
    RP --> D["agv_description<br/>URDF"]
    RP --> G["agv_gazebo<br/>world · sim · bài thử"]
    RP --> L["agv_localization<br/>EKF · imu_bias"]
    RP --> NV["agv_navigation<br/>SLAM · Nav2 · docking"]
    RP --> M["agv_mission<br/>đơn hàng · dashboard"]
    ST --> APP["App<br/>logic"]
    ST --> MOD["Modules<br/>driver"]
    ST --> CORE["Core<br/>CubeMX sinh"]
```

<div align="center">

Đồ án 2 · HK261

</div>
