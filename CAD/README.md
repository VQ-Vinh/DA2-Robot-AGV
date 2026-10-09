# CAD – mô hình cơ khí xe AGV

File: `agv.FCStd` (FreeCAD 1.1). Mở bằng **File → Open**.

## Quy ước

- Đơn vị mm. Gốc toạ độ = `base_footprint` của URDF (mặt đất, tâm xe); X tiến, Y trái, Z lên.
- Giá trị âm trong `Params` phải gõ dạng `=-82 mm` (có dấu `=`), nếu không FreeCAD lưu thành chữ.
- Mọi kích thước nằm trong bảng **`Params`** (đầu cây Model). Sửa giá trị ở cột B rồi nhấn `Ctrl+R`, mô hình tự cập nhật. Cột C ghi nguồn; dòng có chữ `UOC LUONG` là số chưa đo trên linh kiện thật.
- Hai tấm mica là PartDesign Body (sketch + pad + lỗ), dùng để xuất DXF cắt laser. Động cơ, lidar và Pi là model tải về; bánh và gá động cơ tự vẽ.

## Các cụm

| Cụm | Nội dung |
|---|---|
| `Chassis` | 2 tấm mica 300 × 300 × 5 mm, 4 trụ ốc 100 mm. Tầng 1 có 4 chỗ khoét cho bánh, 4 lỗ chữ nhật 16 × 8 mm luồn dây động cơ, lỗ trụ, 8 lỗ M3 bắt gá động cơ, 4 lỗ M3 cho lidar, 4 lỗ M2.5 cho Pi |
| `Drive` | `WheelFL` = bánh "65 mm" (STEP: lốp Ø68 × 27, mâm, khớp nối lục giác 12 mm) + GA25-370 có encoder (STEP) + gá chữ L (tự vẽ); FR/RL/RR là Link tới FL. Bánh nằm gọn trong khung, mặt ngoài thụt vào 1.5 mm |
| `Lidar` | YDLidar X4 (lưới STL) trên 4 trụ M3 × 25 đặt đúng 4 chân đế, ở phía trước xe: tâm đầu quay tại x = 108 mm (`lidar_x`), đuôi quay về sau |
| `RasPi` | Raspberry Pi 4B (STEP) trên 4 trụ 10 mm, ở giữa xe (mép trước chui dưới đuôi lidar 8 mm, hở 5 mm theo chiều cao), xoay ngang: GPIO quay về sau (phía PCB), USB-C/HDMI quay về trước, USB/Ethernet quay sang trái |
| `PCB` | Hình giữ chỗ cho mạch PCB 120 × 120 mm trên 4 trụ 30 mm, phía sau Pi; 4 lỗ M3 trên tầng 1 (vị trí lỗ giả định cách mép 4 mm, sửa khi có file PCB thật) |
| `Battery` | Pin 3S 18650 + mạch BMS (STEP, 54 × 65 × 21 mm), nằm trên tầng 1 ngay dưới PCB, mạch BMS quay lên |
| `Lift` | Cơ cấu nâng dựng thử, theo linh kiện bán sẵn: mặt nâng 260 × 260 × 10 mm có 4 lỗ Ø9 cho đầu vít; mỗi góc một đai ốc đồng T8 (vành 22 mm, cao 15 mm) treo dưới mặt nâng bằng 2 trụ đồng M3 × 20, đi qua lỗ Ø24 ở tầng 2; 4 vít me T8 tại (±103, ±86) mm quay trong ổ bi F688ZZ ở tầng 1; 4 puly GT2 20 răng + đai kín 860 mm; động cơ bước NEMA17 (STEP, thân 42.3 × 42.3 × 40 mm, puly lỗ 5 mm, đầu nối quay về trước) đặt úp ngược trên tầng 1 ở (0, −124), đỡ bằng 4 trụ đồng M3 × 20 và 2 con lăn ép đai. Đổi `lift_pos` (0–30 mm) trong `Params` hoặc chạy `lift_demo.FCMacro` để xem chuyển động. Đai không tự cập nhật theo `Params` |

Chưa có: model 3D thật của PCB, cách giữ pin (dây đai hoặc gá), chốt định vị kệ và công tắc hành trình của cơ cấu nâng.

## Model tải từ GrabCAD

Thư mục `CAD/parts/` không đưa lên git (model của người khác); hình học đã được nhúng sẵn trong `agv.FCStd`.

| Linh kiện | Nguồn | Dùng |
|---|---|---|
| GA25 + encoder | grabcad.com/library/ga25-gear-motor-with-encoder-1 | STEP |
| Gá GA25 | grabcad.com/library/ga25-motor-bracket-1 | chỉ có DWG, FreeCAD chưa đọc được nên gá được vẽ lại theo ảnh, kích thước ước lượng |
| Raspberry Pi 4B | grabcad.com/library/raspberry-pi-4-model-b-1 | STEP |
| YDLidar X4 | grabcad.com/library/ydlidar-x4-4 | STL (lưới, không phải khối đặc) |
| Pin 3S 18650 | grabcad.com/library/3s-18650-li-ion-battery-1 | STEP (chỉ lấy 3 cell + mạch BMS; bỏ dây và băng keo) |
| Bánh 65 mm | grabcad.com/library/65mm-wheel-1 | STEP (đường kính ngoài thực 68 mm; khớp nối trong model là loại lỗ 3 mm, động cơ GA25 trục 4 mm cần loại lỗ 4 mm) |
| NEMA17 42 × 42 | grabcad.com/library/nema-17-stepper-motor-42x42mm-dimension-accurate-cad-model-1 | STEP, có bản vẽ kích thước kèm theo |

## Khác với URDF

- Bánh Ø68 mm thay vì Ø65 mm: trục bánh cao 34 mm.
- `track_width` = 270 mm thay vì 340 mm: bánh nằm trong khung 300 mm.
- `plate1_below_axle` = 17.5 mm thay vì 12.5 mm (theo gá động cơ, ước lượng). Mặt trên tầng 2 ở 161.5 mm (URDF 155 mm); mặt nâng khi hạ ở 173.5 mm (URDF 167 mm, gầm kệ 180 mm).
- Trụ ốc nằm ở (±135, ±105) mm thay vì 4 góc, để tránh chỗ khoét bánh.
- Lidar dời lên trước 108 mm (URDF `lidar_x` = 0), thân cao 58.4 mm (URDF 35 mm), đặt trên trụ 25 mm; mặt quét tính ra ở 119.9 mm.
