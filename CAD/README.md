# CAD – mô hình cơ khí xe AGV

File: `agv.FCStd` (FreeCAD 1.1). Mở bằng **File → Open**.

## Quy ước

- Đơn vị mm. Gốc toạ độ = `base_footprint` của URDF (mặt đất, tâm xe); Z lên. **Đầu xe là phía Pi/PCB/camera, tức hướng −X của file CAD** (ban đầu lidar là đầu xe; đổi theo yêu cầu khi gắn camera, mô hình không dựng lại). `export_meshes.py` quay lưới 180° để +X của URDF là hướng tiến.
- Giá trị âm trong `Params` phải gõ dạng `=-82 mm` (có dấu `=`), nếu không FreeCAD lưu thành chữ.
- Mọi kích thước nằm trong bảng **`Params`** (đầu cây Model). Sửa giá trị ở cột B rồi nhấn `Ctrl+R`, mô hình tự cập nhật. Cột C ghi nguồn; dòng có chữ `UOC LUONG` là số chưa đo trên linh kiện thật.
- Hai tấm mica là PartDesign Body (sketch + pad + lỗ), dùng để xuất DXF cắt laser. Động cơ, lidar và Pi là model tải về; bánh và gá động cơ tự vẽ.

## Các cụm

| Cụm | Nội dung |
|---|---|
| `Chassis` | 2 tấm mica 300 × 300 × 5 mm, 4 trụ ốc 100 mm, vẽ thành 2 đoạn vì thị trường chỉ có trụ tới 50 mm: đoạn dưới đực-cái 50 + đoạn trên cái-cái 50 vặn vào nhau, hai đầu là lỗ ren, bắt vít M3 × 10 xuyên mica. Tầng 1 có 4 chỗ khoét cho bánh, 4 lỗ chữ nhật 16 × 8 mm luồn dây động cơ tại (±64, ±68) mm (cạnh đuôi encoder, không nằm trên thân động cơ), lỗ trụ, 8 lỗ M3 bắt gá động cơ, 4 lỗ M3 cho lidar, 4 lỗ M2.5 cho Pi, 2 lỗ M3 cho gá camera |
| `Drive` | `WheelFL` = bánh "65 mm" (STEP: lốp Ø68 × 27, mâm, khớp nối lục giác 12 mm) + GA25-370 có encoder (STEP) + gá chữ L dày 3 mm (tự vẽ theo gá bán sẵn); FR/RL/RR là Link tới FL. Bánh nằm gọn trong khung, mặt ngoài thụt vào 1.5 mm |
| `Lidar` | YDLidar X2L (STEP từ model SolidWorks grabcad.com/library/ydlidar-x2l-lidar-1, người làm đồ án xuất sang STEP): đế + mạch 101 × 57 × 11 mm, đầu quay Ø60 × 18 mm, động cơ thò xuống 19.4 mm dưới đế; bỏ 3 chân tự vẽ của tác giả model. Ở phía sau xe (+X của file CAD), động cơ quay về sau: tâm đầu quay tại x = 92 mm (`lidar_x`; đuôi lidar ở x = 148.5, mép mica 150). Đáy đế cao hơn mặt tầng 1 50 mm (`lidar_standoff`): 3 trụ đồng M3 cái-cái 50 mm qua 3 lỗ Ø3.5 trên đế, tại (`lidar_x` + 36, ±18) và (`lidar_x` − 40.25, 0). Mặt quét 126.5 mm, giữa đầu quay (ước lượng), cao hơn đỉnh NEMA17 (116.5) và đỉnh linh kiện PCB (112.6); 4 đai ốc nâng không còn cắt mặt quét, chỉ còn 4 trụ tầng, 4 vít me và cờ báo nâng (khi hạ hết) |
| `RasPi` | Raspberry Pi 4B (STEP) trên 4 trụ M2.5 cái-cái 5 mm, **nằm dưới mạch PCB** (tâm Pi = `pcb_x` − 8 mm), cạnh dài 85 mm dọc trục x: **cổng USB/Ethernet quay về phía lidar** (+X của file CAD, phía sau xe; đổi ngày 28/10/2026, trước đó quay sang trái), cạnh có USB-C/micro HDMI quay sang phải (−Y), hàng GPIO ở cạnh +Y. Trước cổng USB còn trống 40 mm để cắm cáp (kiểm tra bằng `common()`); 4 lỗ trên tầng 1 tại (`pi_x` − 10 ± 29, `pi_y` ± 24.5) |
| `PCB` | Mạch thật `PCB/agv_mainboard` 150 × 125 mm (`pcb_len` theo x, `pcb_wid` theo y) trên 4 trụ cái-cái 30 mm, tâm x = −54 mm (`pcb_x`, trước là −56 với tấm giữ chỗ 120 × 120), che lên Pi. 4 lỗ M3 cách mép 4 mm, khớp với H1–H4 trên file KiCad. Mép trước PCB cách gá camera 3.5 mm. Kiểm tra bằng `common()` với mọi chi tiết khác: không chồng lấn. Mô hình 3D của bo Discovery (file STEP gắn trong KiCad) bị lỗi: sinh ra 7 mặt phẳng vô hạn ở z = 101.2 mm, hiện thành một tấm thừa trên PCB. Đã bỏ các mặt này và thay Discovery bằng tấm giữ chỗ `DiscoveryBoard` 66 × 97 × 1.6 mm, đáy cao hơn mặt PCB 11 mm. Nguồn vào là jack XT60 (J1) ở góc trước-trái PCB, miệng cắm quay về đầu xe; trước miệng cắm trống 25 mm (kiểm tra bằng `common()`) |
| `Battery` | Pack 3S: 3 cell 18650 ghép bằng 6 đế ghép pin (STEP, bước 20 mm), mạch BMS 3S 20A HW-375 (STEP, 58.7 × 20 × 3.55 mm) nằm trên pin; cả pack 63 × 67 × 25.6 mm, **treo dưới tầng 1, chính giữa xe** (tâm (0, 0)), đỉnh mạch cách mica 3 mm. Giữ bằng `BatHolder`: máng chữ U in 3D liền khối (94 × 74 × 31.6 mm, đáy và thành 3 mm, không khoét cửa sổ, ~41 g PETG) có 4 tai 12 × 12 × 6 mm ở góc, mỗi tai 1 lỗ M3 tại (±41, ±29) mm và khoét Ø7 × 2.8 ở mặt trên cho đai ốc hãm của vít cấy. Hai lỗ sau nằm dưới bo Pi (đáy Pi cách mica 2.9 mm) nên dùng vít M3 đầu chìm từ trên mica, khoét côn mica; đáy gá cách đất 19.9 mm. File in: `print/battery_holder.stl` / `.step` (đã lật úp sẵn để in không cần đỡ) |
| `Camera` | Raspberry Pi Camera Module 3 Wide (STEP; model là bản thường, bản Wide ống kính cao hơn ~1 mm) nhìn thẳng về đầu xe, cố định, ở mép tầng 1 phía Pi/PCB: trục ống kính tại y = 0, cao 88 mm, đầu ống kính ở x ≈ −148 mm (mép mica −150); đỉnh cụm 104 mm, thấp hơn mặt quét lidar (123 mm). Gá chữ L in 3D `CamBracket` (36 × 16.5 × 47.5 mm): 4 vấu có lỗ mồi Ø1.7 cho vít tự ren M2 × 6 bắt camera, khe luồn cáp dẹt cao 67–77 mm (cáp đi phía trên đai nâng, đỉnh đai 64.5 mm, rồi luồn dưới PCB về Pi), đế bắt 2 vít M3 × 12 từ dưới mica tại (−143, ±12.5) + đai ốc. Gá cách đai nâng 10.7 mm, cách mép PCB 3.5 mm (PCB 150 × 125). Cáp CSI 15 chân 20 cm đi kèm là đủ (đường đi ~10 cm). File in: `print/camera_bracket.stl` / `.step`. Gá là hình tĩnh, đổi `cam_x`/`cam_z` phải dựng lại |
| `Lift` | Cơ cấu nâng dựng thử, theo linh kiện bán sẵn: mặt nâng 260 × 260 × 10 mm có 4 lỗ Ø9 cho đầu vít và 8 lỗ M3 (khoét côn) cho vít bắt trụ treo đai ốc; mỗi góc một đai ốc đồng T8 (vành 22 mm, cao 15 mm) treo dưới mặt nâng bằng 2 trụ đồng M3 cái-cái 20 mm, đi qua lỗ Ø24 ở tầng 2; 4 vít me T8 tại (±115, ±72) mm quay trong ổ bi F688ZZ ở tầng 1 (ổ bi cách mép gá động cơ bánh 8.5 mm, puly cách đầu vít gá 16 mm; `lift_x` + `lift_y` phải giữ = 187 mm để đai vẫn dài 852 mm); 4 puly GT2 20 răng + đai kín 852 mm (cỡ có bán); động cơ bước NEMA17 (STEP, thân 42.3 × 42.3 × 40 mm, puly lỗ 5 mm, đầu nối quay về trước) đặt úp ngược trên tầng 1 ở (0, −110), đỡ bằng 4 trụ đồng M3 đực-cái 20 mm và 2 con lăn ép đai (ròng rọc trơn GT2, lỗ 5 mm – ước lượng). Đổi `lift_pos` (0–30 mm) trong `Params` hoặc chạy `lift_demo.FCMacro` để xem chuyển động. Puly, ổ bi, con lăn theo `Params`; riêng đai là hình tĩnh, không tự cập nhật. **Công tắc hành trình** (2 × KW12, bắt sát mặt dưới tầng 2 tại y = `limit_y` = 100): công tắc DƯỚI ở x = `limit_down_x` = 70 có cần gạt thò lên qua khe 12 × 6 mm trên tầng 2, mặt nâng đè cần xuống 1.5 mm khi hạ hết; công tắc TRÊN ở x = `limit_up_x` = 40 có cần gạt nằm ngang (cao 148 mm), một **cờ** in 3D 6 × 6 × 45.5 mm bắt dưới mặt nâng (vít M3 từ trên) đi qua khe 10 × 10 mm trên tầng 2 và chạm cần khi nâng đủ 30 mm. File in cờ: `print/limit_flag.stl` / `.step` (lỗ mồi Ø2.6 sâu 10 mm ở đầu trên cho vít M3) |

Trụ đồng xuyên mica đều là loại **cái-cái + vít M3 từ phía bên kia tấm mica** (ren đực của trụ chỉ dài ~6 mm, không xuyên nổi mica 5 mm): trụ lidar 35, trụ PCB 30, trụ treo đai ốc nâng 20 (vít đầu chìm M3 × 16 từ trên mặt nâng); riêng trụ đỡ NEMA17 là đực-cái 20 (đầu đực vặn vào lỗ ren của động cơ). Loại trụ ghi ở nhãn từng trụ và cột ghi chú của `Params`.

Mô hình Pi vẫn là Raspberry Pi 4B. Pi 5 có cùng kích thước bo (85 × 56 mm) và cùng 4 lỗ bắt nên không ảnh hưởng gá; khác ở vị trí cổng (Pi 5 có cổng PCIe và cổng camera 22 chân). Công tắc nguồn và nút dừng khẩn chưa có chỗ gắn trong CAD.

## Model tải từ GrabCAD

Thư mục `CAD/parts/` không đưa lên git (model của người khác); hình học đã được nhúng sẵn trong `agv.FCStd`.

| Linh kiện | Nguồn | Dùng |
|---|---|---|
| GA25 + encoder | grabcad.com/library/ga25-gear-motor-with-encoder-1 | STEP |
| Gá GA25 | grabcad.com/library/ga25-motor-bracket-1 | chỉ có DWG, FreeCAD chưa đọc được nên gá được vẽ lại theo ảnh, kích thước ước lượng |
| Raspberry Pi 4B | grabcad.com/library/raspberry-pi-4-model-b-1 | STEP |
| YDLidar X2L | `parts/Lidar Assembly.step` (người làm đồ án xuất từ model SolidWorks grabcad.com/library/ydlidar-x2l-lidar-1) | STEP; 4 khối: đầu quay, đế + mạch, động cơ, khớp nối (thay YDLidar X4 từ 28/10/2026) |
| Đế ghép pin 18650 | grabcad.com/library/cell-spacer-battery-18650-1 | STEP (đế 1 cell, ghép 3 cái mỗi đầu) |
| Mạch BMS 3S 20A HW-375 | grabcad.com/library/18650-lithium-battery-bms-3s-20a-hw-375-rev-2-0-1 | STEP, có ảnh kích thước |
| Bánh 65 mm | grabcad.com/library/65mm-wheel-1 | STEP (đường kính ngoài thực 68 mm; khớp nối trong model là loại lỗ 3 mm, động cơ GA25 trục 4 mm cần loại lỗ 4 mm) |
| Pi Camera Module 3 | grabcad.com/library/raspberry-pi-camera-module-v3-2 | STEP; lỗ bắt 21 × 12.5 mm, bo 25 × 23.9 mm khớp bản vẽ hãng |
| NEMA17 42 × 42 | grabcad.com/library/nema-17-stepper-motor-42x42mm-dimension-accurate-cad-model-1 | STEP, có bản vẽ kích thước kèm theo |

## Đưa vào mô phỏng

`export_meshes.py` xuất lưới STL cho `RasPi/src/agv_description/meshes/` và in các số đo để chép vào `agv.urdf.xacro`:

```bash
"C:/Users/<tên>/AppData/Local/Programs/FreeCAD 1.1/bin/freecadcmd.exe" CAD/export_meshes.py
```

Từ REPORT.md mục 4.20, URDF dùng số đo và hình dạng của file CAD này (mặt quét lidar 123 mm, khung 300 mm, tốc độ nâng theo vít me). Sửa CAD xong phải chạy lại script rồi chép số vào URDF. Đừng mở file trong hai phiên FreeCAD cùng lúc: phiên lưu sau sẽ ghi đè phiên trước.
