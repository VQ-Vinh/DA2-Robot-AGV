# Sơ đồ thời gian các task FreeRTOS

Tài liệu mô tả các task chạy trên STM32F407 theo thời gian, lấy từ code hiện tại
(`App/Src/app.c`, `Core/Src/freertos.c`, `Core/Inc/FreeRTOSConfig.h`). Cuối file
có sẵn một prompt để đưa cho công cụ AI vẽ sơ đồ.

> Các con số thời gian thực thi của `controlTask` là **ước lượng**. Số liệu thật cần
> đo bằng cách bật/tắt một chân GPIO ở đầu và cuối task rồi xem bằng logic analyzer.

## 1. Thông số hệ thống

| Thông số | Giá trị | Nguồn |
|---|---|---|
| CPU | STM32F407, 168 MHz | `SystemClock_Config()` |
| Tick FreeRTOS | 1 kHz (1 tick = 1 ms), lấy từ SysTick | `configTICK_RATE_HZ` |
| Tick của HAL | 1 kHz, lấy từ TIM7 | `.ioc`: Timebase = TIM7 |
| Scheduler | Preemptive, có ưu tiên | `configUSE_PREEMPTION = 1` |
| UART log | USART2, 115200 8N1, **DMA** (DMA1 Stream 6), ngắt ưu tiên 5 | `usart.c`, `uart_tx.c` |

## 2. Các task

| Task | Ưu tiên (số FreeRTOS) | Stack | Kích hoạt | Việc làm |
|---|---|---|---|---|
| `controlTask` | `osPriorityNormal` (24) | 256 words | Chu kỳ **50 ms** (`osDelayUntil`) | Đọc encoder, đọc lệnh CLI, chạy PID tốc độ, đẩy mẫu vào queue, nháy LED PD13. Chỉ đổi tốc độ đặt mỗi 3 s khi đang ở chế độ demo |
| `logTask` | `osPriorityBelowNormal` (16) | 512 words | Khi queue có mẫu | `snprintf` một dòng, gửi USART2 bằng DMA |
| Timer service | 2 | 256 words | Không dùng timer nào, luôn bị block | — |
| Idle | 0 | 128 words | Khi không còn task nào sẵn sàng | — |

Liên kết giữa hai task: message queue `log_queue`, chứa được 8 phần tử `Sample_t`
(`sp`, `duty`, `rpm_x10`). `controlTask` đẩy vào với timeout 0 (queue đầy thì
bỏ mẫu), còn `logTask` chờ với `osWaitForever`.

## 3. Thời gian thực thi

| Việc | Thời gian | Cách tính |
|---|---|---|
| Thân `controlTask` | ~15–30 µs (ước lượng) | Đọc thanh ghi TIM3, vài phép tính int64, nội suy feedforward + PID bằng số thực (có FPU phần cứng), ghi 1 phần tử queue, toggle GPIO |
| Thân `logTask` (CPU) | ~20–40 µs (ước lượng) | `snprintf` một dòng + khởi động DMA |
| Truyền một dòng trên dây UART | **~2.6–3.0 ms**, do DMA làm, CPU không tham gia | 30–34 ký tự × 10 bit / 115200 bit/s ≈ 86.8 µs/ký tự |
| Ngắt SysTick + TIM7 | vài µs mỗi 1 ms | Tick FreeRTOS và tick HAL |
| Ngắt USART2 + DMA1 Stream 6 | vài µs mỗi dòng log | Báo truyền xong, nhả semaphore |

Tải CPU ước tính: `logTask` khoảng 0.1 %, `controlTask` dưới 0.1 %, còn lại gần như toàn
bộ là task Idle.

Trước đây `logTask` gửi UART kiểu polling (`printf` → `HAL_UART_Transmit`), CPU đứng chờ
từng byte suốt 3.3 ms, tức ~7 % CPU. Nay `logTask` giao buffer cho DMA rồi block trên
semaphore, CPU rảnh trong lúc truyền (xem `Modules/Src/uart_tx.c`).

## 4. Diễn biến theo thời gian

Mốc t = 0 là lúc gọi `osKernelStart()`. Trước đó `App_Init()` đã in banner trong
`main()`, khi scheduler chưa chạy.

1. **t = 0 ms**: cả hai task cùng sẵn sàng. `controlTask` có ưu tiên cao hơn nên chạy
   trước: đặt duty = 0, tính `next = 50`, rồi block trong `osDelayUntil`.
2. `logTask` chạy tiếp, thấy queue rỗng nên block. Từ đây chỉ còn Idle chạy.
3. **t = 50 ms**: `controlTask` thức dậy, đọc encoder, đẩy mẫu vào queue, toggle LED,
   rồi block tới t = 100. Việc đẩy mẫu làm `logTask` sẵn sàng, nhưng nó phải chờ
   `controlTask` block xong mới được chạy (ưu tiên thấp hơn).
4. **t ≈ 50.02 ms**: `logTask` chạy `snprintf` (~30 µs), gọi `HAL_UART_Transmit_DMA`
   rồi block trên semaphore.
5. **t ≈ 50.05 → 53.4 ms**: DMA đẩy từng byte ra chân TX, CPU chạy Idle. Byte cuối ra
   khỏi chân TX thì ngắt USART2 gọi `HAL_UART_TxCpltCallback`, nhả semaphore. `logTask`
   thức dậy một chút rồi quay lại chờ queue.
   Sau đó Idle chạy tới t = 100.
6. Chu kỳ lặp lại mỗi 50 ms. LED PD13 đảo trạng thái mỗi 50 ms, tức nháy 10 Hz.
7. **t = 3000, 6000, 9000 … ms** (mỗi 60 mẫu): `controlTask` đổi sang tốc độ đặt kế tiếp
   theo kịch bản `0 → 120 → 180 → 240 → 180 → 120 → 0 → -150 → -220 → 0` (RPM), xoay vòng.
   Việc đổi lệnh nằm ở **cuối** chu kỳ, sau khi đã đẩy mẫu, nên `sp`, `rpm` và `duty`
   trong cùng một dòng log luôn thuộc về cùng một chu kỳ điều khiển.

Chỗ đáng chú ý: nếu sau này `logTask` in lâu hơn 50 ms, thì lúc đang in nó vẫn bị
`controlTask` chen ngang (preempt) đúng mốc 50 ms. Nhịp điều khiển không bị lệch,
chỉ có log bị chậm và queue đầy dần.

```
Tỉ lệ: 1 ký tự = 2 ms. █ = CPU đang chạy task (khối µs được phóng to).
▒ = DMA đang truyền UART, CPU không tham gia.

t (ms)       0                        50                       100                      150
             |                        |                        |                        |
controlTask  █························█························█························█
logTask      █························█························█························█
Idle         ·████████████████████████·████████████████████████·████████████████████████·
UART TX DMA  ·························▒▒·······················▒▒·······················▒
LED PD13     ▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▔
```

## 5. Prompt vẽ sơ đồ

Copy nguyên khối dưới đây đưa cho Claude hoặc công cụ AI khác.

````text
Vẽ sơ đồ thời gian (timing diagram / Gantt theo trục thời gian) cho hệ thống
FreeRTOS chạy trên STM32F407 168 MHz, tick 1 ms, scheduler preemptive.
Xuất ra một file SVG duy nhất (hoặc code Python matplotlib nếu không vẽ SVG được),
nền trắng, font sans-serif, nhãn tiếng Việt.

CÁC TASK (mỗi task là một làn ngang, xếp từ ưu tiên cao xuống thấp):
1. controlTask - ưu tiên 24 (osPriorityNormal) - màu xanh dương
   Chạy định kỳ mỗi 50 ms bằng osDelayUntil, mỗi lần chạy khoảng 15 µs:
   đọc encoder TIM3, đẩy 1 mẫu vào queue, toggle LED PD13.
   Cứ 60 lần (tức mỗi 3000 ms) thì đổi tốc độ đặt theo chuỗi
   0 → 120 → 180 → 240 → 180 → 120 → 0 → -150 → -220 → 0 (RPM) → lặp lại.
2. logTask - ưu tiên 16 (osPriorityBelowNormal) - màu cam
   Block chờ queue. Mỗi khi controlTask đẩy mẫu thì logTask chạy ngay sau khi
   controlTask block, snprintf một dòng (~30 µs CPU), giao cho DMA gửi qua UART
   115200 (mất ~3.3 ms trên dây nhưng CPU rảnh). logTask block chờ semaphore
   báo truyền xong, rồi quay lại chờ queue.
3. Idle - ưu tiên 0 - màu xám nhạt
   Chạy trong mọi khoảng trống còn lại.

LÀN PHỤ (vẽ dưới các task):
- LED PD13: sóng vuông, đảo mức mỗi 50 ms (ban đầu mức thấp, lên cao tại t = 50).
- Số phần tử trong queue log_queue: tăng lên 1 tại mỗi mốc 50 ms,
  giảm về 0 khi logTask lấy ra (ngay sau đó).
- Tick 1 ms: vạch nhỏ trên trục thời gian.

VẼ HAI HÌNH:
Hình A - phóng to 0 → 110 ms:
  - t = 0: controlTask chạy trước (khởi tạo, duty = 0) rồi block;
    logTask chạy rất ngắn, thấy queue rỗng rồi block; sau đó là Idle.
  - t = 50 và t = 100: controlTask chạy ~15 µs, ngay sau đó logTask chạy ~30 µs.
  - Thêm làn "UART TX (DMA)" màu tím nhạt, hình gạch chéo: bận từ ngay sau logTask,
    kéo dài 3.3 ms. Trong khoảng đó làn Idle vẫn tô (CPU rảnh), đây là ý chính của hình.
  - Vì 15 µs quá hẹp so với trục 110 ms, hãy vẽ khối controlTask với độ rộng tối
    thiểu dễ nhìn và ghi chú "~15 µs (không theo tỉ lệ)".
  - Vẽ mũi tên từ controlTask sang logTask ghi "osMessageQueuePut".
  - Ghi chú khoảng thời gian 50 ms giữa hai lần controlTask chạy ("chu kỳ cố định,
    osDelayUntil") và độ rộng 3.3 ms của làn UART ("DMA truyền, CPU không chờ").

Hình B - tổng quan 0 → 6500 ms:
  - Mỗi mốc 50 ms chỉ vẽ một vạch mảnh cho controlTask và logTask (không cần chi tiết).
  - Thêm một làn "Tốc độ đặt (RPM)" dạng bậc thang: 0 (0–3000 ms), 120 (3000–6000),
    180 (6000–9000), 240 (từ 9000), trục tung từ -270 đến 270.
  - Đánh dấu các mốc 3000, 6000, 9000 ms bằng đường đứt nét dọc ghi "đổi tốc độ đặt".

Chú thích (legend) góc trên phải: màu của từng task, ký hiệu "█ = đang chạy".
Tiêu đề: "Sơ đồ thời gian task FreeRTOS - DA2 Robot AGV".
Dưới hình ghi: "Tải CPU ước tính: logTask ≈ 0.1 %, controlTask < 0.1 %, Idle ≈ 99.8 %".
````

## 6. Khi code thay đổi

Cần cập nhật tài liệu này khi thay đổi một trong các thông số sau:
`SAMPLE_PERIOD_MS`, `STEP_PERIOD_MS`, mức ưu tiên task trong `.ioc`, baudrate UART,
hoặc định dạng dòng log (độ dài dòng quyết định thời gian truyền UART).
