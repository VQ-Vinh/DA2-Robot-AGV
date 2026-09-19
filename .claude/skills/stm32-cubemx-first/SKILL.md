---
name: stm32-cubemx-first
description: |
  Quy tắc bắt buộc khi viết hoặc sửa code firmware STM32 trong đồ án
  DA2-Robot-AGV: nếu việc đó cần một ngoại vi, một chân, một ngắt hoặc một cấu
  hình clock chưa có sẵn, phải dừng lại yêu cầu người dùng cấu hình trong
  STM32CubeMX (file .ioc) trước, đợi họ xác nhận, kiểm tra lại file .ioc và code
  đã sinh, rồi mới viết code. Dùng skill này bất cứ khi nào người dùng yêu cầu
  thêm hoặc sửa chức năng trên STM32 — điều khiển động cơ, PWM, đọc encoder,
  UART, I2C, SPI, ADC, ngắt, timer, DMA, cảm biến, LED, nút nhấn — kể cả khi họ
  chỉ nói ngắn gọn kiểu "viết hàm điều khiển động cơ" hay "cho tôi đọc cảm biến"
  mà không nhắc gì tới CubeMX.
---

# Viết code STM32: cấu hình CubeMX trước, code sau

Project này sinh code bằng STM32CubeMX bản độc lập (`STM32.ioc`, CubeMX 6.16.1,
target toolchain CMake). Điều đó đặt ra một ràng buộc mà không được phép quên:
**CubeMX là nguồn sự thật cho phần khởi tạo phần cứng.** Các file `gpio.c`,
`tim.c`, `usart.c`, hàm `SystemClock_Config()`, `HAL_MspInit()` đều do CubeMX
sinh ra và sẽ bị ghi đè ở lần generate sau.

Vì vậy, viết tay phần khởi tạo ngoại vi là công toi: nó chạy được hôm nay, rồi
biến mất vào lần người dùng mở CubeMX chỉnh một thứ khác. Tệ hơn, nó biến mất
lặng lẽ — code vẫn build, chỉ là phần cứng không còn được cấu hình, và người
dùng sẽ đi soi logic trong khi lỗi nằm ở chỗ hoàn toàn khác.

Nên thứ tự đúng là: người dùng cấu hình trong CubeMX → generate code → ta viết
phần logic trong các khối `USER CODE`.

## Khi nào phải dừng lại, khi nào làm luôn

Đây là ranh giới quan trọng nhất của skill này. Chặn nhầm thì phiền người dùng
vì những việc lẽ ra chỉ mất một phút.

**Dừng lại yêu cầu cấu hình CubeMX** khi việc cần làm đòi hỏi thứ chưa tồn tại
trong `.ioc`:

- Bật một ngoại vi mới: TIM, UART/USART, I2C, SPI, ADC, DAC, CAN, DMA, RTC
- Dùng một chân chưa được gán tín hiệu, hoặc đổi chức năng chân đang có
- Bật một ngắt mới trong NVIC, hoặc đổi mức ưu tiên ngắt
- Đổi cấu hình clock: tần số HSE, PLL, prescaler
- Đổi tham số của ngoại vi đã có nhưng do CubeMX quản lý: prescaler và period
  của timer, baudrate UART, độ phân giải ADC, chế độ chân GPIO

**Làm luôn, không cần hỏi**, khi chỉ đụng tới logic chạy trên phần cứng đã cấu
hình sẵn:

- Sửa lỗi cú pháp, lỗi biên dịch
- Viết hoặc sửa thuật toán: PID, lọc nhiễu, máy trạng thái, xử lý gói tin
- Đổi hằng số trong code của mình: ngưỡng, hệ số, duty cycle đang tính bằng
  biến, chu kỳ `HAL_Delay`
- Tái cấu trúc, đổi tên, tách hàm, viết comment
- Đọc/giải thích code, tra cứu

Khi lưỡng lự, cứ chạy script kiểm tra ở dưới trước. Nó cho biết ngay ngoại vi
cần dùng đã có hay chưa, và câu trả lời đó quyết định luôn việc phải chặn hay
không — nhanh hơn là ngồi đoán.

## Quy trình ba bước

### Bước 1 — Nói chính xác cần cấu hình gì

Đừng nói chung chung kiểu "bạn cấu hình TIM3 trong CubeMX nhé". Người dùng phải
mở CubeMX, tìm đúng mục, điền đúng số — nên đưa họ đúng những con số đó, kèm lý
do để họ tự điều chỉnh nếu phần cứng thực tế khác.

Trước khi viết yêu cầu, chạy script kiểm tra để biết hiện trạng, rồi tính toán
tham số dựa trên clock thật của project (đọc `SystemClock_Config()` trong
`Core/Src/main.c` để biết tần số APB1/APB2, đừng giả định 168 MHz).

Trình bày theo dạng này:

> Để làm được việc này cần cấu hình trong CubeMX trước. Bạn mở `STM32/STM32.ioc`
> bằng STM32CubeMX rồi làm giúp tôi:
>
> **1. Bật TIM3 phát PWM**
> - Pinout → Timers → TIM3 → Channel1 = `PWM Generation CH1`
> - Chân PC6 sẽ tự sáng lên, đó là TIM3_CH1
>
> **2. Đặt tham số TIM3** (tab Parameter Settings)
> - Prescaler = `83` — APB1 timer clock 84 MHz, chia 84 còn 1 MHz
> - Counter Period = `999` — 1 MHz / 1000 = 1 kHz, chu kỳ 1 ms
> - Các mục khác giữ mặc định
>
> **3. Sinh code**
> - `Project Manager` → kiểm tra Toolchain vẫn là `CMake`
> - Bấm `GENERATE CODE`
>
> Xong rồi báo tôi, tôi kiểm tra lại rồi viết code điều khiển.

Nếu có nhiều lựa chọn hợp lý (chọn timer nào, chân nào), nêu lựa chọn mình đề
xuất kèm một câu lý do, đừng bắt người dùng tự quyết mà không có thông tin.

### Bước 2 — Kiểm tra lại sau khi người dùng xác nhận

Người dùng báo "xong rồi" **không có nghĩa là xong**. Lỗi hay gặp nhất là cấu
hình đúng nhưng quên bấm `GENERATE CODE`, hoặc generate vào nhầm thư mục. Lúc đó
`.ioc` đã đổi mà code thì chưa. Nếu ta tin lời và viết code luôn, nó sẽ không
build được, hoặc tệ hơn là build được mà chạy sai.

Kiểm tra bằng script đi kèm skill này:

```bash
python .claude/skills/stm32-cubemx-first/scripts/check_ioc.py STM32 --expect TIM3
```

Script in ra: ngoại vi đang bật trong `.ioc`, bảng chân đã gán, các file
`Core/Src/*.c` CubeMX đã sinh, HAL module đã bật trong `stm32f4xx_hal_conf.h`,
và cảnh báo nếu có file chưa được thêm vào `CMakeLists.txt`. Với `--expect`, nó
thoát với mã lỗi nếu ngoại vi cần dùng chưa sẵn sàng.

Cần xem chi tiết hơn (ví dụ prescaler thực tế là bao nhiêu) thì đọc thẳng
`.ioc` — file dạng `khoa=giá trị`, grep được:

```bash
grep -E "^TIM3\.|^PC6\." STM32/STM32.ioc
```

Kết quả kiểm tra quyết định bước tiếp theo:

- **Đủ** → báo ngắn gọn đã xác nhận thấy gì, rồi viết code
- **Thiếu / sai số** → nói rõ thiếu đúng cái gì và nhiều khả năng do đâu (thường
  là chưa generate), hướng dẫn bổ sung, đợi lượt xác nhận tiếp. Đừng tự sửa
  `.ioc` bằng tay để đi tiếp cho nhanh — xem mục cuối.

### Bước 3 — Viết code, chỉ trong khối USER CODE

Project đang bật `KeepUserCode=true`, nghĩa là CubeMX giữ lại nội dung nằm giữa
các cặp `/* USER CODE BEGIN X */` và `/* USER CODE END X */`, và xóa sạch mọi
thứ nằm ngoài. Đây là ràng buộc cứng, không phải khuyến nghị.

- Code logic đặt trong `USER CODE` của `main.c`, hoặc tốt hơn là tách thành
  module riêng trong `Core/Src/` + `Core/Inc/` do mình quản lý hoàn toàn
- File tự viết phải thêm vào `target_sources()` trong `STM32/CMakeLists.txt`
  (phần `# Add user sources here`), không thêm vào
  `cmake/stm32cubemx/CMakeLists.txt` vì file đó CubeMX sẽ ghi đè
- Kiểm tra cặp BEGIN/END bao đúng chỗ. Code lọt ra ngoài cặp đó trông vẫn chạy
  bình thường cho tới lần generate kế tiếp, lúc đó mới mất — rất khó truy
- Dùng handle CubeMX đã tạo (`htim3`, `huart2`), khai báo `extern` qua header
  tương ứng (`tim.h`, `usart.h`), đừng tự định nghĩa lại

Viết xong thì build để xác nhận:

```bash
cd STM32 && build.bat Debug
```

Báo lại kết quả build thật — dung lượng FLASH/RAM, hoặc lỗi nếu có. Đừng nói
"đã xong" khi chưa build.

## Không tự sửa file .ioc

Có thể sẽ nảy ra ý nghĩ tự thêm dòng vào `.ioc` cho nhanh thay vì đợi người dùng.
Đừng làm, vì hai lý do:

`.ioc` chỉ là phần nổi. CubeMX còn kiểm tra xung đột chân, tính lại cây clock,
sinh code khởi tạo và cập nhật `stm32f4xx_hal_conf.h`, `CMakeLists.txt`,
`stm32f4xx_it.c`. Sửa tay `.ioc` cho ra một file mô tả một cấu hình không tồn
tại trong code — trạng thái tệ hơn hẳn so với việc thiếu cấu hình, vì giờ script
kiểm tra cũng báo OK trong khi thực tế thì không.

Lý do thứ hai quan trọng hơn: người dùng cần biết phần cứng của họ đang được cấu
hình thế nào. Đây là đồ án họ phải bảo vệ. Cấu hình xuất hiện sau lưng họ là thứ
họ sẽ không giải thích được khi bị hỏi.

Ngoại lệ duy nhất: người dùng đọc lý do trên và vẫn yêu cầu ta sửa tay. Khi đó
cứ làm, nhưng nói rõ cần mở CubeMX generate lại để code khớp với `.ioc`.
