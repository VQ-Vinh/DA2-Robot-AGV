---
name: report-log
description: |
  Ghi nhật ký quá trình làm đồ án DA2-Robot-AGV vào REPORT.md ở gốc repo, để
  sau này làm báo cáo cho giảng viên: làm gì, dùng gì, sai ở đâu, vì sao sai,
  sửa thế nào, kết quả và bài học. Dùng skill này mỗi khi xong một phần việc
  (một bước, một PR), khi sửa xong một sự cố hoặc một kết luận sai, khi đổi
  hướng thiết kế, và khi người dùng nhắc tới báo cáo, nhật ký, REPORT.md, "ghi
  lại quá trình", "để làm báo cáo" — kể cả khi họ chỉ nói "push lên đi" sau một
  phần việc có sự cố đáng ghi.
---

# Ghi nhật ký quá trình vào REPORT.md

REPORT.md là nguyên liệu cho báo cáo đồ án, không phải bản demo. Giảng viên
chấm **quá trình**: cách phát hiện vấn đề, cách tìm nguyên nhân, cách sửa và
cải thiện. Một nhật ký chỉ toàn thành công vừa không thật, vừa mất phần có giá
trị nhất. Vì vậy **sai lầm phải được ghi lại trung thực**, kể cả những lần đoán
sai, những kết luận phải rút lại, và những lỗi do chính công cụ test gây ra.

## Khi nào ghi

- Xong một bước / một PR: thêm hoặc cập nhật mục của bước đó.
- Sửa xong một sự cố: thêm một dòng vào bảng sự cố của bước tương ứng.
- Phát hiện một kết luận trước đó là sai: **không xoá** kết luận cũ, ghi thêm
  rằng nó sai, vì sao đã tin nó, và điều gì cho thấy nó sai.
- Đổi hướng thiết kế: ghi lý do đổi và phương án bị bỏ.

Ghi ngay khi việc vừa xong, lúc số liệu và log còn trong tay; để sau sẽ chỉ còn
lại phiên bản "đẹp" và mất chi tiết.

## Mỗi mục cần có

1. **Mục tiêu** – định làm gì.
2. **Cách làm** – công cụ, gói phần mềm, thiết kế; nêu nguồn tham khảo nếu có
   (dự án mã nguồn mở, tài liệu chính thức).
3. **Sai lầm / sự cố** – chuyện gì xảy ra, kèm **số liệu** (tỉ lệ thành công,
   sai số, thời gian, thông báo lỗi nguyên văn ngắn).
4. **Nguyên nhân** – và **tìm ra bằng cách nào** (log, đo đạc, đối chiếu dự án
   tham khảo). Nếu đã đoán sai trước khi tìm ra, ghi cả các lần đoán sai.
5. **Cách sửa** và **kết quả sau khi sửa**, có số liệu so sánh trước/sau.
6. **Bài học** – một câu, áp dụng được cho lần sau.

Sự cố nhỏ trong cùng một bước thì gom thành bảng
`# | Sự cố | Nguyên nhân (tìm bằng cách nào) | Cách sửa`.

## Quy tắc viết

- Tiếng Việt có dấu, giọng trung tính ("đã thử…", "nguyên nhân là…"), không
  dùng "mình/tôi".
- Chỉ ghi điều đã kiểm chứng. Điều chưa chắc thì ghi rõ là giả thuyết hoặc
  "chưa kiểm chứng". Không bịa số liệu, không làm tròn cho đẹp.
- Ghi cả kết quả xấu: lần chạy thất bại, tỉ lệ không ổn định (ví dụ "4 lần
  4/4, 1 lần 1/4" chứ không phải "chạy tốt").
- Cập nhật **bảng tổng hợp sai lầm** (mục 5) và **vấn đề còn mở** (mục 6) mỗi
  khi có thay đổi; vấn đề đã giải quyết thì chuyển thành một mục có cách sửa.
- Cập nhật **bảng mốc thời gian** (ngày, việc, PR) ở mục 1.
- REPORT.md không thay README: hướng dẫn sử dụng để ở README, REPORT.md chỉ
  tóm tắt và trỏ tới.

## Khi đẩy code

Commit thay đổi REPORT.md thành một commit riêng
(`docs: update progress report for <phần việc>`), trong cùng nhánh và PR với
phần việc đó, theo skill `git-push-workflow`.
