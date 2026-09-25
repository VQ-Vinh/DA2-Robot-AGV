---
name: git-push-workflow
description: |
  Quy trình đưa code lên GitHub cho đồ án DA2-Robot-AGV: tách thay đổi thành
  nhiều commit nhỏ, luôn làm trên nhánh riêng (không bao giờ commit thẳng vào
  main), rồi mở pull request với mô tả ngắn bằng tiếng Việt. Dùng skill này bất
  cứ khi nào người dùng nhắc tới push code, đẩy code, commit, tạo nhánh, mở PR,
  "lên git", "đưa lên GitHub", "lưu lại code", hoặc khi vừa hoàn thành một phần
  việc và cần đưa kết quả lên remote — kể cả khi họ chỉ nói ngắn gọn "push lên
  đi" mà không nói gì thêm về nhánh hay commit.
---

# Đưa code lên GitHub

Mục tiêu của quy trình này không phải là "cho code lên remote cho xong", mà là
để lịch sử git đọc được như một câu chuyện: mỗi commit là một ý, mỗi PR là một
phần việc. Đồ án này kéo dài nhiều tháng và sẽ phải nhìn lại — một commit khổng
lồ tên "update" sáu tháng sau là vô dụng, còn chuỗi commit rõ ràng thì tìm lại
được chỗ nào làm hỏng chuyện.

Người dùng đã cho phép tự động chạy hết quy trình: tạo nhánh → commit → push →
mở PR, không cần hỏi lại ở từng bước. Cứ làm thẳng tới lúc có link PR, rồi báo
cáo lại. Ngoại lệ duy nhất là các tình huống ở mục "Khi nào phải dừng lại hỏi".

## Bước 1 — Xem thay đổi có gì trước khi quyết định chia commit

```bash
git status --short && git diff --stat && git diff
```

Đọc hết diff trước khi gõ `git add`. Không chia được commit hợp lý nếu chưa biết
mình đang chia cái gì. Với thay đổi lớn, đọc cả `git diff` đầy đủ chứ không chỉ
`--stat`.

Nếu thấy file lẽ ra không nên lên git (thư mục `build/`, `*.elf`, `*.hex`,
`__pycache__/`, file chứa khoá/mật khẩu/token), đừng commit — bổ sung vào
`.gitignore` ở gốc repo và nói cho người dùng biết.

## Bước 2 — Tạo nhánh, không bao giờ commit thẳng vào main

Kiểm tra nhánh hiện tại. Nếu đang ở `main`, tạo nhánh mới ngay — các thay đổi
chưa commit sẽ đi theo sang nhánh mới, không mất gì:

```bash
git checkout -b feat/ten-viec-ngan-gon
```

Tiền tố theo loại thay đổi, phần sau là kebab-case tiếng Anh ngắn:

| Tiền tố     | Dùng khi                                    | Ví dụ                    |
|-------------|---------------------------------------------|--------------------------|
| `feat/`     | Thêm tính năng mới                          | `feat/pwm-motor-driver`  |
| `fix/`      | Sửa lỗi                                     | `fix/uart-timeout`       |
| `refactor/` | Đổi cấu trúc, không đổi hành vi             | `refactor/hal-wrapper`   |
| `docs/`     | Tài liệu, README, báo cáo                   | `docs/build-guide`       |
| `chore/`    | Script build, cấu hình, dọn dẹp             | `chore/gitignore`        |

Nếu một lần push gồm nhiều loại, chọn tiền tố theo phần việc chính.

## Bước 3 — Chia thành nhiều commit

Đây là phần quan trọng nhất và cũng là phần dễ làm ẩu nhất. Một commit nên là
**một thay đổi có thể mô tả trong một câu mà không cần chữ "và"**. Nếu câu mô tả
bắt buộc phải có "và", gần như chắc chắn đó phải là hai commit.

Các đường cắt thường đúng trong repo này:

- **Theo module**: thay đổi trong `STM32/` và trong `RasPi/` hầu như luôn là hai
  commit riêng — chúng chạy trên hai con chip khác nhau, vòng đời khác nhau.
- **Tách hạ tầng khỏi tính năng**: script build, `.gitignore`, cấu hình CMake đi
  riêng khỏi code firmware. Người review muốn xem logic, không muốn logic bị lẫn
  trong đống cấu hình.
- **Tách sửa lỗi khỏi tính năng mới**: để sau này `git revert` được cái này mà
  không mất cái kia.
- **Tách đổi tên / format hàng loạt**: diff đổi format rất ồn, trộn chung sẽ che
  mất thay đổi logic thật sự.

Commit theo từng nhóm file cụ thể, đừng `git add -A` rồi commit một cục:

```bash
git add STM32/Modules/Src/pwm.c STM32/Modules/Inc/pwm.h
git commit -m "feat(stm32): add PWM driver for motor control"
```

### Định dạng commit message — tiếng Anh, Conventional Commits

```
<loại>(<scope>): <mô tả ở thể mệnh lệnh, chữ thường, không chấm cuối câu>
```

Loại: `feat`, `fix`, `refactor`, `docs`, `chore`, `test`, `build`.
Scope: `stm32`, `raspi`, `build`, hoặc tên module cụ thể. Bỏ scope nếu thay đổi
trải rộng toàn repo.

Dòng đầu giữ dưới 72 ký tự. Nếu cần giải thích *tại sao* (không phải *cái gì* —
cái gì đã nằm trong diff rồi), thêm phần thân sau một dòng trống.

**Ví dụ 1**
Việc đã làm: thêm file pwm.c và pwm.h cấu hình TIM3 phát xung điều khiển động cơ
Commit: `feat(stm32): add PWM driver for motor control`

**Ví dụ 2**
Việc đã làm: sửa dấu chấm phẩy còn thiếu làm build lỗi ở main.c
Commit: `fix(stm32): add missing semicolon in main loop`

**Ví dụ 3**
Việc đã làm: viết build.bat và flash.bat, thêm .gitignore
Chia hai commit:
`chore(build): add build and flash scripts for CMake workflow`
`chore: ignore build artifacts and Python caches`

Kết thúc mỗi commit message bằng dòng attribution mà hệ thống yêu cầu
(`Co-Authored-By: ...`) — dùng heredoc để xuống dòng cho đúng:

```bash
git commit -F - << 'EOF'
feat(stm32): add PWM driver for motor control

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>
EOF
```

Trước khi sang bước sau, chạy `git log --oneline main..HEAD` và đọc lại. Chuỗi
commit đó có kể được câu chuyện không? Nếu không, sửa lại bằng `git reset --soft`
rồi chia lại — lúc này chưa push nên sửa còn rẻ.

## Bước 4 — Push

```bash
git push -u origin <ten-nhanh>
```

## Bước 5 — Mở PR, mô tả ngắn bằng tiếng Việt

Tiêu đề PR viết tiếng Anh giống commit chính. Phần mô tả viết tiếng Việt, ngắn —
mục tiêu là người đọc hiểu trong 15 giây, không phải đọc lại toàn bộ diff:

```bash
gh pr create --base main --title "feat(stm32): add PWM driver for motor control" --body "$(cat << 'EOF'
## Thay đổi
- Thêm driver PWM điều khiển động cơ, dùng TIM3 kênh 1 và 2
- Tần số 20 kHz, duty điều chỉnh 0-100%

## Kiểm thử
- Build Debug thành công, FLASH 6.1 KB
- Đã nạp và đo xung trên PD12 bằng oscilloscope
EOF
)"
```

Phần mô tả PR không kèm dòng attribution "Generated with Claude Code" — đây là
đồ án cá nhân, người dùng không muốn dòng đó trong PR. Đừng tự thêm lại.

Giữ mục "Thay đổi" trong khoảng 2-5 gạch đầu dòng. Mục "Kiểm thử" ghi đúng những
gì thật sự đã chạy — nếu chưa build hay chưa nạp thử thì ghi thẳng là chưa, đừng
ghi cho đẹp. PR mô tả sai còn tệ hơn PR không mô tả.

Báo lại cho người dùng link PR kèm một câu tóm tắt đã chia bao nhiêu commit.

## Khi nào phải dừng lại hỏi

Quy trình chạy tự động, trừ mấy trường hợp này — vì làm sai thì khó gỡ:

- **Diff có vẻ chứa khoá, mật khẩu, token, hoặc thông tin cá nhân.** Dừng, chỉ
  rõ file và dòng.
- **Có thay đổi mà người dùng không hề yêu cầu**, hoặc file lạ xuất hiện. Hỏi
  trước xem có cố ý không.
- **Đang ở giữa một trạng thái git dở dang**: merge conflict, rebase đang chạy,
  detached HEAD. Mô tả tình hình rồi hỏi.
- **Người dùng yêu cầu push thẳng vào `main`.** Nói rõ vì sao nên tránh, nhưng
  nếu họ khẳng định lại thì cứ làm — đó là quyết định của họ.
- **Nhánh đã tồn tại từ trước với nội dung khác.** Hỏi xem nên tiếp tục trên
  nhánh đó hay tạo nhánh mới.

Ngoài ra, đừng bao giờ dùng `push --force` lên nhánh chung, và đừng thêm
`--no-verify` để bỏ qua git hook — hook lỗi thì sửa nguyên nhân.
