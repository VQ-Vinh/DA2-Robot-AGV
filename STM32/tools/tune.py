"""Bảng điều khiển dòng lệnh cho DA2-Robot-AGV qua UART.

Hiện đồ thị tốc độ theo thời gian thực, nhận lệnh gõ tay để đặt duty, đặt tốc độ
và chỉnh ba hệ số PID mà không phải biên dịch lại firmware. Toàn bộ log được ghi
ra file, vẽ lại được bằng plot_speed.py.

Cách dùng:
    python tools/tune.py                    # tự tìm cổng COM, có đồ thị trực tiếp
    python tools/tune.py -p COM13
    python tools/tune.py --no-live          # chỉ chạy trong terminal
    python tools/tune.py --replay log.txt   # phát lại log cũ, không cần board

Lệnh gửi xuống board:
    s 150       chạy vòng kín ở 150 RPM        (s -120 để chạy lùi)
    d 40        đặt duty trực tiếp, vòng hở     (d 0 để thả trôi)
    kp 0.12     đổi hệ số tỉ lệ
    ki 0.5      đổi hệ số tích phân
    kd 0.01     đổi hệ số đạo hàm
    demo        quay lại kịch bản tự động
    stop        dừng động cơ
    log 0 / 1   tắt / bật dòng log định kỳ
    ?           xem trạng thái hiện tại

Lệnh chạy tại chỗ trên PC (bắt đầu bằng dấu hai chấm):
    :plot       vẽ đồ thị tĩnh từ toàn bộ log đã ghi
    :mark ghi   chèn một dòng ghi chú vào log
    :q          thoát
"""

import argparse
import queue
import re
import subprocess
import sys
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from plot_speed import FIELD  # dùng chung biểu thức bóc tách với script vẽ tĩnh


def parse_sample(line):
    """Trả về dict các trường đọc được, hoặc None nếu dòng không phải mẫu đo."""
    if "rpm" not in line or line.lstrip().startswith("#"):
        return None
    rec = {}
    for name, pat in FIELD.items():
        m = pat.search(line)
        if m:
            rec[name] = float(m.group(1))
    return rec if "rpm" in rec else None


STATE_FIELD = {
    "kp": re.compile(r"\bkp\s*=\s*(-?\d+(?:\.\d+)?)"),
    "ki": re.compile(r"\bki\s*=\s*(-?\d+(?:\.\d+)?)"),
    "kd": re.compile(r"\bkd\s*=\s*(-?\d+(?:\.\d+)?)"),
    "s":  re.compile(r"\bsp\s*=\s*(-?\d+)"),
}


def parse_state(line):
    """Boc kp/ki/kd/sp tu dong phan hoi '# mode=... kp=0.120 ...'."""
    if not line.lstrip().startswith("#"):
        return {}
    out = {}
    for key, pat in STATE_FIELD.items():
        m = pat.search(line)
        if m:
            out[key] = m.group(1)
    return out


def pick_port(preferred=None):
    from serial.tools import list_ports

    ports = list(list_ports.comports())
    if preferred:
        return preferred
    if not ports:
        sys.exit("Không thấy cổng COM nào. Cắm USB-TTL rồi thử lại.")

    # Loai cac cong chac chan khong phai USB-TTL noi toi PA2/PA3:
    #   - ST-Link VCP: khong noi toi PA2/PA3 tren board Discovery
    #   - Cong Bluetooth ao: Windows tao san hang loat
    def usable_port(p):
        desc = (p.description or "").lower()
        if "bluetooth" in desc:
            return False
        if "stlink" in desc.replace("-", "") or "st-link" in desc:
            return False
        return True

    usable = [p for p in ports if usable_port(p)]
    if len(usable) == 1:
        print(f"Dùng cổng {usable[0].device} ({usable[0].description})")
        skipped = len(ports) - 1
        if skipped:
            print(f"(bỏ qua {skipped} cổng khác: ST-Link VCP và cổng Bluetooth ảo)")
        return usable[0].device

    print("Có nhiều cổng COM, chọn bằng -p:")
    for p in ports:
        print(f"  {p.device:8} {p.description}")
    sys.exit(1)


class SerialLink:
    """Nguồn dữ liệu thật: đọc UART trong luồng riêng."""

    def __init__(self, port, baud):
        import serial

        self.serial_mod = serial
        try:
            self.ser = serial.Serial(port, baud, timeout=0.2)
        except serial.SerialException as e:
            sys.exit(f"Không mở được {port}: {e}\n"
                     "Kiểm tra xem PuTTY hay chương trình khác có đang giữ cổng không.")
        self.lines = queue.Queue()
        self.running = True
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        while self.running:
            try:
                raw = self.ser.readline()
            except self.serial_mod.SerialException:
                self.lines.put(None)
                return
            if raw:
                self.lines.put(raw.decode("utf-8", errors="replace").rstrip("\r\n"))

    def send(self, text):
        try:
            self.ser.write((text + "\n").encode("ascii", errors="ignore"))
        except self.serial_mod.SerialException:
            pass

    def close(self):
        self.running = False
        try:
            self.ser.write(b"stop\n")   # không để động cơ chạy tiếp khi mất điều khiển
            time.sleep(0.2)
            self.ser.close()
        except Exception:
            pass


class ReplayLink:
    """Nguồn dữ liệu giả: phát lại một file log, dùng để thử không cần board."""

    def __init__(self, path, dt_ms=50.0, speed=1.0):
        self.lines = queue.Queue()
        self.running = True
        self._src = Path(path).read_text(encoding="utf-8", errors="replace").splitlines()
        self._delay = (dt_ms / 1000.0) / max(speed, 0.01)
        threading.Thread(target=self._feed, daemon=True).start()

    def _feed(self):
        for line in self._src:
            if not self.running:
                return
            self.lines.put(line)
            time.sleep(self._delay)
        self.lines.put(None)

    def send(self, text):
        self.lines.put(f"# (phát lại, không gửi được: {text})")

    def close(self):
        self.running = False


class LivePlot:
    """Cửa sổ đồ thị cuộn theo thời gian thực."""

    def __init__(self, window_s, title, on_command=None):
        import matplotlib.pyplot as plt
        from matplotlib.widgets import Button, TextBox

        self.plt = plt
        self.window_s = window_s
        self.on_command = on_command or (lambda cmd: None)
        self._updating = False   # chan vong lap khi set_val goi lai on_submit
        self._last_draw = 0.0
        self._draw_interval = 0.1   # ve lai 10 lan/giay la du muot cho mat nguoi
        self.t = deque()
        self.sp = deque()
        self.rpm = deque()
        self.duty = deque()

        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(11, 5.5))
        self.ax2 = self.ax.twinx()

        (self.l_sp,) = self.ax.step([], [], where="post", color="#888888",
                                    linestyle="--", linewidth=1.5,
                                    label="Tốc độ đặt (RPM)")
        (self.l_rpm,) = self.ax.plot([], [], color="#1f77b4", linewidth=1.8,
                                     label="Tốc độ thực tế (RPM)")
        (self.l_duty,) = self.ax2.plot([], [], color="#ff7f0e", linewidth=1.0,
                                       alpha=0.65, label="Duty (%)")

        self.ax.set_xlabel("Thời gian (s)")
        self.ax.set_ylabel("Tốc độ trục ra (RPM)")
        self.ax2.set_ylabel("Duty (%)", color="#ff7f0e")
        self.ax2.tick_params(axis="y", labelcolor="#ff7f0e")
        self.ax2.set_ylim(-110, 110)
        self.ax.grid(True, alpha=0.3)
        self.ax.axhline(0, color="#000000", linewidth=0.8, alpha=0.4)
        self.ax.set_title(title)
        self.ax.legend(
            [self.l_sp, self.l_rpm, self.l_duty],
            [l.get_label() for l in (self.l_sp, self.l_rpm, self.l_duty)],
            loc="upper left", framealpha=0.9)
        self.fig.tight_layout()
        self.fig.subplots_adjust(bottom=0.26)

        # Hang o nhap: [nhan, lenh gui xuong board, gia tri ban dau, vi tri x]
        fields = [
            ("Kp",  "kp", "0.12", 0.11),
            ("Ki",  "ki", "0.50", 0.28),
            ("Kd",  "kd", "0.01", 0.45),
            ("RPM", "s",  "150",  0.63),
        ]
        self.boxes = {}
        for label, cmd, init, x in fields:
            ax_box = self.fig.add_axes([x, 0.06, 0.08, 0.055])
            box = TextBox(ax_box, label + "  ", initial=init)
            box.on_submit(lambda text, c=cmd: self._submit(c, text))
            self.boxes[cmd] = box

        ax_stop = self.fig.add_axes([0.76, 0.06, 0.08, 0.055])
        self.btn_stop = Button(ax_stop, "Dừng")
        self.btn_stop.on_clicked(lambda _e: self.on_command("stop"))

        ax_demo = self.fig.add_axes([0.86, 0.06, 0.08, 0.055])
        self.btn_demo = Button(ax_demo, "Demo")
        self.btn_demo.on_clicked(lambda _e: self.on_command("demo"))

        self.fig.text(0.11, 0.135,
                      "Gõ số rồi Enter để gửi xuống board. Giá trị sẽ tự cập nhật "
                      "theo phản hồi của board.",
                      fontsize=8, color="#666666")
        self.fig.show()

    def _submit(self, cmd, text):
        """Enter trong o nhap: gui lenh xuong board."""
        if self._updating:
            return
        text = text.strip()
        if not text:
            return
        try:
            float(text)
        except ValueError:
            print(f"[{text!r} khong phai so, bo qua]")
            return
        self.on_command(f"{cmd} {text}")

    def set_fields(self, values):
        """Cap nhat o nhap theo trang thai board tra ve, khong kich hoat on_submit."""
        self._updating = True
        try:
            for cmd, val in values.items():
                box = self.boxes.get(cmd)
                if box is not None and box.text != val:
                    box.set_val(val)
        finally:
            self._updating = False

    def add(self, t, rec):
        self.t.append(t)
        self.rpm.append(rec.get("rpm", 0.0))
        self.sp.append(rec.get("sp", 0.0))
        self.duty.append(rec.get("duty", 0.0))

        cutoff = t - self.window_s
        while self.t and self.t[0] < cutoff:
            self.t.popleft()
            self.rpm.popleft()
            self.sp.popleft()
            self.duty.popleft()

    def refresh(self):
        """Xu ly su kien chuot/ban phim moi vong, nhung chi ve lai 10 lan/giay.

        Ve lai toan bo hinh rat nang: neu goi theo nhip vong lap (100 lan/giay)
        thi Tk khong theo kip va Windows bao cua so 'Not responding'.
        """
        self.fig.canvas.flush_events()

        now = time.monotonic()
        if now - self._last_draw < self._draw_interval or not self.t:
            return
        self._last_draw = now

        self.l_sp.set_data(self.t, self.sp)
        self.l_rpm.set_data(self.t, self.rpm)
        self.l_duty.set_data(self.t, self.duty)

        t_end = self.t[-1]
        self.ax.set_xlim(max(0.0, t_end - self.window_s), max(self.window_s, t_end))

        lo = min(min(self.rpm), min(self.sp), 0.0)
        hi = max(max(self.rpm), max(self.sp), 0.0)
        pad = max(20.0, (hi - lo) * 0.12)
        self.ax.set_ylim(lo - pad, hi + pad)

        self.fig.canvas.draw_idle()

    def alive(self):
        return bool(self.plt.get_fignums())

    def close(self):
        self.plt.close(self.fig)


def run_plot(log_path):
    script = HERE / "plot_speed.py"
    print(f"[đang vẽ {log_path} ...]")
    subprocess.run([sys.executable, str(script), str(log_path)])


def stdin_thread(cmd_queue):
    while True:
        try:
            line = input()
        except (EOFError, KeyboardInterrupt):
            # stdin dong (chay khong co terminal): con cua so do thi thi van chay tiep
            cmd_queue.put(":eof")
            return
        cmd_queue.put(line.strip())


def main():
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8", errors="replace")
        except (AttributeError, ValueError):
            pass

    ap = argparse.ArgumentParser(
        description="Điều khiển và ghi log DA2-Robot-AGV qua UART",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__.split("Lệnh gửi xuống board:")[1])
    ap.add_argument("-p", "--port", help="cổng COM, ví dụ COM13 (bỏ trống thì tự tìm)")
    ap.add_argument("-b", "--baud", type=int, default=115200)
    ap.add_argument("-l", "--log", help="file log (mặc định: logs/<thời gian>.txt)")
    ap.add_argument("--no-live", action="store_true", help="tắt đồ thị trực tiếp")
    ap.add_argument("-w", "--window", type=float, default=20.0,
                    help="bề rộng cửa sổ đồ thị tính bằng giây (mặc định 20)")
    ap.add_argument("--replay", help="phát lại một file log thay vì đọc cổng COM")
    ap.add_argument("--replay-speed", type=float, default=1.0,
                    help="tốc độ phát lại, 2 là nhanh gấp đôi")
    ap.add_argument("--list", action="store_true", help="chỉ liệt kê cổng COM rồi thoát")
    args = ap.parse_args()

    if args.list:
        from serial.tools import list_ports
        for p in list_ports.comports():
            print(f"{p.device:8} {p.description}")
        return

    # Nguồn dữ liệu
    if args.replay:
        link = ReplayLink(args.replay, speed=args.replay_speed)
        print(f"Phát lại {args.replay} (tốc độ x{args.replay_speed})")
    else:
        try:
            import serial  # noqa: F401
        except ImportError:
            sys.exit("Thiếu pyserial. Cài bằng:  python -m pip install pyserial")
        link = SerialLink(pick_port(args.port), args.baud)

    # File log
    if args.log:
        log_path = Path(args.log)
    else:
        log_dir = HERE / "logs"
        log_dir.mkdir(exist_ok=True)
        log_path = log_dir / (datetime.now().strftime("%Y%m%d-%H%M%S") + ".txt")

    live = None
    if not args.no_live:
        try:
            live = LivePlot(args.window, "Đáp ứng tốc độ - DA2 Robot AGV",
                            on_command=lambda cmd: link.send(cmd))
        except Exception as e:
            print(f"[không mở được cửa sổ đồ thị: {e}] — chạy tiếp ở chế độ terminal")

    if not args.replay:
        print(f"Log ghi vào {log_path}")
    print("Gõ ? để xem lệnh của board, :q để thoát, :plot để vẽ đồ thị tĩnh.")
    if live:
        print("Đóng cửa sổ đồ thị cũng là thoát.\n")

    cmd_queue = queue.Queue()
    threading.Thread(target=stdin_thread, args=(cmd_queue,), daemon=True).start()

    t0 = time.monotonic()
    stop = False

    with open(log_path, "w", encoding="utf-8") as logfile:
        try:
            while not stop:
                # 1. Dữ liệu mới từ board
                got_any = False
                while True:
                    try:
                        line = link.lines.get_nowait()
                    except queue.Empty:
                        break
                    if line is None:
                        print("\n[hết dữ liệu / mất kết nối]")
                        stop = True
                        break

                    logfile.write(line + "\n")
                    rec = parse_sample(line)
                    if rec is None:
                        print(line)          # phản hồi lệnh, banner: luôn hiện
                        if live is not None:
                            live.set_fields(parse_state(line))
                    elif live is None:
                        print(line)          # không có đồ thị thì in mẫu đo ra
                    if rec is not None and live is not None:
                        live.add(time.monotonic() - t0, rec)
                        got_any = True
                if got_any:
                    logfile.flush()

                # 2. Lệnh người dùng gõ
                while True:
                    try:
                        cmd = cmd_queue.get_nowait()
                    except queue.Empty:
                        break
                    if not cmd:
                        continue
                    if cmd in (":q", ":quit", ":exit"):
                        stop = True
                        break
                    if cmd == ":eof":
                        if live is None:
                            stop = True
                            break
                        print("[stdin da dong - dieu khien bang cac o nhap tren do thi]")
                        continue
                    if cmd == ":plot":
                        logfile.flush()
                        run_plot(log_path)
                    elif cmd.startswith(":mark"):
                        note = cmd[5:].strip() or "mark"
                        logfile.write(f"# {note}\n")
                        logfile.flush()
                        print(f"[đã ghi chú: {note}]")
                    elif cmd.startswith(":"):
                        print("[lệnh PC: :plot | :mark <ghi chú> | :q]")
                    else:
                        link.send(cmd)

                # 3. Vẽ lại
                if live is not None:
                    if not live.alive():
                        stop = True
                    else:
                        live.refresh()
                        time.sleep(0.005)   # nhuong CPU, khong quay vong rong
                else:
                    time.sleep(0.02)
        except KeyboardInterrupt:
            pass
        finally:
            link.close()
            if live is not None and live.alive():
                live.close()

    print(f"\nĐã đóng. Log: {log_path}")
    print(f"Vẽ lại:  python {HERE.name}/plot_speed.py {log_path}")


if __name__ == "__main__":
    main()
