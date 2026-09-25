"""Vẽ đồ thị đáp ứng tốc độ từ log UART của DA2-Robot-AGV.

Đọc file log lưu từ PuTTY, vẽ tốc độ đặt và tốc độ thực tế theo thời gian,
kèm duty, rồi tính các chỉ số đáp ứng cho từng bước nhảy: vọt lố, thời gian
ổn định và sai số xác lập.

Nhận cả hai định dạng log:
    sp= 120  rpm=118.9  duty=  32          (vòng kín)
    duty=  60  count=    1525  rpm=49.9    (vòng hở)

Cách dùng:
    python plot_speed.py log.txt
    python plot_speed.py log.txt -o dothi.png --dt 50
    python plot_speed.py log.txt --show
"""

import argparse
import re
import sys
from pathlib import Path

import matplotlib
import matplotlib.pyplot as plt

FIELD = {
    "sp":    re.compile(r"\bsp\s*=\s*(-?\d+)"),
    "rpm":   re.compile(r"\brpm\s*=\s*(-?\d+(?:\.\d+)?)"),
    "duty":  re.compile(r"\bduty\s*=\s*(-?\d+)"),
    "count": re.compile(r"\bcount\s*=\s*(-?\d+)"),
}


def parse_log(path):
    """Trả về danh sách dict, mỗi dict là một mẫu. Bỏ qua dòng không khớp."""
    samples = []
    for line in Path(path).read_text(encoding="utf-8", errors="replace").splitlines():
        if "rpm" not in line:
            continue
        rec = {}
        for name, pat in FIELD.items():
            m = pat.search(line)
            if m:
                rec[name] = float(m.group(1))
        if "rpm" in rec:
            samples.append(rec)
    return samples


def find_steps(sp, dt_ms):
    """Cắt chuỗi mẫu thành các đoạn có cùng lệnh đặt.

    Trả về list (chỉ số bắt đầu, chỉ số kết thúc, giá trị sp).
    """
    steps = []
    start = 0
    for i in range(1, len(sp)):
        if sp[i] != sp[i - 1]:
            steps.append((start, i, sp[start]))
            start = i
    steps.append((start, len(sp), sp[start]))
    return steps


def step_metrics(rpm, seg_start, seg_end, target, dt_ms, band_pct=2.0):
    """Tính vọt lố (%), thời gian ổn định (ms) và sai số xác lập cho một bước."""
    seg = rpm[seg_start:seg_end]
    if not seg or target == 0:
        return None

    band = abs(target) * band_pct / 100.0

    # Vọt lố phải đo theo HƯỚNG của bước nhảy, không phải cứ lấy đỉnh cao nhất:
    # ở bước giảm tốc (240 -> 120) thì đỉnh cao nhất chính là điểm xuất phát,
    # lấy nhầm sẽ ra "vọt lố +98%" trong khi thực tế là hụt vài phần trăm.
    start = seg[0]
    if start <= target:        # bước tăng: vượt qua đích về phía trên
        peak = max(seg)
        overshoot = (peak - target) / abs(target) * 100.0
    else:                      # bước giảm: vượt qua đích về phía dưới
        peak = min(seg)
        overshoot = (target - peak) / abs(target) * 100.0

    # Thời gian ổn định: mẫu cuối cùng còn nằm ngoài dải, cộng thêm một chu kỳ
    settle_idx = None
    for i in range(len(seg) - 1, -1, -1):
        if abs(seg[i] - target) > band:
            settle_idx = i + 1
            break
    settle_ms = None if settle_idx is None else settle_idx * dt_ms
    if settle_idx is not None and settle_idx >= len(seg):
        settle_ms = None  # chưa kịp ổn định trong đoạn này

    tail = seg[-10:] if len(seg) >= 10 else seg
    err = sum(tail) / len(tail) - target

    return {
        "target": target,
        "peak": peak,
        "overshoot": overshoot,
        "settle_ms": settle_ms,
        "err": err,
        "duration_ms": len(seg) * dt_ms,
    }


def main():
    # Console Windows mac dinh la cp1252, khong in duoc tieng Viet
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8", errors="replace")
        except (AttributeError, ValueError):
            pass

    ap = argparse.ArgumentParser(description="Vẽ đáp ứng tốc độ từ log UART")
    ap.add_argument("logfile", help="file log lưu từ PuTTY")
    ap.add_argument("-o", "--out", help="file ảnh xuất ra (mặc định: <log>.png)")
    ap.add_argument("--dt", type=float, default=50.0,
                    help="chu kỳ lấy mẫu tính bằng ms (mặc định 50, bằng SAMPLE_PERIOD_MS)")
    ap.add_argument("--band", type=float, default=2.0,
                    help="dải ổn định tính theo %% lệnh đặt (mặc định 2)")
    ap.add_argument("--show", action="store_true", help="mở cửa sổ đồ thị")
    ap.add_argument("--title", default="Đáp ứng tốc độ - DA2 Robot AGV")
    args = ap.parse_args()

    samples = parse_log(args.logfile)
    if not samples:
        sys.exit(f"Không đọc được mẫu nào từ {args.logfile} — kiểm tra lại file log")

    dt = args.dt
    t = [i * dt / 1000.0 for i in range(len(samples))]
    rpm = [s["rpm"] for s in samples]
    has_sp = all("sp" in s for s in samples)
    has_duty = all("duty" in s for s in samples)
    sp = [s["sp"] for s in samples] if has_sp else None
    duty = [s["duty"] for s in samples] if has_duty else None

    if not args.show:
        matplotlib.use("Agg")

    fig, ax = plt.subplots(figsize=(12, 6))

    if sp is not None:
        ax.step(t, sp, where="post", color="#888888", linestyle="--",
                linewidth=1.5, label="Tốc độ đặt (RPM)")
    ax.plot(t, rpm, color="#1f77b4", linewidth=1.6, label="Tốc độ thực tế (RPM)")
    ax.set_xlabel("Thời gian (s)")
    ax.set_ylabel("Tốc độ trục ra (RPM)")
    ax.grid(True, alpha=0.3)
    ax.axhline(0, color="#000000", linewidth=0.8, alpha=0.4)

    handles, labels = ax.get_legend_handles_labels()
    if duty is not None:
        ax2 = ax.twinx()
        ax2.plot(t, duty, color="#ff7f0e", linewidth=1.0, alpha=0.65,
                 label="Duty (%)")
        ax2.set_ylabel("Duty (%)", color="#ff7f0e")
        ax2.tick_params(axis="y", labelcolor="#ff7f0e")
        ax2.set_ylim(-110, 110)
        h2, l2 = ax2.get_legend_handles_labels()
        handles += h2
        labels += l2

    ax.legend(handles, labels, loc="upper left", framealpha=0.9)

    # Vạch đứt tại mỗi lần đổi lệnh đặt + bảng chỉ số
    rows = []
    if sp is not None:
        for seg_start, seg_end, target in find_steps(sp, dt):
            if seg_start > 0:
                ax.axvline(t[seg_start], color="#cccccc", linewidth=0.8, zorder=0)
            m = step_metrics(rpm, seg_start, seg_end, target, dt, args.band)
            if m:
                rows.append(m)

    ax.set_title(args.title)
    fig.tight_layout()

    out = args.out or (str(Path(args.logfile).with_suffix("")) + ".png")
    fig.savefig(out, dpi=140)
    print(f"Đã lưu đồ thị: {out}")
    print(f"Số mẫu: {len(samples)}   Thời lượng: {t[-1]:.1f} s   Chu kỳ: {dt:.0f} ms")

    if rows:
        print()
        print(f"{'Lệnh đặt':>9} {'Đỉnh':>8} {'Vọt lố':>9} {'Ổn định':>10} {'Sai số XL':>11}")
        print("-" * 52)
        for m in rows:
            settle = "chưa ổn định" if m["settle_ms"] is None else f"{m['settle_ms']/1000:.2f} s"
            print(f"{m['target']:9.0f} {m['peak']:8.1f} {m['overshoot']:8.1f}% "
                  f"{settle:>10} {m['err']:10.1f}")
        print()
        print(f"Dải ổn định: ±{args.band:.0f}% lệnh đặt. "
              "Sai số XL tính trên 10 mẫu cuối mỗi bước.")

    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
