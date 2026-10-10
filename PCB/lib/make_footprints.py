"""Sinh footprint cho cac module cam tren board me (mach 1 lop, tu an mon).

Toa do lay tu file thiet ke goc, khong do bang tay:
  - TB6612FNG : SparkFun_Motor_Driver-TB6612FNG_v11.brd  (github.com/sparkfun/Motor_Driver-Dual_TB6612FNG)
  - INA169    : Adafruit INA169 CurPowerMonitor.brd       (github.com/adafruit/Adafruit-INA169-Breakout-PCB)
  - XL4015    : XL4015_Converter.kicad_mod                (github.com/rayvburn/KiCad)
So thu tu pad trung voi so chan cua ky hieu trong agv_modules.kicad_sym.

Chay:  python make_footprints.py   (ghi vao agv_modules.pretty/ canh file nay)
"""
from pathlib import Path

OUT = Path(__file__).parent / "agv_modules.pretty"

# Pad cho mach tu an mon: ban rong, lo du lon cho chan hang rao 0.64 mm
HDR_DRILL = 1.0
PWR_DRILL = 1.3


def header(name, descr):
    return (
        f'(footprint "{name}"\n'
        '  (version 20240108)\n'
        '  (generator "make_footprints")\n'
        '  (layer "F.Cu")\n'
        f'  (descr "{descr}")\n'
        '  (attr through_hole)\n'
    )


def text(kind, value, x, y, layer):
    return (
        f'  (property "{kind}" "{value}" (at {x:.3f} {y:.3f} 0) (layer "{layer}")\n'
        '    (effects (font (size 1 1) (thickness 0.15))))\n'
    )


def rect(x1, y1, x2, y2, layer, width):
    pts = [(x1, y1), (x2, y1), (x2, y2), (x1, y2), (x1, y1)]
    out = ""
    for (ax, ay), (bx, by) in zip(pts, pts[1:]):
        out += (
            f'  (fp_line (start {ax:.3f} {ay:.3f}) (end {bx:.3f} {by:.3f})\n'
            f'    (stroke (width {width}) (type solid)) (layer "{layer}"))\n'
        )
    return out


def label(txt, x, y, size=0.8):
    return (
        f'  (fp_text user "{txt}" (at {x:.3f} {y:.3f} 0) (layer "F.SilkS")\n'
        f'    (effects (font (size {size} {size}) (thickness 0.12))))\n'
    )


def pad(num, x, y, sx, sy, drill, shape="oval"):
    return (
        f'  (pad "{num}" thru_hole {shape} (at {x:.3f} {y:.3f}) (size {sx} {sy})'
        f' (drill {drill}) (layers "*.Cu" "*.Mask"))\n'
    )


def outline(x1, y1, x2, y2):
    m = 0.25
    return (
        rect(x1, y1, x2, y2, "F.Fab", 0.1)
        + rect(x1, y1, x2, y2, "F.SilkS", 0.12)
        + rect(x1 - m, y1 - m, x2 + m, y2 + m, "F.CrtYd", 0.05)
    )


def tb6612():
    """Board 20.32 x 20.32 mm, hai hang 1x8 cach nhau 15.24 mm. Goc = pad 1 (VM)."""
    left = ["VM", "VCC", "GND", "AO1", "AO2", "BO2", "BO1", "GND"]
    right = ["PWMA", "AIN2", "AIN1", "STBY", "BIN1", "BIN2", "PWMB", "GND"]
    s = header("TB6612FNG_Module", "Module TB6612FNG kieu SparkFun ROB-14450, 2 hang 1x8 buoc 2.54, cach 15.24 mm")
    s += text("Reference", "REF**", 7.62, -2.4, "F.SilkS")
    s += text("Value", "TB6612FNG_Module", 7.62, 20.3, "F.Fab")
    s += outline(-2.54, -1.27, 17.78, 19.05)
    for i, n in enumerate(left):
        y = i * 2.54
        s += pad(i + 1, 0, y, 2.2, 1.7, HDR_DRILL, "rect" if i == 0 else "oval")
        s += label(n, 3.3, y, 0.7)
    for i, n in enumerate(right):
        y = i * 2.54
        s += pad(i + 9, 15.24, y, 2.2, 1.7, HDR_DRILL)
        s += label(n, 11.7, y, 0.7)
    return s + ")\n"


def ina169():
    """Board 22.86 x 20.97 mm. Goc = chan VCC cua hang rao 1x5.

    Hang rao: VCC, GND, VIN-, VIN+, OUT. Hai lo domino 3.5 mm (VIN-, VIN+) nam cach
    hang rao 14.836 mm; pad trung so voi chan hang rao tuong ung de chia dong.
    So pad theo ky hieu: 1 VIN+, 2 VIN-, 3 VCC, 4 GND, 5 VOUT.
    """
    s = header("INA169_Module", "Module INA169 kieu Adafruit 1164: hang rao 1x5 + 2 lo domino 3.5 mm")
    s += text("Reference", "REF**", 5.08, 3.6, "F.SilkS")
    s += text("Value", "INA169_Module", 5.08, -19.5, "F.Fab")
    s += outline(-6.35, -18.43, 16.51, 2.54)
    hdr = [("3", "VCC"), ("4", "GND"), ("2", "VIN-"), ("1", "VIN+"), ("5", "OUT")]
    for i, (num, n) in enumerate(hdr):
        x = i * 2.54
        s += pad(num, x, 0, 1.7, 2.2, HDR_DRILL, "rect" if i == 0 else "oval")
        s += label(n, x, -2.2, 0.6)
    s += pad("2", 3.253, -14.836, 2.4, 3.0, PWR_DRILL)
    s += pad("1", 6.753, -14.836, 2.4, 3.0, PWR_DRILL)
    s += label("VIN-", 1.0, -12.2, 0.6)
    s += label("VIN+", 9.0, -12.2, 0.6)
    return s + ")\n"


def xl4015():
    """Board 54 x 24 mm, 4 pad o 4 goc: cach 50 mm theo chieu dai, 20 mm theo chieu ngan."""
    s = header("XL4015_Module", "Module buck XL4015 5A, 4 pad goc 50 x 20 mm")
    s += text("Reference", "REF**", 0, -13.2, "F.SilkS")
    s += text("Value", "XL4015_Module", 0, 13.2, "F.Fab")
    s += outline(-27, -12, 27, 12)
    for num, n, x, y in [("1", "IN+", -25, -10), ("2", "IN-", -25, 10),
                         ("3", "OUT+", 25, -10), ("4", "OUT-", 25, 10)]:
        s += pad(num, x, y, 4.0, 3.2, PWR_DRILL, "rect" if num == "1" else "oval")
        s += label(n, x + (5.5 if x < 0 else -5.5), y, 1.0)
    return s + ")\n"


if __name__ == "__main__":
    OUT.mkdir(exist_ok=True)
    for name, fn in [("TB6612FNG_Module", tb6612), ("INA169_Module", ina169), ("XL4015_Module", xl4015)]:
        (OUT / f"{name}.kicad_mod").write_text(fn(), encoding="utf-8", newline="\n")
        print("wrote", name)
