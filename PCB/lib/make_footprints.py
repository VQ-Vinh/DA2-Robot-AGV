"""Sinh footprint cho cac module cam tren board me (mach 1 lop, tu an mon).

Toa do lay tu file thiet ke goc, khong do bang tay:
  - TB6612FNG : SparkFun_Motor_Driver-TB6612FNG_v11.brd  (github.com/sparkfun/Motor_Driver-Dual_TB6612FNG)
  - INA169    : Adafruit INA169 CurPowerMonitor.brd       (github.com/adafruit/Adafruit-INA169-Breakout-PCB)
  - INA219    : Adafruit_INA219_CurPowerMonitor.brd       (github.com/adafruit/Adafruit-INA219-Current-Sensor-PCB)
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
    # 0.8 x 0.15 mm: muc chu nho nhat JLCPCB in ro
    return (
        f'  (fp_text user "{txt}" (at {x:.3f} {y:.3f} 0) (layer "F.SilkS")\n'
        f'    (effects (font (size {size} {size}) (thickness 0.15))))\n'
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

# Mo hinh 3D (tai tu GrabCAD, dat trong lib/3d/models/, khong dua len git):
#   xl4015.step            grabcad.com/library/step-down-xl4015-1
#   ina219.step/.wrl      grabcad.com/library/ina219-dc-current-sensor-1 (CJMCU-219)
#   tb6612fng.step         grabcad.com/library/sparkfun-motor-driver-1 (hang chan cach 15 mm, khop footprint)
#   a4988_socketed.step    grabcad.com/library/a4988-stepper-driver-with-pins-socketed-1
#   gy_bno055.step         grabcad.com/library/gy-bno055-1
#   kf301_2p.step          grabcad.com/library/kf301-5-0-2p-1
#   stm32f4_discovery.step/.wrl grabcad.com/library/stm32f4-1
# Module cam tren hang rao cai: day mach cao 11 mm (hang rao cai 8.5 + nhua hang rao duc 2.5).
M3D = "${KIPRJMOD}/lib/3d/models/"
SOCKET_1X08 = "${KICAD10_3DMODEL_DIR}/Connector_PinSocket_2.54mm.3dshapes/PinSocket_1x08_P2.54mm_Vertical.step"


def model(path, off=(0, 0, 0), rot=(0, 0, 0)):
    return ('  (model "%s"' % path + chr(10) +
            "    (offset (xyz %g %g %g))" % tuple(off) + chr(10) +
            "    (scale (xyz 1 1 1))" + chr(10) +
            "    (rotate (xyz %g %g %g))" % tuple(rot) + chr(10) + "  )" + chr(10))


def tb6612():
    """Board 20.32 x 20.32 mm, hai hang 1x8 cach nhau 15.24 mm. Goc = pad 1 (VM)."""
    left = ["VM", "VCC", "GND", "AO1", "AO2", "BO2", "BO1", "GND"]
    right = ["PWMA", "AIN2", "AIN1", "STBY", "BIN1", "BIN2", "PWMB", "GND"]
    s = header("TB6612FNG_Module", "Module TB6612FNG kieu SparkFun ROB-14450, 2 hang 1x8 buoc 2.54, cach 15.24 mm")
    # 3 chan GND noi chung tren module: khai bao de DRC coi la da noi (khong can noi tren board me)
    s += '  (jumper_pad_groups ("3" "8" "16"))\n'
    s += text("Reference", "REF**", 7.62, -2.4, "F.SilkS")
    s += text("Value", "TB6612FNG_Module", 7.62, 20.3, "F.Fab")
    s += outline(-2.54, -1.27, 17.78, 19.05)
    for i, n in enumerate(left):
        y = i * 2.54
        s += pad(i + 1, 0, y, 2.2, 1.7, HDR_DRILL, "rect" if i == 0 else "oval")
        s += label(n, 3.3, y)
    for i, n in enumerate(right):
        y = i * 2.54
        s += pad(i + 9, 15.24, y, 2.2, 1.7, HDR_DRILL)
        s += label(n, 11.7, y)
    s += model(SOCKET_1X08) + model(SOCKET_1X08, (15.24, 0, 0)) + model(M3D + "tb6612fng.step", (18.1, -8.89, 10.02))
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
        s += label(n, x, -2.2)
    s += pad("2", 3.253, -14.836, 2.4, 3.0, PWR_DRILL)
    s += pad("1", 6.753, -14.836, 2.4, 3.0, PWR_DRILL)
    s += label("VIN-", 1.0, -12.2)
    s += label("VIN+", 9.0, -12.2)
    return s + ")\n"


def ina219():
    """Board 25.4 x 22.24 mm. Goc = chan VCC cua hang rao 1x6.

    Hang rao: VCC, GND, SCL, SDA, VIN-, VIN+. Hai lo domino 3.5 mm (VIN-, VIN+) cach
    hang rao 16.106 mm; pad trung so voi chan hang rao tuong ung. Dong lon phai di qua
    hai lo domino (chan hang rao chi chiu khoang 3 A).
    So pad theo ky hieu: 1 VCC, 2 GND, 3 SCL, 4 SDA, 5 VIN-, 6 VIN+.
    """
    s = header("INA219_Module", "Module INA219 kieu Adafruit 904: hang rao 1x6 + 2 lo domino 3.5 mm")
    s += text("Reference", "REF**", 6.35, 3.9, "F.SilkS")
    s += text("Value", "INA219_Module", 6.35, -20.6, "F.Fab")
    s += outline(-6.731, -19.446, 18.669, 2.794)
    # nhan hang rao buoc 2.54 mm: "VIN-"/"VIN+" co 0.8 mm se de len nhau nen viet tat V-/V+
    for i, n in enumerate(["VCC", "GND", "SCL", "SDA", "V-", "V+"]):
        x = i * 2.54
        # hai chan VIN-/VIN+ cua hang rao de trong (khong gan net): dong lon di qua hai lo domino,
        # tren module hai cap chan nay da noi san voi nhau
        s += pad(i + 1 if i < 4 else "", x, 0, 1.7, 2.2, HDR_DRILL, "rect" if i == 0 else "oval")
        s += label(n, x, -2.2)
    s += pad("5", 4.269, -16.106, 2.6, 3.4, PWR_DRILL)
    s += pad("6", 7.769, -16.106, 2.6, 3.4, PWR_DRILL)
    s += label("VIN-", 1.6, -13.2)
    s += label("VIN+", 10.4, -13.2)
    s += model("${KICAD10_3DMODEL_DIR}/Connector_PinHeader_2.54mm.3dshapes/PinHeader_1x06_P2.54mm_Vertical.step", (0, 0, 0), (0, 0, -90))
    s += model(M3D + "ina219.wrl", (-6.44, -3.0, 2.5), (-90, 0, 0))
    return s + ")" + chr(10)


def xl4015():
    """Board 54 x 24 mm, 4 pad o 4 goc: cach 50 mm theo chieu dai, 20 mm theo chieu ngan."""
    s = header("XL4015_Module", "Module buck XL4015 5A, 4 pad goc 50 x 20 mm")
    s += text("Reference", "REF**", 0, -13.2, "F.SilkS")
    s += text("Value", "XL4015_Module", 0, 13.2, "F.Fab")
    s += outline(-27, -12, 27, 12)
    # canh doc cua khung silkscreen di qua 4 pad o goc: chi giu doan giua hai pad (|y| <= 8)
    for x in (-27, 27):
        bad = '  (fp_line (start %.3f %.3f) (end %.3f %.3f)\n    (stroke (width 0.12) (type solid)) (layer "F.SilkS"))\n'
        s = s.replace(bad % (x, -12 if x > 0 else 12, x, 12 if x > 0 else -12),
                      bad % (x, -8, x, 8))
    for num, n, x, y in [("1", "IN+", -25, -10), ("2", "IN-", -25, 10),
                         ("3", "OUT+", 25, -10), ("4", "OUT-", 25, 10)]:
        s += pad(num, x, y, 4.0, 3.2, PWR_DRILL, "rect" if num == "1" else "oval")
        s += label(n, x + (5.5 if x < 0 else -5.5), y, 1.0)
    s += model(M3D + "xl4015.step", (0, 0, 2.5), (0, 0, 90))
    return s + ")\n"


if __name__ == "__main__":
    OUT.mkdir(exist_ok=True)
    for name, fn in [("TB6612FNG_Module", tb6612), ("INA169_Module", ina169), ("INA219_Module", ina219), ("XL4015_Module", xl4015)]:
        (OUT / f"{name}.kicad_mod").write_text(fn(), encoding="utf-8", newline="\n")
        print("wrote", name)
