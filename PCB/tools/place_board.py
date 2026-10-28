"""Xep linh kien board me 2 lop theo khoi chuc nang (chay bang Python cua KiCad).

    "C:/Users/<ten>/AppData/Local/Programs/KiCad/10.0/bin/python.exe" tools/place_board.py agv_mainboard.kicad_pcb [out.dsn]

Board 150 x 125 mm, goc tren trai tai (50, 50). Discovery dat goc tren phai (cong mini-USB nap code
sat mep tren); ben trai la nguon, driver dong co, co cau nang; dau noi dong co o mep duoi;
I2C / UART / cong tac hanh trinh o dai duoi Discovery. Moi linh kien cho bang goc tren-trai cua
hop courtyard (mm) va goc xoay (do), de doc va sua de hon toa do tam.
Xoa day cu, them 4 lo M3 o goc (khop tru 30 mm trong CAD). Co doi so thu 2 thi xuat file .dsn
cho Freerouting.
"""
import math
import os
import re
import shutil
import sys
import tempfile
import pcbnew
from pcbnew import FromMM, ToMM, VECTOR2I

BOARD = (50.0, 50.0, 200.0, 175.0)
HOLES = [(54, 54), (196, 54), (54, 171), (196, 171)]

# Discovery: chan 1 cua J10 tai (134, 84.5), J11 cach 50.8 mm; than board 66 x 97 mm tu y = 50.7,
# jack audio thoi ra ~4 mm o mep duoi (toi y ~ 151.7)
FIXED = {"J10": (134.0, 84.5, 0), "J11": (184.8, 84.5, 0)}

BOX = {
    # nguon vao, bao ve, do dong, 5 V / 3.3 V
    "J1": (58, 51, 0), "J4": (58, 62, 0),
    "U2": (72, 51, 0),
    "U1": (51, 78, 180), "F1": (79, 78, 0), "Q1": (108, 78, 0), "U3": (108, 86, 0), "C5": (120.5, 85, 0),
    "D2": (107, 93, 180),
    "J5": (51, 104, 90), "J3": (51, 117, 90), "J2": (51, 130, 90), "D1": (62, 104, 90),
    # driver dong co, co cau nang
    "U5": (70, 104, 0), "U4": (95, 104, 0),
    "U6": (70, 128, 90), "JP1": (94, 129, 90), "C12": (94, 137, 0), "C6": (105, 128, 0), "R11": (105, 141, 0),
    # coi, do dung khan, LED
    "BZ1": (51, 145, 0), "Q2": (66, 147, 0), "D6": (75, 148, 0), "R12": (66, 155, 0), "R13": (80, 155, 0),
    "R5": (94, 147, 0), "R6": (94, 152, 0), "R8": (129, 153, 0), "D4": (143, 152.5, 180),
    # IMU: bo GY-BNO055 chia len ~12 mm phia tren de, khong nam duoi jack audio cua Discovery
    "U7": (106, 160, 90),
    # dau noi dong co + buoc o mep duoi
    "J22": (59, 165.5, 0), "J23": (76, 165.5, 0), "J20": (93, 165.5, 0), "J21": (110, 165.5, 0),
    "J30": (128, 166, 0),
    # I2C, UART, cong tac hanh trinh
    "J40": (144, 166, 0), "J41": (160, 166, 0), "J31": (173, 166, 0), "J32": (183, 166, 0),
    "C13": (175, 160, 0), "C14": (185, 160, 0),
}


def courtyards(path):
    """Hop bao F.CrtYd cua tung footprint, tinh tu file .kicad_pcb (mm, toa do board)."""
    txt = open(path, encoding="utf-8").read()
    out = {}
    for blk in re.split(r"\n\t\(footprint ", txt)[1:]:
        m = re.search(r"\(at ([-\d.]+) ([-\d.]+)(?: ([-\d.]+))?\)", blk)
        px, py, rot = float(m.group(1)), float(m.group(2)), math.radians(float(m.group(3) or 0))
        ref = re.search(r'\(property "Reference" "([^"]+)"', blk).group(1)
        pts = []
        for lm in re.finditer(r'\(layer "F\.CrtYd"\)', blk):
            item = blk[blk.rfind("(fp_", 0, lm.start()):lm.start()]
            pts += [(float(x), float(y)) for x, y in re.findall(r"\((?:start|end|xy|center) ([-\d.]+) ([-\d.]+)\)", item)]
        if not pts:
            pts = [(float(x), float(y)) for x, y in re.findall(r"\(pad [^\n]*\n\s*\(at ([-\d.]+) ([-\d.]+)", blk)]
        xs = [px + x * math.cos(rot) + y * math.sin(rot) for x, y in pts]
        ys = [py - x * math.sin(rot) + y * math.cos(rot) for x, y in pts]
        out[ref] = (min(xs), min(ys), max(xs), max(ys))
    return out


def main(path, dsn=None):
    board = pcbnew.LoadBoard(path)
    refs = {fp.GetReference(): fp for fp in board.GetFootprints()}
    for ref, (x, y, rot) in FIXED.items():
        refs[ref].SetOrientationDegrees(rot)
        refs[ref].SetPosition(VECTOR2I(FromMM(x), FromMM(y)))
    for ref, (x, y, rot) in BOX.items():
        fp = refs[ref]
        fp.SetOrientationDegrees(rot)
        cy = fp.GetCourtyard(pcbnew.F_CrtYd)
        bb = cy.BBox() if cy.OutlineCount() else fp.GetBoundingBox(False)
        p = fp.GetPosition()
        fp.SetPosition(VECTOR2I(p.x + FromMM(x) - bb.GetX(), p.y + FromMM(y) - bb.GetY()))
    missing = sorted(r for r in refs if r not in FIXED and r not in BOX and not r.startswith("H"))

    # lo bat M3: chi co tren board (khong co trong so do nen danh dau board-only)
    lib = os.path.join(os.path.dirname(os.path.dirname(sys.executable)), "share", "kicad", "footprints", "MountingHole.pretty")
    for i, (x, y) in enumerate(HOLES, start=1):
        ref = "H%d" % i
        if ref in refs:
            board.Remove(refs[ref])
        fp = pcbnew.FootprintLoad(lib, "MountingHole_3.2mm_M3")
        fp.SetReference(ref)
        fp.SetAttributes(fp.GetAttributes() | pcbnew.FP_BOARD_ONLY | pcbnew.FP_EXCLUDE_FROM_BOM | pcbnew.FP_EXCLUDE_FROM_POS_FILES)
        fp.SetPosition(VECTOR2I(FromMM(x), FromMM(y)))
        board.Add(fp)

    for t in list(board.GetTracks()):
        board.Remove(t)

    # KiCad dang mo board thi SaveBoard len chinh file do khong ghi duoc: luu ra file tam roi chep de
    tmp = os.path.join(tempfile.gettempdir(), "place_board_tmp.kicad_pcb")
    if not pcbnew.SaveBoard(tmp, board):
        raise SystemExit("SaveBoard failed")
    shutil.copyfile(tmp, path)
    if dsn:
        pcbnew.ExportSpecctraDSN(pcbnew.LoadBoard(path), dsn)

    # soat chong lan courtyard, lan mep, cham lo bat (doc lai file da luu)
    boxes = courtyards(path)
    names = sorted(n for n in boxes if not n.startswith("H"))
    x1, y1, x2, y2 = BOARD
    issues = []
    for i, a in enumerate(names):
        ax1, ay1, ax2, ay2 = boxes[a]
        if ax1 < x1 or ay1 < y1 or ax2 > x2 or ay2 > y2:
            issues.append("%s lan mep" % a)
        if a not in ("J10", "J11"):
            for hx, hy in HOLES:
                if ax1 - 3.5 < hx < ax2 + 3.5 and ay1 - 3.5 < hy < ay2 + 3.5:
                    issues.append("%s cham lo (%d, %d)" % (a, hx, hy))
        for b in names[i + 1:]:
            if {a, b} == {"J10", "J11"}:
                continue
            bx1, by1, bx2, by2 = boxes[b]
            ox, oy = min(ax2, bx2) - max(ax1, bx1), min(ay2, by2) - max(ay1, by1)
            if ox > 0.05 and oy > 0.05:
                issues.append("%s-%s %.1fx%.1f" % (a, b, ox, oy))
    print("placed %d | chua co vi tri: %s" % (len(FIXED) + len(BOX), missing))
    print("courtyard issues (%d): %s" % (len(issues), issues))


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else None)
