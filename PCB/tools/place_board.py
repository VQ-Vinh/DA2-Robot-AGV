"""Xep linh kien board me theo khoi chuc nang (chay bang Python cua KiCad, khi KiCad da dong board).

    "C:/Users/<ten>/AppData/Local/Programs/KiCad/10.0/bin/python.exe" tools/place_board.py agv_mainboard.kicad_pcb

Toa do mm, goc do (nguoc chieu kim dong ho). Board 150 x 125 mm, goc tren trai tai (50, 50).
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

PLACEMENT = {
    # MCU: hai hang rao cua Discovery (than board 66 x 97 mm, mep tren tai y = 51.3)
    "J10": (127.0, 85.09, 0), "J11": (177.8, 85.09, 0),
    # Nguon 5 V / 3.3 V
    "U2": (79, 64, 0), "U3": (97, 82, 0), "C5": (95, 89, 0), "D2": (174.16, 80, 180), "J4": (57, 84, 270),
    # Nguon vao, bao ve, sac
    "J1": (54, 170, 0), "F1": (67.8, 171.5, 90), "Q1": (66, 143.5, 0), "J2": (57, 139, 90), "J3": (57, 121, 270), "D1": (65, 136, 90),
    # Do dong, do ap pin
    "U1": (90, 117.9, 180),
    # Dung khan, nguon dong co
    "J5": (101.5, 112, 0), "C6": (113, 150, 0), "R8": (186, 157, 0), "D4": (190, 163.5, 180),
    "R5": (189, 128, 90), "R6": (189, 140.6, 90),
    # Coi
    "BZ1": (75.5, 107, 0), "Q2": (81, 95, 0), "R12": (72, 90.5, 0), "R13": (83.5, 98.8, 180), "D6": (87.6, 110, 90),
    # Driver dong co DC
    "U5": (102.7, 122, 0), "U4": (135.3, 171.2, 90),
    "J22": (76, 168.1, 0), "J23": (91, 168.1, 0), "J20": (106, 168.1, 0), "J21": (121, 168.1, 0),
    # Co cau nang
    "U6": (156.5, 170, 90), "J30": (180.9, 160, 270), "JP1": (166.66, 152.3, 90), "C12": (194.5, 168, 0), "R11": (184, 151, 0),
    "J31": (195, 99, 270), "C13": (189, 104, 90), "J32": (195, 109, 270), "C14": (189, 114, 90),
    # IMU, I2C, UART
    "U7": (188.5, 62, 0), "J40": (195, 85.3, 270), "J41": (100, 96, 0),
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


def main(path):
    board = pcbnew.LoadBoard(path)
    refs = {fp.GetReference(): fp for fp in board.GetFootprints()}
    missing = sorted(set(refs) - set(PLACEMENT)), sorted(set(PLACEMENT) - set(refs))
    for ref, (x, y, rot) in PLACEMENT.items():
        fp = refs[ref]
        fp.SetPosition(VECTOR2I(FromMM(x), FromMM(y)))
        fp.SetOrientationDegrees(rot)

    # INA219: hai chan VIN-/VIN+ tren hang rao khong mang net (dong lon di qua hai lo domino)
    u1 = refs["U1"]
    for pad in u1.Pads():
        if pad.GetNumber() in ("5", "6") and abs(ToMM(pad.GetFPRelativePosition().y)) < 0.5:
            pad.SetNumber("")
            pad.SetNetCode(0)

    # bo day cu va vung phu dong cu (neu co), ve lai vien board
    for t in list(board.GetTracks()):
        board.Remove(t)
    for z in list(board.Zones()):
        board.Remove(z)
    for d in list(board.GetDrawings()):
        if d.GetLayer() == pcbnew.Edge_Cuts:
            board.Remove(d)
    x1, y1, x2, y2 = BOARD
    rect = pcbnew.PCB_SHAPE(board)
    rect.SetShape(pcbnew.SHAPE_T_RECT)
    rect.SetStart(VECTOR2I(FromMM(x1), FromMM(y1)))
    rect.SetEnd(VECTOR2I(FromMM(x2), FromMM(y2)))
    rect.SetLayer(pcbnew.Edge_Cuts)
    rect.SetWidth(FromMM(0.1))
    board.Add(rect)

    # KiCad dang mo board thi SaveBoard len chinh file do khong ghi duoc: luu ra file tam roi chep de
    tmp = os.path.join(tempfile.gettempdir(), "place_board_tmp.kicad_pcb")
    if not pcbnew.SaveBoard(tmp, board):
        raise SystemExit("SaveBoard failed")
    shutil.copyfile(tmp, path)

    # soat chong lan vung chiem cho (courtyard) va lan mep: doc lai file da luu
    boxes = courtyards(path)
    names = sorted(boxes)
    x1, y1, x2, y2 = BOARD
    overlaps = []
    for i, a in enumerate(names):
        ax1, ay1, ax2, ay2 = boxes[a]
        if ax1 < x1 or ay1 < y1 or ax2 > x2 or ay2 > y2:
            overlaps.append("%s lan mep (%.1f %.1f %.1f %.1f)" % (a, ax1, ay1, ax2, ay2))
        for b in names[i + 1:]:
            if {a, b} == {"J10", "J11"}:
                continue
            bx1, by1, bx2, by2 = boxes[b]
            ox, oy = min(ax2, bx2) - max(ax1, bx1), min(ay2, by2) - max(ay1, by1)
            if ox > 0.05 and oy > 0.05:
                overlaps.append("%s-%s %.1fx%.1f" % (a, b, ox, oy))
    print("placed %d footprints | not in table: %s | not on board: %s" % (len(PLACEMENT), missing[0], missing[1]))
    print("courtyard issues (%d): %s" % (len(overlaps), overlaps))


if __name__ == "__main__":
    main(sys.argv[1])
