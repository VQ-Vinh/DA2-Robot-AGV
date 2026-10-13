"""Dua toan bo linh kien ra ngoai vien board, xep thanh tung hang theo khoi chuc nang
(de tu xep lai bang tay trong KiCad). Giu nguyen goc xoay; xoa day va vung phu dong neu con.

    "<KiCad>/bin/python.exe" tools/unplace_board.py agv_mainboard.kicad_pcb
"""
import os
import shutil
import sys
import tempfile

import pcbnew
from pcbnew import FromMM, ToMM, VECTOR2I

from place_board import courtyards

# moi khoi mot hang, ben phai board
GROUPS = [
    ("MCU", ["J10", "J11"]),
    ("Nguon vao - bao ve - sac", ["J1", "F1", "Q1", "J2", "J3", "D1"]),
    ("Do dong / ap pin", ["U1"]),
    ("Nguon 5 V / 3.3 V", ["U2", "U3", "C5", "D2", "J4"]),
    ("Dung khan - VMOT", ["J5", "C6", "R5", "R6", "R8", "D4"]),
    ("Coi", ["BZ1", "Q2", "R12", "R13", "D6"]),
    ("Driver dong co DC", ["U4", "U5", "J20", "J21", "J22", "J23"]),
    ("Co cau nang", ["U6", "J30", "JP1", "C12", "R11", "J31", "C13", "J32", "C14"]),
    ("IMU - I2C - UART", ["U7", "J40", "J41"]),
]
GAP = 4.0       # khoang cach giua cac linh kien (mm)
MARGIN = 15.0   # cach mep phai board (mm)


def main(path):
    boxes = courtyards(path)
    board = pcbnew.LoadBoard(path)
    refs = {fp.GetReference(): fp for fp in board.GetFootprints()}
    listed = [r for _, g in GROUPS for r in g]
    assert sorted(listed) == sorted(refs), (set(refs) ^ set(listed))
    xs, ys = [], []
    for d in board.GetDrawings():
        if d.GetLayer() == pcbnew.Edge_Cuts:
            for p in (d.GetStart(), d.GetEnd()):
                xs.append(ToMM(p.x))
                ys.append(ToMM(p.y))
    x0, y = max(xs) + MARGIN, min(ys)
    for _, group in GROUPS:
        x, row_h = x0, 0.0
        for ref in group:
            bx1, by1, bx2, by2 = boxes[ref]
            w, h = max(bx2 - bx1, 3.0), max(by2 - by1, 3.0)
            fp = refs[ref]
            pos = fp.GetPosition()
            # dat goc tren trai cua courtyard vao (x, y)
            fp.SetPosition(VECTOR2I(pos.x + FromMM(x - bx1), pos.y + FromMM(y - by1)))
            x += w + GAP
            row_h = max(row_h, h)
        y += row_h + GAP + 2
    for t in list(board.GetTracks()):
        board.Remove(t)
    tmp = os.path.join(tempfile.gettempdir(), "unplace_tmp.kicad_pcb")
    if not pcbnew.SaveBoard(tmp, board):
        raise SystemExit("SaveBoard failed")
    shutil.copyfile(tmp, path)
    # soat lai: khong con linh kien nao trong vien board, khong chong lan
    new = courtyards(path)
    inside = [r for r, b in new.items() if b[0] < max(xs) and b[2] > min(xs) and b[1] < max(ys) and b[3] > min(ys)]
    names = sorted(new)
    overlaps = [(a, b) for i, a in enumerate(names) for b in names[i + 1:]
                if min(new[a][2], new[b][2]) - max(new[a][0], new[b][0]) > 0.05
                and min(new[a][3], new[b][3]) - max(new[a][1], new[b][1]) > 0.05]
    print("moved %d footprints | still inside board: %s | overlaps: %s" % (len(refs), inside, overlaps))


if __name__ == "__main__":
    main(sys.argv[1])
