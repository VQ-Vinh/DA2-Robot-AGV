"""Sua silkscreen sau khi di day (Python cua KiCad):
  - chu tren silkscreen thap hon 0.8 mm (nhan chan module 0.7 mm) nang len 0.8 x 0.15 mm (muc in duoc cua JLCPCB);
  - an ten H1..H4 (lo bat);
  - ten linh kien nam sat / lot ra ngoai mep board thi dua vao giua than linh kien.

    "<KiCad>/bin/python.exe" tools/fix_silk.py agv_mainboard.kicad_pcb
"""
import os
import shutil
import sys
import tempfile
import pcbnew
from pcbnew import FromMM, ToMM

BOARD = (50.0, 50.0, 200.0, 175.0)
EDGE = 1.0      # (mm) khoang cach toi thieu tu chu toi mep board
MIN_H = 0.8


def main(path):
    board = pcbnew.LoadBoard(path)
    x1, y1, x2, y2 = BOARD
    raised = moved = 0
    for fp in board.GetFootprints():
        ref = fp.Reference()
        if fp.GetReference().startswith("H"):
            ref.SetVisible(False)
        texts = [ref, fp.Value()] + [t for t in fp.GraphicalItems() if isinstance(t, pcbnew.PCB_TEXT)]
        for t in texts:
            if t.GetLayer() not in (pcbnew.F_SilkS, pcbnew.B_SilkS) or not t.IsVisible():
                continue
            if ToMM(t.GetTextHeight()) < MIN_H - 1e-6:
                t.SetTextSize(pcbnew.VECTOR2I(FromMM(MIN_H), FromMM(MIN_H)))
                t.SetTextThickness(FromMM(0.15))
                raised += 1
        if ref.IsVisible() and ref.GetLayer() == pcbnew.F_SilkS:
            bb = ref.GetBoundingBox()
            if ToMM(bb.GetLeft()) < x1 + EDGE or ToMM(bb.GetTop()) < y1 + EDGE or \
               ToMM(bb.GetRight()) > x2 - EDGE or ToMM(bb.GetBottom()) > y2 - EDGE:
                ref.SetPosition(fp.GetBoundingBox(False).GetCenter())
                moved += 1
    tmp = os.path.join(tempfile.gettempdir(), "fix_silk_tmp.kicad_pcb")
    if not pcbnew.SaveBoard(tmp, board):
        raise SystemExit("SaveBoard failed")
    shutil.copyfile(tmp, path)
    print("chu nang len %.1f mm: %d, ten dua vao trong: %d" % (MIN_H, raised, moved))


if __name__ == "__main__":
    main(sys.argv[1])
