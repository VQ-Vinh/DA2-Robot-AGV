"""Phu dong GND len toan bo mat duoi (B.Cu) cua board me.

    "<KiCad>/bin/python.exe" tools/add_gnd_pour.py agv_mainboard.kicad_pcb [--out khac.kicad_pcb]

Vung phu cu (neu co) bi xoa truoc. Chan GND noi lien voi vung phu (khong dung nhanh tan nhiet), khoang ho toi dong khac 0.8 mm.
"""
import os
import shutil
import sys
import tempfile

import pcbnew
from pcbnew import FromMM, VECTOR2I


def main():
    path = sys.argv[1]
    out = sys.argv[sys.argv.index("--out") + 1] if "--out" in sys.argv else path
    board = pcbnew.LoadBoard(path)
    for z in list(board.Zones()):
        board.Remove(z)
    xs, ys = [], []
    for d in board.GetDrawings():
        if d.GetLayer() == pcbnew.Edge_Cuts:
            for p in (d.GetStart(), d.GetEnd()):
                xs.append(p.x)
                ys.append(p.y)
    zone = pcbnew.ZONE(board)
    zone.SetLayer(pcbnew.B_Cu)
    zone.SetNetCode(board.GetNetcodeFromNetname("GND"))
    zone.SetZoneName("GND")
    e = FromMM(1.0)     # lui vao 1 mm so voi mep board
    for x, y in ((min(xs) + e, min(ys) + e), (max(xs) - e, min(ys) + e), (max(xs) - e, max(ys) - e), (min(xs) + e, max(ys) - e)):
        zone.AppendCorner(VECTOR2I(x, y), -1)
    zone.SetLocalClearance(FromMM(0.8))
    zone.SetMinThickness(FromMM(0.6))
    zone.SetPadConnection(pcbnew.ZONE_CONNECTION_FULL)
    zone.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)
    board.Add(zone)
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    tmp = os.path.join(tempfile.gettempdir(), "gnd_pour_tmp.kicad_pcb")
    if not pcbnew.SaveBoard(tmp, board):
        raise SystemExit("SaveBoard failed")
    shutil.copyfile(tmp, out)
    print("GND pour added, filled area %.0f mm2" % (zone.GetFilledArea() / 1e12))


if __name__ == "__main__":
    main()
