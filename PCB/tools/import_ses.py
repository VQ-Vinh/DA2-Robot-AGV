"""Nhap ket qua Freerouting (.ses) vao board, phu dong GND ca 2 lop roi luu (Python cua KiCad).

    "<KiCad>/bin/python.exe" tools/import_ses.py agv_mainboard.kicad_pcb board.ses

Quy trinh di day 2 lop:
  1. tools/place_board.py agv_mainboard.kicad_pcb board.dsn   (xep linh kien, xuat DSN)
  2. java -jar freerouting-2.1.0.jar -de board.dsn -do board.ses -mp 100 --gui.enabled=false
     (chay trong WSL bang Java 21; ban CLI native 2.5.0 bi loi SerializationConstructorAccessor)
  3. tools/import_ses.py agv_mainboard.kicad_pcb board.ses
  4. kicad-cli pcb drc --refill-zones agv_mainboard.kicad_pcb
Day cu va vung phu dong cu bi xoa truoc khi nhap.
"""
import os
import shutil
import sys
import tempfile
import pcbnew
from pcbnew import FromMM

BOARD = (50.0, 50.0, 200.0, 175.0)


def main(pcb, ses):
    board = pcbnew.LoadBoard(pcb)
    for t in list(board.GetTracks()):
        board.Remove(t)
    if not pcbnew.ImportSpecctraSES(board, ses):
        raise SystemExit("import SES failed")

    # Giu vung phu GND da co (go zone bang RemoveNative roi do dong lai lam KiCad 10 segfault);
    # chi tao moi tren lop chua co
    gnd = board.FindNet("GND")
    have = {L for i in range(board.GetAreaCount()) for L in (pcbnew.F_Cu, pcbnew.B_Cu)
            if board.GetArea(i).GetNetname() == "GND" and board.GetArea(i).IsOnLayer(L)}
    x1, y1, x2, y2 = BOARD
    for layer in (pcbnew.F_Cu, pcbnew.B_Cu):
        if layer in have:
            continue
        z = pcbnew.ZONE(board)
        z.SetLayer(layer)
        z.SetNet(gnd)
        z.SetLocalClearance(FromMM(0.3))
        z.SetMinThickness(FromMM(0.25))
        z.SetPadConnection(pcbnew.ZONE_CONNECTION_THERMAL)
        z.SetThermalReliefGap(FromMM(0.4))
        z.SetThermalReliefSpokeWidth(FromMM(0.5))
        ol = pcbnew.SHAPE_POLY_SET()
        ol.NewOutline()
        for x, y in ((x1, y1), (x2, y1), (x2, y2), (x1, y2)):
            ol.Append(FromMM(x), FromMM(y))
        z.SetOutline(ol)
        board.Add(z)
    pcbnew.ZONE_FILLER(board).Fill([board.GetArea(i) for i in range(board.GetAreaCount())])

    tmp = os.path.join(tempfile.gettempdir(), "import_ses_tmp.kicad_pcb")
    if not pcbnew.SaveBoard(tmp, board):
        raise SystemExit("SaveBoard failed")
    shutil.copyfile(tmp, pcb)


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
