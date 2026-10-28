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
    # board.Remove(zone) loi SWIG ('SwigPyObject' khong co thisown) o KiCad 10: dung RemoveNative
    for z in [board.GetArea(i) for i in range(board.GetAreaCount())]:
        board.RemoveNative(z)
    for t in list(board.GetTracks()):
        board.Remove(t)
    if not pcbnew.ImportSpecctraSES(board, ses):
        raise SystemExit("import SES failed")

    gnd = board.FindNet("GND")
    x1, y1, x2, y2 = BOARD
    for layer in (pcbnew.F_Cu, pcbnew.B_Cu):
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
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())

    tmp = os.path.join(tempfile.gettempdir(), "import_ses_tmp.kicad_pcb")
    if not pcbnew.SaveBoard(tmp, board):
        raise SystemExit("SaveBoard failed")
    shutil.copyfile(tmp, pcb)


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
