"""Thay footprint cua 1 linh kien tren board, giu ten, lien ket so do va net theo so chan (Python cua KiCad).

    "<KiCad>/bin/python.exe" tools/swap_footprint.py agv_mainboard.kicad_pcb J1 lib/agv_modules.pretty XT60PW-M_Battery_Pin1Plus

Dung khi doi footprint trong so do ma khong muon chay lai "Update PCB from schematic" (lenh do cua MCP
do sai net tren so do co day, xem REPORT 4.17). Vi tri giu nguyen; xep lai bang place_board.py.
"""
import os
import shutil
import sys
import tempfile
import pcbnew


def main(path, ref, lib, name):
    board = pcbnew.LoadBoard(path)
    old = board.FindFootprintByReference(ref)
    new = pcbnew.FootprintLoad(lib, name)
    if new is None:
        raise SystemExit("khong tim thay %s trong %s" % (name, lib))
    new.SetReference(ref)
    new.SetValue(old.GetValue())
    new.SetPath(old.GetPath())
    new.SetPosition(old.GetPosition())
    new.SetOrientation(old.GetOrientation())
    nets = {p.GetNumber(): p.GetNet() for p in old.Pads()}
    for p in new.Pads():
        if p.GetNumber() in nets:
            p.SetNet(nets[p.GetNumber()])
    board.Remove(old)
    board.Add(new)
    tmp = os.path.join(tempfile.gettempdir(), "swap_fp_tmp.kicad_pcb")
    if not pcbnew.SaveBoard(tmp, board):
        raise SystemExit("SaveBoard failed")
    shutil.copyfile(tmp, path)
    print(ref, "->", name, {p.GetNumber(): p.GetNetname() for p in new.Pads() if p.GetNumber()})


if __name__ == "__main__":
    main(*sys.argv[1:5])
