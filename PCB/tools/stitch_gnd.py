"""Noi pad GND bi co lap (vung phu dong khong lot toi) vao vung GND chinh bang mot doan day ngan,
neu can thi them 1 via sang lop kia (Python cua KiCad).

    "<KiCad>/bin/python.exe" tools/stitch_gnd.py agv_mainboard.kicad_pcb C6.2 U4.3

Cach tim: thoi phong dong cua moi net khac GND (pad, day, via) them khoang ho + nua be rong day thanh
vung cam; xet cac diem cuoi tren luoi 0.25 mm trong ban kinh 12 mm quanh pad, giu diem nam trong
manh dong GND lon nhat cua lop do va doan thang pad -> diem khong cat vung cam. Chon doan ngan nhat;
khong co thi thu dat via (ca hai lop phai trong) roi noi tu via sang manh GND lon nhat cua lop kia.
"""
import math
import os
import shutil
import sys
import tempfile
import pcbnew
from pcbnew import FromMM, ToMM, VECTOR2I

CLEAR = 0.25     # mm, khoang ho toi thieu cua net class
TRACK = 0.6      # mm, be rong day GND (lop Ground)
VIA_D, VIA_H = 0.8, 0.4
LAYERS = (pcbnew.F_Cu, pcbnew.B_Cu)


def obstacles(board, layer, inflate):
    ps = pcbnew.SHAPE_POLY_SET()
    err = FromMM(0.005)
    for fp in board.GetFootprints():
        for p in fp.Pads():
            if p.GetNetname() != "GND" and p.IsOnLayer(layer):
                p.TransformShapeToPolygon(ps, layer, FromMM(inflate), err, pcbnew.ERROR_INSIDE)
    for t in board.GetTracks():
        if t.GetNetname() != "GND" and t.IsOnLayer(layer):
            t.TransformShapeToPolygon(ps, layer, FromMM(inflate), err, pcbnew.ERROR_INSIDE)
    ps.Simplify()
    return ps


def main_fill(board, layer):
    best, area = None, 0
    for i in range(board.GetAreaCount()):
        z = board.GetArea(i)
        if z.GetNetname() != "GND" or not z.IsOnLayer(layer):
            continue
        fill = z.GetFilledPolysList(layer)
        for k in range(fill.OutlineCount()):
            one = pcbnew.SHAPE_POLY_SET(fill.COutline(k))
            a = one.Area()
            if a > area:
                best, area = one, a
    return best


def seg_clear(obs, a, b, skip_r):
    """Doan a->b khong cham vung cam (bo qua doan nam trong pad nguon, ban kinh skip_r)."""
    n = max(2, int(math.dist(a, b) / 0.1))
    for i in range(n + 1):
        x = a[0] + (b[0] - a[0]) * i / n
        y = a[1] + (b[1] - a[1]) * i / n
        if math.dist((x, y), a) < skip_r:
            continue
        if obs.Contains(VECTOR2I(FromMM(x), FromMM(y))):
            return False
    return True


def add_track(board, net, a, b, layer):
    t = pcbnew.PCB_TRACK(board)
    t.SetStart(VECTOR2I(FromMM(a[0]), FromMM(a[1])))
    t.SetEnd(VECTOR2I(FromMM(b[0]), FromMM(b[1])))
    t.SetWidth(FromMM(TRACK))
    t.SetLayer(layer)
    t.SetNet(net)
    board.Add(t)


def stitch(board, ref, num):
    fp = board.FindFootprintByReference(ref)
    pad = [p for p in fp.Pads() if p.GetNumber() == num][0]
    net = pad.GetNet()
    c = (ToMM(pad.GetPosition().x), ToMM(pad.GetPosition().y))
    skip = ToMM(max(pad.GetSize(pcbnew.F_Cu).x, pad.GetSize(pcbnew.F_Cu).y)) / 2
    obs_t = {L: obstacles(board, L, CLEAR + TRACK / 2) for L in LAYERS}
    obs_v = {L: obstacles(board, L, CLEAR + VIA_D / 2) for L in LAYERS}
    fills = {L: main_fill(board, L) for L in LAYERS}
    pts = []
    r = float(os.environ.get("STITCH_R", "12"))
    steps = int(r / 0.25)
    for i in range(-steps, steps + 1):
        for j in range(-steps, steps + 1):
            p = (c[0] + i * 0.25, c[1] + j * 0.25)
            d = math.dist(p, c)
            if skip < d <= r:
                pts.append((d, p))
    pts.sort()

    def inside_fill(L, p, margin):
        # diem cuoi phai nam sau trong manh GND chinh de day noi chac vao vung phu
        if fills[L] is None:
            return False
        for dx, dy in ((0, 0), (margin, 0), (-margin, 0), (0, margin), (0, -margin)):
            if not fills[L].Contains(VECTOR2I(FromMM(p[0] + dx), FromMM(p[1] + dy))):
                return False
        return True

    # 1) chi mot doan day tren cung lop
    for d, p in pts:
        for L in LAYERS:
            if inside_fill(L, p, TRACK) and seg_clear(obs_t[L], c, p, skip):
                add_track(board, net, c, p, L)
                return "%s.%s: day %.2f mm tren %s toi (%.2f, %.2f)" % (ref, num, d, board.GetLayerName(L), p[0], p[1])
    # 2) day + via sang lop kia
    for d, p in pts:
        v = VECTOR2I(FromMM(p[0]), FromMM(p[1]))
        if any(obs_v[L].Contains(v) for L in LAYERS):
            continue
        for L in LAYERS:
            other = LAYERS[1] if L == LAYERS[0] else LAYERS[0]
            if seg_clear(obs_t[L], c, p, skip) and inside_fill(other, p, VIA_D):
                add_track(board, net, c, p, L)
                via = pcbnew.PCB_VIA(board)
                via.SetPosition(v)
                via.SetWidth(FromMM(VIA_D))
                via.SetDrill(FromMM(VIA_H))
                via.SetNet(net)
                board.Add(via)
                return "%s.%s: day %.2f mm tren %s + via tai (%.2f, %.2f)" % (ref, num, d, board.GetLayerName(L), p[0], p[1])
    return "%s.%s: KHONG tim duoc duong" % (ref, num)


def tie(board, a, b):
    """Noi thang 2 pad GND (vd chan GND cua cung mot module) bang 1 doan day neu doan thang khong pham."""
    def pad(rp):
        ref, num = rp.split(".")
        return [p for p in board.FindFootprintByReference(ref).Pads() if p.GetNumber() == num][0]
    pa, pb = pad(a), pad(b)
    ca = (ToMM(pa.GetPosition().x), ToMM(pa.GetPosition().y))
    cb = (ToMM(pb.GetPosition().x), ToMM(pb.GetPosition().y))
    skip = ToMM(max(pa.GetSize(pcbnew.F_Cu).x, pa.GetSize(pcbnew.F_Cu).y)) / 2
    for L in LAYERS:
        obs = obstacles(board, L, CLEAR + TRACK / 2)
        if seg_clear(obs, ca, cb, skip) and seg_clear(obs, cb, ca, skip):
            add_track(board, pa.GetNet(), ca, cb, L)
            return "%s-%s: day thang %.2f mm tren %s" % (a, b, math.dist(ca, cb), board.GetLayerName(L))
    return "%s-%s: KHONG noi thang duoc" % (a, b)


def via_fence(board, pitch=5.0):
    """Rai via GND theo luoi `pitch` mm o moi cho ca 2 lop deu co dong GND va khong pham khoang ho,
    de cac manh GND tren 2 lop noi voi nhau (manh khong noi duoc se bi xoa khi do lai dong)."""
    obs_v = {L: obstacles(board, L, CLEAR + VIA_D / 2) for L in LAYERS}
    fills = {}
    for L in LAYERS:
        fs = pcbnew.SHAPE_POLY_SET()
        for i in range(board.GetAreaCount()):
            z = board.GetArea(i)
            if z.GetNetname() == "GND" and z.IsOnLayer(L):
                fs.Append(z.GetFilledPolysList(L))
        fills[L] = fs
    net = board.FindNet("GND")
    x1, y1, x2, y2 = 50.0, 50.0, 200.0, 175.0
    n = 0
    y = y1 + 2.0
    while y < y2 - 1.5:
        x = x1 + 2.0
        while x < x2 - 1.5:
            v = VECTOR2I(FromMM(x), FromMM(y))
            ok = all(not obs_v[L].Contains(v) for L in LAYERS)
            # via phai nam han trong dong GND ca 2 lop (xet 4 diem tren vanh via)
            for dx, dy in ((0, 0), (VIA_D, 0), (-VIA_D, 0), (0, VIA_D), (0, -VIA_D)):
                p = VECTOR2I(FromMM(x + dx), FromMM(y + dy))
                ok = ok and all(fills[L].Contains(p) for L in LAYERS)
            if ok:
                via = pcbnew.PCB_VIA(board)
                via.SetPosition(v)
                via.SetWidth(FromMM(VIA_D))
                via.SetDrill(FromMM(VIA_H))
                via.SetNet(net)
                board.Add(via)
                n += 1
            x += pitch
        y += pitch
    return n


def main(path, pads):
    board = pcbnew.LoadBoard(path)
    zones = [board.GetArea(i) for i in range(board.GetAreaCount())]
    for z in zones:
        z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_NEVER)
    pcbnew.ZONE_FILLER(board).Fill(zones)
    for rp in pads:
        if rp == "--fence":
            print("via GND rai luoi:", via_fence(board))
            continue
        if "-" in rp:
            print(tie(board, *rp.split("-")))
            continue
        ref, num = rp.split(".")
        print(stitch(board, ref, num))
    for z in zones:
        z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)
    pcbnew.ZONE_FILLER(board).Fill([board.GetArea(i) for i in range(board.GetAreaCount())])
    tmp = os.path.join(tempfile.gettempdir(), "stitch_gnd_tmp.kicad_pcb")
    if not pcbnew.SaveBoard(tmp, board):
        raise SystemExit("SaveBoard failed")
    shutil.copyfile(tmp, path)


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2:])
