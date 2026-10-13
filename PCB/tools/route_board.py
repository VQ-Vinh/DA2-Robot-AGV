"""Di day thu cho board me 1 lop (B.Cu) bang thuat toan me cung tren luoi (A*), co go day di lai.

Chay bang Python cua KiCad:
    "<KiCad>/bin/python.exe" tools/route_board.py agv_mainboard.kicad_pcb [--order short|signal|power]
        [--seed N] [--minutes M] [--report out.json] [--out khac.kicad_pcb] [--verbose]

Cach lam:
  - Chia board thanh luoi 0.254 mm. Moi net co hai be rong: "rong" (di binh thuong) va "hep"
    (chi dung o cho chat, vi du luc thoat ra khoi chan linh kien); o chi di duoc be rong hep
    bi tinh chi phi cao hon nen duong day tu that lai dung cho can.
  - Buoc 1: di lan luot tung net (net pin truoc, sau do net ngan truoc), chua di GND.
  - Buoc 2 (go day di lai): voi moi net con thieu ket noi, tim xem day cua net nao dang chan
    duong; go cac net do ra, di net bi thieu truoc roi di lai cac net vua go. Chi giu thay doi
    neu tong so ket noi thieu giam.
  - Buoc 3: di GND bang day; sau do dung add_gnd_pour.py de phu dong GND.
  - Khong co lop thu hai nen khong co via: ket noi nao khong tim duoc duong thi de lai,
    bao cao la "can day nhay" (jumper).
Day cu tren board bi xoa truoc khi di lai.
"""
import heapq
import json
import math
import os
import random
import re
import shutil
import sys
import tempfile
import time

import numpy as np
import pcbnew
from pcbnew import FromMM, ToMM, VECTOR2I

G = 0.254                 # buoc luoi (mm)
EDGE_CLEARANCE = 1.0      # khoang ho dong - mep board (mm)
MARGIN = 0.2              # du phong sai so do luoi (mm)
NARROW_COST = 4           # he so chi phi cho o chi di duoc be rong hep
NEAR_PAD = 2.0            # (mm) vung quanh chan cua net khac: di qua bi tinh them chi phi,
NEAR_COST = 2             # de day khong om sat roi bit loi thoat cua chan chua di day
SOFT_COST = 60            # (chi khi do duong bi chan) chi phi di xuyen qua day cua net khac
MAX_BLOCKERS = 4          # go toi da bay nhieu net cho mot lan thu

# (ten lop, bieu thuc ten net, be rong rong, be rong hep, khoang ho) - khoang ho trung voi .kicad_pro
CLASSES = [
    ("Battery", r"^(BAT_IN|BAT_FUSED|VBAT_PROT|VBAT_SW|VBAT|CHG_IN)$", 4.0, 1.6, 0.8),
    ("MotorSupply", r"^VMOT$", 2.5, 1.0, 0.6),
    ("MotorOut", r"^(MOT\d_[PN]|STEP_\w+)$", 1.0, 0.8, 0.5),
    ("Supply", r"^(\+5V|\+5V_DISC|\+3V3)$", 1.2, 0.8, 0.5),
    ("Ground", r"^GND$", 1.2, 0.8, 0.5),
    ("Default", r".*", 0.8, 0.6, 0.5),
]
ORDERS = {
    "power": ["Battery", "MotorSupply", "MotorOut", "Supply", "Default", "Ground"],
    "signal": ["Battery", "MotorSupply", "MotorOut", "Default", "Supply", "Ground"],
    # chi uu tien duong pin, con lai xep theo do dai (net ngan di truoc)
    "short": ["Battery", "Default", "Default", "Default", "Default", "Ground"],
}

DIRS = [(1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1)]   # (dj, di)
TURN_COST = (0, 4, 12)


def net_class(name):
    n = name.lstrip("/")
    for cls, pat, wide, narrow, clr in CLASSES:
        if re.match(pat, n):
            return cls, wide, narrow, clr


def dilate(mask, r_mm):
    """No vung True cua mask ra ban kinh r_mm (hinh tron)."""
    r = r_mm / G
    R = int(math.floor(r))
    h, w = mask.shape
    out = np.zeros_like(mask)
    m = mask.astype(np.int32)
    for dy in range(-R, R + 1):
        span = int(math.floor(math.sqrt(r * r - dy * dy)))
        padded = np.zeros((h, w + 2 * span + 1), dtype=np.int32)
        padded[:, span + 1:span + 1 + w] = m
        cs = np.cumsum(padded, axis=1)
        hor = (cs[:, 2 * span + 1:] - cs[:, :w]) > 0
        if dy >= 0:
            out[dy:, :] |= hor[:h - dy, :]
        else:
            out[:h + dy, :] |= hor[-dy:, :]
    return out


class Grid:
    def __init__(self, board):
        xs, ys = [], []
        for d in board.GetDrawings():
            if d.GetLayer() == pcbnew.Edge_Cuts:
                for p in (d.GetStart(), d.GetEnd()):
                    xs.append(ToMM(p.x))
                    ys.append(ToMM(p.y))
        self.x1, self.y1, self.x2, self.y2 = min(xs), min(ys), max(xs), max(ys)
        self.W = int(round((self.x2 - self.x1) / G)) + 1
        self.H = int(round((self.y2 - self.y1) / G)) + 1
        jj, ii = np.meshgrid(np.arange(self.W), np.arange(self.H))
        self.X = self.x1 + jj * G
        self.Y = self.y1 + ii * G

    def cell(self, x, y):
        return int(round((y - self.y1) / G)), int(round((x - self.x1) / G))

    def xy(self, c):
        i, j = divmod(c, self.W)
        return self.x1 + j * G, self.y1 + i * G

    def edge_block(self, r):
        return ((self.X < self.x1 + r) | (self.X > self.x2 - r) | (self.Y < self.y1 + r) | (self.Y > self.y2 - r))

    def pad_mask(self, pad, grow=0.0):
        """O luoi nam trong dong cua pad (pad chu nhat, oval, tron), co the co/no them `grow` mm."""
        x, y, sx, sy, rot, shape = pad["x"], pad["y"], pad["sx"], pad["sy"], pad["rot"], pad["shape"]
        rad = math.hypot(sx, sy) / 2 + abs(grow) + G
        i0, j0 = self.cell(x - rad, y - rad)
        i1, j1 = self.cell(x + rad, y + rad)
        i0, j0, i1, j1 = max(i0, 0), max(j0, 0), min(i1, self.H - 1), min(j1, self.W - 1)
        X, Y = self.X[i0:i1 + 1, j0:j1 + 1] - x, self.Y[i0:i1 + 1, j0:j1 + 1] - y
        a = math.radians(rot)
        u = X * math.cos(a) - Y * math.sin(a)
        v = X * math.sin(a) + Y * math.cos(a)
        hx, hy = sx / 2 + grow, sy / 2 + grow
        if hx <= 0 or hy <= 0:
            inside = np.zeros_like(X, dtype=bool)
        elif shape == "rect":
            inside = (np.abs(u) <= hx) & (np.abs(v) <= hy)
        else:   # tron / oval: hinh "san van dong"
            r = min(hx, hy)
            du = np.maximum(np.abs(u) - (hx - r), 0)
            dv = np.maximum(np.abs(v) - (hy - r), 0)
            inside = du * du + dv * dv <= r * r
        return (slice(i0, i1 + 1), slice(j0, j1 + 1)), inside


def read_pads(board):
    pads = []
    for fp in board.GetFootprints():
        ref = fp.GetReference()
        for p in fp.Pads():
            pos, size = p.GetPosition(), p.GetSize()
            shape = "rect" if p.GetShape() in (pcbnew.PAD_SHAPE_RECT, pcbnew.PAD_SHAPE_ROUNDRECT) else "oval"
            sx, sy = ToMM(size.x), ToMM(size.y)
            if p.GetAttribute() == pcbnew.PAD_ATTRIB_NPTH:
                d = p.GetDrillSize()
                sx, sy = max(sx, ToMM(d.x)), max(sy, ToMM(d.y))
            pads.append(dict(ref=ref, num=p.GetNumber(), net=p.GetNetname(), code=p.GetNetCode(),
                             x=ToMM(pos.x), y=ToMM(pos.y), sx=sx, sy=sy,
                             rot=p.GetOrientationDegrees(), shape=shape))
    return pads


def astar(cost, W, H, comp, A, sources, tj, ti):
    """Tim duong tu cac o `sources` (thanh phan A) toi o bat ky thuoc thanh phan khac cua cung net."""
    best, parent, heap = {}, {}, []
    for s in sources:
        i, j = divmod(s, W)
        dj, di = abs(j - tj), abs(i - ti)
        best[s * 9 + 8] = 0
        heap.append((10 * max(dj, di) + 4 * min(dj, di), 0, s, 8))
    heapq.heapify(heap)
    while heap:
        f, g, c, d = heapq.heappop(heap)
        key = c * 9 + d
        if g > best.get(key, 1 << 60):
            continue
        cc = comp[c]
        if cc and cc != A:
            path = [c]
            while key in parent:
                key = parent[key]
                path.append(key // 9)
            return path[::-1]
        i, j = divmod(c, W)
        for nd in range(8):
            if d != 8:
                turn = (nd - d) % 8
                if turn > 4:
                    turn = 8 - turn
                if turn > 2:
                    continue
                tc = TURN_COST[turn]
            else:
                tc = 0
            dj, di = DIRS[nd]
            nj, ni = j + dj, i + di
            if nj < 0 or ni < 0 or nj >= W or ni >= H:
                continue
            n = ni * W + nj
            k = cost[n]
            if not k:
                continue
            if dj and di and not (cost[c + dj] and cost[c + di * W]):
                continue
            ng = g + (14 if dj and di else 10) * k + tc
            nkey = n * 9 + nd
            if ng < best.get(nkey, 1 << 60):
                best[nkey] = ng
                parent[nkey] = key
                a, b = abs(nj - tj), abs(ni - ti)
                heapq.heappush(heap, (ng + 10 * max(a, b) + 4 * min(a, b), ng, n, nd))
    return None


def reachable(cost, W, H, comp, A, sources):
    """Loang nhanh: tu thanh phan A co toi duoc thanh phan khac cua net khong."""
    seen = bytearray(W * H)
    stack = list(sources)
    for s in stack:
        seen[s] = 1
    while stack:
        c = stack.pop()
        cc = comp[c]
        if cc and cc != A:
            return True
        i, j = divmod(c, W)
        for dj, di in DIRS:
            nj, ni = j + dj, i + di
            if nj < 0 or ni < 0 or nj >= W or ni >= H:
                continue
            n = ni * W + nj
            if seen[n] or not cost[n]:
                continue
            if dj and di and not (cost[c + dj] and cost[c + di * W]):
                continue
            seen[n] = 1
            stack.append(n)
    return False


class Router:
    def __init__(self, board, log=print):
        self.grid = grid = Grid(board)
        self.log = log
        self.pads = read_pads(board)
        self.codes, nets = {}, {}
        self.owner = np.zeros((grid.H, grid.W), dtype=np.int32)     # 0 trong, -1 dong khong net, >0 ma net
        for p in self.pads:
            sl, inside = grid.pad_mask(p)
            self.owner[sl][inside] = p["code"] if p["net"] else -1
            if p["net"]:
                self.codes[p["net"]] = p["code"]
                nets.setdefault(p["net"], []).append(p)
        self.padowner = self.owner.copy()
        self.nets = {n: pl for n, pl in nets.items() if len(pl) > 1}
        self.names = {c: n for n, c in self.codes.items()}
        # khoang ho giua hai net = gia tri lon hon cua hai lop net
        self.levels = {}
        for name, code in self.codes.items():
            self.levels.setdefault(net_class(name)[3], []).append(code)
        self.fixed = np.array([c for n, c in self.codes.items() if net_class(n)[0] == "Battery"])
        self.res = {}       # ket qua tung net: segments, idx (o da to dong), groups, jumpers, length

    def hpwl(self, name):
        xs, ys = [p["x"] for p in self.nets[name]], [p["y"] for p in self.nets[name]]
        return (max(xs) - min(xs)) + (max(ys) - min(ys))

    def total(self):
        return sum(r["jumpers"] for r in self.res.values())

    def blocked(self, other, clr, half):
        """Vung tam day khong duoc vao: cach dong `other` it hon half + khoang ho."""
        blk = self.grid.edge_block(half + EDGE_CLEARANCE + MARGIN)
        rest = other.copy()
        for lv, lv_codes in self.levels.items():
            if lv > clr:
                m = other & np.isin(self.owner, lv_codes)
                if m.any():
                    rest &= ~m
                    blk |= dilate(m, half + lv + MARGIN)
        return blk | dilate(rest, half + clr + MARGIN)

    def unroute(self, name):
        r = self.res.pop(name, None)
        if r is not None and len(r["idx"]):
            self.owner.ravel()[r["idx"]] = 0

    def route_net(self, name, relaxed=False):
        """Di toan bo mot net. relaxed=True: chi do duong, cho phep di xuyen day cua net khac
        (chi phi cao) va tra ve tap net bi di xuyen; khong ghi gi len board."""
        t0 = time.time()
        grid, owner = self.grid, self.owner
        W, H = grid.W, grid.H
        cls, wide, narrow, clr = net_class(name)
        code, plist = self.codes[name], self.nets[name]
        other = (owner != 0) & (owner != code)
        if relaxed:
            # day cua net khac (tru duong pin) coi nhu di xuyen duoc
            soft_src = other & (self.padowner == 0) & ~np.isin(owner, self.fixed)
            hard = other & ~soft_src
            blk_n = self.blocked(hard, clr, narrow / 2)
            blk_w = self.blocked(hard, clr, wide / 2)
            soft = dilate(soft_src, narrow / 2 + 0.8 + MARGIN) & ~blk_n
        else:
            blk_n = self.blocked(other, clr, narrow / 2)
            blk_w = self.blocked(other, clr, wide / 2)
        if clr > 0.5 and not relaxed:
            # sat chan cua chinh net nay cho phep khoang ho 0.5 mm: chan TO-220 buoc 2.54 mm
            # khong the giu duoc 0.8 mm cua lop Battery
            own = np.zeros((H, W), dtype=bool)
            for p in plist:
                sl, inside = grid.pad_mask(p)
                own[sl] |= inside
            blk_n &= ~(dilate(own, 4.0) & ~dilate(other, narrow / 2 + 0.5 + MARGIN) & ~grid.edge_block(narrow / 2 + EDGE_CLEARANCE + MARGIN))
        near = dilate((self.padowner != 0) & (self.padowner != code), NEAR_PAD)
        wideg = ~blk_w
        costg = np.where(blk_n, 0, np.where(blk_w, NARROW_COST, 1) + np.where(near, NEAR_COST, 0)).astype(np.uint8)
        if relaxed:
            costg[soft] = SOFT_COST
            wideg &= ~soft
        comp = np.zeros((H, W), dtype=np.int32)
        padidx = np.full((H, W), -1, dtype=np.int32)
        for k, p in enumerate(plist):
            # loi cua pad (lui vao nua be rong hep) luon di duoc, de day thoat ra khoi chan
            sl, core = grid.pad_mask(p, -narrow / 2 + G)
            ci, cj = grid.cell(p["x"], p["y"])
            comp[sl][core] = k + 1
            padidx[sl][core] = k
            comp[ci, cj] = k + 1
            padidx[ci, cj] = k
            sub = costg[sl]
            sub[core & (sub == 0)] = NARROW_COST
            if costg[ci, cj] == 0:
                costg[ci, cj] = NARROW_COST
        cost = costg.ravel().tolist()
        widef = wideg.ravel().tolist()
        compf = comp.ravel()
        padf = padidx.ravel()
        alive = set(range(1, len(plist) + 1))
        members = {k + 1: [k] for k in range(len(plist))}
        isolated, length, segments, stamped, crossed = [], 0.0, [], [], set()
        while len(alive) > 1:
            # nguon: thanh phan lon nhat con song (cay dang lon dan)
            A = max(alive, key=lambda a: (len(members[a]), -a))
            comp_list = compf.tolist()
            sources = np.nonzero(compf == A)[0].tolist()
            path = None
            if reachable(cost, W, H, comp_list, A, sources):
                # dich goi y: pad gan nhat thuoc thanh phan khac
                best = None
                for B in alive - {A}:
                    for kb in members[B]:
                        for ka in members[A]:
                            d = math.hypot(plist[ka]["x"] - plist[kb]["x"], plist[ka]["y"] - plist[kb]["y"])
                            if best is None or d < best[0]:
                                best = (d, kb)
                ti, tj = grid.cell(plist[best[1]]["x"], plist[best[1]]["y"])
                path = astar(cost, W, H, comp_list, A, sources, tj, ti)
            if path is None:
                # A bi co lap: de lai lam "dao", lay thanh phan khac lam nguon
                isolated.append(A)
                alive.discard(A)
                continue
            B = int(compf[path[-1]])
            if relaxed:
                pm = np.zeros((H, W), dtype=bool)
                pm.ravel()[[c for c in path if cost[c] >= SOFT_COST]] = True
                if pm.any():
                    hit = dilate(pm, narrow / 2 + 0.8 + 2 * MARGIN) & soft_src
                    crossed |= set(int(c) for c in np.unique(owner[hit]))
            else:
                # doan day: gop cac buoc thang hang, cung be rong
                pts = [grid.xy(c) for c in path]
                wid = [wide if widef[a] and widef[b] else narrow for a, b in zip(path, path[1:])]
                segs = []
                for k in range(len(path) - 1):
                    d = path[k + 1] - path[k]
                    if segs and segs[-1][4] == wid[k] and segs[-1][5] == d:
                        segs[-1][2:4] = pts[k + 1]
                    else:
                        segs.append([pts[k][0], pts[k][1], pts[k + 1][0], pts[k + 1][1], wid[k], d])
                for c in (path[0], path[-1]):
                    k = int(padf[c])
                    if k >= 0:
                        px, py = plist[k]["x"], plist[k]["y"]
                        cx, cy = grid.xy(c)
                        if abs(px - cx) > 1e-4 or abs(py - cy) > 1e-4:
                            segs.append([px, py, cx, cy, narrow, None])
                for s in segs:
                    segments.append((code, s[0], s[1], s[2], s[3], s[4]))
                    length += math.hypot(s[2] - s[0], s[3] - s[1])
                # ghi dong moi vao luoi
                pm_w = np.zeros((H, W), dtype=bool)
                pm_n = np.zeros((H, W), dtype=bool)
                for k, c in enumerate(path):
                    is_wide = (k < len(wid) and wid[k] == wide) and (k == 0 or wid[k - 1] == wide)
                    (pm_w if is_wide else pm_n).ravel()[c] = True
                stamp = (dilate(pm_w, wide / 2) | dilate(pm_n, narrow / 2)) & (owner == 0)
                owner[stamp] = code
                stamped.append(np.nonzero(stamp.ravel())[0])
            for c in path:
                if compf[c] == 0:
                    compf[c] = A
            compf[compf == B] = A
            members[A] += members.pop(B)
            alive.discard(B)
        if relaxed:
            return crossed
        groups = [members[a] for a in list(alive) + isolated]
        groups = [sorted("%s.%s" % (plist[k]["ref"], plist[k]["num"]) for k in g) for g in groups]
        self.res[name] = dict(cls=cls, pads=len(plist), jumpers=len(groups) - 1, length=round(length, 1),
                              groups=groups if len(groups) > 1 else [], segments=segments,
                              idx=np.concatenate(stamped) if stamped else np.zeros(0, dtype=np.int64))
        self.log("%-14s %-11s pads %2d  jumpers %d  %.0f mm  %.1fs" % (name, cls, len(plist), len(groups) - 1, length, time.time() - t0))
        return self.res[name]["jumpers"]

    def initial_order(self, order_name, seed=0):
        rnd = random.Random(seed)
        todo = []
        for name in self.nets:
            cls = net_class(name)[0]
            if cls == "Ground":
                continue
            rank = 0 if cls == "Battery" else (1 if order_name == "short" else ORDERS[order_name].index(cls))
            todo.append((rank, self.hpwl(name) * (rnd.uniform(0.5, 1.5) if seed else 1), name))
        return [t[2] for t in sorted(todo)]

    def ripup(self, deadline):
        """Go day di lai: giam dan so ket noi thieu cho toi khi het cai thien hoac het gio."""
        tried, improved = set(), True
        while improved and time.time() < deadline:
            improved = False
            failing = sorted((n for n, r in self.res.items() if r["jumpers"]), key=lambda n: (self.res[n]["pads"], self.hpwl(n)))
            for name in failing:
                if time.time() > deadline:
                    break
                if name not in self.res or not self.res[name]["jumpers"]:
                    continue
                before = self.total()
                snap_owner, snap_res = self.owner.copy(), dict(self.res)
                self.unroute(name)
                crossed = self.route_net(name, relaxed=True)
                blockers = sorted((self.names[c] for c in crossed if c > 0 and self.names.get(c) in self.res), key=self.hpwl)
                key = (name, tuple(blockers), before)
                if not blockers or len(blockers) > MAX_BLOCKERS or key in tried:
                    self.owner, self.res = snap_owner, snap_res
                    continue
                tried.add(key)
                for b in blockers:
                    self.unroute(b)
                self.route_net(name)
                for b in blockers:
                    self.route_net(b)
                after = self.total()
                if after < before:
                    improved = True
                    print("  rip-up %-14s (go %s): %d -> %d" % (name, ", ".join(blockers), before, after), flush=True)
                else:
                    self.owner, self.res = snap_owner, snap_res


def main():
    path = sys.argv[1]

    def arg(flag, default=None):
        return sys.argv[sys.argv.index(flag) + 1] if flag in sys.argv else default

    order_name, seed, minutes = arg("--order", "short"), int(arg("--seed", 0)), float(arg("--minutes", 6))
    board = pcbnew.LoadBoard(path)
    for t in list(board.GetTracks()):
        board.Remove(t)
    for z in list(board.Zones()):
        board.Remove(z)
    r = Router(board, print if "--verbose" in sys.argv else (lambda *a: None))
    t0 = time.time()
    for name in r.initial_order(order_name, seed):
        r.route_net(name)
    print("pass 1 (%s, seed %d): %d jumpers, %.0fs" % (order_name, seed, r.total(), time.time() - t0), flush=True)
    r.ripup(t0 + minutes * 60)
    print("after rip-up: %d jumpers, %.0fs" % (r.total(), time.time() - t0), flush=True)
    sig_jumpers = r.total()
    if "GND" in r.nets:
        r.route_net("GND")
    for res in r.res.values():
        for code, xa, ya, xb, yb, w in res["segments"]:
            t = pcbnew.PCB_TRACK(board)
            t.SetStart(VECTOR2I(FromMM(xa), FromMM(ya)))
            t.SetEnd(VECTOR2I(FromMM(xb), FromMM(yb)))
            t.SetWidth(FromMM(w))
            t.SetLayer(pcbnew.B_Cu)
            t.SetNetCode(code)
            board.Add(t)
    conns = sum(v["pads"] - 1 for v in r.res.values())
    print("RESULT order=%s seed=%d: %d/%d connections routed; jumpers: %d (khong ke GND) + %d (GND, truoc khi phu dong)"
          % (order_name, seed, conns - r.total(), conns, sig_jumpers, r.total() - sig_jumpers), flush=True)
    if arg("--report"):
        rep = {n: {k: v for k, v in res.items() if k not in ("segments", "idx")} for n, res in r.res.items()}
        with open(arg("--report"), "w", encoding="utf-8") as f:
            json.dump(rep, f, indent=1)
    if arg("--dump"):
        np.save(arg("--dump"), r.owner)
    # KiCad dang mo board thi khong ghi thang len file duoc: luu ra file tam roi chep de
    tmp = os.path.join(tempfile.gettempdir(), "route_board_tmp_%d.kicad_pcb" % os.getpid())
    if not pcbnew.SaveBoard(tmp, board):
        raise SystemExit("SaveBoard failed")
    shutil.copyfile(tmp, arg("--out", path))
    os.remove(tmp)


if __name__ == "__main__":
    main()
