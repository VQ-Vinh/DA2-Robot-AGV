"""To mau theo tung mat cho mo hinh STEP khong co mau, xuat VRML cho KiCad.

Mo hinh la mot khoi lien nen khong tach duoc theo linh kien; mau duoc doan theo vi tri
va kich thuoc cua tung mat (tam mach, chan cam, nhua hang rao, linh kien).
KiCad doc VRML theo don vi 0.1 inch nen toa do duoc chia cho 2.54.
"""
import FreeCAD, Part, os, sys
base = r"D:\MyBK\HK261\DA2\Src\PCB\lib\3d\models"
COL = {"pcb_green": (0.10, 0.45, 0.22), "pcb_blue": (0.10, 0.25, 0.70), "gold": (0.85, 0.70, 0.25),
       "silver": (0.75, 0.75, 0.78), "black": (0.08, 0.08, 0.08), "dark": (0.22, 0.22, 0.24), "white": (0.9, 0.9, 0.88)}

def classify_discovery(f):
    b = f.BoundBox   # mach nam trong mat XZ, day theo Y: 0..1.5; mat tren la +Y
    if b.YMin >= -0.05 and b.YMax <= 1.55:
        return "pcb_green"
    small = b.XLength < 1.1 and b.ZLength < 1.1
    if small:
        return "gold"                                  # chan hang rao
    row = lambda z: abs(z + 58.15) < 2.9 or abs(z + 7.65) < 2.9
    cz = b.Center.z
    if row(cz) and b.XMin > 0.5 and b.XMax < 65.5 and -2.8 <= b.YMin and b.YMax <= 4.2 and b.XLength < 64:
        return "black"                                 # nhua hang rao P1/P2
    if b.YMax <= 0.05:
        return "black"
    if b.XMin > 86 or b.XMax < 9:                      # cong USB va jack o hai dau
        return "black" if (b.XMax < 14 and abs(cz + 47) < 6) else "silver"
    if b.ZLength < 1.1 or b.XLength < 1.1:
        return "gold" if b.YLength > 3 else "dark"
    return "dark"

def classify_ina219(f):
    b = f.BoundBox   # mach nam trong mat XZ, day theo Y: 0..1.5
    if b.YMin >= -0.05 and b.YMax <= 1.55:
        return "pcb_blue"
    if b.YMax > 1.55:
        big = b.XLength > 5 and b.ZLength > 2.5
        return "black" if big or b.YMax > 2.0 else "silver"
    return "dark"

JOBS = [("stm32f4_discovery", classify_discovery), ("ina219", classify_ina219)]
for name, fn in JOBS:
    sh = Part.Shape(); sh.read(os.path.join(base, name + ".step"))
    sh.tessellate(0.08)          # tao luoi cho ca khoi truoc, neu khong cac mat lon nhieu lo tra ve rong
    groups = {}
    empty = []
    for f in sh.Faces:
        key = fn(f)
        pts, tris = f.tessellate(0.08)
        if not tris:
            empty.append((key, round(f.Area, 1)))
            continue
        g = groups.setdefault(key, ([], []))
        off = len(g[0])
        g[0].extend(pts)
        rev = (f.Orientation == "Reversed")
        for a, b, c in tris:
            g[1].append((off + a, off + b, off + c))   # tessellate() da tinh huong mat, khong dao lai
    out = ["#VRML V2.0 utf8", "# sinh tu %s.step bang step_to_wrl.py" % name]
    for key, (pts, tris) in groups.items():
        r, g_, b_ = COL[key]
        out.append("Shape { appearance Appearance { material Material { diffuseColor %.3f %.3f %.3f specularColor 0.2 0.2 0.2 shininess 0.3 } }" % (r, g_, b_))
        out.append(" geometry IndexedFaceSet { solid FALSE creaseAngle 0.5 coord Coordinate { point [")
        out.append(",".join("%.4f %.4f %.4f" % (p.x / 2.54, p.y / 2.54, p.z / 2.54) for p in pts))
        out.append(" ] } coordIndex [")
        out.append(",".join("%d,%d,%d,-1" % t for t in tris))
        out.append(" ] } }")
    open(os.path.join(base, name + ".wrl"), "w", encoding="utf-8", newline="\n").write("\n".join(out) + "\n")
    print(name, {k: len(v[1]) for k, v in groups.items()}, '| faces without mesh:', len(empty), sorted(empty, key=lambda e: -e[1])[:4])
