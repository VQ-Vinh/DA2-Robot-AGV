"""Xuat luoi STL tu CAD/agv.FCStd cho mo hinh xe trong Gazebo / RViz (agv_description/meshes).

Chay bang FreeCAD dong lenh (khong can mo cua so):
  "C:/Users/<ten>/AppData/Local/Programs/FreeCAD 1.1/bin/freecadcmd.exe" CAD/export_meshes.py

Moi file la mot nhom chi tiet cung mau, don vi met, trong frame cua link URDF tuong ung:
  base_plates.stl  2 tam mica                               frame base_link (tam truc banh)
  base_parts.stl   tru oc, dong co + ga, Pi, PCB, pin, phan co dinh cua co cau nang   base_link
  lidar.stl        YDLidar X4 (de, dong co, chan) + 4 tru do   base_link
  lidar_head.stl   dau quay cua lidar (mau xanh)             base_link
  wheel.stl        lop + mam + khop noi (banh trai; banh phai quay 180 deg)   frame banh
  lift_plate.stl   mat nang + 4 dai oc + tru                frame lift_plate (tam mat nang)
DAU XE la phia Pi/camera = -X cua file CAD: luoi than xe va mat nang duoc quay 180 deg quanh Z
de +X cua URDF la huong tien (trong URDF: lidar_x = -Params.lidar_x).
In ra cac kich thuoc can chep vao agv.urdf.xacro.
"""
import math
import os

import FreeCAD as App
import Mesh
import MeshPart

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, '..', 'RasPi', 'src', 'agv_description', 'meshes')
doc = App.openDocument(os.path.join(HERE, 'agv.FCStd'))
print()
sh = doc.Params


def P(alias):
    return float(sh.get(alias))


def tess(shape, dev=0.4):
    return MeshPart.meshFromShape(Shape=shape, LinearDeflection=dev, AngularDeflection=0.6, Relative=False)


def glob(name, dev=0.4, min_vol=0.0):
    o = doc.getObject(name)
    s = o.Shape.copy()
    s.Placement = o.getGlobalPlacement()
    if min_vol and s.Solids:
        import Part
        s = Part.makeCompound([x for x in s.Solids if x.Volume > min_vol])
    return tess(s, dev)


def write(mesh, fname, shift_z, reduce=0.0, flip=True):
    if reduce:
        mesh.decimate(0.05, reduce)      # giam so tam giac (sai lech toi da 0.05 mm) cho Gazebo ve nhe
    m = App.Matrix()
    if flip:                         # dau xe = phia Pi/camera = -X cua file CAD -> quay 180 deg quanh Z
        m.rotateZ(math.pi)
    m.move(App.Vector(0, 0, -shift_z))
    mesh.transform(m)
    k = App.Matrix()
    k.scale(0.001, 0.001, 0.001)
    mesh.transform(k)
    path = os.path.join(OUT, fname)
    mesh.write(path)
    bb = mesh.BoundBox
    print('%-16s %7d tam giac, %5.1f MB, hop bao x %.3f..%.3f y %.3f..%.3f z %.3f..%.3f' % (
        fname, mesh.CountFacets, os.path.getsize(path) / 1e6, bb.XMin, bb.XMax, bb.YMin, bb.YMax, bb.ZMin, bb.ZMax))


axle = P('axle_z')

plates = Mesh.Mesh()
for n in ('Plate1', 'Plate2'):
    plates.addMesh(glob(n))
write(plates, 'base_plates.stl', axle)

parts = Mesh.Mesh()
names = ['Standoff%d' % i for i in range(1, 5)] + ['StandoffTop%d' % i for i in range(1, 5)] + ['PiPost%d' % i for i in range(1, 5)] + \
        ['PCBBoard'] + ['PCBPost%d' % i for i in range(1, 5)] + ['BatCells', 'BatSpacers', 'BatBMS', 'BatHolder', 'PiCam3', 'CamBracket'] + \
        ['LiftScrew%d' % i for i in range(1, 5)] + ['LiftBearing%d' % i for i in range(1, 5)] + \
        ['LiftPulley%d' % i for i in range(1, 5)] + ['LiftIdler1', 'LiftIdler2', 'LiftIdlerStud1', 'LiftIdlerStud2',
                                                      'LiftBelt', 'LiftMotorPulley', 'LiftMotor'] + \
        ['LiftMotorPost%d' % i for i in range(1, 5)]
for n in names:
    parts.addMesh(glob(n))
parts.addMesh(glob('Pi4B', dev=0.6, min_vol=150.0))        # bo linh kien dan nho cua Pi cho nhe luoi
for w in ('WheelFL', 'WheelFR', 'WheelRL', 'WheelRR'):     # dong co + ga di theo than xe
    pl = doc.getObject(w).Placement
    for n in ('MotorGA25', 'BracketV', 'BracketH'):
        o = doc.getObject(n)
        s = o.Shape.copy()
        s.Placement = pl.multiply(o.Placement)
        parts.addMesh(tess(s))
write(parts, 'base_parts.stl', axle, 0.6)

def placed(name):
    """Luoi STL trong CAD (Mesh::Feature): dua phep dat (vi tri tren xe) vao toa do dinh."""
    m = doc.getObject(name).Mesh.copy()
    pl = m.Placement
    m.Placement = App.Placement()
    m.transform(pl.toMatrix())
    return m


# Lidar trong CAD tach thanh 4 luoi theo mau: de, dau quay (xanh), dong co, 4 chan
lx = P('lidar_x')
head = placed('LidarHead')
body = Mesh.Mesh()
for n in ('LidarX4', 'LidarMotor', 'LidarLegs'):
    body.addMesh(placed(n))
head_top = head.BoundBox.ZMax
# phan khong quay (de, tai bat, duoi dong co) nam ngoai ban kinh dau quay: mat quet phai cao hon no
out_top = max(p.z for p in body.Points if ((p.x - lx) ** 2 + p.y ** 2) ** 0.5 > P('lidar_head_dia') / 2 + 1.0)
for i in range(1, 5):
    body.addMesh(glob('LidarPost%d' % i))
write(body, 'lidar.stl', axle)
write(head, 'lidar_head.stl', axle)

wheel = Mesh.Mesh()
for n in ('Tire', 'Rim', 'HexCoupler'):
    wheel.addMesh(tess(doc.getObject(n).Shape, 0.6))
write(wheel, 'wheel.stl', 0.0, 0.85, flip=False)

lift_z = P('plate2_top_z') + P('lift_gap') + P('lift_plate_thick') / 2
lift = Mesh.Mesh()
for n in ['LiftPlateCut'] + ['LiftNut%d' % i for i in range(1, 5)] + ['LiftNutPost%d%s' % (i, k) for i in range(1, 5) for k in 'ab']:
    lift.addMesh(glob(n))
write(lift, 'lift_plate.stl', lift_z)

print('--- kich thuoc cho URDF (m) ---')
for a in ('plate_size', 'plate_thickness', 'plate_gap', 'plate1_below_axle', 'wheel_dia', 'wheel_width', 'wheelbase',
          'track_width', 'axle_z', 'plate1_top_z', 'plate2_top_z', 'standoff_inset', 'standoff_inset_y', 'lidar_x',
          'lidar_scan_z', 'lidar_base_z', 'lidar_height', 'lift_plate_size', 'lift_plate_thick', 'lift_gap',
          'lift_stroke', 'lift_x', 'lift_y', 'lift_screw_dia', 'lift_pos'):
    print('%-20s %.4f' % (a, P(a) / 1000.0))
print('lidar: de (phan khong quay) cao toi z = %.4f m, dinh dau quay z = %.4f m -> mat quet o giua: z = %.4f m (Params.lidar_scan_z dang la %.4f m)'
      % (out_top / 1000.0, head_top / 1000.0, (out_top + head_top) / 2000.0, P('lidar_scan_z') / 1000.0))
