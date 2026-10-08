#!/usr/bin/env python3
"""Do chuyen dong that (ground truth) cua xe theo tung buoc don hang. Chay lan luot cac don (tu xac nhan),
ghi quy dao 20 Hz vao ~/agv_tests/motion_<tag>.csv, in:
  - moi buoc: thoi gian, quang duong, tong goc quay, goc quay "thua" (tong - goc can quay), so lan dao chieu quay
  - sau buoc tra ke (return_dock): lech huong xe so voi truc o, lech ngang / doc
  - sau don: ke lech tam o, ke xoay
  - ke lech so voi xe luc cho nguoi lay hang (xe dung yen; do bang gz model, mat ~1 s nen khong do luc chay)
  - "AMCL sai max": chi tham khao, /amcl_pose chi phat khi AMCL cap nhat nen co the cu
Dung de tim cho xe "vong vong" (REPORT.md 4.14). Chay cung sim.launch.py nav:=true (Gazebo + order_manager).
Doi so: tag, danh sach ke cach nhau dau phay (mac dinh 8 ke)."""
import json
import math
import os
import re
import subprocess
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import String

TAG = sys.argv[1] if len(sys.argv) > 1 else 'm'
SHELVES = sys.argv[2].split(',') if len(sys.argv) > 2 else \
    ['ke_05', 'ke_01', 'ke_06', 'ke_03', 'ke_08', 'ke_02', 'ke_07', 'ke_04']

rclpy.init()
n = rclpy.create_node('motion_test')
st = {'state': None, 'log': [], 'gt': None, 'amcl': None}


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def on_gt(m):
    p = m.pose.pose
    st['gt'] = (p.position.x + 4.5, p.position.y, yaw_of(p.orientation))


def on_amcl(m):
    p = m.pose.pose
    st['amcl'] = (p.position.x, p.position.y, yaw_of(p.orientation))


n.create_subscription(String, '/order/state', lambda m: st.__setitem__('state', json.loads(m.data)),
                      QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
n.create_subscription(String, '/order/status', lambda m: st['log'].append((time.time(), m.data)),
                      QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
n.create_subscription(Odometry, '/ground_truth', on_gt, qos_profile_sensor_data)
n.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', on_amcl,
                      QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
cmd = n.create_publisher(String, '/order/command', 10)
os.makedirs(os.path.expanduser('~/agv_tests'), exist_ok=True)
csv = open(os.path.expanduser(f'~/agv_tests/motion_{TAG}.csv'), 'w')
csv.write('t,order,step,x,y,yaw,ax,ay,ayaw\n')


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def gz_pose(name):
    out = subprocess.run(['gz', 'model', '-m', name, '-p'], capture_output=True, text=True).stdout
    num = r'-?\d+(?:\.\d+)?(?:e-?\d+)?'
    g = re.findall(rf'\[\s*({num})\s+({num})\s+({num})\s*\]', out)
    if len(g) < 2:
        return None
    return float(g[0][0]) + 4.5, float(g[0][1]), float(g[1][2])


class StepStat:
    def __init__(self, oid, step):
        self.oid, self.step = oid, step
        self.t0 = time.time()
        self.pts = []
        self.amcl_err = 0.0

    def add(self, gt, amcl):
        self.pts.append(gt)
        if amcl:
            self.amcl_err = max(self.amcl_err, math.hypot(gt[0] - amcl[0], gt[1] - amcl[1]))

    def summary(self):
        p = self.pts
        if len(p) < 2:
            return None
        dist = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(p, p[1:]))
        dy = [wrap(b[2] - a[2]) for a, b in zip(p, p[1:])]
        rot = sum(abs(d) for d in dy)
        net = abs(wrap(p[-1][2] - p[0][2]))
        # dao chieu quay: gom cac doan cung dau, chi tinh doan >= 10 deg
        segs, cur = [], 0.0
        for d in dy:
            if abs(d) < 1e-3:
                continue
            if cur == 0 or (d > 0) == (cur > 0):
                cur += d
            else:
                segs.append(cur)
                cur = d
        if cur:
            segs.append(cur)
        big = [s for s in segs if abs(s) >= math.radians(10)]
        rev = sum(1 for a, b in zip(big, big[1:]) if (a > 0) != (b > 0))
        return {'order': self.oid, 'step': self.step, 'dur': time.time() - self.t0, 'dist': dist,
                'rot': math.degrees(rot), 'extra': math.degrees(rot - net), 'rev': rev,
                'amcl': self.amcl_err}


def spin(sec):
    t0 = time.time()
    while time.time() - t0 < sec:
        rclpy.spin_once(n, timeout_sec=0.02)
        tick()


cur_stat = None
slip = {}          # order -> [max lech tam ke so voi xe (cm), max xoay ke so voi xe (deg)]


def slip_loop():
    """1 Hz: dang cho ke thi do ke truot tren xe (vi tri that ke so voi xe)."""
    while True:
        s, gt = st['state'], st['gt']
        if s and gt and s.get('carrying') and s.get('current') and s.get('step') == 'wait_confirm':
            shelf = next((o['shelf'] for o in s['orders'] if o['id'] == s['current']), None)
            gp = gz_pose(shelf) if shelf else None
            gt = st['gt']
            if gp:
                d = 100 * math.hypot(gp[0] - gt[0], gp[1] - gt[1])
                r = abs((math.degrees(gp[2] - gt[2]) + 45) % 90 - 45)
                a = slip.setdefault(s['current'], [0.0, 0.0, s.get('step'), s.get('step')])
                if d > a[0]:
                    a[0], a[2] = d, s.get('step')
                if r > a[1]:
                    a[1], a[3] = r, s.get('step')
        time.sleep(1.0)


threading.Thread(target=slip_loop, daemon=True).start()
stats, docks, shelves_out = [], [], []
last_rec = 0.0


def tick():
    global cur_stat, last_rec
    s, gt = st['state'], st['gt']
    if not s or not gt or time.time() - last_rec < 0.05:
        return
    last_rec = time.time()
    oid, step = s.get('current'), s.get('step') if s.get('current') else 'idle'
    if cur_stat is None or (cur_stat.oid, cur_stat.step) != (oid, step):
        if cur_stat is not None:
            sm = cur_stat.summary()
            if sm:
                stats.append(sm)
            if cur_stat.step == 'return_dock' and step == 'lower':
                sl = s['slots'][s['shelves'][next(o['shelf'] for o in s['orders'] if o['id'] == cur_stat.oid)]]
                c, sn = math.cos(sl['yaw']), math.sin(sl['yaw'])
                ex, ey = gt[0] - sl['x'], gt[1] - sl['y']
                docks.append({'order': cur_stat.oid, 'yaw_err': math.degrees(wrap(gt[2] - sl['yaw'])),
                              'lon': ex * c + ey * sn, 'lat': -ex * sn + ey * c})
        cur_stat = StepStat(oid, step)
    cur_stat.add(gt, st['amcl'])
    a = st['amcl'] or (float('nan'),) * 3
    csv.write(f'{time.time():.2f},{oid},{step},{gt[0]:.3f},{gt[1]:.3f},{gt[2]:.3f},{a[0]:.3f},{a[1]:.3f},{a[2]:.3f}\n')


t0 = time.time()
while not any('nhan don' in m for _, m in st['log']) and time.time() - t0 < 240:
    spin(1.0)
while cmd.get_subscription_count() == 0:
    spin(0.5)
print('san sang', flush=True)

results = []
for shelf in SHELVES:
    cmd.publish(String(data=f'add {shelf} tram_lay_hang 0'))
    spin(2.0)
    t1 = time.time()
    o = None
    while time.time() - t1 < 600:
        spin(0.5)
        s = st['state']
        if s.get('step') == 'wait_confirm':
            spin(2.0)
            cmd.publish(String(data='confirm'))
            spin(1.0)
        if s.get('error'):
            print(f'{shelf}: SU CO {s["error"]}', flush=True)
            break
        mine = [x for x in s['orders'] if x['shelf'] == shelf]
        if mine and mine[-1]['status'] in ('done', 'failed', 'cancelled') and s.get('current') is None:
            o = mine[-1]
            break
    s = st['state']
    slot = s['shelves'].get(shelf)
    gp = gz_pose(shelf)
    if gp and slot:
        sl = s['slots'][slot]
        rot = (math.degrees(gp[2]) + 45) % 90 - 45
        shelves_out.append((shelf, 100 * math.hypot(gp[0] - sl['x'], gp[1] - sl['y']), rot))
    results.append((shelf, o and o['status'], time.time() - t1))
    print(f'{shelf}: {o and o["status"]} sau {time.time() - t1:.0f} s', flush=True)
    if s.get('error'):
        break
    spin(3.0)

csv.close()
print('\n== Tung buoc (dur s, quang duong m, tong quay deg, quay thua deg, dao chieu, AMCL sai max m)', flush=True)
for x in stats:
    if x['step'] in ('idle',):
        continue
    flag = '  <==' if x['extra'] > 120 or x['rev'] >= 3 else ''
    print(f"  #{x['order']} {x['step']:15s} {x['dur']:6.1f} {x['dist']:5.2f} {x['rot']:6.0f} {x['extra']:6.0f} "
          f"{x['rev']:3d} {x['amcl']:.2f}{flag}", flush=True)
agg = {}
for x in stats:
    a = agg.setdefault(x['step'], [0, 0, 0, 0, 0])
    a[0] += 1; a[1] += x['dur']; a[2] += x['extra']; a[3] += x['rev']; a[4] = max(a[4], x['extra'])
print('\n== Trung binh theo buoc (n, dur, quay thua TB, quay thua max, dao chieu TB)', flush=True)
for k, a in agg.items():
    print(f'  {k:15s} {a[0]:2d} {a[1] / a[0]:6.1f} {a[2] / a[0]:6.0f} {a[4]:6.0f} {a[3] / a[0]:5.1f}', flush=True)
print('\n== Xe khi tra ke xong (lech huong deg, doc cm, ngang cm)', flush=True)
for d in docks:
    print(f"  #{d['order']} {d['yaw_err']:+6.1f} {100 * d['lon']:+6.1f} {100 * d['lat']:+6.1f}", flush=True)
print('\n== Ke sau don (lech tam cm, xoay deg)', flush=True)
for sh, dc, r in shelves_out:
    print(f'  {sh} {dc:5.1f} {r:+6.1f}', flush=True)
print('\n== Ke truot tren xe khi cho (max lech tam cm @buoc, max xoay deg @buoc)', flush=True)
for k, a in slip.items():
    print(f'  #{k} {a[0]:5.1f} @{a[2]}  {a[1]:5.1f} @{a[3]}', flush=True)
ok = sum(1 for _, r, _ in results if r == 'done')
print(f'KET QUA motion: {ok}/{len(SHELVES)} don xong; max lech huong tra ke '
      f'{max((abs(d["yaw_err"]) for d in docks), default=float("nan")):.1f} deg', flush=True)
