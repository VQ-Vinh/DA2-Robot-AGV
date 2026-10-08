#!/usr/bin/env python3
"""Thu xu ly su co cua order_manager (giai doan 5). 3 don, moi don mot su co khi dang cho ke toi tram:
  A ke_03: tuong chan ngang kho 75 s -> mong doi: canh bao (15 s), su co duong_bi_chan (60 s), go tuong + ack, don xong
  B ke_06: DUNG KHAN, 5 s sau "ack"                     -> mong doi: xe dung that, lam tiep, don xong
  C ke_02: dich ke ra khoi xe (roi ke)                   -> mong doi: xe dung, don loi, ke chua ro vi tri
In KET QUA."""
import json
import math
import subprocess
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import String

rclpy.init()
n = rclpy.create_node('fault_test')
st = {'state': None, 'log': [], 'gt': None}
n.create_subscription(String, '/order/state', lambda m: st.__setitem__('state', json.loads(m.data)),
                      QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
n.create_subscription(String, '/order/status', lambda m: st['log'].append((time.time(), m.data)),
                      QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
n.create_subscription(Odometry, '/ground_truth',
                      lambda m: st.__setitem__('gt', (m.pose.pose.position.x, m.pose.pose.position.y)),
                      qos_profile_sensor_data)
cmd = n.create_publisher(String, '/order/command', 10)
results = []


def spin(sec):
    t0 = time.time()
    while time.time() - t0 < sec:
        rclpy.spin_once(n, timeout_sec=0.05)


def send(text):
    cmd.publish(String(data=text))
    spin(0.5)


def gz(args):
    return subprocess.run(['gz', 'service'] + args + ['--timeout', '3000'], capture_output=True, text=True).stdout


WALL = '/tmp/fault_wall.sdf'
open(WALL, 'w').write('<?xml version="1.0"?><sdf version="1.9"><model name="block_wall"><static>true</static>'
                      '<link name="l"><collision name="c"><geometry><box><size>0.3 6.4 0.6</size></box></geometry>'
                      '</collision><visual name="v"><geometry><box><size>0.3 6.4 0.6</size></box></geometry>'
                      '</visual></link></model></sdf>')


def wait_step(order_shelf, step, timeout=300):
    t0 = time.time()
    while time.time() - t0 < timeout:
        spin(0.2)
        s = st['state']
        if s and s.get('step') == step:
            cur = next((o for o in s['orders'] if o['id'] == s.get('current')), None)
            if cur and cur['shelf'] == order_shelf:
                return True
    return False


def auto_confirm():
    s = st['state']
    if s and s.get('step') == 'wait_confirm':
        send('confirm')


def wait_order_end(shelf, timeout=600):
    t0 = time.time()
    while time.time() - t0 < timeout:
        spin(0.5)
        auto_confirm()
        s = st['state']
        mine = [o for o in s['orders'] if o['shelf'] == shelf] if s else []
        if mine and mine[-1]['status'] in ('done', 'failed', 'cancelled') and s.get('current') is None:
            return mine[-1]
    return None


t0 = time.time()
while not any('nhan don' in m for _, m in st['log']) and time.time() - t0 < 180:
    spin(1.0)
while cmd.get_subscription_count() == 0:
    spin(0.5)

# ---- A: duong bi chan ----
send('add ke_03 tram_lay_hang 0')
if wait_step('ke_03', 'to_station'):
    spin(3.0)
    print(gz(['-s', '/world/warehouse/create', '--reqtype', 'gz.msgs.EntityFactory', '--reptype', 'gz.msgs.Boolean',
              '--req', f'sdf_filename: "{WALL}", pose: {{position: {{x: -2.3, y: 0.0, z: 0.3}}}}']).strip(), flush=True)
    tw = time.time()
    alarm_seen, fault_seen = False, None
    while time.time() - tw < 75:
        spin(0.5)
        ss = st['state'] or {}
        alarm_seen |= bool(ss.get('alarm'))
        fault_seen = fault_seen or ss.get('fault_kind')
    gz(['-s', '/world/warehouse/remove', '--reqtype', 'gz.msgs.Entity', '--reptype', 'gz.msgs.Boolean',
        '--req', 'name: "block_wall", type: MODEL'])
    print(f'A: go tuong sau 75 s, canh bao {alarm_seen}, su co {fault_seen}', flush=True)
    # Su co "duong bi chan" co the toi sau (Nav2 tu hoi phuc + thu lai + cho 60 s): cho toi khi co su co
    # (bam ack) hoac don xong
    o, t2 = None, time.time()
    while time.time() - t2 < 300:
        spin(0.5)
        auto_confirm()
        ss = st['state']
        fault_seen = fault_seen or ss.get('fault_kind')
        if ss.get('fault_kind'):
            spin(1.0)
            send('ack')
        mine = [x for x in ss['orders'] if x['shelf'] == 'ke_03']
        if mine and mine[-1]['status'] in ('done', 'failed') and ss.get('current') is None:
            o = mine[-1]
            break
    results.append(('A duong bi chan', alarm_seen and fault_seen == 'duong_bi_chan' and o and o['status'] == 'done',
                    f'canh bao {alarm_seen}, su co {fault_seen}, don {o and o["status"]}'))
else:
    results.append(('A duong bi chan', False, 'khong toi buoc cho ke'))

# ---- B: dung khan ----
send('add ke_06 tram_lay_hang 0')
if wait_step('ke_06', 'to_station'):
    spin(4.0)
    send('estop')
    spin(1.5)
    p0 = st['gt']
    spin(3.0)
    p1 = st['gt']
    moved = math.hypot(p1[0] - p0[0], p1[1] - p0[1]) if p0 and p1 else float('nan')
    fault = st['state'].get('fault_kind')
    print(f'B: sau DUNG KHAN 1.5 s, xe di them {100 * moved:.1f} cm trong 3 s, su co {fault}', flush=True)
    spin(1.0)
    send('ack')
    o = wait_order_end('ke_06')
    results.append(('B dung khan', moved < 0.03 and fault == 'dung_khan' and o and o['status'] == 'done',
                    f'xe di them {100 * moved:.1f} cm khi dung, don {o and o["status"]}'))
else:
    results.append(('B dung khan', False, 'khong toi buoc cho ke'))

# ---- C: roi ke ----
send('add ke_02 tram_lay_hang 0')
if wait_step('ke_02', 'to_station'):
    spin(4.0)
    gz(['-s', '/world/warehouse/set_pose', '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
        '--req', 'name: "ke_02", position: {x: -5.5, y: 2.0, z: 0.0}'])
    t1 = time.time()
    fault = None
    while time.time() - t1 < 10 and not fault:
        spin(0.2)
        fault = st['state'].get('fault_kind')
    dt = time.time() - t1
    print(f'C: su co sau {dt:.1f} s: {fault}', flush=True)
    spin(2.0)
    send('ack')
    o = wait_order_end('ke_02', timeout=60)
    slot = st['state']['shelves'].get('ke_02')
    send('shelf ke_02 N5')
    spin(2.0)
    slot2 = st['state']['shelves'].get('ke_02')
    results.append(('C roi ke', fault == 'mat_ke' and o and o['status'] == 'failed' and slot is None and slot2 == 'N5',
                    f'su co {fault} sau {dt:.1f} s, don {o and o["status"]}, ke {slot} -> sau lenh shelf: {slot2}'))
else:
    results.append(('C roi ke', False, 'khong toi buoc cho ke'))

spin(5.0)
print(f'KET QUA su co: {sum(1 for _, ok, _ in results if ok)}/{len(results)} dat', flush=True)
for name, ok, info in results:
    print(f'  {name:18s} {"DAT " if ok else "SAI "} {info}', flush=True)
print('  buoc cuoi:', st['state'].get('step_label'), '| su co:', st['state'].get('error'), flush=True)
