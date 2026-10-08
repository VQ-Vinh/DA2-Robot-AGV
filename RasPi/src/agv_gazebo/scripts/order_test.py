#!/usr/bin/env python3
"""Thu order_manager dau-cuoi: 3 don (1 gap), 1 don huy, tu bam xac nhan o tram. In KET QUA."""
import json
import math
import re
import subprocess
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

rclpy.init()
n = rclpy.create_node('order_test')
latched1 = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
latched10 = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
st = {'state': None, 'log': []}
n.create_subscription(String, '/order/state', lambda m: st.__setitem__('state', json.loads(m.data)), latched1)
n.create_subscription(String, '/order/status', lambda m: st['log'].append((time.time(), m.data)), latched10)
cmd = n.create_publisher(String, '/order/command', 10)


def spin(sec):
    t0 = time.time()
    while time.time() - t0 < sec:
        rclpy.spin_once(n, timeout_sec=0.05)


def send(text):
    cmd.publish(String(data=text))
    spin(1.0)


t0 = time.time()
while not any('nhan don' in s for _, s in st['log']) and time.time() - t0 < 180:
    spin(1.0)
print('order_manager san sang' if any('nhan don' in s for _, s in st['log']) else 'KHONG san sang', flush=True)
while cmd.get_subscription_count() == 0:
    spin(0.5)
send('add ke_03 tram_lay_hang 0')
send('add ke_06 tram_lay_hang 0')
send('add ke_01 tram_lay_hang 1')      # gap: phai lam truoc ke_06
send('add ke_07 tram_lay_hang 0')
spin(1.0)
ids = sorted(o['id'] for o in st['state']['queue']) if st['state'] else []
q = st['state']['queue'] if st['state'] else []
k7 = [o['id'] for o in st['state']['orders'] if o['shelf'] == 'ke_07' and o['status'] == 'pending']
if k7:
    send(f'cancel {k7[0]}')
print('hang doi:', [(o['id'], o['shelf'], o['priority']) for o in q], flush=True)

order_seq, confirmed = [], set()
t0 = time.time()
while time.time() - t0 < 1500:
    spin(0.5)
    s = st['state']
    if not s:
        continue
    cur = s.get('current')
    if cur and (not order_seq or order_seq[-1] != cur):
        order_seq.append(cur)
    if s.get('step') == 'wait_confirm' and cur not in confirmed:
        spin(3.0)                         # nguoi lay hang mat 3 s
        send('confirm')
        confirmed.add(cur)
    if s.get('error'):
        print('LOI xe dung:', s['error'], flush=True)
        break
    mine = [o for o in s['orders'] if o['shelf'] in ('ke_03', 'ke_06', 'ke_01', 'ke_07') and o['created'] >= t0 - 120]
    if mine and all(o['status'] in ('done', 'failed', 'cancelled') for o in mine) and not cur:
        break

s = st['state']
rows = [o for o in s['orders'] if o['created'] >= t0 - 120]
id2shelf = {o['id']: o['shelf'] for o in rows}
print('thu tu chay:', [id2shelf.get(i, i) for i in order_seq], flush=True)
for o in rows:
    dt = (o['finished'] - o['started']) if o['started'] and o['finished'] else None
    print(f"  #{o['id']} {o['shelf']} uu_tien {o['priority']}: {o['status']}"
          + (f', {dt:.1f} s' if dt else '') + (f", loi: {o['error']}" if o['error'] else ''), flush=True)
# ke co ve dung o khong (vi tri that)
lay = s['slots']
for shelf in ('ke_03', 'ke_06', 'ke_01'):
    out = subprocess.run(['gz', 'model', '-m', shelf, '-p'], capture_output=True, text=True).stdout
    num = r'-?\d+(?:\.\d+)?(?:e-?\d+)?'
    g = re.findall(rf'\[\s*({num}\s+{num}\s+{num})\s*\]', out)
    if g:
        x, y, _ = [float(v) for v in g[0].split()]
        sl = lay[s['shelves'][shelf]]
        print(f'  {shelf} cach tam o {s["shelves"][shelf]}: {100 * math.hypot(x + 4.5 - sl["x"], y - sl["y"]):.1f} cm', flush=True)
done = sum(1 for o in rows if o['status'] == 'done')
print(f"KET QUA don hang: {done} xong, {sum(1 for o in rows if o['status'] == 'cancelled')} huy, "
      f"{sum(1 for o in rows if o['status'] == 'failed')} loi, tong {time.time() - t0:.0f} s; stats {s['stats']}", flush=True)
