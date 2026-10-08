#!/usr/bin/env python3
"""Thu chinh sach pin cua order_manager: pin dau 40 %, hao / sac nhanh x20, 3 don. In dong thoi gian + KET QUA."""
import json
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

rclpy.init()
n = rclpy.create_node('battery_test')
st = {'state': None, 'log': []}
n.create_subscription(String, '/order/state', lambda m: st.__setitem__('state', json.loads(m.data)),
                      QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
n.create_subscription(String, '/order/status', lambda m: st['log'].append(m.data),
                      QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
cmd = n.create_publisher(String, '/order/command', 10)


def spin(sec):
    t0 = time.time()
    while time.time() - t0 < sec:
        rclpy.spin_once(n, timeout_sec=0.05)


t0 = time.time()
while not any('nhan don' in s for s in st['log']) and time.time() - t0 < 180:
    spin(1.0)
while cmd.get_subscription_count() == 0:
    spin(0.5)
spin(20.0)       # de xe vao tram sac luc dau
for shelf in ('ke_03', 'ke_06', 'ke_02'):
    cmd.publish(String(data=f'add {shelf} tram_lay_hang 0'))
    spin(1.0)

T0 = time.time()
last, confirmed, timeline = None, set(), []
charged_between, hold_seen = False, False
while time.time() - T0 < 1500:
    spin(0.5)
    s = st['state']
    if not s or not s.get('battery'):
        continue
    b = s['battery']
    key = (s.get('current'), s.get('step'), s.get('charge_hold'), b['charging'])
    if key != last:
        last = key
        line = (f"{time.time() - T0:6.0f} s  pin {b['pct']:5.1f} %{' sac' if b['charging'] else '    '}  "
                f"don {s.get('current')}  buoc {s.get('step_label')}  giu_sac {s.get('charge_hold')}")
        timeline.append(line)
        print(line, flush=True)
    if s.get('charge_hold'):
        hold_seen = True
        if s.get('current'):
            print('SAI: dang giu sac ma van chay don moi', flush=True)
    if hold_seen and b['charging']:
        charged_between = True
    if s.get('step') == 'wait_confirm' and s.get('current') not in confirmed:
        spin(3.0)
        cmd.publish(String(data='confirm'))
        confirmed.add(s.get('current'))
    rows = [o for o in s['orders'] if o['created'] >= T0 - 30]
    if len(rows) >= 3 and all(o['status'] in ('done', 'failed', 'cancelled') for o in rows) \
            and s.get('step') in ('charging', 'docked'):
        break
s = st['state']
rows = [o for o in s['orders'] if o['created'] >= T0 - 30]
print(f"KET QUA pin: {sum(o['status'] == 'done' for o in rows)}/{len(rows)} don xong, "
      f"co giu sac {hold_seen}, sac giua cac don {charged_between}, ket thuc o '{s.get('step_label')}' "
      f"pin {s['battery']['pct']} %, tong {time.time() - T0:.0f} s", flush=True)
for o in rows:
    print(f"  #{o['id']} {o['shelf']}: {o['status']}" + (f", loi {o['error']}" if o['error'] else ''), flush=True)
