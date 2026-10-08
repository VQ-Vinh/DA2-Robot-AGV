#!/usr/bin/env python3
"""Quan ly don hang kho kieu Kiva: hang doi don, lay ke -> cho toi tram -> tra ke (dung chung xe that).

Mot don = "dua ke <ke> toi tram <tram>". Xe lam lan luot:
  toi staging truoc o -> chui gam (shelf_dock, theo chan ke) -> nang -> lui ra -> cho ke toi tram
  -> CHO NGUOI XAC NHAN da lay hang -> toi staging -> tra ke vao o (slot_dock) -> ha -> lui ra
  Truoc khi nang / ha: kiem tra xe thang truc o (checked_dock), sai thi lui ra vao lai.
Het don thi vao tram sac (DockRobot dock charger_<idle_station>) va sac.

Pin (/battery_state, battery_sim hoac INA226 tren xe that): truoc moi don, pin < battery_low +
order_reserve (phan du tru cho chinh don do) thi khong nhan don, vao sac toi battery_full roi moi lam
tiep (don dang chay van lam xong). Chi so voi battery_low thi xe nhan don luc pin 34 % roi can 0 %
giua chung (REPORT.md 4.12). Dang dung o
tram sac ma co don va pin du thi roi tram (UndockRobot) roi lam don.

Lenh: /order/command (std_msgs/String)
  add <ke> <tram> [uu_tien]   them don (uu_tien lon lam truoc, mac dinh 0)
  cancel <id>                 huy don dang cho (don dang chay: chi huy duoc truoc khi nang ke)
  confirm [id]                nguoi o tram da lay hang xong -> xe tra ke
  pause | resume              tam dung nhan don moi / tiep tuc (don dang chay van lam xong)
  estop                       DUNG KHAN: huy moi lenh chay, xe dung; "ack" thi lam tiep buoc dang do
  ack                         xac nhan da xu ly su co
  shelf <ke> <o>              nguoi dat lai ke vao o (sau khi ke roi / chuyen tay)
Trang thai: /order/state (std_msgs/String, JSON, giu tin cuoi): hang doi, don dang chay + buoc,
vi tri ke, lich su gan day. Nhat ky de doc: /order/status (String, giu tin cuoi).

Du lieu: SQLite (tham so db_path): bang orders (lich su, thong ke) va shelves (ke dang o o nao).
Mo phong: reset_shelves:=true nap lai vi tri ke tu shelves.yaml moi lan chay (world cung reset).

Loi va su co (giai doan 5, REPORT.md 4.13):
  - moi chang thu lai 1 lan sau khi xoa costmap
  - chang di chuyen van loi = duong bi chan: canh bao, cho blocked_retry_s roi thu lai, toi da
    blocked_wait_s; van chan -> xe dung cho nguoi don duong, "ack" thi di tiep
  - loi truoc khi nang ke -> don tra lai hang doi (toi da max_attempts lan), lam don khac
  - dung khan (estop): huy moi action, van toc 0; "ack" thi lam lai buoc dang do
  - roi ke khi dang cho (mat tin hieu co ke > 1 s) / co cau nang loi -> xe dung, don loi, ke "chua ro
    vi tri" (nguoi dat lai roi gui "shelf <ke> <o>"); "ack" thi xe ve sac
"""
import json
import math
import os
import sqlite3
import threading
import time

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from geometry_msgs.msg import Point
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import BackUp, DockRobot, NavigateThroughPoses, NavigateToPose, Spin, UndockRobot
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import BatteryState
from rclpy.time import Time
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener

STEP_NAME = {
    'staging': 'toi truoc o ke', 'dock': 'chui gam', 'lift': 'nang ke', 'undock': 'lui ra',
    'to_station': 'cho ke toi tram', 'wait_confirm': 'cho nguoi lay hang', 'return_staging': 'toi truoc o',
    'return_dock': 'tra ke vao o', 'lower': 'ha ke', 'return_undock': 'lui ra', 'home': 've tram sac',
    'to_charger': 'vao tram sac', 'charging': 'dang sac', 'docked': 'dung o tram sac', 'leave_charger': 'roi tram sac',
}


class OrderError(Exception):
    pass


class Fault(Exception):
    """Su co can nguoi xu ly. resumable: sau khi nguoi bam "Da xu ly" thi lam lai buoc dang do."""

    def __init__(self, kind, msg, resumable):
        super().__init__(msg)
        self.kind, self.msg, self.resumable = kind, msg, resumable


class OrderManager(Node):
    def __init__(self):
        super().__init__('order_manager')
        self.declare_parameter('shelves_file', '')
        self.declare_parameter('stations_file', '')
        self.db_path = os.path.expanduser(self.declare_parameter('db_path', '~/.agv/orders.db').value)
        self.reset_shelves = self.declare_parameter('reset_shelves', False).value
        self.idle_station = self.declare_parameter('idle_station', 'tram_sac').value
        self.straight = self.declare_parameter('straight_approach', 1.2).value
        self.auto_confirm_s = self.declare_parameter('auto_confirm_s', 0.0).value   # 0 = cho nguoi bam
        self.retries = self.declare_parameter('retries', 1).value
        self.battery_low = self.declare_parameter('battery_low', 0.30).value     # duoi muc nay: di sac
        self.battery_full = self.declare_parameter('battery_full', 0.80).value   # sac toi muc nay moi nhan don
        self.order_reserve = self.declare_parameter('order_reserve', 0.10).value  # pin can cho mot don
        # = staging_x_offset cua simple_charging_dock (nav2.yaml), dau duong
        self.charger_staging = self.declare_parameter('charger_staging_offset', 0.7).value
        self.battery = None        # (phan tram 0..1, dang sac?)
        self.charge_hold = False   # dang giu de sac, khong nhan don
        self.blocked_retry_s = self.declare_parameter('blocked_retry_s', 10.0).value   # duong bi chan: cho roi thu lai
        self.blocked_wait_s = self.declare_parameter('blocked_wait_s', 60.0).value     # ... toi da, roi bao nguoi
        self.stall_alarm_s = self.declare_parameter('stall_alarm_s', 15.0).value       # khong tien ve dich -> canh bao
        self.max_attempts = self.declare_parameter('max_attempts', 2).value            # lan thu mot don truoc khi bo
        # Docking chi kiem khoang cach toi tam o, khong kiem huong -> xe co the "vao xong" ma lech 22-50 deg
        # (REPORT.md 4.14). Kiem tra truoc khi nang / ha (checked_dock): lech huong > dock_yaw_tol
        # hoac (tra ke) lech vi tri > dock_pos_tol thi lui ra staging vao lai (toi da dock_attempts lan)
        self.dock_yaw_tol = math.radians(self.declare_parameter('dock_yaw_tol_deg', 8.0).value)
        self.dock_pos_tol = self.declare_parameter('dock_pos_tol', 0.10).value   # ke cach ke ben canh 0.35 m
        self.dock_attempts = self.declare_parameter('dock_attempts', 3).value
        # Spin quay thua ~30 deg (vung chet 1.5 rad/s + tre odom + ham, do o lan thu s2): chi quay tai cho khi
        # lech > turn_min, va xin bot spin_coast
        self.spin_coast = math.radians(self.declare_parameter('spin_coast_deg', 30.0).value)
        self.turn_min = math.radians(self.declare_parameter('turn_min_deg', 40.0).value)
        self.fault = None          # {'kind', 'msg', 'resumable'}: xe dung, cho nguoi
        self.alarm = None          # canh bao khong dung xe (vd duong bi chan, dang cho thu lai)
        self.active = []           # goal handle dang chay (de huy khi co su co); ClientGoalHandle khong hash duoc -> list
        self.drop_since = None

        self.layout = yaml.safe_load(open(self.get_parameter('shelves_file').value))
        self.stations = yaml.safe_load(open(self.get_parameter('stations_file').value))['stations']
        self.slots = self.layout['slots']
        self.staging_offset = self.layout['shelf']['staging_offset']

        self.lock = threading.RLock()
        self.wake = threading.Event()
        self.confirmed = threading.Event()
        self.acked = threading.Event()
        self.paused = False
        self.current = None        # dict don dang chay
        self.step = None
        self.lift_state, self.has_load = None, None
        self.cancel_requested = set()

        self.open_db()
        latched = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(String, 'order/state', QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.status_pub = self.create_publisher(String, 'order/status', latched)
        self.lift_pub = self.create_publisher(String, 'lift/command', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(String, 'order/command', self.on_command, 10)
        self.create_subscription(String, 'lift/state', lambda m: setattr(self, 'lift_state', m.data), latched)
        self.create_subscription(Bool, 'lift/has_load', lambda m: setattr(self, 'has_load', m.data), latched)
        self.create_subscription(BatteryState, 'battery_state', self.on_battery, 10)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.dock = ActionClient(self, DockRobot, 'dock_robot')
        self.undock = ActionClient(self, UndockRobot, 'undock_robot')
        self.backup = ActionClient(self, BackUp, 'backup')
        self.nav_through = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        self.spin_client = ActionClient(self, Spin, 'spin')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # Nav2 chi nhan dich khi da "active" (action server co the co truoc do va tu choi moi dich)
        self.state_clients = [self.create_client(GetState, f'{n}/get_state')
                              for n in ('bt_navigator', 'docking_server')]
        self.clear_clients = [self.create_client(ClearEntireCostmap, s) for s in (
            'global_costmap/clear_entirely_global_costmap', 'local_costmap/clear_entirely_local_costmap')]
        self.create_timer(1.0, self.publish_state)
        self.create_timer(0.2, self.monitor)
        threading.Thread(target=self.worker, daemon=True).start()
        self.say(f'{len(self.slots)} o ke, {self.count_pending()} don dang cho. Cho Nav2...')

    # ---------- du lieu ----------
    def open_db(self):
        os.makedirs(os.path.dirname(self.db_path), exist_ok=True)
        self.db = sqlite3.connect(self.db_path, check_same_thread=False)
        self.db.executescript('''
            CREATE TABLE IF NOT EXISTS orders (
              id INTEGER PRIMARY KEY AUTOINCREMENT, shelf TEXT, station TEXT, priority INTEGER,
              status TEXT, created REAL, started REAL, finished REAL, error TEXT);
            CREATE TABLE IF NOT EXISTS shelves (name TEXT PRIMARY KEY, slot TEXT);''')
        if 'attempts' not in [r[1] for r in self.db.execute('PRAGMA table_info(orders)')]:
            self.db.execute('ALTER TABLE orders ADD COLUMN attempts INTEGER DEFAULT 0')
        with self.lock:
            if self.reset_shelves or not self.db.execute('SELECT COUNT(*) FROM shelves').fetchone()[0]:
                self.db.execute('DELETE FROM shelves')
                self.db.executemany('INSERT INTO shelves VALUES (?, ?)', self.layout['shelves'].items())
            if self.reset_shelves:
                # Mo phong: world dat lai ke -> don cu cua lan chay truoc khong con y nghia
                self.db.execute("UPDATE orders SET status='cancelled', error='mo phong khoi dong lai' "
                                "WHERE status='pending'")
            # Don dang chay luc tat may: khong biet xe da lam toi dau -> danh dau loi, nguoi kiem tra
            self.db.execute("UPDATE orders SET status='failed', error='mat dien / khoi dong lai' "
                            "WHERE status='running'")
            self.db.commit()

    def count_pending(self):
        with self.lock:
            return self.db.execute("SELECT COUNT(*) FROM orders WHERE status='pending'").fetchone()[0]

    def shelf_slot(self, shelf):
        with self.lock:
            row = self.db.execute('SELECT slot FROM shelves WHERE name=?', (shelf,)).fetchone()
        return row[0] if row else None

    def set_order(self, oid, **kw):
        with self.lock:
            cols = ', '.join(f'{k}=?' for k in kw)
            self.db.execute(f'UPDATE orders SET {cols} WHERE id=?', (*kw.values(), oid))
            self.db.commit()

    # ---------- tien ich ----------
    def say(self, text):
        self.get_logger().info(text)
        self.status_pub.publish(String(data=text))

    def publish_state(self):
        with self.lock:
            rows = self.db.execute(
                'SELECT id, shelf, station, priority, status, created, started, finished, error FROM orders '
                "WHERE status IN ('pending', 'running') OR id IN "
                '(SELECT id FROM orders ORDER BY id DESC LIMIT 15) ORDER BY id').fetchall()
            shelves = dict(self.db.execute('SELECT name, slot FROM shelves').fetchall())
            done = self.db.execute("SELECT COUNT(*), AVG(finished - started) FROM orders WHERE status='done'").fetchone()
        keys = ('id', 'shelf', 'station', 'priority', 'status', 'created', 'started', 'finished', 'error')
        orders = [dict(zip(keys, r)) for r in rows]
        state = {
            'paused': self.paused,
            'error': self.fault['msg'] if self.fault else None,
            'fault_kind': self.fault['kind'] if self.fault else None,
            'alarm': self.alarm,
            'current': self.current['id'] if self.current else None,
            'step': self.step,
            'step_label': STEP_NAME.get(self.step),
            'carrying': self.current.get('carrying', False) if self.current else False,
            # Hang doi theo dung thu tu xe se lam (uu tien cao truoc, cung uu tien thi don cu truoc)
            'queue': sorted((o for o in orders if o['status'] == 'pending'), key=lambda o: (-o['priority'], o['id'])),
            'orders': orders,
            'shelves': shelves,
            'slots': self.slots,
            'stats': {'done': done[0], 'avg_s': round(done[1], 1) if done[1] else None},
            'battery': ({'pct': round(100 * self.battery[0], 1), 'charging': self.battery[1]}
                        if self.battery else None),
            'charge_hold': self.charge_hold,
        }
        self.state_pub.publish(String(data=json.dumps(state, ensure_ascii=False)))

    def on_battery(self, m):
        self.battery = (m.percentage, m.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING)

    # ---------- lenh ----------
    def on_command(self, msg):
        p = msg.data.strip().split()
        if not p:
            return
        cmd = p[0].lower()
        if cmd == 'add':
            if len(p) < 3:
                self.say('Dung: add <ke> <tram> [uu_tien]')
                return
            shelf, station = p[1], p[2]
            try:
                prio = int(p[3]) if len(p) > 3 else 0
            except ValueError:
                self.say(f'uu tien "{p[3]}" khong phai so')
                return
            with self.lock:
                known = self.db.execute('SELECT slot FROM shelves WHERE name=?', (shelf,)).fetchone()
            if known is None:
                self.say(f'Khong co ke "{shelf}"')
                return
            if known[0] is None:
                self.say(f'{shelf} chua ro vi tri (roi khi dang cho?). Dat lai ke roi gui "shelf {shelf} <o>"')
                return
            if self.stations.get(station, {}).get('kind') != 'pick':
                self.say(f'"{station}" khong phai tram lay hang')
                return
            with self.lock:
                cur = self.db.execute('INSERT INTO orders (shelf, station, priority, status, created) '
                                      "VALUES (?, ?, ?, 'pending', ?)", (shelf, station, prio, time.time()))
                self.db.commit()
            self.say(f'Them don #{cur.lastrowid}: {shelf} -> {station} (uu tien {prio})')
            self.wake.set()
        elif cmd == 'cancel' and len(p) > 1:
            try:
                oid = int(p[1])
            except ValueError:
                self.say(f'id "{p[1]}" khong phai so')
                return
            with self.lock:
                n = self.db.execute("UPDATE orders SET status='cancelled', finished=? "
                                    "WHERE id=? AND status='pending'", (time.time(), oid)).rowcount
                self.db.commit()
            if n:
                self.say(f'Da huy don #{oid}')
            elif self.current and self.current['id'] == oid:
                self.cancel_requested.add(oid)
                self.say(f'Don #{oid} dang chay: se huy neu chua nang ke')
            else:
                self.say(f'Khong huy duoc don #{oid}')
        elif cmd == 'confirm':
            if self.step == 'wait_confirm':
                self.confirmed.set()
                self.say('Da xac nhan lay hang, xe tra ke')
            else:
                self.say('Khong co don nao dang cho xac nhan')
        elif cmd == 'pause':
            self.paused = True
            self.say('Tam dung nhan don moi (don dang chay van lam xong)')
        elif cmd == 'resume':
            self.paused = False
            self.say('Tiep tuc nhan don')
            self.wake.set()
        elif cmd == 'ack':
            if self.fault:
                self.acked.set()
                self.say('Da xac nhan xu ly su co')
        elif cmd == 'estop':
            self.trigger_fault('dung_khan', 'nguoi van hanh bam DUNG KHAN', resumable=True)
        elif cmd == 'shelf' and len(p) > 2:
            # Nguoi dat lai ke vao o (vd sau khi ke roi): cap nhat vi tri trong CSDL
            shelf, slot = p[1], p[2]
            if slot not in self.slots:
                self.say(f'Khong co o "{slot}"')
                return
            with self.lock:
                taken = self.db.execute('SELECT name FROM shelves WHERE slot=? AND name<>?', (slot, shelf)).fetchone()
                if taken:
                    self.say(f'O {slot} dang co {taken[0]}')
                    return
                self.db.execute('UPDATE shelves SET slot=? WHERE name=?', (slot, shelf))
                self.db.commit()
            self.say(f'{shelf} dat o {slot}')
        else:
            self.say(f'Lenh khong hieu: "{msg.data}". Dung: add | cancel | confirm | pause | resume | ack | estop | shelf')
        self.publish_state()

    # ---------- action (chay trong worker thread, executor spin o thread khac) ----------
    def wait_future(self, fut, timeout):
        t0 = time.time()
        while not fut.done():
            if time.time() - t0 > timeout:
                return False
            time.sleep(0.05)
        return True

    def action(self, client, goal, timeout):
        """Gui goal, cho ket qua. Su co (self.fault) xay ra trong luc cho -> giam sat da huy goal -> Fault."""
        if self.fault:
            raise Fault(**self.fault)
        if not client.wait_for_server(timeout_sec=10.0):
            raise OrderError('action server khong san sang')
        # Nav2 tu hoi phuc (xoa costmap, quay, cho, lui; toi 6 lan) truoc khi bao loi -> co the mat hang phut
        # ma khong ai biet xe dang ket. Feedback number_of_recoveries tang = bao dong ngay (REPORT.md 4.13).
        # Nhung Nav2 con co the KHONG loi ma di vong mai (lap duong qua phan vat can lidar chua thay):
        # theo doi tien do distance_remaining, dung tien > stall_alarm_s thi canh bao, > blocked_wait_s
        # thi huy goal va bao su co "duong bi chan" cho nguoi.
        recov = {'n': 0, 'alarm': False, 'best': float('inf'), 't': time.time(), 'stall': False}
        target = None
        if isinstance(goal, NavigateToPose.Goal):
            target = (goal.pose.pose.position.x, goal.pose.pose.position.y)
        elif isinstance(goal, NavigateThroughPoses.Goal) and goal.poses:
            target = (goal.poses[-1].pose.position.x, goal.poses[-1].pose.position.y)

        def on_feedback(msg):
            fb = msg.feedback
            n = getattr(fb, 'number_of_recoveries', 0)
            if n > recov['n']:
                recov['n'], recov['alarm'] = n, True
                self.alarm = f'Nav2 dang hoi phuc (lan {n}): co the duong bi chan'
                self.say(self.alarm)
                self.publish_state()
            # Tien do = khoang cach thang tu xe (current_pose) toi dich cuoi. distance_remaining cua
            # NavigateThroughPoses khong giam deu (co luc = 0) -> bao "ket" nham khi xe van chay (REPORT 4.13)
            p = getattr(fb, 'current_pose', None)
            if p is not None and target is not None:
                d = math.hypot(p.pose.position.x - target[0], p.pose.position.y - target[1])
                if d < recov['best'] - 0.3:
                    recov['best'], recov['t'] = d, time.time()

        fut = client.send_goal_async(goal, feedback_callback=on_feedback)
        if not self.wait_future(fut, 10.0) or not fut.result().accepted:
            return False, None
        handle = fut.result()
        with self.lock:
            self.active.append(handle)
        try:
            res = handle.get_result_async()
            t0 = time.time()
            nav = isinstance(goal, (NavigateToPose.Goal, NavigateThroughPoses.Goal))
            while not res.done():
                if time.time() - t0 > timeout:
                    handle.cancel_goal_async()
                    return False, None
                stalled = time.time() - recov['t']
                if nav and stalled > self.stall_alarm_s and not recov['stall']:
                    recov['stall'] = recov['alarm'] = True
                    self.alarm = f'xe khong tien ve dich {stalled:.0f} s: co the duong bi chan'
                    self.say(self.alarm)
                    self.publish_state()
                if nav and stalled > self.blocked_wait_s:
                    self.alarm = None
                    # trigger_fault gan self.fault (de lenh "ack" duoc nhan), huy goal, van toc 0
                    self.trigger_fault('duong_bi_chan', f'xe khong tien ve dich qua {self.blocked_wait_s:.0f} s, '
                                       'can nguoi don duong', resumable=True)
                    raise Fault(**self.fault)
                time.sleep(0.1)
        finally:
            with self.lock:
                if handle in self.active:
                    self.active.remove(handle)
            if recov['alarm'] and self.alarm and self.alarm.startswith('Nav2 dang hoi phuc'):
                self.alarm = None
        if self.fault:
            raise Fault(**self.fault)
        return res.result().status == GoalStatus.STATUS_SUCCEEDED, res.result().result

    def pose(self, x, y, yaw):
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.pose.position.x, p.pose.position.y = float(x), float(y)
        p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        return p

    def clear_costmaps(self):
        for c in self.clear_clients:
            if c.service_is_ready():
                c.call_async(ClearEntireCostmap.Request())
        time.sleep(1.0)

    # ---------- su co ----------
    def trigger_fault(self, kind, msg, resumable):
        """Dung xe ngay: huy moi action dang chay, gui van toc 0. Buoc dang chay se nem Fault."""
        with self.lock:
            if self.fault:
                return
            self.fault = {'kind': kind, 'msg': msg, 'resumable': resumable}
            handles = list(self.active)
        for h in handles:
            h.cancel_goal_async()
        for _ in range(5):
            self.cmd_vel_pub.publish(Twist())
        self.say(f'SU CO ({kind}): {msg}. Xe dung, cho nguoi xu ly roi bam "Da xu ly" (ack).')
        self.publish_state()

    def wait_ack(self):
        self.acked.clear()
        self.acked.wait()
        with self.lock:
            self.fault = None
        self.alarm = None
        self.publish_state()

    def monitor(self):
        """5 Hz: phat hien su co trong luc chay don (chay o thread cua executor)."""
        o = self.current
        if not o or self.fault:
            self.drop_since = None
            return
        if self.lift_state == 'error':
            self.trigger_fault('co_cau_nang', 'co cau nang bao loi', resumable=False)
            return
        # Roi ke: dang cho ma mat tin hieu "co ke" > 1 s (ngoai luc dang nang / ha)
        if o.get('carrying') and self.step not in ('lift', 'lower') and self.has_load is False:
            self.drop_since = self.drop_since or time.time()
            if time.time() - self.drop_since > 1.0:
                self.trigger_fault('mat_ke', f'mat ke {o["shelf"]} khi dang cho', resumable=False)
        else:
            self.drop_since = None

    def retry(self, name, fn, nav=False):
        """Chay fn() -> (ok, info). Loi: xoa costmap thu lai self.retries lan. Chang di chuyen (nav) van loi
        thi coi la duong bi chan: bao dong, cho blocked_retry_s roi thu lai, toi da blocked_wait_s; van chan
        -> su co can nguoi (Fault tiep tuc duoc). Chang khac van loi -> OrderError."""
        label = STEP_NAME.get(name, name)
        for attempt in range(self.retries + 1):
            ok, info = fn()
            if ok:
                return
            self.say(f'{label} loi{": " + info if info else ""}'
                     + (', xoa costmap va thu lai' if attempt < self.retries else ''))
            if attempt < self.retries:
                self.clear_costmaps()
        if not nav:
            raise OrderError(f'{label} that bai')
        t0 = time.time()
        while time.time() - t0 < self.blocked_wait_s:
            self.alarm = f'{label}: duong bi chan, cho {self.blocked_retry_s:.0f} s roi thu lai'
            self.say(self.alarm)
            self.publish_state()
            for _ in range(int(self.blocked_retry_s * 10)):
                if self.fault:
                    raise Fault(**self.fault)
                time.sleep(0.1)
            self.clear_costmaps()
            ok, info = fn()
            if ok:
                self.alarm = None
                self.say(f'{label}: het chan, di tiep')
                return
        self.alarm = None
        self.trigger_fault('duong_bi_chan', f'{label}: duong bi chan qua {self.blocked_wait_s:.0f} s, '
                           'can nguoi don duong', resumable=True)
        raise Fault(**self.fault)

    # ---------- cac buoc ----------
    def straight_to(self, x, y, yaw, d):
        """Toi staging cach (x, y) d m doc truc yaw: di qua diem xa hon straight_approach m cung truc ->
        doan cuoi thang, mui xe thang truc. Quay tai cho cuoi chang thi vung chet dong co lam lech toi 30-50 deg
        (REPORT 4.9); dung, quay tai cho roi di thang cung khong duoc vi Spin quay thua ~30 deg (REPORT 4.14)."""
        c, s = math.cos(yaw), math.sin(yaw)
        pre = d + self.straight
        g = NavigateThroughPoses.Goal()
        g.poses = [self.pose(x - pre * c, y - pre * s, yaw), self.pose(x - d * c, y - d * s, yaw)]
        ok, r = self.action(self.nav_through, g, 180.0)
        return ok, '' if ok or r is None else f'error_code {r.error_code}'

    def go_staging(self, slot):
        sl = self.slots[slot]
        return self.straight_to(sl['x'], sl['y'], sl['yaw'], self.staging_offset)

    def coarse_turn(self, yaw):
        """Lech huong lon (> turn_min): quay tai cho cho bot lech. Vung chet day Spin len >= 1.5 rad/s, cong tre
        odom va ham: moi lan Spin quay thua ~30 deg (xin 3 deg -> quay 40, lan thu s2) nen khong sua duoc lech
        nho tai cho; lech nho de bo dieu khien docking sua trong luc di (lui ra, vao lai)."""
        for _ in range(2):
            p = self.robot_pose()
            if p is None:
                return
            e = math.atan2(math.sin(yaw - p[2]), math.cos(yaw - p[2]))
            if abs(e) <= self.turn_min:
                return
            cmd = e - math.copysign(self.spin_coast, e)
            self.spin(cmd)
            time.sleep(0.5)
            q = self.robot_pose()
            if q is not None:
                got = math.atan2(math.sin(q[2] - p[2]), math.cos(q[2] - p[2]))
                self.get_logger().info(f'quay bot lech: lech {math.degrees(e):+.0f} deg, xin {math.degrees(cmd):+.0f}, '
                                       f'quay that {math.degrees(got):+.0f}')

    def dock_into(self, slot, detect):
        g = DockRobot.Goal()
        g.navigate_to_staging_pose = False
        if detect:
            g.use_dock_id, g.dock_id = True, f'slot_{slot}'          # shelf_dock: theo chan ke
        else:
            sl = self.slots[slot]
            g.use_dock_id, g.dock_type = False, 'slot_dock'          # o trong: theo ban do
            g.dock_pose = self.pose(sl['x'], sl['y'], sl['yaw'])
        ok, r = self.action(self.dock, g, 120.0)
        ok = ok and r is not None and r.success
        return ok, '' if ok or r is None else f'error_code {r.error_code}'

    def robot_pose(self):
        """(x, y, yaw) cua base_link trong map (AMCL), None neu chua co TF."""
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', Time())
        except TransformException:
            return None
        q = t.transform.rotation
        return (t.transform.translation.x, t.transform.translation.y,
                math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    def slot_error(self, slot):
        """Sai lech xe so voi o: (huong rad, doc m, ngang m) theo truc o, None neu chua co TF."""
        p = self.robot_pose()
        if p is None:
            return None
        sl = self.slots[slot]
        c, s = math.cos(sl['yaw']), math.sin(sl['yaw'])
        ex, ey = p[0] - sl['x'], p[1] - sl['y']
        return math.atan2(math.sin(p[2] - sl['yaw']), math.cos(p[2] - sl['yaw'])), ex * c + ey * s, -ex * s + ey * c

    def spin(self, angle):
        g = Spin.Goal()
        g.target_yaw = float(angle)
        g.time_allowance.sec = 15
        ok, _ = self.action(self.spin_client, g, 30.0)
        return ok

    def checked_dock(self, slot, detect):
        """Docking vao o roi KIEM TRA tu the (AMCL) so voi truc o truoc khi nang / ha ke. Docking chi kiem
        khoang cach toi dich, khong kiem huong: voi vung chet dong co xe co the "vao xong" ma lech 22-50 deg
        (REPORT.md 4.14) -> ha ke xeo, hoac nang ke xeo (chan ke lot ra ngoai vung loc scan).
        Sai: lech lon thi quay bot tai cho (giua o, an toan), lui thang ra staging, vao lai (dock_attempts lan)."""
        info = ''
        for attempt in range(self.dock_attempts):
            if attempt:
                err = self.slot_error(slot)
                if err is not None:
                    self.coarse_turn(self.slots[slot]['yaw'])
                    err = self.slot_error(slot) or err
                    if err[1] + self.staging_offset > 0.15:
                        self.back_up(err[1] + self.staging_offset)
                self.clear_costmaps()
            ok, info = self.dock_into(slot, detect)
            if not ok:
                continue
            time.sleep(0.5)                 # AMCL / TF cap nhat sau khi dung
            err = self.slot_error(slot)
            if err is None:
                return True, ''
            yaw, lon, lat = err
            # Lay ke: ke co the dat lech trong o va docking bam theo chan ke -> chi kiem huong
            if abs(yaw) <= self.dock_yaw_tol and (detect or math.hypot(lon, lat) <= self.dock_pos_tol):
                if attempt:
                    self.say(f'vao o {slot}: tu the dat sau {attempt + 1} lan (lech {math.degrees(yaw):+.1f} deg, '
                             f'{100 * lon:+.0f} / {100 * lat:+.0f} cm)')
                return True, ''
            info = f'xe lech {math.degrees(yaw):+.0f} deg, doc {100 * lon:+.0f} cm, ngang {100 * lat:+.0f} cm'
            self.say(f'vao o {slot}: {info}, lui ra vao lai (lan {attempt + 1}/{self.dock_attempts})')
        return False, info

    def undock_from(self, dock_type):
        g = UndockRobot.Goal()
        g.dock_type, g.max_undocking_time = dock_type, 30.0
        ok, r = self.action(self.undock, g, 60.0)
        ok = ok and r is not None and r.success
        return ok, '' if ok or r is None else f'error_code {r.error_code}'

    def lift(self, cmd):
        want = ('up', True) if cmd == 'up' else ('down', False)
        self.lift_pub.publish(String(data=cmd))
        t0 = time.time()
        time.sleep(0.3)
        while time.time() - t0 < 10.0:
            if self.fault:
                raise Fault(**self.fault)
            if (self.lift_state, self.has_load) == want:
                return True, ''
            if self.lift_state == 'error':
                return False, 'co cau nang bao loi'
            time.sleep(0.1)
        return False, f'state {self.lift_state}, has_load {self.has_load}'

    def go_station(self, name):
        st = self.stations[name]
        g = NavigateToPose.Goal()
        g.pose = self.pose(st['x'], st['y'], st['yaw'])
        ok, r = self.action(self.nav, g, 240.0)
        return ok, '' if ok or r is None else f'error_code {r.error_code}'

    def set_step(self, step):
        self.step = step
        self.publish_state()

    def wait_confirm(self, o):
        self.say(f'Don #{o["id"]}: {o["shelf"]} da toi {o["station"]}, cho nguoi lay hang (bam xac nhan)')
        self.confirmed.clear()
        t0 = time.time()
        while not self.confirmed.wait(0.5):
            if self.fault:
                raise Fault(**self.fault)
            if self.auto_confirm_s > 0 and time.time() - t0 > self.auto_confirm_s:
                self.say(f'Tu xac nhan sau {self.auto_confirm_s:.0f} s (auto_confirm_s)')
                break

    def run_order(self, o):
        """Chay don theo danh sach buoc. Su co tiep tuc duoc (dung khan, duong bi chan): cho nguoi bam
        "Da xu ly" roi lam lai DUNG buoc dang do. Su co khong tiep tuc duoc (roi ke, co cau nang) -> Fault len worker."""
        oid, shelf, station = o['id'], o['shelf'], o['station']
        slot = self.shelf_slot(shelf)
        self.set_order(oid, status='running', started=time.time())
        self.say(f'Don #{oid}: lay {shelf} (o {slot}) -> {station}')

        def lift_up():
            ok, info = self.lift('up')
            if not ok:
                raise OrderError(f'nang ke that bai: {info}')
            o['carrying'] = True
            # payload_manager xoa costmap khi mat nang len het; cho costmap cap nhat lai (scan da loc chan
            # ke) roi moi lui ra, khong thi doi khi con "chan ke ma" -> docking bao va cham
            time.sleep(1.5)

        def lower():
            ok, info = self.lift('down')
            if not ok:
                self.trigger_fault('co_cau_nang', f'ha ke that bai: {info}', resumable=False)
                raise Fault(**self.fault)
            o['carrying'] = False

        steps = [
            ('staging', lambda: self.retry('staging', lambda: self.go_staging(slot), nav=True)),
            ('dock', lambda: self.retry('dock', lambda: self.checked_dock(slot, detect=True))),
            ('lift', lift_up),
            ('undock', lambda: self.retry('undock', lambda: self.undock_from('shelf_dock'))),
            ('to_station', lambda: self.retry('to_station', lambda: self.go_station(station), nav=True)),
            ('wait_confirm', lambda: self.wait_confirm(o)),
            ('return_staging', lambda: self.retry('return_staging', lambda: self.go_staging(slot), nav=True)),
            ('return_dock', lambda: self.retry('return_dock', lambda: self.checked_dock(slot, detect=False))),
            ('lower', lower),
            ('return_undock', lambda: self.retry('return_undock', self.leave_slot)),
        ]
        # Tu buoc nang ke tro di (ca sau khi da ha ke, xe con duoi gam ke) loi nao cung KHONG duoc tra don
        # ve hang doi / chay lai tu dau: thanh su co cho nguoi, lam lai dung buoc (REPORT.md 4.13)
        lift_index = [name for name, _ in steps].index('lift')
        i = 0
        while i < len(steps):
            name, fn = steps[i]
            if name in ('staging', 'dock') and oid in self.cancel_requested:
                raise OrderError('nguoi dung huy')
            self.set_step(name)
            try:
                try:
                    fn()
                except OrderError as e:
                    if i < lift_index:
                        raise          # chua nang ke: worker tra don ve hang doi
                    where = f'dang cho {shelf}' if o.get('carrying') else f'xe con o gam {shelf}'
                    self.trigger_fault('buoc_loi_sau_khi_nang', f'{e} ({where})', resumable=True)
                    raise Fault(**self.fault)
                i += 1
            except Fault as f:
                if not f.resumable:
                    raise
                self.wait_ack()
                self.say(f'Don #{oid}: tiep tuc buoc "{STEP_NAME.get(name, name)}"')
        with self.lock:
            dt = time.time() - self.db.execute('SELECT started FROM orders WHERE id=?', (oid,)).fetchone()[0]
        self.set_order(oid, status='done', finished=time.time())
        self.say(f'Don #{oid} xong sau {dt:.1f} s')

    def next_order(self):
        with self.lock:
            row = self.db.execute("SELECT id, shelf, station FROM orders WHERE status='pending' "
                                  'ORDER BY priority DESC, id LIMIT 1').fetchone()
        return {'id': row[0], 'shelf': row[1], 'station': row[2]} if row else None

    def go_charger(self):
        """Vao tram sac: tu di thang vao staging doc truc tram (nhu voi ke), roi DockRobot cho bao dang sac.
        De docking_server tu di thi xe dang trong 0.5 m quanh staging se chui cheo vao luon va quet goc
        khoi tiep diem (REPORT.md 4.12)."""
        st = self.stations[self.idle_station]
        if not (self.battery and self.battery[1]):
            # Chua cham tiep diem: di thang vao staging truoc. Dang sac san (xe xuat phat ngay o tram) thi
            # KHONG di: phai quay dau sat khoi tiep diem va bi ket (REPORT.md 4.13); DockRobot thang se bao
            # "already docked" va ghi nho dock cho UndockRobot sau nay
            ok, info = self.straight_to(st['x'], st['y'], st['yaw'], self.charger_staging)
            if not ok:
                return False, f'khong toi duoc truoc tram sac: {info}'
        g = DockRobot.Goal()
        g.use_dock_id, g.dock_id = True, f'charger_{self.idle_station}'
        g.navigate_to_staging_pose = False
        ok, r = self.action(self.dock, g, 120.0)
        ok = ok and r is not None and r.success
        if not ok:
            # Xe co the dang sat khoi tiep diem: lui thang ra truoc, quay tai cho o day se quet vao khoi
            # (lan thu f8: nhan don ngay sau khi vao tram loi, xe ket o day 60 s - REPORT.md 4.13)
            self.back_up(0.5)
        return ok, '' if ok or r is None else f'error_code {r.error_code}'

    def leave_slot(self):
        """Lui ra khoi gam sau khi ha ke. UndockRobot loi (ke vua dat lech, chan ke sat footprint -> du bao va
        cham) thi lui thang 1 m bang BackUp: xe da thang truc o (checked_dock) nen duong lui la duong vua vao."""
        ok, info = self.undock_from('slot_dock')
        if ok:
            return True, ''
        ok2 = self.back_up(1.0)
        return ok2, f'undock loi ({info}), lui thang 1 m: {"OK" if ok2 else "loi"}'

    def back_up(self, dist):
        g = BackUp.Goal()
        g.target = Point(x=float(dist))
        g.speed = 0.15
        g.time_allowance.sec = 15
        ok, _ = self.action(self.backup, g, 30.0)
        return ok

    def leave_charger(self):
        """Roi tram sac. UndockRobot loi (vd xe bat khoi tiep diem > docking_threshold: "Robot is not in
        the dock") thi lui thang 0.5 m bang behavior BackUp: xe dang sat khoi tiep diem, quay tai cho se quet vao."""
        ok, info = self.undock_from('simple_charging_dock')
        if ok:
            return True, ''
        ok2 = self.back_up(0.5)
        return ok2, f'undock loi ({info}), lui thang 0.5 m: {"OK" if ok2 else "loi"}'

    def nav2_active(self):
        for c in self.state_clients:
            if not c.wait_for_service(timeout_sec=2.0):
                return False
            fut = c.call_async(GetState.Request())
            if not self.wait_future(fut, 5.0) or fut.result().current_state.id != State.PRIMARY_STATE_ACTIVE:
                return False
        return True

    def update_charge_hold(self):
        if self.battery is None:
            return
        pct = self.battery[0]
        need = self.battery_low + self.order_reserve
        if not self.charge_hold and pct < need:
            self.charge_hold = True
            self.say(f'Pin {100 * pct:.0f} % < {100 * need:.0f} % (nguong {100 * self.battery_low:.0f} % + du tru '
                     f'mot don): dung nhan don, di sac toi {100 * self.battery_full:.0f} %')
        elif self.charge_hold and pct >= self.battery_full:
            self.charge_hold = False
            self.say(f'Pin {100 * pct:.0f} %: nhan don tiep')
            self.wake.set()

    def worker(self):
        while rclpy.ok() and not self.nav2_active():
            time.sleep(1.0)
        self.say('Nav2 / docking san sang, nhan don')
        # Luc dau coi nhu chua o tram (du xe xuat phat sat tiep diem): DockRobot mot lan de docking_server
        # biet xe dang o dock nao, sau do UndockRobot moi chay duoc
        docked = False
        while rclpy.ok():
            self.update_charge_hold()
            o = None if (self.paused or self.charge_hold) else self.next_order()
            if self.fault:          # su co luc ranh (vd dung khan khi dang vao tram sac)
                self.wait_ack()
                continue
            if o is None:
                if not docked:
                    self.set_step('to_charger')
                    try:
                        ok, info = self.go_charger()
                    except Fault:
                        continue
                    docked = ok
                    if not ok:
                        self.say(f'Vao tram sac loi{": " + info if info else ""}, thu lai sau')
                self.set_step(('charging' if self.battery and self.battery[1] else 'docked') if docked else None)
                self.wake.wait(2.0)
                self.wake.clear()
                continue
            if docked:
                self.set_step('leave_charger')
                try:
                    ok, info = self.leave_charger()
                except Fault:
                    continue
                if info:
                    self.say(f'Roi tram sac: {info}')
                if not ok:
                    time.sleep(2.0)
                    continue
                docked = False
            self.current = o
            try:
                self.run_order(o)
            except Fault as f:
                # Su co khong tiep tuc duoc (roi ke, co cau nang): don loi, ke chua ro vi tri neu dang cho
                self.set_order(o['id'], status='failed', finished=time.time(), error=f.msg)
                if o.get('carrying') or f.kind == 'mat_ke':
                    with self.lock:
                        self.db.execute('UPDATE shelves SET slot=NULL WHERE name=?', (o['shelf'],))
                        self.db.commit()
                    self.say(f'{o["shelf"]} chua ro vi tri: dat lai ke roi gui "shelf {o["shelf"]} <o>"')
                o['carrying'] = False
                self.wait_ack()
            except OrderError as e:
                # Loi truoc khi nang ke (xe khong cho gi): tra don ve hang doi, toi da max_attempts lan
                with self.lock:
                    n = self.db.execute('SELECT attempts FROM orders WHERE id=?', (o['id'],)).fetchone()[0] or 0
                if n + 1 < self.max_attempts and 'huy' not in str(e):
                    self.set_order(o['id'], status='pending', attempts=n + 1, error=str(e))
                    self.say(f'Don #{o["id"]} loi ({e}), tra ve hang doi (lan {n + 1}/{self.max_attempts})')
                else:
                    self.set_order(o['id'], status='failed', finished=time.time(), error=str(e), attempts=n + 1)
                    self.say(f'Don #{o["id"]} LOI: {e}')
            finally:
                self.cancel_requested.discard(o['id'])
                self.current = None
                self.set_step(None)


def main():
    rclpy.init()
    node = OrderManager()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
