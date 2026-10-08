#!/usr/bin/env python3
"""Quan ly don hang kho kieu Kiva: hang doi don, lay ke -> cho toi tram -> tra ke (dung chung xe that).

Mot don = "dua ke <ke> toi tram <tram>". Xe lam lan luot:
  toi staging truoc o -> chui gam (shelf_dock, theo chan ke) -> nang -> lui ra -> cho ke toi tram
  -> CHO NGUOI XAC NHAN da lay hang -> toi staging -> tra ke vao o (slot_dock) -> ha -> lui ra
Het don thi ve idle_station (tram sac).

Lenh: /order/command (std_msgs/String)
  add <ke> <tram> [uu_tien]   them don (uu_tien lon lam truoc, mac dinh 0)
  cancel <id>                 huy don dang cho (don dang chay: chi huy duoc truoc khi nang ke)
  confirm [id]                nguoi o tram da lay hang xong -> xe tra ke
  pause | resume              tam dung nhan don moi / tiep tuc (don dang chay van lam xong)
  ack                         xac nhan da xu ly loi (xe dang o trang thai error)
Trang thai: /order/state (std_msgs/String, JSON, giu tin cuoi): hang doi, don dang chay + buoc,
vi tri ke, lich su gan day. Nhat ky de doc: /order/status (String, giu tin cuoi).

Du lieu: SQLite (tham so db_path): bang orders (lich su, thong ke) va shelves (ke dang o o nao).
Mo phong: reset_shelves:=true nap lai vi tri ke tu shelves.yaml moi lan chay (world cung reset).

Loi: moi chang di chuyen / chui gam thu lai 1 lan (xoa costmap). Van loi:
  - chua nang ke -> don "failed", lam don tiep theo
  - dang cho ke -> xe dung, trang thai "error", cho nguoi xu ly roi gui "ack" (giai doan 5 se tu xu ly)
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
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import DockRobot, NavigateThroughPoses, NavigateToPose, UndockRobot
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String

STEP_NAME = {
    'staging': 'toi truoc o ke', 'dock': 'chui gam', 'lift': 'nang ke', 'undock': 'lui ra',
    'to_station': 'cho ke toi tram', 'wait_confirm': 'cho nguoi lay hang', 'return_staging': 'toi truoc o',
    'return_dock': 'tra ke vao o', 'lower': 'ha ke', 'return_undock': 'lui ra', 'home': 've tram sac',
}


class OrderError(Exception):
    pass


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
        self.error = None
        self.lift_state, self.has_load = None, None
        self.cancel_requested = set()

        self.open_db()
        latched = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(String, 'order/state', QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.status_pub = self.create_publisher(String, 'order/status', latched)
        self.lift_pub = self.create_publisher(String, 'lift/command', 10)
        self.create_subscription(String, 'order/command', self.on_command, 10)
        self.create_subscription(String, 'lift/state', lambda m: setattr(self, 'lift_state', m.data), latched)
        self.create_subscription(Bool, 'lift/has_load', lambda m: setattr(self, 'has_load', m.data), latched)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_through = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        self.dock = ActionClient(self, DockRobot, 'dock_robot')
        self.undock = ActionClient(self, UndockRobot, 'undock_robot')
        self.clear_clients = [self.create_client(ClearEntireCostmap, s) for s in (
            'global_costmap/clear_entirely_global_costmap', 'local_costmap/clear_entirely_local_costmap')]
        self.create_timer(1.0, self.publish_state)
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
        with self.lock:
            if self.reset_shelves or not self.db.execute('SELECT COUNT(*) FROM shelves').fetchone()[0]:
                self.db.execute('DELETE FROM shelves')
                self.db.executemany('INSERT INTO shelves VALUES (?, ?)', self.layout['shelves'].items())
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
            'error': self.error,
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
        }
        self.state_pub.publish(String(data=json.dumps(state, ensure_ascii=False)))

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
            if self.shelf_slot(shelf) is None:
                self.say(f'Khong co ke "{shelf}"')
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
            if self.error:
                self.acked.set()
                self.say('Da xac nhan xu ly loi')
        else:
            self.say(f'Lenh khong hieu: "{msg.data}". Dung: add | cancel | confirm | pause | resume | ack')
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
        if not client.wait_for_server(timeout_sec=10.0):
            raise OrderError('action server khong san sang')
        fut = client.send_goal_async(goal)
        if not self.wait_future(fut, 10.0) or not fut.result().accepted:
            return False, None
        handle = fut.result()
        res = handle.get_result_async()
        if not self.wait_future(res, timeout):
            handle.cancel_goal_async()
            return False, None
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

    def retry(self, name, fn):
        """Chay fn() -> (ok, info); loi thi xoa costmap thu lai self.retries lan, van loi -> OrderError."""
        for attempt in range(self.retries + 1):
            ok, info = fn()
            if ok:
                return
            self.say(f'{STEP_NAME.get(name, name)} loi{": " + info if info else ""}'
                     + (', xoa costmap va thu lai' if attempt < self.retries else ''))
            if attempt < self.retries:
                self.clear_costmaps()
        raise OrderError(f'{STEP_NAME.get(name, name)} that bai')

    # ---------- cac buoc ----------
    def go_staging(self, slot):
        sl = self.slots[slot]
        c, s = math.cos(sl['yaw']), math.sin(sl['yaw'])
        d, pre = self.staging_offset, self.staging_offset + self.straight
        g = NavigateThroughPoses.Goal()
        # Di thang vao staging doc truc o: quay tai cho voi vung chet dong co lech toi 30-50 deg (REPORT 4.9)
        g.poses = [self.pose(sl['x'] - pre * c, sl['y'] - pre * s, sl['yaw']),
                   self.pose(sl['x'] - d * c, sl['y'] - d * s, sl['yaw'])]
        ok, r = self.action(self.nav_through, g, 180.0)
        return ok, '' if ok or r is None else f'error_code {r.error_code}'

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

    def run_order(self, o):
        oid, shelf, station = o['id'], o['shelf'], o['station']
        slot = self.shelf_slot(shelf)
        self.set_order(oid, status='running', started=time.time())
        self.say(f'Don #{oid}: lay {shelf} (o {slot}) -> {station}')
        steps_before_lift = [('staging', lambda: self.go_staging(slot)),
                             ('dock', lambda: self.dock_into(slot, detect=True))]
        for name, fn in steps_before_lift:
            if oid in self.cancel_requested:
                raise OrderError('nguoi dung huy')
            self.set_step(name)
            self.retry(name, fn)
        self.set_step('lift')
        ok, info = self.lift('up')
        if not ok:
            raise OrderError(f'nang ke that bai: {info}')
        o['carrying'] = True
        for name, fn in [('undock', lambda: self.undock_from('shelf_dock')),
                         ('to_station', lambda: self.go_station(station))]:
            self.set_step(name)
            self.retry(name, fn)
        self.set_step('wait_confirm')
        self.say(f'Don #{oid}: {shelf} da toi {station}, cho nguoi lay hang (bam xac nhan)')
        self.confirmed.clear()
        t0 = time.time()
        while not self.confirmed.wait(0.5):
            if self.auto_confirm_s > 0 and time.time() - t0 > self.auto_confirm_s:
                self.say(f'Tu xac nhan sau {self.auto_confirm_s:.0f} s (auto_confirm_s)')
                break
        for name, fn in [('return_staging', lambda: self.go_staging(slot)),
                         ('return_dock', lambda: self.dock_into(slot, detect=False))]:
            self.set_step(name)
            self.retry(name, fn)
        self.set_step('lower')
        ok, info = self.lift('down')
        if not ok:
            raise OrderError(f'ha ke that bai: {info}')
        o['carrying'] = False
        self.set_step('return_undock')
        self.retry('return_undock', lambda: self.undock_from('slot_dock'))
        with self.lock:
            dt = time.time() - self.db.execute('SELECT started FROM orders WHERE id=?', (oid,)).fetchone()[0]
        self.set_order(oid, status='done', finished=time.time())
        self.say(f'Don #{oid} xong sau {dt:.1f} s')

    def next_order(self):
        with self.lock:
            row = self.db.execute("SELECT id, shelf, station FROM orders WHERE status='pending' "
                                  'ORDER BY priority DESC, id LIMIT 1').fetchone()
        return {'id': row[0], 'shelf': row[1], 'station': row[2]} if row else None

    def worker(self):
        while not self.dock.wait_for_server(timeout_sec=2.0) and rclpy.ok():
            pass
        self.say('Nav2 / docking san sang, nhan don')
        at_home = False
        while rclpy.ok():
            o = None if self.paused else self.next_order()
            if o is None:
                if not at_home and self.idle_station in self.stations:
                    self.set_step('home')
                    ok, _ = self.go_station(self.idle_station)
                    at_home = ok
                    self.set_step(None)
                self.wake.wait(2.0)
                self.wake.clear()
                continue
            at_home = False
            self.current = o
            try:
                self.run_order(o)
            except OrderError as e:
                carrying = o.get('carrying', False)
                self.set_order(o['id'], status='failed', finished=time.time(), error=str(e))
                self.say(f'Don #{o["id"]} LOI: {e}')
                if carrying:
                    # Dang cho ke ma loi: khong tu xu ly (giai doan 5), xe dung cho nguoi
                    self.error = f'don #{o["id"]}: {e} (dang cho {o["shelf"]})'
                    self.say(f'XE DUNG: {self.error}. Xu ly xong gui "ack".')
                    self.publish_state()
                    self.acked.clear()
                    self.acked.wait()
                    self.error = None
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
