#!/usr/bin/env python3
"""Co cau nang trong mo phong, cung giao dien topic voi xe that (cau noi UART <-> STM32):

  /lift/command   std_msgs/String  "up" | "down"           (vao)
  /lift/state     std_msgs/String  down | moving_up | up | moving_down | error   (ra, giu tin cuoi)
  /lift/has_load  std_msgs/Bool    co ke tren mat nang     (ra, giu tin cuoi)

Ben trong: gui vi tri khop lift_joint cho JointPositionController cua Gazebo (/lift/cmd_pos), tang
dan voi toc do speed (vit me: toc do gan nhu khong doi bat ke tai, khong nhay thang toi dich),
doc vi tri that tu /joint_states, "co ke" tu cam bien tiep xuc mat nang (/lift/contact).
Qua timeout_s chua toi vi tri (ket co khi, qua tai) -> state "error", giong firmware se bao.

Khoa ke (lock_shelves): nang het len ma cam bien thay ke thi noi khop co dinh ke <-> mat nang (plugin
DetachableJoint trong moi ke cua world, topic gz /shelf/<ke>/attach | detach), lenh ha thi mo khoa truoc
roi moi ha. Giong chot dinh vi tren mat nang xe that; chi nho ma sat thi ke truot toi 10 cm khi xe quay
tai cho (REPORT.md 4.14). Plugin noi khop moi ke voi xe ngay khi xe xuat hien: moi giay so trang thai
that cua tung khop (/shelf/<ke>/state) voi trang thai mong muon (chi ke dang cho la "attached") va gui
lenh toi khi khop. Truoc day gui mot lan bang `gz topic -p`: tin co the mat (chua kip ket noi voi plugin),
xe con dinh vao 8 ke, banh quay tai cho ma odom tuong da di 1.5 m (lan thu m8).

Khong dung use_sim_time (chi dem thoi gian may): tranh nhan /clock 333 tin/s (REPORT.md 4.8).
"""
import re
import subprocess
import threading
import time

import rclpy
from gz.msgs10.empty_pb2 import Empty
from gz.msgs10.stringmsg_pb2 import StringMsg
from gz.transport13 import Node as GzNode
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, String


class LiftSim(Node):
    def __init__(self):
        super().__init__('lift_sim')
        self.up_pos = self.declare_parameter('up_position', 0.03).value
        self.tol = self.declare_parameter('tolerance', 0.004).value   # 2 mm: lan s4 mat nang dung o 2.0 mm sau khi mo khoa ke -> bao loi
        self.timeout = self.declare_parameter('timeout_s', 5.0).value
        self.load_hold = self.declare_parameter('load_hold_s', 0.5).value
        self.speed = self.declare_parameter('speed', 0.02).value     # m/s, khop lift_speed (URDF)
        self.period = 0.05
        self.lock_shelves = self.declare_parameter('lock_shelves', True).value
        self.contact_shelf = None  # ke dang cham mat nang (ten model trong Gazebo)
        self.locked = None         # ke dang khoa vao mat nang
        self.unlocking = False     # dang mo khoa, chua bat dau ha
        self.joints = {}           # ke -> {'attach': pub, 'detach': pub, 'state': str | None, 'sent': so lan}
        self.joint_lock = threading.Lock()
        self.gz = GzNode()
        self.attach_until = 0.0    # chi gui lenh khoa trong cua so nay sau khi nang (ke bi go ra thi thoi)
        self.robot_seen = False

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.cmd_pub = self.create_publisher(Float64, 'lift/cmd_pos', 10)
        self.state_pub = self.create_publisher(String, 'lift/state', latched)
        self.load_pub = self.create_publisher(Bool, 'lift/has_load', latched)
        self.create_subscription(String, 'lift/command', self.on_command, 10)
        self.create_subscription(JointState, 'joint_states', self.on_joints, 10)
        self.create_subscription(Contacts, 'lift/contact', self.on_contact, 10)

        self.target = 0.0
        self.cmd = 0.0             # vi tri dang gui (tien dan toi target)
        self.pos = None
        self.state = None
        self.cmd_time = 0.0
        self.last_contact = 0.0
        self.has_load = None
        self.set_state('down')
        self.create_timer(self.period, self.tick)
        if self.lock_shelves:
            threading.Thread(target=self.find_shelves, daemon=True).start()
            self.create_timer(1.0, self.check_joints)

    def set_state(self, state):
        if state != self.state:
            self.state = state
            self.state_pub.publish(String(data=state))
            self.get_logger().info(f'mat nang: {state}')

    def on_command(self, msg):
        cmd = msg.data.strip().lower()
        if cmd not in ('up', 'down'):
            self.get_logger().warn(f'lenh nang khong hieu: "{msg.data}" (up | down)')
            return
        if cmd == 'down' and self.locked:
            # Mo khoa truoc roi moi ha: con khoa thi chan ke cham san ma mat nang van keo xuong
            shelf, self.locked = self.locked, None
            self.unlocking = True
            self.send_joint(shelf)
            self.set_state('moving_down')
            threading.Thread(target=self.unlock_then_lower, args=(shelf,), daemon=True).start()
            return
        self.target = self.up_pos if cmd == 'up' else 0.0
        self.cmd_time = time.monotonic()
        self.set_state('moving_up' if cmd == 'up' else 'moving_down')

    def find_shelves(self):
        """Tim cac ke co khop khoa (topic /shelf/<ke>/detach), tao publisher thuong truc va nghe trang thai."""
        while rclpy.ok():
            out = subprocess.run(['gz', 'topic', '-l'], capture_output=True, text=True, timeout=10).stdout
            names = sorted(set(re.findall(r'^/shelf/([^/]+)/detach$', out, re.M)))
            if names:
                break
            time.sleep(1.0)
        for name in names:
            j = {'attach': self.gz.advertise(f'/shelf/{name}/attach', Empty),
                 'detach': self.gz.advertise(f'/shelf/{name}/detach', Empty), 'state': None, 'sent': 0}
            self.gz.subscribe(StringMsg, f'/shelf/{name}/state', lambda m, n=name: self.on_joint_state(n, m.data))
            with self.joint_lock:
                self.joints[name] = j
        self.get_logger().info(f'{len(names)} ke co khop khoa: {" ".join(names)}')

    def on_joint_state(self, name, state):
        with self.joint_lock:
            j = self.joints.get(name)
            if j is not None and j['state'] != state:
                j['state'], j['sent'] = state, 0
                self.get_logger().info(f'{name}: {state}')

    def send_joint(self, name):
        """Gui lenh dua khop cua ke ve trang thai mong muon (chi ke dang khoa la attached)."""
        with self.joint_lock:
            j = self.joints.get(name)
            if j is None:
                return
            if name == self.locked:
                if time.monotonic() > self.attach_until:
                    return          # da het cua so khoa: khong tu khoa lai ke da bi go ra (bai thu roi ke)
                want = 'attached'
            else:
                want = 'detached'
            # Chua nghe duoc trang thai (plugin chi phat khi doi?): gui toi da 3 lan roi coi nhu da khop
            if j['state'] == want or (j['state'] is None and j['sent'] >= 3):
                return
            j['sent'] += 1
            pub = j['attach'] if want == 'attached' else j['detach']
        pub.publish(Empty())

    def reset_sent(self):
        with self.joint_lock:
            for j in self.joints.values():
                j['sent'] = 0

    def check_joints(self):
        for name in list(self.joints):
            self.send_joint(name)

    def joint_state(self, name):
        with self.joint_lock:
            j = self.joints.get(name)
            return j['state'] if j else None

    def unlock_then_lower(self, shelf):
        t0 = time.monotonic()
        while self.joint_state(shelf) not in ('detached', None) and time.monotonic() - t0 < 3.0:
            self.send_joint(shelf)
            time.sleep(0.2)
        if self.joint_state(shelf) == 'attached':
            self.get_logger().error(f'khong mo khoa duoc {shelf} sau 3 s, van ha')
        time.sleep(0.2)
        self.target = 0.0
        self.cmd_time = time.monotonic()
        self.unlocking = False

    def on_joints(self, msg):
        if 'lift_joint' in msg.name:
            self.pos = msg.position[msg.name.index('lift_joint')]
            if not self.robot_seen:
                # Xe vua xuat hien trong Gazebo: plugin se noi khop moi ke voi xe -> dem lai so lan gui
                self.robot_seen = True
                threading.Timer(2.0, self.reset_sent).start()

    def on_contact(self, msg):
        if msg.contacts:
            self.last_contact = time.monotonic()
            c = msg.contacts[0]
            for e in (c.collision1, c.collision2):
                model = e.name.split('::')[0]
                if model and model != 'agv':
                    self.contact_shelf = model

    def tick(self):
        now = time.monotonic()
        step = self.speed * self.period
        self.cmd += max(-step, min(step, self.target - self.cmd))
        # Gui moi chu ky ke ca khi da toi: bridge / Gazebo co the chua san sang luc dau
        self.cmd_pub.publish(Float64(data=self.cmd))
        load = now - self.last_contact < self.load_hold
        if load != self.has_load:
            self.has_load = load
            self.load_pub.publish(Bool(data=load))
        if self.pos is None or self.unlocking or self.state not in ('moving_up', 'moving_down'):
            return
        if abs(self.pos - self.target) < self.tol:
            if self.target > 0 and self.lock_shelves and self.has_load and self.contact_shelf and not self.locked:
                self.locked = self.contact_shelf
                self.attach_until = time.monotonic() + 3.0
                with self.joint_lock:
                    if self.locked in self.joints:
                        self.joints[self.locked]['sent'] = 0
                self.send_joint(self.locked)
            self.set_state('up' if self.target > 0 else 'down')
        elif now - self.cmd_time > self.timeout:
            self.get_logger().error(f'mat nang khong toi {self.target:.3f} m sau {self.timeout:.0f} s '
                                    f'(dang o {self.pos:.3f} m)')
            self.set_state('error')


def main():
    rclpy.init()
    node = LiftSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
