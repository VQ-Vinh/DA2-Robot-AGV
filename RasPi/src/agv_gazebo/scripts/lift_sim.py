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
tai cho (REPORT.md 4.14). Plugin noi khop san luc mo phong bat dau: khi thay xe, mo khoa het cac ke.

Khong dung use_sim_time (chi dem thoi gian may): tranh nhan /clock 333 tin/s (REPORT.md 4.8).
"""
import re
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, String


class LiftSim(Node):
    def __init__(self):
        super().__init__('lift_sim')
        self.up_pos = self.declare_parameter('up_position', 0.03).value
        self.tol = self.declare_parameter('tolerance', 0.002).value
        self.timeout = self.declare_parameter('timeout_s', 5.0).value
        self.load_hold = self.declare_parameter('load_hold_s', 0.5).value
        self.speed = self.declare_parameter('speed', 0.02).value     # m/s, khop lift_speed (URDF)
        self.period = 0.05
        self.lock_shelves = self.declare_parameter('lock_shelves', True).value
        self.contact_shelf = None  # ke dang cham mat nang (ten model trong Gazebo)
        self.locked = None         # ke dang khoa vao mat nang
        self.unlocked_all = False
        self.unlocking = False     # dang mo khoa, chua bat dau ha

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
            self.set_state('moving_down')
            threading.Thread(target=self.unlock_then_lower, args=(shelf,), daemon=True).start()
            return
        self.target = self.up_pos if cmd == 'up' else 0.0
        self.cmd_time = time.monotonic()
        self.set_state('moving_up' if cmd == 'up' else 'moving_down')

    def gz_joint(self, shelf, action):
        r = subprocess.run(['gz', 'topic', '-t', f'/shelf/{shelf}/{action}', '-m', 'gz.msgs.Empty', '-p', ' '],
                           capture_output=True, text=True, timeout=10)
        if r.returncode:
            self.get_logger().error(f'khong {action} duoc {shelf}: {r.stderr.strip()}')
        else:
            self.get_logger().info(f'{"khoa" if action == "attach" else "mo khoa"} {shelf}')

    def unlock_then_lower(self, shelf):
        self.gz_joint(shelf, 'detach')
        time.sleep(0.2)
        self.target = 0.0
        self.cmd_time = time.monotonic()
        self.unlocking = False

    def unlock_all(self):
        """Plugin noi khop moi ke voi xe ngay khi xe xuat hien: mo het (lap lai cho chac)."""
        time.sleep(2.0)
        out = subprocess.run(['gz', 'topic', '-l'], capture_output=True, text=True, timeout=10).stdout
        shelves = sorted(set(re.findall(r'^/shelf/([^/]+)/detach$', out, re.M)))
        # Goi song song: moi lenh gz mat ~1 s, goi lan luot 8 ke x 2 lan = 16 s thi xe da nhan don
        # va chay khi con dinh vao ke (lan thu s1)
        for _ in range(2):
            procs = [subprocess.Popen(['gz', 'topic', '-t', f'/shelf/{sh}/detach', '-m', 'gz.msgs.Empty',
                                       '-p', ' '], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                     for sh in shelves]
            for p in procs:
                p.wait(timeout=10)
        self.get_logger().info(f'da mo khoa {len(shelves)} ke luc khoi dong')

    def on_joints(self, msg):
        if 'lift_joint' in msg.name:
            self.pos = msg.position[msg.name.index('lift_joint')]
            if self.lock_shelves and not self.unlocked_all:
                self.unlocked_all = True       # xe da co trong Gazebo
                threading.Thread(target=self.unlock_all, daemon=True).start()

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
                threading.Thread(target=self.gz_joint, args=(self.locked, 'attach'), daemon=True).start()
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
