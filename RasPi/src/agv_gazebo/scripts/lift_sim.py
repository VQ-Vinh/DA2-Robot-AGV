#!/usr/bin/env python3
"""Co cau nang trong mo phong, cung giao dien topic voi xe that (cau noi UART <-> STM32):

  /lift/command   std_msgs/String  "up" | "down"           (vao)
  /lift/state     std_msgs/String  down | moving_up | up | moving_down | error   (ra, giu tin cuoi)
  /lift/has_load  std_msgs/Bool    co ke tren mat nang     (ra, giu tin cuoi)

Ben trong: gui vi tri khop lift_joint cho JointPositionController cua Gazebo (/lift/cmd_pos), tang
dan voi toc do speed (vit me: toc do gan nhu khong doi bat ke tai, khong nhay thang toi dich),
doc vi tri that tu /joint_states, "co ke" tu cam bien tiep xuc mat nang (/lift/contact).
Qua timeout_s chua toi vi tri (ket co khi, qua tai) -> state "error", giong firmware se bao.

Khong dung use_sim_time (chi dem thoi gian may): tranh nhan /clock 333 tin/s (REPORT.md 4.8).
"""
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
        self.target = self.up_pos if cmd == 'up' else 0.0
        self.cmd_time = time.monotonic()
        self.set_state('moving_up' if cmd == 'up' else 'moving_down')

    def on_joints(self, msg):
        if 'lift_joint' in msg.name:
            self.pos = msg.position[msg.name.index('lift_joint')]

    def on_contact(self, msg):
        if msg.contacts:
            self.last_contact = time.monotonic()

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
        if self.pos is None or self.state not in ('moving_up', 'moving_down'):
            return
        if abs(self.pos - self.target) < self.tol:
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
