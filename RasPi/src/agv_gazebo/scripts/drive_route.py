#!/usr/bin/env python3
"""Tu lai xe qua cac loi di trong kho de lap ban do (chi dung trong mo phong).

Doc vi tri that /ground_truth (frame world), gui /cmd_vel - lenh van di qua
motor_model nen xe chiu dung gioi han dong co. Tren xe that thi lai tay
(teleop) de lap ban do, khong dung node nay.

  ros2 run agv_gazebo drive_route.py
  ros2 run agv_gazebo drive_route.py --ros-args -p speed:=0.5

Cach lai don gian: lech huong nhieu thi quay tai cho, lech it thi vua di vua
chinh huong. Quay cham (turn_speed 0.5 rad/s, toi da 1.0 nhu Husky cua Clearpath)
vi lidar 7 Hz: quay nhanh thi SLAM ghep scan lech goc.

Khi bat vung chet dong co (rpm_min 100), lenh quay nao cung thanh >= 1.5 rad/s va
phanh mat ~0.4 rad, nen phai dung quay som:
  --ros-args -p rot_start:=0.6 -p rot_stop:=0.45
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

# Lo trinh (x, y) trong frame world cua warehouse.sdf, xuat phat mac dinh (-4.5, 0).
# Ke: x -2.7..0.3 va 1.3..4.3, hang y = +-0.8, +-2.4 (rong 0.6 m).
# Loi doc: y = 0, +-1.6, +-3.4. Loi ngang: x = 0.8 (giua 2 day ke), x = 5.2 (phia dong).
ROUTE = [
    (-4.5, -3.4),   # xuong goc tay nam
    (0.8, -3.4),    # loi duoi cung, sang giua kho
    (0.8, -1.6),
    (5.2, -1.6),    # loi giua hang C-D, sang phia dong (giua 2 pallet)
    (5.2, 1.6),
    (0.8, 1.6),     # loi giua hang A-B
    (0.8, 3.4),
    (-4.5, 3.4),    # loi tren cung, ve phia tay
    (-4.5, 0.0),
    (5.2, 0.0),     # loi chinh y = 0, xuyen ca kho
    (-4.5, 0.0),    # quay ve cho xuat phat -> SLAM khep vong (loop closure)
]


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class DriveRoute(Node):

    def __init__(self):
        super().__init__('drive_route')
        self.speed = self.declare_parameter('speed', 0.4).value             # m/s khi di thang
        self.reach = self.declare_parameter('reach_tolerance', 0.15).value  # m
        self.turn_speed = min(self.declare_parameter('turn_speed', 0.5).value, 1.0)  # rad/s
        self.rot_start = self.declare_parameter('rot_start', 0.3).value     # rad: lech hon -> quay tai cho
        self.rot_stop = self.declare_parameter('rot_stop', 0.08).value      # rad: thoi quay, de xe tu phanh
        self.wp_timeout = self.declare_parameter('waypoint_timeout', 90.0).value

        self.pose = None
        self.idx = 0
        self.rotating = False
        self.wp_start = None
        self.done = False

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Odometry, 'ground_truth', self.on_pose, 10)
        self.create_timer(0.05, self.step)

    def on_pose(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_of(p.orientation))

    def send(self, v, w):
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.pub.publish(cmd)

    def step(self):
        if self.pose is None or self.done:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.wp_start is None:
            self.wp_start = now
            self.get_logger().info(f'diem {self.idx + 1}/{len(ROUTE)}: {ROUTE[self.idx]}')

        x, y, yaw = self.pose
        tx, ty = ROUTE[self.idx]
        dist = math.hypot(tx - x, ty - y)
        err = wrap(math.atan2(ty - y, tx - x) - yaw)

        timed_out = now - self.wp_start > self.wp_timeout
        if dist < self.reach or timed_out:
            if timed_out:
                self.get_logger().warn(f'khong toi duoc {ROUTE[self.idx]} sau {self.wp_timeout:.0f} s, bo qua')
            self.idx += 1
            self.wp_start = None
            self.rotating = False
            if self.idx >= len(ROUTE):
                self.send(0.0, 0.0)
                self.done = True
                self.get_logger().info('xong lo trinh')
            return

        if abs(err) > self.rot_start:
            self.rotating = True
        elif abs(err) < self.rot_stop:
            self.rotating = False

        if self.rotating:
            self.send(0.0, math.copysign(self.turn_speed, err))
        else:
            # Gan diem thi bot chinh huong de khong lac vong quanh diem
            w = max(-0.6, min(0.6, 2.0 * err)) if dist > 0.3 else 0.0
            self.send(self.speed, w)


def main():
    rclpy.init()
    node = DriveRoute()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.send(0.0, 0.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
