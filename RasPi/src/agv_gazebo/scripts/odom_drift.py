#!/usr/bin/env python3
"""Do sai so cua odom so voi vi tri that (ground truth) trong mo phong.

Lai xe mot luc roi Ctrl+C, node in bang tong ket: sai so vi tri, sai so goc,
va sai so tinh theo % quang duong. Dung de so sanh odom banh xe voi EKF (buoc 2).

  ros2 run agv_gazebo odom_drift.py                                  # /odom
  ros2 run agv_gazebo odom_drift.py --ros-args -p odom_topic:=/odometry/filtered

Hai nguon duoc dong bo ve cung goc toa do o mau dau tien, sau do ghep tung mau
odom voi mau ground truth gan nhat theo thoi gian (ca hai deu dung sim time).
"""
import bisect
import math
from collections import deque

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def stamp_s(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def pose2d(msg):
    p = msg.pose.pose
    return p.position.x, p.position.y, yaw_of(p.orientation)


class OdomDrift(Node):

    def __init__(self):
        super().__init__('odom_drift')
        self.odom_topic = self.declare_parameter('odom_topic', '/odom').value
        self.gt_topic = self.declare_parameter('ground_truth_topic', '/ground_truth').value
        self.period = self.declare_parameter('print_period', 2.0).value

        self.gt = deque(maxlen=300)   # (t, x, y, yaw), ~10 s o 30 Hz
        self.origin = None            # (odom0, gt0) luc bat dau
        self.dist = 0.0               # quang duong that, m
        self.last_gt_xy = None
        self.err = None               # (dx_m, dyaw_rad) moi nhat
        self.max_pos = 0.0
        self.max_yaw = 0.0

        self.create_subscription(Odometry, self.gt_topic, self.on_gt, 50)
        self.create_subscription(Odometry, self.odom_topic, self.on_odom, 50)
        self.create_timer(self.period, self.report)
        self.get_logger().info(f'so sanh {self.odom_topic} voi {self.gt_topic}')

    def on_gt(self, msg):
        x, y, yaw = pose2d(msg)
        self.gt.append((stamp_s(msg), x, y, yaw))
        if self.origin is not None and self.last_gt_xy is not None:
            self.dist += math.hypot(x - self.last_gt_xy[0], y - self.last_gt_xy[1])
        self.last_gt_xy = (x, y)

    def nearest_gt(self, t):
        times = [g[0] for g in self.gt]
        i = bisect.bisect_left(times, t)
        best = min((j for j in (i - 1, i) if 0 <= j < len(self.gt)),
                   key=lambda j: abs(times[j] - t))
        return self.gt[best]

    def on_odom(self, msg):
        if not self.gt:
            return
        t = stamp_s(msg)
        g = self.nearest_gt(t)
        if abs(g[0] - t) > 0.1:
            return
        o = pose2d(msg)

        if self.origin is None:
            self.origin = (o, g[1:])
            self.last_gt_xy = (g[1], g[2])
            return

        # Dua odom ve toa do world: xoay + tinh tien sao cho mau dau trung ground truth
        (ox0, oy0, oyaw0), (gx0, gy0, gyaw0) = self.origin
        rot = gyaw0 - oyaw0
        dx, dy = o[0] - ox0, o[1] - oy0
        wx = gx0 + dx * math.cos(rot) - dy * math.sin(rot)
        wy = gy0 + dx * math.sin(rot) + dy * math.cos(rot)
        wyaw = o[2] + rot

        e_pos = math.hypot(wx - g[1], wy - g[2])
        e_yaw = wrap(wyaw - g[3])
        self.err = (e_pos, e_yaw)
        self.max_pos = max(self.max_pos, e_pos)
        self.max_yaw = max(self.max_yaw, abs(e_yaw))

    def report(self):
        if self.err is None:
            self.get_logger().info(f'cho du lieu tu {self.odom_topic} va {self.gt_topic}...')
            return
        e_pos, e_yaw = self.err
        pct = 100.0 * e_pos / self.dist if self.dist > 0.05 else 0.0
        self.get_logger().info(
            f'quang duong {self.dist:6.2f} m | lech vi tri {e_pos:5.3f} m ({pct:4.1f} %) | '
            f'lech goc {math.degrees(e_yaw):6.1f} deg')

    def summary(self):
        if self.err is None:
            print('Chua co du lieu.')
            return
        e_pos, e_yaw = self.err
        pct = 100.0 * e_pos / self.dist if self.dist > 0.05 else 0.0
        print(f'\n=== {self.odom_topic} so voi {self.gt_topic} ===')
        print(f'Quang duong that : {self.dist:.2f} m')
        print(f'Lech vi tri cuoi : {e_pos:.3f} m ({pct:.1f} % quang duong), lon nhat {self.max_pos:.3f} m')
        print(f'Lech goc cuoi    : {math.degrees(e_yaw):.1f} deg, lon nhat {math.degrees(self.max_yaw):.1f} deg')


def main():
    rclpy.init()
    node = OdomDrift()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.summary()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
