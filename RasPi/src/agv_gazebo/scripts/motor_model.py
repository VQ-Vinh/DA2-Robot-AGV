#!/usr/bin/env python3
"""Mo phong gioi han dong co cua xe that, dat giua /cmd_vel va DiffDrive cua Gazebo.

DiffDrive cua Gazebo dat dung toc do banh nhu duoc yeu cau, ke ca 0.01 m/s.
Xe that thi khong: firmware STM32 (SpeedCtrl_SetTarget) ep moi lenh khac 0
vao khoang [SPD_RPM_MIN, SPD_RPM_MAX] = [100, 270] RPM, vi duoi ~30 % duty
dong co GA25-370 khong quay noi. Node nay lam y het de Nav2 va EKF trong mo
phong gap dung nhung gi se gap tren xe that.

  /cmd_vel (Twist) -> toc do 2 ben banh (RPM) -> gioi han -> /cmd_vel_limited (Twist)

Cac buoc 1-2 la viec cua node cau noi Pi <-> STM32 (sau nay phai lam y het),
buoc 3 la firmware:
  1. Giu ban kinh cua (preserve_turning_radius, giong diff_drive_controller cua Husky):
     banh nao khac 0 ma duoi rpm_min thi nhan CA HAI banh cung ti le cho toi khi banh
     cham nhat dat rpm_min. Xe chay nhanh hon nhung van dung duong cong.
     Khong co buoc nay: Nav2 xin v=0.25, w=0.33 (52 / 95 RPM) -> firmware day ca hai
     len 100 RPM -> xe di thang, mat lai -> troi vao ke trong loi hep.
  2. Banh nao vuot rpm_max thi thu nho CA HAI ben cung ti le (cung giu ban kinh cua).
  3. Firmware: banh nao con khac 0 ma duoi rpm_min thi day len rpm_min.
     Vi du khi tat buoc 1: Nav2 xin 0.1 m/s -> 29 RPM -> 100 RPM -> xe chay 0.34 m/s.

Tham so (mac dinh khop xe that va firmware hien tai):
  wheel_radius  0.0325  m
  track         0.45    m, track hieu dung (effective_track trong URDF)
  rpm_min       100     dat 0 de tat vung chet, xem xe chay "ly tuong" ra sao
  rpm_max       270
  preserve_turning_radius  true   false = chi co gioi han cua firmware (buoc 2-3)
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class MotorModel(Node):

    def __init__(self):
        super().__init__('motor_model')
        self.r = self.declare_parameter('wheel_radius', 0.0325).value
        self.track = self.declare_parameter('track', 0.45).value
        self.rpm_min = self.declare_parameter('rpm_min', 100.0).value
        self.rpm_max = self.declare_parameter('rpm_max', 270.0).value
        self.preserve_radius = self.declare_parameter('preserve_turning_radius', True).value

        self.pub = self.create_publisher(Twist, 'cmd_vel_limited', 10)
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd, 10)
        self.get_logger().info(
            f'banh {self.rpm_min:.0f}-{self.rpm_max:.0f} RPM '
            f'(v banh {self.rpm_to_v(self.rpm_min):.2f}-{self.rpm_to_v(self.rpm_max):.2f} m/s)')

    def v_to_rpm(self, v):
        return v / self.r * 60.0 / (2.0 * math.pi)

    def rpm_to_v(self, rpm):
        return rpm * 2.0 * math.pi / 60.0 * self.r

    def limit(self, rpm_l, rpm_r):
        # Firmware nhan so nguyen: duoi 0.5 RPM la 0
        rpm_l = 0.0 if abs(rpm_l) < 0.5 else rpm_l
        rpm_r = 0.0 if abs(rpm_r) < 0.5 else rpm_r

        # 1. Giu ban kinh cua: day ca hai banh len cung ti le cho banh cham nhat dat rpm_min
        moving = [abs(x) for x in (rpm_l, rpm_r) if x != 0.0]
        if self.preserve_radius and moving and min(moving) < self.rpm_min:
            k = self.rpm_min / min(moving)
            rpm_l *= k
            rpm_r *= k

        # 2. Qua toc: thu nho ca hai ben cung ti le
        peak = max(abs(rpm_l), abs(rpm_r))
        if peak > self.rpm_max:
            rpm_l *= self.rpm_max / peak
            rpm_r *= self.rpm_max / peak

        # 3. Vung chet cua firmware: khac 0 ma nho thi day len rpm_min
        def deadband(rpm):
            rpm = round(rpm)
            if rpm == 0:
                return 0.0
            return math.copysign(max(abs(rpm), self.rpm_min), rpm)

        return deadband(rpm_l), deadband(rpm_r)

    def on_cmd(self, msg):
        v, w = msg.linear.x, msg.angular.z
        rpm_l = self.v_to_rpm(v - w * self.track / 2.0)
        rpm_r = self.v_to_rpm(v + w * self.track / 2.0)
        out_l, out_r = self.limit(rpm_l, rpm_r)

        v_l, v_r = self.rpm_to_v(out_l), self.rpm_to_v(out_r)
        out = Twist()
        out.linear.x = (v_l + v_r) / 2.0
        out.angular.z = (v_r - v_l) / self.track
        self.pub.publish(out)

        if abs(out_l - rpm_l) > 1.0 or abs(out_r - rpm_r) > 1.0:
            self.get_logger().debug(
                f'xin v={v:.2f} w={w:.2f} ({rpm_l:.0f}/{rpm_r:.0f} RPM) -> '
                f'v={out.linear.x:.2f} w={out.angular.z:.2f} ({out_l:.0f}/{out_r:.0f} RPM)')


def main():
    rclpy.init()
    node = MotorModel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
