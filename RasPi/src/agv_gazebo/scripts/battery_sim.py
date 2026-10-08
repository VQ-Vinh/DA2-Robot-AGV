#!/usr/bin/env python3
"""Pin trong mo phong, cung giao dien voi xe that (STM32 doc INA226 qua cau noi UART):

  /battery_state  sensor_msgs/BatteryState  1 Hz: percentage 0..1, voltage, current (A, + = dang sac),
                                            power_supply_status CHARGING / DISCHARGING

Mo hinh don gian (khong phai dien hoa that), du de thu chinh sach sac cua order_manager:
  - pin 3S Li-ion: dien ap 9.9 V (0 %) .. 12.6 V (100 %), tuyen tinh
  - hao: idle_draw + move_draw * |v| (+ lift_draw khi dang cho ke), %/phut
  - sac khi xe cham tiep diem tram sac: xe (vi tri that /ground_truth) cach diem dock < contact_tol
    va lech goc < contact_yaw_tol, sac charge_rate %/phut, dong +charge_current A
  - time_scale: tang toc do hao / sac de thu nhanh (1 = thoi gian that)
Khong dung use_sim_time (dem thoi gian may), tranh /clock 333 tin/s (REPORT.md 4.8).
"""
import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class BatterySim(Node):
    def __init__(self):
        super().__init__('battery_sim')
        self.soc = self.declare_parameter('initial_percentage', 0.9).value
        self.idle_draw = self.declare_parameter('idle_draw', 0.3).value       # %/phut (Pi + STM32 + lidar)
        self.move_draw = self.declare_parameter('move_draw', 2.0).value       # %/phut tai 1 m/s
        self.lift_draw = self.declare_parameter('lift_draw', 0.5).value       # %/phut khi cho ke
        self.charge_rate = self.declare_parameter('charge_rate', 2.0).value   # %/phut
        self.charge_current = self.declare_parameter('charge_current', 2.0).value
        self.time_scale = self.declare_parameter('time_scale', 1.0).value
        # Diem dock cua tram sac (frame world): tram_sac map (-0.45, 0, pi) (stations.yaml) = world (-4.95, 0)
        self.dock = self.declare_parameter('dock_world_pose', [-4.95, 0.0, 3.14159]).value
        self.contact_tol = self.declare_parameter('contact_tolerance', 0.06).value
        self.contact_yaw_tol = self.declare_parameter('contact_yaw_tolerance', 0.2).value

        self.speed = 0.0
        self.pose = None
        self.carrying = False
        self.pub = self.create_publisher(BatteryState, 'battery_state', 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.create_subscription(Odometry, 'ground_truth', self.on_gt, qos_profile_sensor_data)
        self.create_subscription(Bool, 'payload/carrying', lambda m: setattr(self, 'carrying', m.data),
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.last = time.monotonic()
        self.was_charging = None
        self.create_timer(1.0, self.tick)

    def on_odom(self, m):
        self.speed = math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y)

    def on_gt(self, m):
        p = m.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_of(p.orientation))

    def on_contacts(self):
        if self.pose is None:
            return False
        x, y, yaw = self.pose
        dyaw = math.atan2(math.sin(yaw - self.dock[2]), math.cos(yaw - self.dock[2]))
        return math.hypot(x - self.dock[0], y - self.dock[1]) < self.contact_tol and abs(dyaw) < self.contact_yaw_tol

    def tick(self):
        now = time.monotonic()
        minutes = (now - self.last) / 60.0 * self.time_scale
        self.last = now
        charging = self.on_contacts()
        if charging:
            self.soc = min(1.0, self.soc + self.charge_rate / 100.0 * minutes)
            current = self.charge_current if self.soc < 1.0 else 0.6     # dong nho khi day (giu > nguong 0.5 A)
        else:
            draw = self.idle_draw + self.move_draw * self.speed + (self.lift_draw if self.carrying else 0.0)
            self.soc = max(0.0, self.soc - draw / 100.0 * minutes)
            current = -(0.8 + 2.5 * self.speed + (0.4 if self.carrying else 0.0))
        if charging != self.was_charging:
            self.get_logger().info(f'{"dang sac" if charging else "khong sac"}, pin {100 * self.soc:.1f} %')
            self.was_charging = charging
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.percentage = float(self.soc)
        msg.voltage = float(9.9 + 2.7 * self.soc)
        msg.current = float(current)
        msg.power_supply_status = (BatteryState.POWER_SUPPLY_STATUS_CHARGING if charging
                                   else BatteryState.POWER_SUPPLY_STATUS_DISCHARGING)
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        msg.present = True
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = BatterySim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
