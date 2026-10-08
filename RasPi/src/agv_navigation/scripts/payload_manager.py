#!/usr/bin/env python3
"""Thay doi theo tai cua xe kieu Kiva (dung chung mo phong va xe that).

Doc trang thai co cau nang (/lift/state, /lift/has_load) va:
  1. Loc scan /scan_raw -> /scan: bo diem roi vao than xe (4 tru oc giua 2 tang che lidar), va
     khi dang cho ke thi bo ca diem trong khung ke (4 chan ke di theo xe, khong phai vat can).
     Giong LaserScanBoxFilter cua laser_filters (diem bi loc = NaN, AMCL / SLAM / costmap bo qua),
     nhung doi khung loc theo tai nen tu viet.
  2. Doi footprint cua 2 costmap: xe 0.32 x 0.38 m <-> ke 0.88 (doc) x 0.78 (ngang) m.
  3. Gioi han toc do Nav2 khi cho ke (/speed_limit).
  4. Khi mat nang da len het ("up"): xoa 2 costmap. Luc mat nang con ha, lidar da danh dau 4 chan
     ke vao costmap; sau khi nang, diem chan ke bi loc (NaN) nen khong tia nao xoa duoc -> "chan ke
     ma" nam trong footprint moi, docking_server bao va cham khi lui ra. Xoa ngay luc bat dau nang
     thi chua du: costmap van giu scan cuoi chua loc va danh dau lai ngay sau khi xoa (REPORT.md).
Phat /payload/carrying (Bool, giu tin cuoi) cho cac node khac.

Dang cho = co ke tren mat nang, hoac mat nang khong o vi tri ha (dang nang / ha / da nang).
"""
import math

import rclpy
from geometry_msgs.msg import Point32, Polygon
from nav2_msgs.msg import SpeedLimit
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String


def rect(hx, hy):
    return Polygon(points=[Point32(x=hx, y=hy), Point32(x=hx, y=-hy),
                           Point32(x=-hx, y=-hy), Point32(x=-hx, y=hy)])


class PayloadManager(Node):
    def __init__(self):
        super().__init__('payload_manager')
        # Khung tinh trong frame laser (lidar dat giua xe): than xe gom 4 tru oc o +-0.135 m
        self.body = self.declare_parameter('body_half', [0.17, 0.17]).value
        # Ke 0.75 m (agv_mission/config/shelves.yaml), chan ngoai cung o 0.375 m. Ke nam lech tren
        # mat nang toi ~8 cm (AMCL lech khi chui gam) -> 0.375 + 0.1 (0.41 da thu: chan ke lot ra)
        self.shelf = self.declare_parameter('shelf_half', [0.48, 0.48]).value
        self.robot_fp = self.declare_parameter('robot_footprint_half', [0.16, 0.19]).value
        # Footprint khi cho (nua doc, nua ngang): ke 0.75 m; nhan dien chan ke cho lech ngang <= 1 cm nhung
        # xe dung qua tam ke 4-6 cm theo chieu doc (REPORT.md 4.10) -> doc 0.375 + 0.065, ngang 0.375 + 0.015.
        # Vuong 0.42 x 0.42 truoc day lam khe toi ke ben canh khi tra ke chi con ~17 cm (REPORT.md 4.13).
        self.shelf_fp = self.declare_parameter('shelf_footprint_half', [0.44, 0.39]).value
        self.carry_speed = self.declare_parameter('carry_speed', 0.35).value   # m/s

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        self.fp_pubs = [self.create_publisher(Polygon, t, 10)
                        for t in ('local_costmap/footprint', 'global_costmap/footprint')]
        self.speed_pub = self.create_publisher(SpeedLimit, 'speed_limit', 10)
        self.carry_pub = self.create_publisher(Bool, 'payload/carrying', latched)
        self.clear_clients = [self.create_client(ClearEntireCostmap, srv) for srv in (
            'local_costmap/clear_entirely_local_costmap', 'global_costmap/clear_entirely_global_costmap')]
        self.create_subscription(LaserScan, 'scan_raw', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(String, 'lift/state', self.on_lift_state, latched)
        self.create_subscription(Bool, 'lift/has_load', self.on_load, latched)

        self.lift_state = 'down'
        self.has_load = False
        self.carrying = None
        self.cache = None          # (so tia, angle_min, increment) -> (cos, sin)
        self.update()
        # Costmap co the khoi dong sau node nay: gui lai footprint dinh ky
        self.create_timer(2.0, self.publish_footprint)

    def on_lift_state(self, msg):
        self.lift_state = msg.data
        self.update()
        if msg.data == 'up':
            self.clear_costmaps()

    def clear_costmaps(self):
        for c in self.clear_clients:
            if c.service_is_ready():
                c.call_async(ClearEntireCostmap.Request())
            else:
                self.get_logger().warn(f'chua co dich vu {c.srv_name}, khong xoa duoc costmap')

    def on_load(self, msg):
        self.has_load = msg.data
        self.update()

    def update(self):
        carrying = self.has_load or self.lift_state != 'down'
        if carrying == self.carrying:
            return
        self.carrying = carrying
        self.carry_pub.publish(Bool(data=carrying))
        self.publish_footprint()
        # speed_limit = 0 nghia la bo gioi han
        self.speed_pub.publish(SpeedLimit(percentage=False,
                                          speed_limit=self.carry_speed if carrying else 0.0))
        self.get_logger().info('dang cho ke: footprint ke, gioi han toc do' if carrying
                               else 'khong cho ke: footprint xe, bo gioi han toc do')

    def publish_footprint(self):
        fp = rect(*(self.shelf_fp if self.carrying else self.robot_fp))
        for p in self.fp_pubs:
            p.publish(fp)

    def on_scan(self, scan):
        n = len(scan.ranges)
        key = (n, scan.angle_min, scan.angle_increment)
        if self.cache is None or self.cache[0] != key:
            angles = [scan.angle_min + i * scan.angle_increment for i in range(n)]
            self.cache = (key, [math.cos(a) for a in angles], [math.sin(a) for a in angles])
        _, cos_a, sin_a = self.cache
        hx, hy = self.shelf if self.carrying else self.body
        nan = float('nan')
        ranges = list(scan.ranges)
        for i, r in enumerate(ranges):
            if r < 0.7 and abs(r * cos_a[i]) < hx and abs(r * sin_a[i]) < hy:   # 0.7 = nua duong cheo khung loc (0.68)
                ranges[i] = nan
        scan.ranges = ranges
        self.scan_pub.publish(scan)


def main():
    rclpy.init()
    node = PayloadManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
