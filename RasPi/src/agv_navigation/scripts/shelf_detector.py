#!/usr/bin/env python3
"""Nhan dien ke mini bang lidar: tim 4 chan ke (hinh vuong) trong /scan_raw, phat tam ke.

  /scan_raw            sensor_msgs/LaserScan   (vao, scan chua loc: can thay ca chan ke)
  /detected_dock_pose  geometry_msgs/PoseStamped (ra, frame cua lidar, stamp = stamp cua scan)

docking_server (loai dock shelf_dock, use_external_detection_pose) doc topic nay de chui gam theo
ke THAT, khong theo vi tri o tren ban do: ke tra ve lech vai cm moi lan, lau dan se lech nhieu;
AMCL cung lech ~5 cm. Dung chung mo phong va xe that.

Cach tim (moi scan):
  1. Gom diem lien tiep cach nhau < cluster_gap thanh cum; giu cum nho (< max_leg_width): chan ke,
     khong phai tuong. Tam cum lui them nua be rong chan theo tia (lidar chi thay mat truoc).
  2. Tim 4 cum tao hinh vuong canh leg_spacing (sai so side_tol): 2 cap chan canh nhau + duong cheo.
  3. Chon hinh vuong gan xe nhat trong vung truoc xe (max_range). Huong = huong canh vuong gan voi
     huong xe nhat (xe dang huong vao ke), tuc yaw trong [-45, 45] deg so voi lidar.
Khong thay ke thi khong phat gi: docking_server tu bao "Lost detection" sau external_detection_timeout.
"""
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ShelfDetector(Node):
    def __init__(self):
        super().__init__('shelf_detector')
        # Ke 0.75 m, chan 0.03 m (agv_mission/config/shelves.yaml) -> tam chan cach nhau 0.72 m
        self.side = self.declare_parameter('leg_spacing', 0.72).value
        self.side_tol = self.declare_parameter('side_tolerance', 0.05).value
        self.leg_width = self.declare_parameter('leg_width', 0.03).value
        self.max_leg_width = self.declare_parameter('max_leg_width', 0.08).value
        self.cluster_gap = self.declare_parameter('cluster_gap', 0.05).value
        self.min_range = self.declare_parameter('min_range', 0.2).value     # bo tru oc cua xe
        self.max_range = self.declare_parameter('max_range', 2.5).value
        self.pub = self.create_publisher(PoseStamped, 'detected_dock_pose', 10)
        self.create_subscription(LaserScan, 'scan_raw', self.on_scan, qos_profile_sensor_data)
        self.found = None

    def legs(self, scan):
        """Tam cac cum diem nho (ung vien chan ke) trong frame lidar."""
        out, cur = [], []

        def close():
            if not cur:
                return
            xs, ys = [p[0] for p in cur], [p[1] for p in cur]
            width = math.hypot(xs[-1] - xs[0], ys[-1] - ys[0])
            if width < self.max_leg_width:
                cx, cy = sum(xs) / len(xs), sum(ys) / len(ys)
                d = math.hypot(cx, cy)
                # lidar chi thay mat truoc cua chan -> lui tam ra sau nua be rong chan
                k = (d + self.leg_width / 2) / d
                out.append((cx * k, cy * k))

        prev = None
        for i, r in enumerate(scan.ranges):
            if not (self.min_range < r < self.max_range + 1.0) or math.isinf(r) or math.isnan(r):
                close()
                cur, prev = [], None
                continue
            a = scan.angle_min + i * scan.angle_increment
            p = (r * math.cos(a), r * math.sin(a))
            if prev is not None and math.hypot(p[0] - prev[0], p[1] - prev[1]) > self.cluster_gap:
                close()
                cur = []
            cur.append(p)
            prev = p
        close()
        # Cum dau va cuoi cua scan 360 do co the la mot cum bi cat doi: bo qua truong hop hiem nay
        return out

    def squares(self, pts):
        s, tol = self.side, self.side_tol
        diag = s * math.sqrt(2.0)
        d = lambda a, b: math.hypot(a[0] - b[0], a[1] - b[1])   # noqa: E731
        n = len(pts)
        found = []
        for i in range(n):
            for j in range(i + 1, n):
                if abs(d(pts[i], pts[j]) - diag) > tol * 1.5:
                    continue
                # i, j la 2 dinh doi dien: 2 dinh con lai doi xung qua tam
                cx, cy = (pts[i][0] + pts[j][0]) / 2, (pts[i][1] + pts[j][1]) / 2
                hx, hy = (pts[j][0] - pts[i][0]) / 2, (pts[j][1] - pts[i][1]) / 2
                expect = [(cx - hy, cy + hx), (cx + hy, cy - hx)]
                match = []
                for e in expect:
                    best = min(range(n), key=lambda k: d(pts[k], e))
                    if best in (i, j) or d(pts[best], e) > tol:
                        break
                    match.append(best)
                if len(match) == 2 and match[0] != match[1]:
                    quad = [pts[i], pts[match[0]], pts[j], pts[match[1]]]
                    found.append(quad)
        return found

    def on_scan(self, scan):
        pts = [p for p in self.legs(scan) if math.hypot(*p) < self.max_range]
        best = None
        for quad in self.squares(pts):
            cx = sum(p[0] for p in quad) / 4
            cy = sum(p[1] for p in quad) / 4
            if cx < -0.3:          # chi xet ke o truoc xe (hoac xe dang o duoi ke)
                continue
            dist = math.hypot(cx, cy)
            if best is None or dist < best[0]:
                best = (dist, cx, cy, quad)
        if best is None:
            if self.found is not False:
                self.get_logger().info('khong thay ke')
                self.found = False
            return
        _, cx, cy, quad = best
        # Huong canh vuong: trung binh 4 canh dua ve [-45, 45) deg (doi xung 90 deg) -> gan huong xe
        angs = []
        for k in range(4):
            a, b = quad[k], quad[(k + 1) % 4]
            th = math.atan2(b[1] - a[1], b[0] - a[0])
            angs.append(4 * th)              # nhan 4 de goc 90 deg trung nhau
        yaw = math.atan2(sum(math.sin(a) for a in angs), sum(math.cos(a) for a in angs)) / 4
        if self.found is not True:
            self.get_logger().info(f'thay ke: tam ({cx:.2f}, {cy:.2f}) m, goc {math.degrees(yaw):.1f} deg')
            self.found = True
        msg = PoseStamped()
        msg.header = scan.header
        msg.pose.position.x, msg.pose.position.y = cx, cy
        msg.pose.orientation.z, msg.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = ShelfDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
