#!/usr/bin/env python3
"""Thu van chuyen ke kieu Kiva trong mo phong (Nav2 + docking_server), do sai so bang vi tri that.

  ros2 launch agv_gazebo sim.launch.py nav:=true
  ros2 run agv_gazebo shelf_transport_test.py --ros-args -p shelf:=ke_03 -p station:=tram_lay_hang
  # ke dat lech khoi tam o (dx m, dy m, dyaw deg) nhu sau nhieu lan tra ke; detect:=false de so voi
  # cach chui gam theo vi tri o tren ban do
  ... -p shelf_offset:="[0.08, -0.06, 4.0]" -p detect:=false

Chuoi buoc (giong mot don hang, giai doan sau se do order_manager lam):
  toi staging (NavigateToPose, do sai so) -> chui gam (DockRobot) -> nang -> lui ra (UndockRobot)
  -> cho ke toi tram (NavigateToPose) -> doi -> toi staging -> tra ke ve o (DockRobot) -> ha -> lui ra
  -> ve tram sac.
Do: sai so chui gam (xe so voi tam o, theo /ground_truth), vi tri ke sau khi tra (gz model),
thoi gian tung buoc. In dong "KET QUA" cuoi cung.

Chi dung trong mo phong (/ground_truth, lenh gz). Frame map = world + (4.5, 0).
"""
import math
import os
import re
import subprocess
import time

import rclpy
from action_msgs.msg import GoalStatus
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import DockRobot, NavigateThroughPoses, NavigateToPose, UndockRobot
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import Bool, String

MAP_X, MAP_Y = 4.5, 0.0


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class ShelfTest(Node):
    def __init__(self):
        super().__init__('shelf_transport_test')
        cfg = os.path.join(get_package_share_directory('agv_mission'), 'config')
        self.layout = yaml.safe_load(open(os.path.join(cfg, 'shelves.yaml')))
        self.stations = yaml.safe_load(open(os.path.join(cfg, 'stations.yaml')))['stations']
        self.shelf = self.declare_parameter('shelf', 'ke_03').value
        self.station = self.declare_parameter('station', 'tram_lay_hang').value
        self.wait_s = self.declare_parameter('wait_s', 3.0).value
        # Doan thang truoc staging (m), xem staging()
        self.straight = self.declare_parameter('straight_approach', 1.2).value
        self.slot = self.layout['shelves'][self.shelf]
        # Lay ke: shelf_dock (theo chan ke shelf_detector tim) hoac slot_dock (theo o tren ban do)
        self.detect = self.declare_parameter('detect', True).value
        self.offset = list(self.declare_parameter('shelf_offset', [0.0, 0.0, 0.0]).value)
        self.dock_type = None

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.lift_state, self.has_load, self.gt, self.localized = None, None, None, False
        self.create_subscription(String, 'lift/state', lambda m: setattr(self, 'lift_state', m.data), latched)
        self.create_subscription(Bool, 'lift/has_load', lambda m: setattr(self, 'has_load', m.data), latched)
        self.create_subscription(Odometry, 'ground_truth', self.on_gt, qos_profile_sensor_data)
        # AMCL phat amcl_pose kieu giu tin cuoi va chi khi xe di chuyen -> phai dung QoS giu tin,
        # khong thi node bat sau AMCL se cho mai
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose',
                                 lambda m: setattr(self, 'localized', True), latched)
        self.lift_pub = self.create_publisher(String, 'lift/command', 10)
        self.dock = ActionClient(self, DockRobot, 'dock_robot')
        self.undock = ActionClient(self, UndockRobot, 'undock_robot')
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_through = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        self.state_cli = self.create_client(GetState, 'bt_navigator/get_state')
        self.steps = []

    def on_gt(self, m):
        p = m.pose.pose
        self.gt = (p.position.x + MAP_X, p.position.y + MAP_Y, yaw_of(p.orientation))

    def spin(self, sec):
        t0 = time.time()
        while time.time() - t0 < sec:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for(self, cond, timeout):
        t0 = time.time()
        while not cond() and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
        return cond()

    def wait_nav2(self):
        def active():
            if not self.state_cli.service_is_ready():
                return False
            f = self.state_cli.call_async(GetState.Request())
            rclpy.spin_until_future_complete(self, f, timeout_sec=2.0)
            return f.result() is not None and f.result().current_state.id == State.PRIMARY_STATE_ACTIVE
        t0 = time.time()
        while time.time() - t0 < 120:
            if active() and self.localized and self.dock.server_is_ready():
                return True
            self.spin(1.0)
        return False

    def run_action(self, client, goal, timeout):
        """Tra ve (thanh cong?, result). Thanh cong = trang thai SUCCEEDED (khong chi la co ket qua)."""
        f = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, f, timeout_sec=10.0)
        h = f.result()
        if h is None or not h.accepted:
            return False, None
        r = h.get_result_async()
        rclpy.spin_until_future_complete(self, r, timeout_sec=timeout)
        if not r.done():
            h.cancel_goal_async()
            return False, None
        return r.result().status == GoalStatus.STATUS_SUCCEEDED, r.result().result

    def step(self, name, fn):
        t0 = time.time()
        ok, info = fn()
        dt = time.time() - t0
        self.steps.append((name, ok, dt, info))
        self.get_logger().info(f'{name}: {"OK" if ok else "LOI"} {dt:.1f} s {info}')
        return ok

    def error_to(self, px, py, pyaw):
        """Sai so cua xe (vi tri that) so voi (px, py, pyaw): doc truc, ngang truc, goc."""
        x, y, yaw = self.gt
        c, s = math.cos(pyaw), math.sin(pyaw)
        along, across = (x - px) * c + (y - py) * s, -(x - px) * s + (y - py) * c
        return (f'lech doc {100 * along:+.1f} cm, ngang {100 * across:+.1f} cm, '
                f'goc {math.degrees(wrap(yaw - pyaw)):+.1f} deg')

    def pose_stamped(self, x, y, yaw):
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.pose.position.x, p.pose.position.y = float(x), float(y)
        p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        return p

    def staging(self):
        """Toi staging (truoc o staging_offset, huong vao o) qua diem pre-staging xa hon tren cung
        truc o: doan cuoi la duong thang nen xe toi noi gan dung huong, khong phai quay tai cho
        (quay voi vung chet dong co thi lo toi 30-50 deg). Do sai so tai staging."""
        sl = self.layout['slots'][self.slot]
        d = self.layout['shelf']['staging_offset']
        c, s = math.cos(sl['yaw']), math.sin(sl['yaw'])
        px, py = sl['x'] - d * c, sl['y'] - d * s
        g = NavigateThroughPoses.Goal()
        # Doan thang 1.2 m (> lookahead toi da 0.9 m cua RPP): 0.7 m chua du, xe toi tu huong cheo
        # van lech ~37 deg (REPORT.md)
        pre = d + self.straight
        g.poses = [self.pose_stamped(sl['x'] - pre * c, sl['y'] - pre * s, sl['yaw']),
                   self.pose_stamped(px, py, sl['yaw'])]
        ok, _ = self.run_action(self.nav_through, g, 180.0)
        self.spin(1.0)
        return ok, self.error_to(px, py, sl['yaw'])

    def place_shelf(self):
        """Dat ke lech khoi tam o (shelf_offset), qua dich vu set_pose cua Gazebo."""
        dx, dy, dyaw = self.offset
        if not any(self.offset):
            return
        sl = self.layout['slots'][self.slot]
        x, y, yaw = sl['x'] - MAP_X + dx, sl['y'] - MAP_Y + dy, math.radians(dyaw)
        req = (f'name: "{self.shelf}", position: {{x: {x}, y: {y}, z: 0.0}}, '
               f'orientation: {{z: {math.sin(yaw / 2)}, w: {math.cos(yaw / 2)}}}')
        subprocess.run(['gz', 'service', '-s', '/world/warehouse/set_pose', '--reqtype', 'gz.msgs.Pose',
                        '--reptype', 'gz.msgs.Boolean', '--timeout', '3000', '--req', req],
                       capture_output=True, text=True)
        self.spin(2.0)
        self.get_logger().info(f'dat {self.shelf} lech o {100 * dx:+.0f}, {100 * dy:+.0f} cm, {dyaw:+.0f} deg')

    def error_to_shelf(self):
        """Sai so xe so voi ke THAT (khong phai tam o): doc truc, ngang truc, goc (ke vuong: mod 90 deg)."""
        sp = self.shelf_pose()
        if sp is None:
            return 'khong doc duoc vi tri ke'
        sl = self.layout['slots'][self.slot]
        x, y, yaw = self.gt
        c, s = math.cos(sl['yaw']), math.sin(sl['yaw'])
        along, across = (x - sp[0]) * c + (y - sp[1]) * s, -(x - sp[0]) * s + (y - sp[1]) * c
        dyaw = wrap(4 * (yaw - sp[3])) / 4
        return f'so voi ke: doc {100 * along:+.1f} cm, ngang {100 * across:+.1f} cm, goc {math.degrees(dyaw):+.1f} deg'

    def dock_goal(self, detect):
        g = DockRobot.Goal()
        g.navigate_to_staging_pose = False      # da tu toi staging (buoc truoc)
        if detect:
            g.use_dock_id, g.dock_id = True, f'slot_{self.slot}'      # loai shelf_dock (co nhan dien)
            self.dock_type = 'shelf_dock'
        else:
            sl = self.layout['slots'][self.slot]
            g.use_dock_id, g.dock_type = False, 'slot_dock'
            g.dock_pose = self.pose_stamped(sl['x'], sl['y'], sl['yaw'])
            self.dock_type = 'slot_dock'
        return g

    def dock_slot(self, detect=False, vs_shelf=False):
        _, r = self.run_action(self.dock, self.dock_goal(detect), 120.0)
        self.spin(1.0)
        sl = self.layout['slots'][self.slot]
        info = self.error_to(sl['x'], sl['y'], sl['yaw']) + (' | ' + self.error_to_shelf() if vs_shelf else '')
        if r is None:
            return False, 'het gio / bi tu choi | ' + info
        return bool(r.success), (f'error_code {r.error_code}, thu lai {r.num_retries} | ' if not r.success
                                 else f'thu lai {r.num_retries} | ') + info

    def lift_and_measure(self):
        ok, info = self.lift('up')
        sp = self.shelf_pose()
        if sp is not None and self.gt is not None:
            info += f', ke lech tam xe {100 * math.hypot(sp[0] - self.gt[0], sp[1] - self.gt[1]):.1f} cm'
        return ok, info

    def undock_slot(self):
        g = UndockRobot.Goal()
        g.dock_type, g.max_undocking_time = self.dock_type, 30.0
        ok, r = self.run_action(self.undock, g, 60.0)
        return ok and bool(r.success), ('' if r is None else f'error_code {r.error_code}')

    def lift(self, cmd):
        want_state, want_load = ('up', True) if cmd == 'up' else ('down', False)
        self.lift_pub.publish(String(data=cmd))
        self.spin(0.3)
        ok = self.wait_for(lambda: self.lift_state == want_state and self.has_load == want_load, 8.0)
        return ok, f'state {self.lift_state}, has_load {self.has_load}'

    def goto_pose(self, x, y, yaw):
        g = NavigateToPose.Goal()
        g.pose = self.pose_stamped(x, y, yaw)
        ok, r = self.run_action(self.nav, g, 180.0)
        return ok, ('' if ok or r is None else f'error_code {r.error_code}')

    def goto(self, name):
        st = self.stations[name]
        return self.goto_pose(st['x'], st['y'], st['yaw'])

    def shelf_pose(self):
        out = subprocess.run(['gz', 'model', '-m', self.shelf, '-p'], capture_output=True, text=True).stdout
        num = r'-?\d+(?:\.\d+)?(?:e-?\d+)?'
        groups = re.findall(rf'\[\s*({num}\s+{num}\s+{num})\s*\]', out)
        if len(groups) < 2:
            return None
        (x, y, z), (_, _, yaw) = [[float(v) for v in g.split()] for g in groups[:2]]
        return x + MAP_X, y + MAP_Y, z, yaw

    def run(self):
        if not self.wait_nav2():
            self.get_logger().error('Nav2 / docking_server khong san sang')
            return
        self.place_shelf()
        self.get_logger().info(f'Lay {self.shelf} o {self.slot} -> {self.station} -> tra ve '
                               f'({"nhan dien chan ke" if self.detect else "theo o tren ban do"})')
        t0 = time.time()
        ok = (self.step('toi staging', self.staging)
              and self.step('chui gam lay ke', lambda: self.dock_slot(self.detect, vs_shelf=True))
              and self.step('nang ke', self.lift_and_measure)
              and self.step('lui ra (co ke)', self.undock_slot)
              and self.step(f'cho ke toi {self.station}', lambda: self.goto(self.station)))
        if ok:
            self.spin(self.wait_s)
            ok = (self.step('toi staging (co ke)', self.staging)
                  and self.step('tra ke vao o', lambda: self.dock_slot(False))
                  and self.step('ha ke', lambda: self.lift('down'))
                  and self.step('lui ra (khong ke)', self.undock_slot))
        sp = self.shelf_pose()
        sl = self.layout['slots'][self.slot]
        place = (f'ke cach tam o {100 * math.hypot(sp[0] - sl["x"], sp[1] - sl["y"]):.1f} cm, '
                 f'z {100 * sp[2]:.1f} cm' if sp else 'khong doc duoc vi tri ke')
        if ok:
            self.step('ve tram sac', lambda: self.goto('tram_sac'))
        n_ok = sum(1 for _, good, _, _ in self.steps if good)
        print(f'KET QUA {self.shelf}: {n_ok}/{len(self.steps)} buoc OK, tong {time.time() - t0:.1f} s, {place}',
              flush=True)
        for name, good, dt, info in self.steps:
            print(f'  {name:24s} {"OK " if good else "LOI"} {dt:6.1f} s  {info}', flush=True)


def main():
    rclpy.init()
    node = ShelfTest()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
