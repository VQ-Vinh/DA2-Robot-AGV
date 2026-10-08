#!/usr/bin/env python3
"""Dieu phoi nhiem vu kho: di qua chuoi vi tri co ten bang Nav2 (NavigateToPose).

Hai cach ra lenh:
  1. Tu dong khi khoi dong: tham so autostart_mission (ten trong missions.yaml).
  2. Go lenh vao topic /mission/command (std_msgs/String), de nhat la dung agv_cmd.py:
       goto <vi_tri>     di toi mot vi tri (huy viec dang lam)
       goto_xy x y [yaw] di toi mot diem bat ky tren ban do (m, rad, frame map)
       run <nhiem_vu>    chay nhiem vu dinh san
       seq <buoc> ...    chay chuoi buoc tu tao, moi buoc = vi_tri[:task[:wait_s]]
                         vd: seq khu_nhap_hang:pick tram_lay_hang:drop:5 tram_sac:charge
       cancel            dung lai
       list              liet ke vi tri va nhiem vu
       status            viec dang lam

Trang thai phat tren /mission/status (std_msgs/String, giu tin cuoi cho node vao sau) de nguoi
doc, va /mission/state (JSON trong std_msgs/String, giu tin cuoi) de chuong trinh doc (web dashboard).
Vi tri ve tren RViz qua /mission/stations (MarkerArray).

Moi chang: gui dich cho Nav2; that bai thi xoa costmap va thu lai `retries` lan; van that
bai thi bo qua chang do va di tiep. Het nhiem vu thi in dong "TONG KET" voi thoi gian
tung chang (thoi gian mo phong / thoi gian ROS).

Khong dung nav2_simple_commander.BasicNavigator: ham cho cua no gui lai initial pose moi
lan spin_once tra ve, de lam AMCL reset lien tuc (xem REPORT.md, buoc 4).
"""
import json
import math

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

TASK_NAME = {'pick': 'lay hang', 'drop': 'tra hang', 'charge': 'sac pin', 'pass': 'di qua'}
DEFAULT_WAIT = {'pick': 3.0, 'drop': 3.0, 'charge': 0.0, 'pass': 0.0}
KIND_COLOR = {'charge': (0.1, 0.8, 0.2), 'pick': (1.0, 0.55, 0.0), 'inbound': (0.2, 0.5, 1.0),
              'waypoint': (0.6, 0.6, 0.6)}


class MissionServer(Node):

    def __init__(self):
        super().__init__('mission_server')
        self.declare_parameter('stations_file', '')
        self.declare_parameter('missions_file', '')
        self.autostart = self.declare_parameter('autostart_mission', '').value
        self.repeat = self.declare_parameter('repeat', 1).value
        self.retries = self.declare_parameter('retries', 1).value
        self.leg_timeout = self.declare_parameter('leg_timeout_s', 180.0).value

        self.stations = yaml.safe_load(open(self.get_parameter('stations_file').value))['stations']
        self.missions = yaml.safe_load(open(self.get_parameter('missions_file').value))['missions']

        latched = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, 'mission/status', latched)
        self.state_pub = self.create_publisher(String, 'mission/state', latched)
        self.last_state = None
        self.marker_pub = self.create_publisher(MarkerArray, 'mission/stations', latched)
        self.create_subscription(String, 'mission/command', self.on_command, 10)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.on_amcl, 10)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.clear_clients = [self.create_client(ClearEntireCostmap, s) for s in (
            'global_costmap/clear_entirely_global_costmap',
            'local_costmap/clear_entirely_local_costmap')]

        # Nav2 nhan dich chi khi bt_navigator da "active" (action server co the da san sang
        # tu truoc, luc do no tu choi moi dich) -> hoi trang thai vong doi, giong nav2_simple_commander
        self.state_client = self.create_client(GetState, 'bt_navigator/get_state')
        self.nav_active = False
        self.state_pending = False
        self.localized = False
        self.ready_reported = False
        self.queue = []            # cac buoc con lai
        self.step = None           # buoc dang lam
        self.phase = 'idle'        # idle | navigating | waiting | retry_wait
        self.goal_handle = None
        self.goal_id = 0           # tang moi lan gui dich, de bo qua ket qua cua dich cu
        self.tries_left = 0
        self.leg_start = 0.0       # bat dau chang (tinh tong thoi gian chang)
        self.try_start = 0.0       # bat dau lan thu hien tai (tinh timeout)
        self.wait_until = 0.0
        self.mission = None        # ten nhiem vu dang chay
        self.runs_left = 0
        self.results = []          # (vi tri, ok, thoi gian)
        self.mission_start = 0.0

        self.mission_steps = []
        self.publish_markers()
        self.publish_state()
        self.create_timer(0.2, self.tick)
        self.say(f'{len(self.stations)} vi tri, {len(self.missions)} nhiem vu. Cho Nav2 san sang...')

    # ---------- tien ich ----------
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def say(self, text):
        self.get_logger().info(text)
        self.status_pub.publish(String(data=text))

    def on_amcl(self, _msg):
        self.localized = True

    def ready(self):
        if not self.nav_active and not self.state_pending and self.state_client.service_is_ready():
            self.state_pending = True
            self.state_client.call_async(GetState.Request()).add_done_callback(self.on_nav_state)
        return self.nav_active and self.localized and self.nav.server_is_ready()

    def on_nav_state(self, future):
        self.state_pending = False
        res = future.result()
        if res is not None and res.current_state.id == State.PRIMARY_STATE_ACTIVE:
            self.nav_active = True

    # ---------- lenh ----------
    def on_command(self, msg):
        parts = msg.data.strip().split()
        if not parts:
            return
        cmd, arg = parts[0].lower(), (parts[1] if len(parts) > 1 else '')
        if cmd == 'goto':
            if arg not in self.stations:
                self.say(f'Khong co vi tri "{arg}". Go "list" de xem danh sach.')
                return
            self.start_mission(None, [{'station': arg, 'task': 'pass', 'wait_s': 0.0}], runs=1)
        elif cmd == 'goto_xy':
            try:
                x, y = float(parts[1]), float(parts[2])
                yaw = float(parts[3]) if len(parts) > 3 else 0.0
            except (IndexError, ValueError):
                self.say('Dung: goto_xy <x> <y> [yaw]  (m, m, rad, frame map)')
                return
            if not all(math.isfinite(v) for v in (x, y, yaw)):
                self.say('goto_xy: toa do khong hop le')
                return
            name = f'diem ({x:.2f}, {y:.2f})'
            self.start_mission(None, [{'station': name, 'task': 'pass', 'wait_s': 0.0,
                                       'pose': {'x': x, 'y': y, 'yaw': yaw}}], runs=1)
        elif cmd == 'seq':
            steps = []
            for tok in parts[1:]:
                f = tok.split(':')
                task = f[1] if len(f) > 1 and f[1] else 'pass'
                if f[0] not in self.stations or task not in TASK_NAME:
                    self.say(f'seq: buoc "{tok}" sai (vi tri hoac task khong co)')
                    return
                try:
                    wait = float(f[2]) if len(f) > 2 else DEFAULT_WAIT[task]
                except ValueError:
                    self.say(f'seq: thoi gian cho "{f[2]}" khong phai so')
                    return
                if not math.isfinite(wait):
                    self.say(f'seq: thoi gian cho "{f[2]}" khong hop le')
                    return
                steps.append({'station': f[0], 'task': task, 'wait_s': max(0.0, min(wait, 600.0))})
            if not steps:
                self.say('Dung: seq <vi_tri>[:task[:wait_s]] ...')
                return
            self.start_mission('tu_tao', steps, runs=1)
        elif cmd == 'run':
            if arg not in self.missions:
                self.say(f'Khong co nhiem vu "{arg}". Go "list" de xem danh sach.')
                return
            self.start_mission(arg, self.missions[arg]['steps'], runs=1)
        elif cmd == 'cancel':
            self.stop_current()
            self.queue = []
            self.mission = None
            self.say('Da huy, xe dung lai.')
        elif cmd == 'list':
            self.say('Vi tri: ' + ', '.join(self.stations))
            self.say('Nhiem vu: ' + '; '.join(f'{k} ({v["description"]})' for k, v in self.missions.items()))
        elif cmd == 'status':
            if self.step:
                self.say(f'Dang {self.phase}: {self.step["station"]}, con {len(self.queue)} buoc')
            else:
                self.say('Ranh, dang cho lenh.')
        else:
            self.say(f'Lenh khong hieu: "{msg.data}". Dung: goto | goto_xy | run | seq | cancel | list | status')

    def start_mission(self, name, steps, runs):
        self.stop_current()
        self.mission = name
        self.mission_steps = list(steps)
        self.runs_left = runs - 1
        self.queue = list(steps)
        self.results = []
        self.mission_start = self.now()
        what = f'nhiem vu {name}' if name else f'di toi {steps[0]["station"]}'
        self.say(f'Bat dau {what}: ' + ' -> '.join(s['station'] for s in steps))

    def stop_current(self):
        self.goal_id += 1      # ket qua cua dich dang chay se bi bo qua
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self.goal_handle = None
        self.step = None
        self.phase = 'idle'

    def publish_state(self):
        """Trang thai dang may doc cho web dashboard; chi phat khi co thay doi."""
        state = {
            'ready': self.ready_reported,
            'mission': self.mission,
            'phase': self.phase if self.step else 'idle',
            'step': self.step,
            'queue': self.queue,
            'done': [{'station': st, 'ok': ok, 'time_s': round(dt, 1)} for st, ok, dt in self.results],
            'total': len(self.results) + (1 if self.step else 0) + len(self.queue),
            'runs_left': self.runs_left if self.mission else 0,
        }
        text = json.dumps(state, ensure_ascii=False)
        if text != self.last_state:
            self.last_state = text
            self.state_pub.publish(String(data=text))

    # ---------- vong lap ----------
    def tick(self):
        self.publish_state()
        if not self.ready():
            return
        if not self.ready_reported:
            self.ready_reported = True
            self.say('Nav2 san sang, cho lenh.')
            if self.autostart:
                if self.autostart in self.missions:
                    self.start_mission(self.autostart, self.missions[self.autostart]['steps'], self.repeat)
                else:
                    self.say(f'autostart_mission "{self.autostart}" khong co trong missions.yaml')

        t = self.now()
        if self.phase == 'navigating' and t - self.try_start > self.leg_timeout:
            self.say(f'Qua {self.leg_timeout:.0f} s chua toi {self.step["station"]}, huy chang nay')
            self.stop_current_goal_only()
            self.leg_failed()
        elif self.phase == 'waiting' and t >= self.wait_until:
            self.phase = 'idle'
            self.step = None
        elif self.phase == 'retry_wait' and t >= self.wait_until:
            self.send_goal()
        if self.phase == 'idle':
            if self.queue:
                self.step = self.queue.pop(0)
                self.tries_left = self.retries
                self.leg_start = self.now()
                self.send_goal()
            elif self.results:
                self.finish_mission()

    def stop_current_goal_only(self):
        self.goal_id += 1
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self.goal_handle = None

    # ---------- Nav2 ----------
    def send_goal(self):
        s = self.step.get('pose') or self.stations[self.step['station']]
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'   # stamp = 0: Nav2 dung TF moi nhat
        goal.pose.pose.position.x = float(s['x'])
        goal.pose.pose.position.y = float(s['y'])
        goal.pose.pose.orientation.z = math.sin(s['yaw'] / 2.0)
        goal.pose.pose.orientation.w = math.cos(s['yaw'] / 2.0)
        self.phase = 'navigating'
        self.try_start = self.now()
        self.goal_id += 1
        gid = self.goal_id
        self.say(f'-> {s.get("label", self.step["station"])} ({TASK_NAME[self.step["task"]]})')
        self.nav.send_goal_async(goal).add_done_callback(lambda f: self.on_goal_response(f, gid))

    def on_goal_response(self, future, gid):
        if gid != self.goal_id:
            return
        handle = future.result()
        if not handle.accepted:
            self.say('Nav2 tu choi dich')
            self.leg_failed()
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(lambda f: self.on_result(f, gid))

    def on_result(self, future, gid):
        if gid != self.goal_id:
            return   # dich cu (da huy / thay the)
        self.goal_handle = None
        if future.result().status == GoalStatus.STATUS_SUCCEEDED:
            dt = self.now() - self.leg_start
            self.results.append((self.step['station'], True, dt))
            wait = float(self.step.get('wait_s', 0.0))
            self.say(f'Toi {self.step["station"]} sau {dt:.1f} s'
                     + (f', {TASK_NAME[self.step["task"]]} {wait:.0f} s' if wait > 0 else ''))
            self.phase = 'waiting'
            self.wait_until = self.now() + wait
        else:
            self.leg_failed()

    def leg_failed(self):
        if self.tries_left > 0:
            self.tries_left -= 1
            self.say(f'Chua toi duoc {self.step["station"]}, xoa costmap va thu lai')
            for c in self.clear_clients:
                if c.service_is_ready():
                    c.call_async(ClearEntireCostmap.Request())
            # Doi 3 s cho costmap xoa xong / Nav2 hoi phuc roi moi gui lai
            self.phase = 'retry_wait'
            self.wait_until = self.now() + 3.0
            return
        dt = self.now() - self.leg_start
        self.results.append((self.step['station'], False, dt))
        self.say(f'Bo qua {self.step["station"]} (that bai sau {dt:.1f} s)')
        self.phase = 'idle'
        self.step = None

    def finish_mission(self):
        ok = sum(1 for _, good, _ in self.results if good)
        legs = ', '.join(f'{st} {"OK" if good else "LOI"} {dt:.1f}s' for st, good, dt in self.results)
        name = self.mission or 'goto'
        self.say(f'TONG KET {name}: {ok}/{len(self.results)} chang thanh cong, '
                 f'tong {self.now() - self.mission_start:.1f} s | {legs}')
        self.results = []
        if self.runs_left > 0 and self.mission:
            self.runs_left -= 1
            self.queue = list(self.mission_steps)
            self.mission_start = self.now()
            self.say(f'Lap lai {self.mission}, con {self.runs_left} lan sau lan nay')
        else:
            self.mission = None

    # ---------- RViz ----------
    def publish_markers(self):
        arr = MarkerArray()
        for i, (name, s) in enumerate(self.stations.items()):
            r, g, b = KIND_COLOR.get(s.get('kind', 'waypoint'), (0.7, 0.7, 0.7))
            disc = Marker()
            disc.header.frame_id = 'map'
            disc.ns, disc.id, disc.type = 'station', i, Marker.CYLINDER
            disc.pose.position.x, disc.pose.position.y, disc.pose.position.z = float(s['x']), float(s['y']), 0.01
            disc.pose.orientation.w = 1.0
            disc.scale.x = disc.scale.y = 0.5 if s.get('kind') != 'waypoint' else 0.25
            disc.scale.z = 0.02
            disc.color.r, disc.color.g, disc.color.b, disc.color.a = r, g, b, 0.7
            text = Marker()
            text.header.frame_id = 'map'
            text.ns, text.id, text.type = 'label', i, Marker.TEXT_VIEW_FACING
            text.pose.position.x, text.pose.position.y, text.pose.position.z = float(s['x']), float(s['y']) + 0.35, 0.3
            text.pose.orientation.w = 1.0
            text.scale.z = 0.25
            text.color.r = text.color.g = text.color.b = 0.1   # chu toi tren nen ban do trang
            text.color.a = 1.0
            text.text = s.get('label', name)
            arr.markers += [disc, text]
        self.marker_pub.publish(arr)


def main():
    rclpy.init()
    node = MissionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
