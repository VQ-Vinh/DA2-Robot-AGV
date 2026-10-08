#!/usr/bin/env python3
"""Web dashboard giao nhiem vu cho xe: mo trinh duyet toi http://<may chay node>:8080

  ros2 launch agv_mission mission.launch.py              # xe that (dashboard bat mac dinh)
  ros2 launch agv_gazebo sim.launch.py nav:=true         # mo phong (tu goi mission.launch.py)

Trinh duyet khong noi chuyen voi ROS/DDS: node nay dung giua, chi dung thu vien chuan Python.
  GET  /                 file tinh trong share/agv_mission/web
  GET  /api/config       vi tri (stations.yaml) va nhiem vu (missions.yaml)
  GET  /api/map          ban do /map: kich thuoc, do phan giai, goc, o ban do (base64), version
  GET  /api/keepout      mat na vung cam /keepout_filter_mask (cac o ke mini), cung dang
  GET  /api/events       (?after=<id nhat ky cuoi>&boot=<ma phien>) Server-Sent Events ~5 Hz: version cac lop ban do, vi tri xe, trang thai nhiem vu, duong di Nav2,
                         dong nhat ky moi, ket noi toi mission_server
  POST /api/command      {"cmd": "run tuan_tra"} -> /mission/command (chi nhan cac lenh trong ALLOWED)

Chua co dang nhap: chi mo trong LAN / Tailscale.
"""
import base64
import json
import math
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

ALLOWED = {'goto', 'goto_xy', 'run', 'seq', 'cancel', 'status', 'list'}
MAX_CMD_LEN = 1000
MAX_PATH_POINTS = 200
LOG_KEEP = 100
EVENT_PERIOD = 0.2
MIME = {'.html': 'text/html; charset=utf-8', '.js': 'text/javascript; charset=utf-8',
        '.css': 'text/css; charset=utf-8', '.svg': 'image/svg+xml', '.png': 'image/png',
        '.ico': 'image/x-icon'}


class Dashboard(Node):

    def __init__(self):
        super().__init__('web_dashboard')
        share = get_package_share_directory('agv_mission')
        self.host = self.declare_parameter('host', '0.0.0.0').value
        self.port = self.declare_parameter('port', 8080).value
        stations_file = self.declare_parameter(
            'stations_file', os.path.join(share, 'config', 'stations.yaml')).value
        missions_file = self.declare_parameter(
            'missions_file', os.path.join(share, 'config', 'missions.yaml')).value
        self.web_dir = os.path.abspath(self.declare_parameter('web_dir', os.path.join(share, 'web')).value)
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value

        self.config = {
            'stations': yaml.safe_load(open(stations_file))['stations'],
            'missions': yaml.safe_load(open(missions_file))['missions'],
        }

        # Du lieu chung giua thread ROS va cac thread HTTP
        self.lock = threading.Lock()
        self.grids = {'map': None, 'keepout': None}   # dict da ma hoa san cho /api/<ten>
        self.grid_version = 0      # tang moi khi mot lop doi; moi lop giu version cua no
        self.pose = None           # {x, y, yaw}
        self.path = []             # [[x, y], ...]
        self.state = None          # dict tu /mission/state
        self.log = []              # [(id, thoi gian, dong)]
        self.log_id = 0
        self.boot = os.urandom(4).hex()   # doi moi lan node khoi dong: trinh duyet xoa nhat ky cu

        latched = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.cmd_pub = self.create_publisher(String, 'mission/command', 10)
        self.create_subscription(String, 'mission/status', self.on_status, latched)
        self.create_subscription(String, 'mission/state', self.on_state, latched)
        self.create_subscription(OccupancyGrid, 'map', lambda m: self.on_grid('map', m), map_qos)
        # Vung cam khong nam trong /map (lidar quet qua phia tren pallet), Nav2 doc tu topic rieng
        self.create_subscription(OccupancyGrid, self.declare_parameter(
            'keepout_topic', 'keepout_filter_mask').value, lambda m: self.on_grid('keepout', m), map_qos)
        self.create_subscription(Path, 'plan', self.on_path, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(EVENT_PERIOD, self.update_pose)

    # ---------- ROS ----------
    def on_status(self, msg):
        with self.lock:
            self.log_id += 1
            self.log.append((self.log_id, time.strftime('%H:%M:%S'), msg.data))
            del self.log[:-LOG_KEEP]

    def on_state(self, msg):
        try:
            state = json.loads(msg.data)
        except ValueError:
            return
        with self.lock:
            self.state = state

    def on_grid(self, layer, msg):
        # -1 (chua biet) -> 255, 0..100 giu nguyen; 1 byte moi o
        cells = bytes(255 if v < 0 else v for v in msg.data)
        info = msg.info
        with self.lock:
            self.grid_version += 1
            self.grids[layer] = {
                'version': self.grid_version,
                'width': info.width, 'height': info.height, 'resolution': info.resolution,
                'origin': [info.origin.position.x, info.origin.position.y],
                'data': base64.b64encode(cells).decode('ascii'),
            }
        self.get_logger().info(f'Lop {layer}: {info.width}x{info.height} o, {info.resolution:.3f} m/o')

    def on_path(self, msg):
        pts = [[round(p.pose.position.x, 3), round(p.pose.position.y, 3)] for p in msg.poses]
        if len(pts) > MAX_PATH_POINTS:
            step = len(pts) / MAX_PATH_POINTS
            pts = [pts[int(i * step)] for i in range(MAX_PATH_POINTS)] + [pts[-1]]
        with self.lock:
            self.path = pts

    def update_pose(self):
        try:
            t = self.tf_buffer.lookup_transform('map', self.base_frame, Time())
        except TransformException:
            return
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with self.lock:
            self.pose = {'x': round(t.transform.translation.x, 3),
                         'y': round(t.transform.translation.y, 3),
                         'yaw': round(yaw, 3)}

    # ---------- cho HTTP ----------
    def snapshot(self, after_log_id):
        with self.lock:
            return {
                'boot': self.boot,
                'pose': self.pose,
                'path': self.path,
                'state': self.state,
                'grids': {k: (g['version'] if g else 0) for k, g in self.grids.items()},
                'log': [{'id': i, 't': t, 'text': s} for i, t, s in self.log if i > after_log_id],
                'link': self.cmd_pub.get_subscription_count() > 0,
            }

    def send_command(self, text):
        words = text.split()
        if not words or words[0].lower() not in ALLOWED:
            return False, f'Lenh khong duoc phep: {words[0] if words else "(trong)"}'
        if self.cmd_pub.get_subscription_count() == 0:
            return False, 'Khong thay mission_server'
        self.cmd_pub.publish(String(data=' '.join(words)))
        return True, 'Da gui'


def make_handler(node):

    class Handler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'

        def log_message(self, fmt, *args):
            pass   # khong in moi request ra terminal

        def send_json(self, obj, code=200):
            body = json.dumps(obj, ensure_ascii=False).encode('utf-8')
            self.send_response(code)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            path = self.path.split('?', 1)[0]
            if path == '/api/config':
                self.send_json(node.config)
            elif path in ('/api/map', '/api/keepout'):
                with node.lock:
                    g = node.grids[path[5:]]
                if g is None:
                    self.send_json({'error': 'chua nhan duoc lop nay'}, 404)
                else:
                    self.send_json(g)
            elif path == '/api/events':
                query = parse_qs(self.path.split('?', 1)[1] if '?' in self.path else '')
                try:
                    after = int(query.get('after', ['0'])[0])
                except ValueError:
                    after = 0
                if query.get('boot', [''])[0] != node.boot:
                    after = 0   # trinh duyet dang giu nhat ky cua lan chay truoc: gui lai tu dau
                self.stream_events(after)
            else:
                self.send_static(path)

        def do_POST(self):
            if self.path != '/api/command':
                self.send_json({'error': 'khong co'}, 404)
                return
            try:
                length = int(self.headers.get('Content-Length', 0))
            except ValueError:
                length = -1
            if length < 0 or length > MAX_CMD_LEN:
                self.send_json({'ok': False, 'message': 'Lenh qua dai'}, 413)
                return
            try:
                cmd = json.loads(self.rfile.read(length) or b'{}').get('cmd', '')
            except (ValueError, AttributeError):
                cmd = None
            if not isinstance(cmd, str):
                self.send_json({'ok': False, 'message': 'Can JSON {"cmd": "..."}'}, 400)
                return
            ok, message = node.send_command(cmd)
            self.send_json({'ok': ok, 'message': message}, 200 if ok else 400)

        def stream_events(self, last_log):
            self.send_response(200)
            self.send_header('Content-Type', 'text/event-stream')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Connection', 'close')
            self.end_headers()
            self.close_connection = True
            try:
                while rclpy.ok():
                    snap = node.snapshot(last_log)
                    if snap['log']:
                        last_log = snap['log'][-1]['id']
                    data = json.dumps(snap, ensure_ascii=False)
                    self.wfile.write(f'data: {data}\n\n'.encode('utf-8'))
                    self.wfile.flush()
                    time.sleep(EVENT_PERIOD)
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass   # trinh duyet dong trang

        def send_static(self, path):
            rel = 'index.html' if path in ('', '/') else path.lstrip('/')
            # abspath (khong phai realpath): chan '..' nhung giu symlink cua --symlink-install
            full = os.path.abspath(os.path.join(node.web_dir, rel))
            if not full.startswith(node.web_dir + os.sep) or not os.path.isfile(full):
                self.send_json({'error': 'khong co'}, 404)
                return
            with open(full, 'rb') as f:
                body = f.read()
            self.send_response(200)
            self.send_header('Content-Type', MIME.get(os.path.splitext(full)[1], 'application/octet-stream'))
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-cache')
            self.end_headers()
            self.wfile.write(body)

    return Handler


def main():
    rclpy.init()
    node = Dashboard()
    server = ThreadingHTTPServer((node.host, node.port), make_handler(node))
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    node.get_logger().info(f'Web dashboard: http://{node.host}:{node.port}  (web: {node.web_dir})')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
