#!/usr/bin/env python3
"""Sinh world kho kieu Kiva va danh sach dock cua Nav2 tu agv_mission/config/shelves.yaml.

  python3 make_warehouse.py      # ghi agv_gazebo/worlds/warehouse.sdf va khoi DOCKS trong
                                 # agv_navigation/config/nav2.yaml

Mot nguon du lieu (shelves.yaml, stations.yaml) -> world, dock, vung cam deu khop nhau.
Sua bo tri thi sua yaml roi chay lai file nay (khong sua tay warehouse.sdf).

Kho 12 x 8 m (world x: -6..6, y: -4..4). Frame map = world + (4.5, 0).
  - Ke co dinh sat tuong bac / nam (cao toi san, lidar thay) -> moc cho AMCL
  - Khu luu pallet o giua: 2 day o, moi o co chan do 4 tru co dinh, pallet co thung hang nam tren; loi giua rong
  - Tram sac, tram lay hang, khu nhap hang ben trai; 2 pallet ben phai
"""
import math
import os

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, '..', '..')
MISSION_CFG = os.path.join(SRC, 'agv_mission', 'config')
WORLD = os.path.join(HERE, '..', 'worlds', 'warehouse.sdf')
NAV2 = os.path.join(SRC, 'agv_navigation', 'config', 'nav2.yaml')

MAP_X, MAP_Y = 4.5, 0.0          # map = world + (MAP_X, MAP_Y)
FIXED_RACKS = [(-1.2, 3.45), (2.8, 3.45), (-1.2, -3.45), (2.8, -3.45)]   # world, 3.0 x 0.6 x 1.2 m
PALLETS = [(5.0, 2.8), (5.0, -2.8)]                                       # world, 1.0 x 0.8 x 0.15 m
MARKER_COLOR = {'charge': '0.1 0.7 0.2', 'pick': '0.95 0.55 0.0', 'inbound': '0.2 0.45 0.9'}
MARKER_SIZE = {'charge': (0.6, 0.6), 'pick': (1.0, 0.8), 'inbound': (1.0, 0.8)}


def box(name, pose, size, color, collision=True):
    c = f'<collision name="{name}_c"><pose>{pose}</pose><geometry><box><size>{size}</size></box></geometry></collision>' \
        if collision else ''
    return (f'{c}<visual name="{name}_v"><pose>{pose}</pose><geometry><box><size>{size}</size></box></geometry>'
            f'<material><ambient>{color} 1</ambient><diffuse>{color} 1</diffuse></material></visual>')


def static_model(name, x, y, z, size, color, collision=True, yaw=0.0):
    return (f'    <model name="{name}"><static>true</static><pose>{x} {y} {z} 0 0 {yaw}</pose>\n'
            f'      <link name="link">{box(name, "0 0 0 0 0 0", size, color, collision)}</link>\n'
            f'    </model>\n')


def stand_model(name, x, y, yaw, cfg):
    """Chan do pallet co dinh tai o: 4 tru vuong o 4 goc, cao toi mat duoi pallet (lidar 0.12 m thay).
    O trong van con 4 tru -> moc de nhan dien o khi tra hang; khong di theo pallet."""
    s, leg, h0 = cfg['size'], cfg['leg'], cfg['clearance']
    off = s / 2 - leg / 2
    parts = [box(f'post{i}', f'{sx * off} {sy * off} {h0 / 2} 0 0 0', f'{leg} {leg} {h0}', '0.3 0.3 0.35')
             for i, (sx, sy) in enumerate([(1, 1), (1, -1), (-1, 1), (-1, -1)])]
    return (f'    <model name="{name}"><static>true</static><pose>{x} {y} 0 0 0 {yaw}</pose>\n'
            f'      <link name="link">{"".join(parts)}</link>\n'
            f'    </model>\n')


def pallet_model(name, x, y, yaw, cfg):
    """Pallet go co thung hang gan cung, nam tren chan do (khong co chan rieng). Tu duoi len: 3 van day
    (truoc, giua, sau theo huong xe vao; van giua la cho mat nang day len), 3 thanh do doc, 5 van mat,
    roi 4 thung 2 x 2 + 1 thung chong len. Xe nang pallet: lift_sim khoa pallet vao mat nang (khop co dinh
    DetachableJoint, giong chot dinh vi tren mat nang xe that), ha thi mo khoa. Plugin noi khop ngay khi
    thay model agv: lift_sim mo khoa het luc khoi dong."""
    s, h0, ph = cfg['size'], cfg['clearance'], cfg['pallet_height']
    m = cfg['mass'] + cfg['goods_mass']
    board, stringer = 0.02, ph - 0.04
    wood, goods = '0.62 0.48 0.3', '0.72 0.55 0.35'
    parts = []
    # x = huong xe vao (truc o), y = ngang
    for i, bx in enumerate((-(s - 0.1) / 2, 0.0, (s - 0.1) / 2)):
        parts.append(box(f'bottom{i}', f'{bx} 0 {h0 + board / 2} 0 0 0', f'0.1 {s} {board}', wood))
    for i, sy in enumerate((-(s - 0.08) / 2, 0.0, (s - 0.08) / 2)):
        parts.append(box(f'stringer{i}', f'0 {sy} {h0 + board + stringer / 2} 0 0 0', f'{s} 0.08 {stringer}', wood))
    top_z = h0 + board + stringer
    for i in range(5):
        bx = -s / 2 + 0.065 + i * (s - 0.13) / 4
        parts.append(box(f'top{i}', f'{bx} 0 {top_z + board / 2} 0 0 0', f'0.13 {s} {board}', wood))
    deck = top_z + board
    g, gh = 0.32, 0.2
    for i, (gx, gy) in enumerate([(1, 1), (1, -1), (-1, 1), (-1, -1)]):
        parts.append(box(f'goods{i}', f'{gx * 0.17} {gy * 0.17} {deck + gh / 2} 0 0 0', f'{g} {g} {gh}', goods))
    parts.append(box('goods4', f'0 0 {deck + gh + 0.09} 0 0 0', '0.3 0.26 0.18', goods))
    top = deck + gh + 0.18
    zc = (h0 + top) / 2
    ixx = m * (s * s + (top - h0) ** 2) / 12
    izz = m * (2 * s * s) / 12
    return (f'    <model name="{name}"><pose>{x} {y} 0.001 0 0 {yaw}</pose>\n'
            f'      <link name="link">\n'
            f'        <inertial><pose>0 0 {zc:.3f} 0 0 0</pose><mass>{m}</mass>'
            f'<inertia><ixx>{ixx:.4f}</ixx><iyy>{ixx:.4f}</iyy><izz>{izz:.4f}</izz>'
            f'<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>\n'
            f'        {"".join(parts)}\n'
            f'      </link>\n'
            f'      <plugin filename="gz-sim-detachable-joint-system" name="gz::sim::systems::DetachableJoint">'
            f'<parent_link>link</parent_link><child_model>agv</child_model><child_link>lift_plate</child_link>'
            f'<attach_topic>/shelf/{name}/attach</attach_topic><detach_topic>/shelf/{name}/detach</detach_topic>'
            f'<output_topic>/shelf/{name}/state</output_topic><suppress_child_warning>true</suppress_child_warning>'
            f'</plugin>\n'
            f'    </model>\n')


def wall(name, x, y, sx, sy):
    return static_model(name, x, y, 0.5, f'{sx} {sy} 1.0', '0.8 0.8 0.8')


def main():
    layout = yaml.safe_load(open(os.path.join(MISSION_CFG, 'shelves.yaml')))
    stations = yaml.safe_load(open(os.path.join(MISSION_CFG, 'stations.yaml')))['stations']
    cfg, slots = layout['shelf'], layout['slots']

    out = ['<?xml version="1.0"?>\n',
           '<!-- SINH TU DONG boi agv_gazebo/scripts/make_warehouse.py tu agv_mission/config/shelves.yaml.\n'
           '     Khong sua tay: sua yaml roi chay lai script. -->\n',
           '<sdf version="1.9">\n  <world name="warehouse">\n',
           # Buoc 3 ms nhu world mau cua Nav2 (xem REPORT.md muc 4.8)
           '    <physics name="3ms" type="ignored">\n      <max_step_size>0.003</max_step_size>\n'
           '      <real_time_factor>1.0</real_time_factor>\n    </physics>\n',
           '    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>\n'
           '    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>\n'
           '    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>\n'
           '    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">\n'
           '      <render_engine>ogre2</render_engine>\n    </plugin>\n'
           '    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>\n'
           '    <!-- Cam bien tiep xuc mat nang cua xe ("co ke") -->\n'
           '    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>\n',
           '    <scene><ambient>0.6 0.6 0.6 1</ambient><background>0.7 0.7 0.7 1</background>'
           '<grid>false</grid></scene>\n',
           '    <light type="directional" name="sun"><cast_shadows>true</cast_shadows><pose>0 0 10 0 0 0</pose>'
           '<diffuse>0.9 0.9 0.9 1</diffuse><specular>0.2 0.2 0.2 1</specular>'
           '<direction>-0.3 0.2 -0.9</direction></light>\n',
           '    <model name="floor"><static>true</static><link name="link">'
           '<collision name="c"><geometry><plane><normal>0 0 1</normal><size>30 30</size></plane></geometry></collision>'
           '<visual name="v"><geometry><plane><normal>0 0 1</normal><size>30 30</size></plane></geometry>'
           '<material><ambient>0.55 0.55 0.5 1</ambient><diffuse>0.55 0.55 0.5 1</diffuse></material></visual>'
           '</link></model>\n',
           '    <!-- Tuong cao 1 m -->\n',
           wall('wall_north', 0, 4.05, 12.2, 0.1), wall('wall_south', 0, -4.05, 12.2, 0.1),
           wall('wall_east', 6.05, 0, 0.1, 8.0), wall('wall_west', -6.05, 0, 0.1, 8.0),
           '    <!-- Ke co dinh sat tuong (toi san, lidar thay) -->\n']
    for i, (x, y) in enumerate(FIXED_RACKS):
        out.append(static_model(f'rack_{i + 1}', x, y, 0.6, '3.0 0.6 1.2', '0.1 0.3 0.7'))
    out.append('    <!-- Pallet 0.15 m: lidar o 0.12 m thay duoc, khong can vung cam -->\n')
    for i, (x, y) in enumerate(PALLETS):
        out.append(static_model(f'pallet_{i + 1}', x, y, 0.075, '1.0 0.8 0.15', '0.6 0.4 0.2'))

    out.append('    <!-- Vach son san (khong collision): tram va o ke -->\n')
    for name, st in stations.items():
        kind = st.get('kind')
        if kind in MARKER_SIZE:
            sx, sy = MARKER_SIZE[kind]
            out.append(static_model(f'mark_{name}', st['x'] - MAP_X, st['y'] - MAP_Y, 0.001,
                                    f'{sx} {sy} 0.002', MARKER_COLOR[kind], collision=False))
    # Tram sac: khoi tiep diem truoc diem dock (mat khoi cach tam xe 0.18 m: mica truoc 0.15 m + 3 cm
    # cho xe dung qua). Cao 0.15 m de lidar (0.12 m) THAY: ban dau lam thap 8 cm cho ban do khong doi,
    # xe hu vao no ma odom van tang, AMCL tuong da di 5 m (REPORT.md 4.12).
    # Tiep diem "cham" do battery_sim tinh theo vi tri xe.
    chargers = [(name, st) for name, st in stations.items() if st.get('kind') == 'charge']
    for name, st in chargers:
        c, s = math.cos(st['yaw']), math.sin(st['yaw'])
        out.append(static_model(f'charger_{name}', st['x'] + 0.23 * c - MAP_X, st['y'] + 0.23 * s - MAP_Y, 0.075,
                                '0.1 0.3 0.15', '0.15 0.15 0.15', yaw=st['yaw']))
    for name, sl in slots.items():
        out.append(static_model(f'slot_{name}', sl['x'] - MAP_X, sl['y'] - MAP_Y, 0.0005,
                                f'{cfg["size"] + 0.08} {cfg["size"] + 0.08} 0.001', '0.85 0.85 0.3',
                                collision=False))

    out.append(f'    <!-- Chan do pallet co dinh o moi o: 4 tru {cfg["leg"]} m, cao {cfg["clearance"]} m -->\n')
    for name, sl in slots.items():
        out.append(stand_model(f'stand_{name}', sl['x'] - MAP_X, sl['y'] - MAP_Y, sl['yaw'], cfg))
    out.append(f'    <!-- Pallet {cfg["size"]} m co thung hang, day pallet o {cfg["clearance"]} m, '
               f'{cfg["mass"] + cfg["goods_mass"]} kg (pallet + hang) -->\n')
    for shelf, slot in layout['shelves'].items():
        sl = slots[slot]
        out.append(pallet_model(shelf, sl['x'] - MAP_X, sl['y'] - MAP_Y, sl['yaw'], cfg))
    out.append('  </world>\n</sdf>\n')
    with open(WORLD, 'w', newline='\n') as f:
        f.write(''.join(out))

    # Khoi dock trong nav2.yaml (giua 2 dong danh dau), moi o ke la mot dock "shelf_dock"
    names = [f'slot_{n}' for n in slots] + [f'charger_{n}' for n, _ in chargers]
    block = [f"    docks: [{', '.join(repr(n) for n in names)}]\n"]
    for n, st in chargers:
        block.append(f"    charger_{n}:\n      type: 'simple_charging_dock'\n      frame: map\n"
                     f"      pose: [{st['x']}, {st['y']}, {st['yaw']}]\n")
    for n, sl in slots.items():
        block.append(f"    slot_{n}:\n      type: 'shelf_dock'\n      frame: map\n"
                     f"      pose: [{sl['x']}, {sl['y']}, {sl['yaw']}]\n")
    text = open(NAV2, encoding='utf-8').read()
    begin, end = '    # BEGIN DOCKS (make_warehouse.py)\n', '    # END DOCKS\n'
    i, j = text.index(begin) + len(begin), text.index(end)
    with open(NAV2, 'w', encoding='utf-8', newline='\n') as f:
        f.write(text[:i] + ''.join(block) + text[j:])
    print(f'world: {len(layout["shelves"])} pallet, {len(slots)} o; nav2.yaml: {len(names)} dock')


if __name__ == '__main__':
    main()
