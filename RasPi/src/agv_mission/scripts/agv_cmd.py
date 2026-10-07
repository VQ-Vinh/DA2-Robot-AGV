#!/usr/bin/env python3
"""Go lenh cho xe tu terminal, gui toi mission_server va in phan hoi.

  ros2 run agv_mission agv_cmd.py list               # xem vi tri va nhiem vu
  ros2 run agv_mission agv_cmd.py goto ke_C2         # di toi mot vi tri
  ros2 run agv_mission agv_cmd.py run giao_hang      # chay nhiem vu dinh san
  ros2 run agv_mission agv_cmd.py cancel             # dung lai
  ros2 run agv_mission agv_cmd.py status
  ros2 run agv_mission agv_cmd.py watch              # chi theo doi trang thai (Ctrl+C de thoat)

Them --follow sau lenh de tiep tuc in trang thai cho toi khi co dong "TONG KET".
"""
import sys
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--ros-args')]
    follow = '--follow' in args
    args = [a for a in args if a != '--follow']
    if not args:
        print(__doc__)
        return

    rclpy.init()
    node = rclpy.create_node('agv_cmd')
    latched = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    sent = [args[0] == 'watch']
    seen_summary = [False]

    def on_status(msg):
        # Kenh trang thai giu lai cac tin cu (ke ca "TONG KET" cua nhiem vu truoc): bo qua
        # moi tin nhan duoc truoc khi gui lenh, ke ca khi ket noi mat nhieu giay.
        if not sent[0]:
            return
        print(msg.data, flush=True)
        if msg.data.startswith('TONG KET'):
            seen_summary[0] = True

    node.create_subscription(String, 'mission/status', on_status, latched)
    pub = node.create_publisher(String, 'mission/command', 10)

    if args[0] != 'watch':
        # Cho ket noi voi mission_server truoc khi gui (discovery trong WSL co the mat vai giay)
        t0 = time.time()
        while pub.get_subscription_count() == 0 and time.time() - t0 < 15.0:
            rclpy.spin_once(node, timeout_sec=0.2)
        if pub.get_subscription_count() == 0:
            print('Khong thay mission_server. Da chay "sim.launch.py nav:=true" (hoac navigation) chua?')
            node.destroy_node()
            rclpy.try_shutdown()
            return
        t0 = time.time()
        while time.time() - t0 < 1.0:            # nhan het tin cu (toi 10 tin) truoc khi gui
            rclpy.spin_once(node, timeout_sec=0.1)
        sent[0] = True
        pub.publish(String(data=' '.join(args)))

    try:
        end = time.time() + 3.0
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            if args[0] == 'watch' or (follow and not seen_summary[0]):
                continue
            if time.time() > end:
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
