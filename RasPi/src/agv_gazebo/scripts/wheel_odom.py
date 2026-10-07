#!/usr/bin/env python3
"""Them covariance cho odom banh xe cua Gazebo truoc khi dua vao EKF.

DiffDrive cua Gazebo gui odom voi covariance toan 0, nghia la "chac chan tuyet doi".
EKF (robot_localization) se tin tuyet doi vao no va bo qua IMU. Node nay gan
covariance hop ly roi gui ra /wheel/odom - dung topic va dinh dang ma node cau noi
STM32 tren xe that se gui, de cau hinh EKF dung chung cho ca mo phong lan xe that.

  /sim/wheel_odom (Gazebo, covariance = 0) -> /wheel/odom (co covariance) -> EKF

Gia tri covariance (phuong sai, don vi^2):
  vx     linear_var    1e-3   (~3 cm/s)  encoder chinh xac khi di thang
  vy     linear_var    1e-3              xe khong di ngang duoc, vy = 0 la rang buoc
  wz     angular_var   0.1    (~0.3 rad/s)  skid-steer truot ngang khi quay -> rat kem,
                                          de EKF lay toc do quay tu gyro
"""
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class WheelOdom(Node):

    def __init__(self):
        super().__init__('wheel_odom')
        self.linear_var = self.declare_parameter('linear_var', 1e-3).value
        self.angular_var = self.declare_parameter('angular_var', 0.1).value
        self.pub = self.create_publisher(Odometry, 'wheel/odom', 10)
        self.create_subscription(Odometry, 'sim/wheel_odom', self.on_odom, 10)

    def on_odom(self, msg):
        # Ma tran 6x6 theo thu tu x, y, z, roll, pitch, yaw. Pose khong dua vao EKF
        # (chi dung twist) nhung van dat lon, de node khac khong tin nham.
        pose_cov = [0.0] * 36
        twist_cov = [0.0] * 36
        for i, var in enumerate([0.05, 0.05, 1e6, 1e6, 1e6, 0.5]):
            pose_cov[i * 7] = var
        for i, var in enumerate([self.linear_var, self.linear_var, 1e6, 1e6, 1e6, self.angular_var]):
            twist_cov[i * 7] = var
        msg.pose.covariance = pose_cov
        msg.twist.covariance = twist_cov
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = WheelOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
