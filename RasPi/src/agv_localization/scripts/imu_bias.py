#!/usr/bin/env python3
"""Tru bias con quay (truc z) cua IMU, uoc luong luc xe dung yen. Dung chung mo phong va xe that.

  /imu/raw (Gazebo hoac cau noi IMU) -> tru bias -> /imu -> EKF

Gyro nao cung co bias (mo phong: 0.0005 rad/s, ~1.7 deg/phut). EKF cong bias vao yaw ca khi xe dung
yen; AMCL tin odom (alpha 0.05) nen khong keo lai duoc: dung o tram sac 10 phut thi ban do va lidar lech
24 deg (REPORT.md 4.15). Banh xe dung yen thi xe skid-steer khong the quay, nen moi toc do quay gyro do
duoc luc do la bias: lay trung binh truot (hang so thoi gian tau_s) va tru vao moi mau.

Da thu va bo: dat covariance wz cua odom banh rat nho luc dung (1e-9): EKF van theo IMU (71 Hz) giua hai
tin odom (20 Hz) vi nhieu qua trinh, troi chi giam mot nua.

Tham so:
  tau_s        5.0    s, hang so thoi gian trung binh truot bias
  still_s      1.0    s, banh phai dung yen it nhat bay lau moi bat dau hoc bias (cho xe dung han)
  still_v      1e-3   m/s, rad/s: nguong "banh dung yen" tren /wheel/odom
"""
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


class ImuBias(Node):

    def __init__(self):
        super().__init__('imu_bias')
        self.tau = self.declare_parameter('tau_s', 5.0).value
        self.still_s = self.declare_parameter('still_s', 1.0).value
        self.still_v = self.declare_parameter('still_v', 1e-3).value
        self.bias = 0.0
        self.still_since = None     # thoi diem (stamp IMU, s) banh bat dau dung yen
        self.wheels_still = False
        self.last_t = None
        self.logged = 0.0
        self.pub = self.create_publisher(Imu, 'imu', 10)
        self.create_subscription(Imu, 'imu/raw', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Odometry, 'wheel/odom', self.on_odom, 10)

    def on_odom(self, msg):
        t = msg.twist.twist
        self.wheels_still = abs(t.linear.x) < self.still_v and abs(t.angular.z) < self.still_v

    def on_imu(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        dt = 0.0 if self.last_t is None else max(0.0, min(t - self.last_t, 0.5))
        self.last_t = t
        if not self.wheels_still:
            self.still_since = None
        elif self.still_since is None:
            self.still_since = t
        elif t - self.still_since > self.still_s:
            a = dt / (self.tau + dt)
            self.bias += a * (msg.angular_velocity.z - self.bias)
            if t - self.logged > 60.0:
                self.logged = t
                self.get_logger().info(f'bias gyro z {self.bias:+.5f} rad/s')
        msg.angular_velocity.z -= self.bias
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = ImuBias()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
