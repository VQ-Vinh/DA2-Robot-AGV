"""EKF gop /wheel/odom + /imu -> /odom va TF odom -> base_footprint.
imu_bias: /imu/raw -> tru bias gyro (hoc luc banh dung yen) -> /imu.

  ros2 launch agv_localization ekf.launch.py                    # xe that
  ros2 launch agv_localization ekf.launch.py use_sim_time:=true # mo phong (sim.launch.py tu goi)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        # Khong dung use_sim_time: chi dung stamp cua tin IMU, tranh doc /clock (REPORT.md 4.8)
        Node(package='agv_localization', executable='imu_bias.py', output='screen'),
        Node(
            package='robot_localization', executable='ekf_node', name='ekf_filter_node',
            parameters=[PathJoinSubstitution([FindPackageShare('agv_localization'), 'config', 'ekf.yaml']),
                        {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}],
            # Ten chuan ma SLAM / Nav2 doc la /odom
            remappings=[('odometry/filtered', '/odom')],
            output='screen'),
    ])
