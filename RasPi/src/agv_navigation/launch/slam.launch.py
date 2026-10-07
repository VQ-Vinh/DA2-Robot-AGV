"""Lap ban do bang slam_toolbox: /scan + TF odom -> /map va TF map -> odom.

  ros2 launch agv_navigation slam.launch.py                    # xe that
  ros2 launch agv_gazebo sim.launch.py slam:=true              # mo phong (tu goi file nay)

Luu ban do khi lai xong (ghi de maps/warehouse.pgm + .yaml trong repo):
  ros2 run nav2_map_server map_saver_cli -f <repo>/RasPi/src/agv_navigation/maps/warehouse
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py'])),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'slam_params_file': PathJoinSubstitution(
                    [FindPackageShare('agv_navigation'), 'config', 'slam.yaml']),
            }.items()),
    ])
