"""Node dieu phoi nhiem vu kho (chay cung Nav2).

  ros2 launch agv_mission mission.launch.py                          # xe that, cho lenh
  ros2 launch agv_mission mission.launch.py mission:=giao_hang       # chay ngay mot nhiem vu
  ros2 launch agv_gazebo sim.launch.py nav:=true mission:=giao_hang  # mo phong (tu goi file nay)

Vi tri: config/stations.yaml, nhiem vu: config/missions.yaml.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('agv_mission')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('mission', default_value='',
                              description='Nhiem vu chay ngay khi Nav2 san sang (de trong = cho lenh)'),
        DeclareLaunchArgument('repeat', default_value='1', description='So lan chay nhiem vu tu dong'),
        Node(
            package='agv_mission', executable='mission_server.py',
            parameters=[{
                'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                'stations_file': PathJoinSubstitution([pkg, 'config', 'stations.yaml']),
                'missions_file': PathJoinSubstitution([pkg, 'config', 'missions.yaml']),
                'autostart_mission': ParameterValue(LaunchConfiguration('mission'), value_type=str),
                'repeat': ParameterValue(LaunchConfiguration('repeat'), value_type=int),
            }],
            output='screen'),
    ])
