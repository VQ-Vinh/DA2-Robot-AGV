"""Node dieu phoi nhiem vu kho (chay cung Nav2).

  ros2 launch agv_mission mission.launch.py                          # xe that, cho lenh
  ros2 launch agv_mission mission.launch.py mission:=tuan_tra        # chay ngay mot nhiem vu
  ros2 launch agv_gazebo sim.launch.py nav:=true mission:=tuan_tra   # mo phong (tu goi file nay)

Web dashboard bat mac dinh: http://<may chay>:8080  (dashboard:=false de tat, port:=... de doi cong).

Vi tri: config/stations.yaml, nhiem vu: config/missions.yaml.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
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
        # Quan ly don hang kieu Kiva (lay ke -> tram -> tra ke). Mo phong: reset_shelves:=true vi
        # world dat lai ke vao o goc moi lan chay; xe that giu vi tri ke trong SQLite.
        DeclareLaunchArgument('reset_shelves', default_value='false',
                              description='Nap lai vi tri ke tu shelves.yaml (mo phong)'),
        DeclareLaunchArgument('auto_confirm_s', default_value='0.0',
                              description='Tu xac nhan lay hang sau N giay (0 = cho nguoi bam)'),
        Node(
            package='agv_mission', executable='order_manager.py',
            parameters=[{
                'shelves_file': PathJoinSubstitution([pkg, 'config', 'shelves.yaml']),
                'stations_file': PathJoinSubstitution([pkg, 'config', 'stations.yaml']),
                'reset_shelves': ParameterValue(LaunchConfiguration('reset_shelves'), value_type=bool),
                'auto_confirm_s': ParameterValue(LaunchConfiguration('auto_confirm_s'), value_type=float),
            }],
            output='screen'),
        DeclareLaunchArgument('dashboard', default_value='true', description='Chay web dashboard'),
        DeclareLaunchArgument('port', default_value='8080', description='Cong web dashboard'),
        Node(
            package='agv_mission', executable='web_dashboard.py',
            # Khong dung use_sim_time: chi lay TF moi nhat, khong can dong ho. Bat len thi node phai
            # nhan /clock moi buoc vat ly cua Gazebo (333 tin/s), ton CPU chi de doc dong ho.
            parameters=[{
                'stations_file': PathJoinSubstitution([pkg, 'config', 'stations.yaml']),
                'missions_file': PathJoinSubstitution([pkg, 'config', 'missions.yaml']),
                'port': ParameterValue(LaunchConfiguration('port'), value_type=int),
            }],
            output='screen',
            condition=IfCondition(LaunchConfiguration('dashboard'))),
    ])
