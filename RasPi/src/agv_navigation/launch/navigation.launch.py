"""Dan duong bang Nav2 tren ban do da lap: AMCL dinh vi + lap duong + bam duong.

  ros2 launch agv_navigation navigation.launch.py                 # xe that
  ros2 launch agv_gazebo sim.launch.py nav:=true                  # mo phong (tu goi file nay)

Mac dinh dung maps/warehouse.yaml; AMCL dat san vi tri ban dau tai goc frame map
(cho xe xuat phat luc lap ban do). Xe dat cho khac thi bam "2D Pose Estimate" tren RViz.
Gui dich: bam "Nav2 Goal" tren RViz.

Vung cam (pallet thap, lidar khong thay) lay tu maps/keepout_mask.yaml qua 2 server
cua Nav2, giong nav2_costmap_filters_demo. Tao lai mat na: scripts/make_keepout_mask.py
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('agv_navigation')
    use_sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    params = LaunchConfiguration('params_file')

    # Mat na vung cam: map_server rieng phat /keepout_filter_mask + server thong tin bo loc
    # (nhanh khong composition cua nav2_costmap_filters_demo). Da thu nap vao chung
    # nav2_container (nhanh composition): xe khong chay duoc, nen giu tien trinh rieng.
    keepout_nodes = GroupAction([
        Node(package='nav2_map_server', executable='map_server', name='filter_mask_server',
             parameters=[params, {'use_sim_time': use_sim_time,
                                  'yaml_filename': LaunchConfiguration('keepout_mask')}],
             output='screen'),
        Node(package='nav2_map_server', executable='costmap_filter_info_server',
             name='costmap_filter_info_server',
             parameters=[params, {'use_sim_time': use_sim_time}],
             output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_costmap_filters',
             parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                          'node_names': ['filter_mask_server', 'costmap_filter_info_server']}],
             output='screen'),
    ])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('map', default_value=PathJoinSubstitution([pkg, 'maps', 'warehouse.yaml']),
                              description='Ban do .yaml'),
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution([pkg, 'config', 'nav2.yaml']),
                              description='Tham so Nav2'),
        DeclareLaunchArgument('keepout_mask',
                              default_value=PathJoinSubstitution([pkg, 'maps', 'keepout_mask.yaml']),
                              description='Mat na vung cam .yaml'),
        # Giong linorobot2: goi thang bringup_launch.py cua Nav2 (map_server + AMCL + navigation)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py'])),
            launch_arguments={
                'map': LaunchConfiguration('map'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'params_file': LaunchConfiguration('params_file'),
                'autostart': 'true',
                # Ghi ro: sim.launch.py co tham so 'slam' (= 'false') va launch con ke thua no,
                # ma bringup_launch.py tinh PythonExpression('not ' + slam) -> phai la 'False'.
                'slam': 'False',
            }.items()),
        keepout_nodes,
    ])
