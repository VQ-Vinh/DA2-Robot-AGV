"""Mo phong AGV trong kho: Gazebo + spawn xe + bridge + robot_state_publisher + RViz.

Vi du:
  ros2 launch agv_gazebo sim.launch.py                 # GUI Gazebo (co bang Teleop) + RViz + rqt_robot_steering
  ros2 launch agv_gazebo sim.launch.py rviz:=false steering:=false
  ros2 launch agv_gazebo sim.launch.py headless:=true  # chi chay server, khong mo cua so nao
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    desc_pkg = FindPackageShare('agv_description')
    gz_pkg = FindPackageShare('agv_gazebo')

    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')
    rviz = LaunchConfiguration('rviz')
    steering = LaunchConfiguration('steering')

    robot_description = ParameterValue(
        Command(['xacro ', PathJoinSubstitution([desc_pkg, 'urdf', 'agv.urdf.xacro'])]),
        value_type=str)

    # -r: chay ngay; -s: chi server (khong GUI) khi headless:=true
    # --gui-config: giao dien co them bang Teleop
    gz_args = [PythonExpression(["'-r -s ' if '", headless, "' == 'true' else '-r '"]),
               '--gui-config ', PathJoinSubstitution([gz_pkg, 'config', 'gui.config']), ' ',
               world]

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
        launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items())

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen')

    spawn = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-topic', 'robot_description', '-name', 'agv',
                   '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
                   '-z', '0.02', '-Y', LaunchConfiguration('yaw')],
        output='screen')

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        parameters=[{'config_file': PathJoinSubstitution([gz_pkg, 'config', 'bridge.yaml']),
                     'use_sim_time': True}],
        output='screen')

    # Gioi han dong co nhu firmware STM32: /cmd_vel -> /cmd_vel_limited -> DiffDrive.
    # rpm_min:=0 de tat vung chet, xem xe "ly tuong" chay ra sao.
    motor_model = Node(
        package='agv_gazebo', executable='motor_model.py',
        parameters=[{'rpm_min': ParameterValue(LaunchConfiguration('rpm_min'), value_type=float),
                     'use_sim_time': True}],
        output='screen')

    rviz_node = Node(
        package='rviz2', executable='rviz2',
        arguments=['-d', PathJoinSubstitution([desc_pkg, 'rviz', 'agv.rviz'])],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(rviz))

    # 2 thanh truot van toc, gui /cmd_vel qua ROS -> dung duoc ca voi xe that.
    # Lan dau: bo tick o "stamped" (bridge nhan Twist), rqt se nho lua chon nay.
    steering_node = Node(
        package='rqt_robot_steering', executable='rqt_robot_steering',
        condition=IfCondition(PythonExpression(["'", steering, "' == 'true' and '", headless, "' != 'true'"])))

    return LaunchDescription([
        DeclareLaunchArgument('world',
                              default_value=PathJoinSubstitution([gz_pkg, 'worlds', 'warehouse.sdf']),
                              description='Duong dan file world .sdf'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='true: khong mo cua so Gazebo'),
        DeclareLaunchArgument('rviz', default_value='true', description='Mo RViz'),
        DeclareLaunchArgument('steering', default_value='true',
                              description='Mo rqt_robot_steering (thanh truot lai xe)'),
        DeclareLaunchArgument('x', default_value='-4.5', description='Vi tri xuat phat x (m)'),
        DeclareLaunchArgument('y', default_value='0.0', description='Vi tri xuat phat y (m)'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Huong xuat phat (rad)'),
        DeclareLaunchArgument('rpm_min', default_value='100.0',
                              description='RPM nho nhat cua banh (vung chet firmware), 0 = tat'),
        gazebo,
        robot_state_publisher,
        spawn,
        bridge,
        motor_model,
        rviz_node,
        steering_node,
    ])
