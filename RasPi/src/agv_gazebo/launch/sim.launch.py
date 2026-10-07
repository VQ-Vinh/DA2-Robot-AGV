"""Mo phong AGV trong kho: Gazebo + spawn xe + bridge + robot_state_publisher + RViz.

Vi du:
  ros2 launch agv_gazebo sim.launch.py                 # GUI Gazebo (co bang Teleop) + RViz + rqt_robot_steering
  ros2 launch agv_gazebo sim.launch.py rviz:=false steering:=false
  ros2 launch agv_gazebo sim.launch.py headless:=true  # chi chay server, khong mo cua so nao
  ros2 launch agv_gazebo sim.launch.py slam:=true      # them slam_toolbox, RViz hien ban do
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
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
    slam = LaunchConfiguration('slam')

    robot_description = ParameterValue(
        Command(['xacro ', PathJoinSubstitution([desc_pkg, 'urdf', 'agv.urdf.xacro'])]),
        value_type=str)

    # Server (vat ly, cam bien) va cua so Gazebo chay 2 tien trinh rieng, giong linorobot2.
    # Chi server la bat buoc: cua so Gazebo chet (driver GPU trong WSL thinh thoang crash
    # luc khoi tao OpenGL) thi mo phong, RViz, SLAM van chay; mo lai bang `gz sim -g`.
    gz_sim_launch = PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments={'gz_args': ['-r -s ', world], 'on_exit_shutdown': 'true'}.items())

    # Mo cua so sau server vai giay, de 2 tien trinh khong khoi tao OpenGL cung luc
    gazebo_gui = TimerAction(period=5.0, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments={
            # --gui-config: giao dien co them bang Teleop
            'gz_args': ['-g --gui-config ', PathJoinSubstitution([gz_pkg, 'config', 'gui.config'])],
            'on_exit_shutdown': 'false'}.items())],
        condition=UnlessCondition(headless))

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

    # Odom banh xe cua Gazebo khong co covariance -> them vao, gui /wheel/odom nhu xe that
    wheel_odom = Node(
        package='agv_gazebo', executable='wheel_odom.py',
        parameters=[{'use_sim_time': True}],
        output='screen')

    # EKF dung chung voi xe that: /wheel/odom + /imu -> /odom + TF odom -> base_footprint
    ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('agv_localization'), 'launch', 'ekf.launch.py'])),
        launch_arguments={'use_sim_time': 'true'}.items())

    # Lap ban do: dung chung launch voi xe that
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('agv_navigation'), 'launch', 'slam.launch.py'])),
        launch_arguments={'use_sim_time': 'true'}.items(),
        condition=IfCondition(slam))

    # RViz: khi co SLAM thi nhin tu tren xuong theo frame map, hien ban do
    rviz_rule = ["'", rviz, "' == 'true' and '", headless, "' != 'true' and '", slam, "' "]
    rviz_node = Node(
        package='rviz2', executable='rviz2',
        arguments=['-d', PathJoinSubstitution([desc_pkg, 'rviz', 'agv.rviz'])],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(PythonExpression(rviz_rule + ["!= 'true'"])))
    rviz_slam_node = Node(
        package='rviz2', executable='rviz2',
        arguments=['-d', PathJoinSubstitution([FindPackageShare('agv_navigation'), 'rviz', 'slam.rviz'])],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(PythonExpression(rviz_rule + ["== 'true'"])))

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
        DeclareLaunchArgument('slam', default_value='false',
                              description='true: chay slam_toolbox lap ban do'),
        DeclareLaunchArgument('rpm_min', default_value='100.0',
                              description='RPM nho nhat cua banh (vung chet firmware), 0 = tat'),
        gazebo_server,
        gazebo_gui,
        robot_state_publisher,
        spawn,
        bridge,
        motor_model,
        wheel_odom,
        ekf,
        slam_launch,
        rviz_node,
        rviz_slam_node,
        steering_node,
    ])
