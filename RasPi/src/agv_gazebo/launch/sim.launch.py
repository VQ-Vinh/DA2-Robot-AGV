"""Mo phong AGV trong kho: Gazebo + spawn xe + bridge + robot_state_publisher + RViz.

Vi du:
  ros2 launch agv_gazebo sim.launch.py                 # GUI Gazebo (co bang Teleop) + RViz + rqt_robot_steering
  ros2 launch agv_gazebo sim.launch.py rviz:=false steering:=false
  ros2 launch agv_gazebo sim.launch.py headless:=true  # chi chay server, khong mo cua so nao
  ros2 launch agv_gazebo sim.launch.py slam:=true      # them slam_toolbox, RViz hien ban do
  ros2 launch agv_gazebo sim.launch.py nav:=true       # Nav2 tren ban do da lap, bam "Nav2 Goal" de xe tu di
  ros2 launch agv_gazebo sim.launch.py nav:=true mission:=tuan_tra    # + chay nhiem vu kho tu dong
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
    steering = LaunchConfiguration('steering')
    slam = LaunchConfiguration('slam')
    nav = LaunchConfiguration('nav')

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

    # Co cau nang (mat nang + cam bien co ke): cung topic /lift/* voi cau noi STM32 cua xe that
    lift = Node(package='agv_gazebo', executable='lift_sim.py', output='screen')

    # Loc scan (than xe, chan ke dang cho) + doi footprint / toc do theo tai: dung chung voi xe that.
    # Khong dung use_sim_time (giu nguyen stamp cua scan, khong can doc /clock).
    payload_manager = Node(package='agv_navigation', executable='payload_manager.py', output='screen')

    # Pin (tram sac theo vi tri that): cung topic /battery_state voi cau noi STM32 (INA226) cua xe that.
    # battery_time_scale:=20 de thu chinh sach sac nhanh (hao / sac nhanh gap 20).
    battery = Node(
        package='agv_gazebo', executable='battery_sim.py', output='screen',
        parameters=[{'time_scale': ParameterValue(LaunchConfiguration('battery_time_scale'), value_type=float),
                     'initial_percentage': ParameterValue(LaunchConfiguration('battery_initial'), value_type=float)}])

    # Nhan dien ke mini tu 4 chan (scan chua loc) -> /detected_dock_pose cho docking_server: dung chung xe that
    shelf_detector = Node(package='agv_navigation', executable='shelf_detector.py', output='screen')

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

    # Dan duong Nav2 tren ban do da lap: dung chung launch voi xe that
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('agv_navigation'), 'launch', 'navigation.launch.py'])),
        launch_arguments={'use_sim_time': 'true'}.items(),
        condition=IfCondition(nav))

    # Dieu phoi nhiem vu kho (luon chay cung Nav2, cho lenh; mission:=<ten> de chay ngay)
    mission_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('agv_mission'), 'launch', 'mission.launch.py'])),
        # World dat lai ke vao o goc moi lan chay -> order_manager nap lai vi tri ke tu shelves.yaml
        launch_arguments={'use_sim_time': 'true', 'reset_shelves': 'true'}.items(),
        condition=IfCondition(nav))

    # RViz: SLAM -> nhin tu tren xuong, hien ban do; Nav2 -> nav.rviz (goc: cau hinh mac dinh cua Nav2,
    # (co nut "2D Pose Estimate", "Nav2 Goal"); con lai -> xem xe theo frame odom
    def rviz(config, mode):
        # PythonExpression ghep chuoi: "'true' == 'true' and 'false' != 'true' and ..."
        cond = ["'", LaunchConfiguration('rviz'), "' == 'true' and '", headless, "' != 'true' and "]
        if mode == 'slam':
            cond += ["'", slam, "' == 'true'"]
        elif mode == 'nav':
            cond += ["'", nav, "' == 'true' and '", slam, "' != 'true'"]
        else:
            cond += ["'", slam, "' != 'true' and '", nav, "' != 'true'"]
        return Node(package='rviz2', executable='rviz2', arguments=['-d', config],
                    parameters=[{'use_sim_time': True}],
                    condition=IfCondition(PythonExpression(cond)))

    rviz_node = rviz(PathJoinSubstitution([desc_pkg, 'rviz', 'agv.rviz']), 'plain')
    rviz_slam_node = rviz(PathJoinSubstitution([FindPackageShare('agv_navigation'), 'rviz', 'slam.rviz']), 'slam')
    rviz_nav_node = rviz(PathJoinSubstitution([FindPackageShare('agv_navigation'), 'rviz', 'nav.rviz']), 'nav')

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
        DeclareLaunchArgument('nav', default_value='false',
                              description='true: chay Nav2 tren ban do maps/warehouse (khong dung cung slam)'),
        DeclareLaunchArgument('mission', default_value='',
                              description='Nhiem vu kho chay ngay khi Nav2 san sang (can nav:=true)'),
        DeclareLaunchArgument('repeat', default_value='1', description='So lan lap nhiem vu tu dong'),
        DeclareLaunchArgument('battery_time_scale', default_value='1.0',
                              description='Tang toc hao / sac pin mo phong (1 = thoi gian that)'),
        DeclareLaunchArgument('battery_initial', default_value='0.9', description='Pin luc dau (0..1)'),
        DeclareLaunchArgument('rpm_min', default_value='100.0',
                              description='RPM nho nhat cua banh (vung chet firmware), 0 = tat'),
        gazebo_server,
        gazebo_gui,
        robot_state_publisher,
        spawn,
        bridge,
        motor_model,
        lift,
        payload_manager,
        battery,
        shelf_detector,
        wheel_odom,
        ekf,
        slam_launch,
        nav_launch,
        mission_launch,
        rviz_node,
        rviz_slam_node,
        rviz_nav_node,
        steering_node,
    ])
