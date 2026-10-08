cmake_minimum_required(VERSION 3.8)
project(agv_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch worlds config
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/motor_model.py scripts/odom_drift.py scripts/wheel_odom.py
  scripts/drive_route.py scripts/lift_sim.py scripts/shelf_transport_test.py scripts/battery_sim.py
  scripts/order_test.py scripts/battery_test.py scripts/fault_test.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
