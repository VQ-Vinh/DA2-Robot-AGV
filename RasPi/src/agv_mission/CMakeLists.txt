cmake_minimum_required(VERSION 3.8)
project(agv_mission)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch web
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/mission_server.py scripts/agv_cmd.py scripts/web_dashboard.py scripts/order_manager.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
