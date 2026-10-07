cmake_minimum_required(VERSION 3.8)
project(agv_mission)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/mission_server.py scripts/agv_cmd.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
