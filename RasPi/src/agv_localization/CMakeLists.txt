cmake_minimum_required(VERSION 3.8)
project(agv_localization)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/imu_bias.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
