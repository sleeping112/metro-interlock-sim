cmake_minimum_required(VERSION 3.16)
project(metro_bringup NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  rviz
  worlds
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/acceptance_test.py
  scripts/inspect_interfaces.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
