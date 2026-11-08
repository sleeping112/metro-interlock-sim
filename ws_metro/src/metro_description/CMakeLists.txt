cmake_minimum_required(VERSION 3.16)
project(metro_description NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  urdf
  models
  meshes
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
