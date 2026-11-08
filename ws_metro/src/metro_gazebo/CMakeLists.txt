cmake_minimum_required(VERSION 3.16)
project(metro_gazebo CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# --- Gazebo target resolution -------------------------------------------------
# Mandatory target: Gazebo Harmonic (gz-sim8).  Older release images used for
# development in this repository may instead expose the unversioned vendor
# package; that fallback is only for compile/smoke testing and does not change
# the runtime target declared in package.xml.
find_package(gz-sim8 QUIET)
if(gz-sim8_FOUND)
  set(GZ_SIM_TARGET gz-sim8::gz-sim8)
  find_package(gz-transport12 REQUIRED)
  find_package(gz-msgs9 REQUIRED)
  find_package(gz-plugin2 REQUIRED)
  set(GZ_TRANSPORT_TARGET gz-transport12::gz-transport12)
  set(GZ_MSGS_TARGET gz-msgs9::gz-msgs9)
  set(GZ_PLUGIN_TARGET gz-plugin2::gz-plugin2)
  message(STATUS "metro_gazebo: using Gazebo Harmonic (gz-sim8)")
else()
  find_package(gz-sim REQUIRED)
  find_package(gz-transport REQUIRED)
  find_package(gz-msgs REQUIRED)
  find_package(gz-plugin REQUIRED)
  set(GZ_SIM_TARGET gz-sim::gz-sim)
  set(GZ_TRANSPORT_TARGET gz-transport::gz-transport)
  set(GZ_MSGS_TARGET gz-msgs::gz-msgs)
  set(GZ_PLUGIN_TARGET gz-plugin::gz-plugin)
  message(WARNING "metro_gazebo: gz-sim8 not found; using unversioned Gazebo vendor packages for local validation only")
endif()

add_library(metro_door_controller SHARED src/door_controller.cpp)
target_include_directories(metro_door_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(metro_door_controller PUBLIC
  ${GZ_SIM_TARGET}
  ${GZ_TRANSPORT_TARGET}
  ${GZ_MSGS_TARGET}
  ${GZ_PLUGIN_TARGET}
)
target_link_libraries(metro_door_controller PUBLIC
  rclcpp::rclcpp
  std_msgs::std_msgs
  std_srvs::std_srvs
)

add_library(metro_train_plugin SHARED src/train_longitudinal.cpp)
target_include_directories(metro_train_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(metro_train_plugin PUBLIC
  ${GZ_SIM_TARGET}
  ${GZ_TRANSPORT_TARGET}
  ${GZ_MSGS_TARGET}
  ${GZ_PLUGIN_TARGET}
)

install(TARGETS metro_door_controller metro_train_plugin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(metro_door_controller metro_train_plugin)
ament_export_dependencies(rclcpp std_msgs std_srvs)
ament_package()
