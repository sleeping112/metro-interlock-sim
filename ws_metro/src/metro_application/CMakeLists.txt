cmake_minimum_required(VERSION 3.16)
project(metro_application CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(metro_application_core SHARED
  src/fault_manager.cpp
  src/train_speed_controller.cpp
)
target_include_directories(metro_application_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(metro_application_core PUBLIC
  rclcpp::rclcpp
  std_msgs::std_msgs
  geometry_msgs::geometry_msgs
  nav_msgs::nav_msgs
)

add_executable(interlock_node src/interlock_node.cpp)
target_link_libraries(interlock_node PUBLIC metro_application_core)
target_link_libraries(interlock_node PUBLIC
  rclcpp::rclcpp
  std_msgs::std_msgs
  std_srvs::std_srvs
  geometry_msgs::geometry_msgs
  nav_msgs::nav_msgs
  rosgraph_msgs::rosgraph_msgs
)

add_executable(train_speed_controller_node
  src/speed_controller_main.cpp
  src/train_speed_controller_node.cpp
)

add_executable(scenario_orchestrator src/scenario_orchestrator.cpp)
target_link_libraries(scenario_orchestrator PUBLIC
  metro_application_core
  rclcpp::rclcpp
  std_msgs::std_msgs
  std_srvs::std_srvs
  geometry_msgs::geometry_msgs
  nav_msgs::nav_msgs
  rcl_interfaces::rcl_interfaces
)

add_executable(platform_monitor src/platform_monitor.cpp)
target_link_libraries(platform_monitor PUBLIC
  metro_application_core
  rclcpp::rclcpp
  std_msgs::std_msgs
  geometry_msgs::geometry_msgs
  nav_msgs::nav_msgs
  visualization_msgs::visualization_msgs
)
target_link_libraries(train_speed_controller_node PUBLIC metro_application_core)
target_link_libraries(train_speed_controller_node PUBLIC
  rclcpp::rclcpp
  std_msgs::std_msgs
  geometry_msgs::geometry_msgs
  nav_msgs::nav_msgs
)

install(TARGETS
  metro_application_core
  interlock_node
  train_speed_controller_node
  scenario_orchestrator
  platform_monitor
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(metro_application_core)
ament_export_dependencies(rclcpp std_msgs geometry_msgs nav_msgs rcl_interfaces visualization_msgs)
ament_package()
