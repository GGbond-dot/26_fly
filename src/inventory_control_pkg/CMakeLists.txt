cmake_minimum_required(VERSION 3.8)
project(inventory_control_pkg)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(inventory_mission_node
  src/inventory_mission_main.cpp
  src/inventory_mission_node.cpp
)
target_include_directories(inventory_mission_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(inventory_mission_node
  rclcpp
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
if(MSVC)
  target_compile_options(inventory_mission_node PRIVATE /W4)
else()
  target_compile_options(inventory_mission_node PRIVATE -Wall -Wextra -Wpedantic)
endif()

install(TARGETS inventory_mission_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.launch.py")

ament_package()
