cmake_minimum_required(VERSION 3.8)
project(fire_link_pkg)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(fire_link_node src/fire_link_node.cpp)
target_include_directories(fire_link_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(fire_link_node
  rclcpp
  std_msgs
)
if(MSVC)
  target_compile_options(fire_link_node PRIVATE /W4)
else()
  target_compile_options(fire_link_node PRIVATE -Wall -Wextra -Wpedantic)
endif()

install(TARGETS fire_link_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.launch.py")

ament_package()
