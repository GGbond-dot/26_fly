cmake_minimum_required(VERSION 3.8)
project(vl53_ranging_pkg)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(serial_comm REQUIRED)

add_executable(vl53_ranging_node
  src/vl53_ranging_node.cpp
)
ament_target_dependencies(vl53_ranging_node
  rclcpp
  std_msgs
  serial_comm
  Boost
)
if(MSVC)
  target_compile_options(vl53_ranging_node PRIVATE /W4)
else()
  target_compile_options(vl53_ranging_node PRIVATE -Wall -Wextra -Wpedantic)
endif()

install(TARGETS vl53_ranging_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.launch.py"
)

ament_package()
