cmake_minimum_required(VERSION 3.8)
project(standoff_estimator_pkg)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(standoff_estimator_node
  src/standoff_estimator_node.cpp
)
ament_target_dependencies(standoff_estimator_node
  rclcpp
  sensor_msgs
  std_msgs
)
if(MSVC)
  target_compile_options(standoff_estimator_node PRIVATE /W4)
else()
  target_compile_options(standoff_estimator_node PRIVATE -Wall -Wextra -Wpedantic)
endif()

install(TARGETS standoff_estimator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.launch.py"
)

ament_package()
