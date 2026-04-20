cmake_minimum_required(VERSION 3.5)
project(data_comm_analysis)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(serial_comm REQUIRED)

add_executable(data_comm_analysis_node
  src/data_comm_analysis_node.cpp
  src/data_comm_analysis.cpp
)

target_include_directories(data_comm_analysis_node PRIVATE
  include
)

ament_target_dependencies(data_comm_analysis_node
  rclcpp
  std_msgs
  serial_comm
)

install(TARGETS data_comm_analysis_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_export_include_directories(include)
ament_export_dependencies(
  Boost
  rclcpp
  std_msgs
  serial_comm
)

ament_package()
