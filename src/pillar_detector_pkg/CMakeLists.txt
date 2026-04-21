cmake_minimum_required(VERSION 3.8)
project(pillar_detector_pkg)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ── 原有的静态检测节点（不变） ──
add_executable(pillar_detector_node
  src/pillar_detector_node.cpp
  src/pillar_detector_main.cpp
)
target_include_directories(pillar_detector_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(pillar_detector_node
  rclcpp
  sensor_msgs
  std_msgs
)
if(MSVC)
  target_compile_options(pillar_detector_node PRIVATE /W4)
else()
  target_compile_options(pillar_detector_node PRIVATE -Wall -Wextra -Wpedantic)
endif()

# ── 新增：基于 TF 的飞行中检测节点 ──
add_executable(pillar_detector_tf
  src/pillar_detector_node_tf.cpp
  src/pillar_detector_tf_main.cpp
)
target_include_directories(pillar_detector_tf PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(pillar_detector_tf
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
if(MSVC)
  target_compile_options(pillar_detector_tf PRIVATE /W4)
else()
  target_compile_options(pillar_detector_tf PRIVATE -Wall -Wextra -Wpedantic)
endif()

install(TARGETS
  pillar_detector_node
  pillar_detector_tf
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.launch.py"
)

ament_package()
