cmake_minimum_required(VERSION 3.8)
project(fire_video_link_pkg)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# 注意：这里**不需要** find_package(sensor_msgs)。桥用 GenericSubscription/
# GenericPublisher 转发已序列化的字节流，消息类型在运行时按名字动态加载，
# 所以加新话题只改 yaml、不用改 CMakeLists 也不用重新编译。

add_executable(fire_video_bridge_node src/fire_video_bridge_node.cpp)
ament_target_dependencies(fire_video_bridge_node rclcpp)

if(MSVC)
  target_compile_options(fire_video_bridge_node PRIVATE /W4)
else()
  target_compile_options(fire_video_bridge_node PRIVATE -Wall -Wextra -Wpedantic)
endif()

install(TARGETS fire_video_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# video_test.launch.py 的 fake:=true / tune:=true 会调这两个脚本
install(PROGRAMS
  ${CMAKE_CURRENT_SOURCE_DIR}/../../scripts/video_link_test.py
  ${CMAKE_CURRENT_SOURCE_DIR}/../../scripts/camera_tune.py
  DESTINATION share/${PROJECT_NAME}/scripts
)

ament_package()
