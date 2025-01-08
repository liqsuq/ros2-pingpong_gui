cmake_minimum_required(VERSION 3.8)
project(pingpong)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pingpong_msgs REQUIRED)

ament_export_dependencies(ament_cmake rclcpp pingpong_msgs)

add_executable(ping src/ping.cpp)
target_include_directories(ping PRIVATE include)
ament_target_dependencies(ping rclcpp pingpong_msgs)

add_executable(pong src/pong.cpp)
target_include_directories(pong PRIVATE include)
ament_target_dependencies(pong rclcpp pingpong_msgs)

add_executable(logger src/logger.cpp)
target_include_directories(logger PRIVATE include)
ament_target_dependencies(logger rclcpp pingpong_msgs)

install(
  TARGETS
    ping pong logger
  DESTINATION
    lib/${PROJECT_NAME}
)

ament_package()
