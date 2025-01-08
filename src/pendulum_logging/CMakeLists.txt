cmake_minimum_required(VERSION 3.8)
project(pendulum_logging)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pendulum_msgs REQUIRED)

ament_export_dependencies(ament_cmake rclcpp pingpong_msgs)

add_executable(logging src/logging.cpp)
target_include_directories(logging PRIVATE include)
ament_target_dependencies(logging rclcpp pendulum_msgs)

install(
  TARGETS
    logging
  DESTINATION
    lib/${PROJECT_NAME}
)

ament_package()
