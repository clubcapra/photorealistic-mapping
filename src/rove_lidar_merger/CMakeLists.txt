cmake_minimum_required(VERSION 3.8)
project(rove_lidar_merger)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)

add_executable(lidar_merger src/lidar_merger.cpp)
ament_target_dependencies(lidar_merger rclcpp sensor_msgs message_filters)

install(TARGETS lidar_merger
  DESTINATION lib/${PROJECT_NAME})

ament_package()
