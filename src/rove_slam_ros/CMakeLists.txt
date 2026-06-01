cmake_minimum_required(VERSION 3.16)
project(rove_slam_ros CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra)

# ──── ament + ROS deps ────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)

# ──── rove_slam core (vendored submodule) ────
# We pull the SLAM C++ core in via add_subdirectory rather than reimplementing
# it in the package. The submodule lives at external/rove_slam.
# Switch off the upstream's own ROS-bridge target — we provide our own node
# below, properly ament-built. Leave KISS-ICP ON (required for SLAM). Leave
# GTSAM OFF (loop closure not used in this minimal bridge; flip to ON for
# Phase 3.3 loop closure).
set(ROVE_SLAM_BUILD_ROS_BRIDGE OFF CACHE BOOL "" FORCE)
set(ROVE_SLAM_USE_KISS_ICP ON CACHE BOOL "" FORCE)
set(ROVE_SLAM_USE_GTSAM OFF CACHE BOOL "" FORCE)
# nanoflann and other vendored deps each try to add an `uninstall` target
# at the top level. ament/colcon already provides one in some configurations
# (the failure mode is "target uninstall already exists"). Telling nanoflann
# the master project owns `uninstall` makes it skip.
set(MASTER_PROJECT_HAS_TARGET_UNINSTALL ON CACHE BOOL "" FORCE)
add_subdirectory(external/rove_slam EXCLUDE_FROM_ALL)

# ──── the node ────
add_executable(rove_slam_node src/rove_slam_node.cpp)
target_include_directories(rove_slam_node PRIVATE
  external/rove_slam/include
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(rove_slam_node rove_slam_core)
ament_target_dependencies(rove_slam_node
  rclcpp sensor_msgs nav_msgs geometry_msgs
  tf2 tf2_ros tf2_eigen
)

install(TARGETS rove_slam_node
        DESTINATION lib/${PROJECT_NAME})

# Install the cmd_vel→Rove bridge as an executable under the package's
# lib/ so `ros2 run rove_slam_ros cmd_vel_to_rove.py` works.
# The script lives in the submodule; we copy it in so it stays self-
# contained.
install(PROGRAMS external/rove_slam/tools/cmd_vel_to_rove.py
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
        DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)  # skip cpplint (style-only)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
