cmake_minimum_required(VERSION 3.5)
project(core_map_representation_interface)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(core_trajectory_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Include directories
include_directories(
  include
  ${ament_INCLUDE_DIRS}
)

# Declare a C++ library (if you have any)
# add_library(${PROJECT_NAME}
#   src/your_cpp_file.cpp
# )

# Link against dependencies (if you have a library)
# ament_target_dependencies(${PROJECT_NAME}
#   rclcpp
#   sensor_msgs
#   std_msgs
#   tf2
#   tf2_ros
#   visualization_msgs
#   pluginlib
#   core_trajectory_msgs
#   geometry_msgs
#   nav_msgs
# )

# Install include files
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp"
)

# Install Python scripts (if you have any)
# install(PROGRAMS
#   scripts/your_python_script.py
#   DESTINATION lib/${PROJECT_NAME}
# )


# Export dependencies
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  rclpy
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  visualization_msgs
  pluginlib
  core_trajectory_msgs
  geometry_msgs
  nav_msgs
)

ament_package()
