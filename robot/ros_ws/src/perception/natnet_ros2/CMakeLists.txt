cmake_minimum_required(VERSION 3.8)
project(natnet_ros2)

# Compiler settings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Pure Python package - no SDK required
# Uses pure Python parsing of NatNet protocol (no C++ dependencies)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

# Install executable scripts and shared modules into the same directory
# so natnet_ros2_node.py can "from natnet_client import ..." at runtime
install(PROGRAMS
  src/natnet_client.py
  src/natnet_ros2_node.py
  src/vision_pose_converter_node.py
  DESTINATION lib/${PROJECT_NAME})

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_files()
endif()

ament_package()
