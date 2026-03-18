cmake_minimum_required(VERSION 3.8)

# TODO: Replace with your package name
project(your_package_name)

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ============================================
# Find Dependencies
# ============================================

find_package(ament_cmake REQUIRED)

# ROS 2 Core
find_package(rclcpp REQUIRED)

# AirStack packages
find_package(airstack_msgs REQUIRED)
find_package(airstack_common REQUIRED)

# Standard ROS 2 messages
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Transforms
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# TODO: Add your module-specific dependencies
# find_package(opencv REQUIRED)
# find_package(pcl_conversions REQUIRED)
# find_package(pluginlib REQUIRED)

# ============================================
# Create Executable(s)
# ============================================

# Main node executable
# TODO: Replace with your node name and source files
add_executable(your_node_name 
  src/your_node.cpp
  # TODO: Add additional source files as needed
)

target_include_directories(your_node_name PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(your_node_name PUBLIC c_std_99 cxx_std_17)

# Link dependencies
ament_target_dependencies(
  your_node_name
  "rclcpp"
  "airstack_msgs"
  "airstack_common"
  "std_msgs"
  "geometry_msgs"
  "nav_msgs"
  "sensor_msgs"
  "tf2"
  "tf2_ros"
  # TODO: Add your dependencies here
)

# ============================================
# Create Library (if needed)
# ============================================

# Uncomment if you want to create a library
# add_library(your_library_name SHARED
#   src/your_library.cpp
# )

# target_include_directories(your_library_name PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)

# target_compile_features(your_library_name PUBLIC c_std_99 cxx_std_17)

# ament_target_dependencies(
#   your_library_name
#   "rclcpp"
#   # ... dependencies
# )

# ============================================
# Install Targets
# ============================================

# Install executables
install(TARGETS your_node_name
  DESTINATION lib/${PROJECT_NAME})

# Install library (if created)
# install(TARGETS your_library_name
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

# Install headers (if you have a library with public headers)
# install(DIRECTORY include/
#   DESTINATION include/)

# ============================================
# Testing
# ============================================

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  
  # Standard linters
  ament_lint_auto_find_test_dependencies()
  
  # TODO: Add unit tests
  # find_package(ament_cmake_gtest REQUIRED)
  # ament_add_gtest(${PROJECT_NAME}_test
  #   test/test_your_node.cpp
  # )
  # target_include_directories(${PROJECT_NAME}_test PUBLIC
  #   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  #   $<INSTALL_INTERFACE:include>
  # )
  # ament_target_dependencies(${PROJECT_NAME}_test
  #   rclcpp
  #   # ... test dependencies
  # )
endif()

# ============================================
# Export (if creating a library for other packages)
# ============================================

# Uncomment if this package provides libraries/headers for other packages
# ament_export_targets(your_library_name HAS_LIBRARY_TARGET)
# ament_export_dependencies(
#   rclcpp
#   # ... exported dependencies
# )

ament_package()
