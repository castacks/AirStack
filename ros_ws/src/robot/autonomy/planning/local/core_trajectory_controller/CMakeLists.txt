cmake_minimum_required(VERSION 3.8)
project(trajectory_controller)
set (CMAKE_INCLUDE_CURRENT_DIR ON)
# Use C++14 standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(trajectory_library REQUIRED)
find_package(airstack_msgs REQUIRED)
find_package(tflib REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Include directories
include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${std_srvs_INCLUDE_DIRS}
  ${airstack_msgs_INCLUDE_DIRS}  # Add this line
  ${rosidl_default_runtime_INCLUDE_DIRS} 
)

# Declare a C++ executable
add_executable(trajectory_controller src/trajectory_controller.cpp)

ament_target_dependencies(trajectory_controller
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  trajectory_library
  airstack_msgs
  tflib
  rosidl_default_generators
  std_srvs
)
target_link_libraries(trajectory_controller
  tf2::tf2
  tf2_ros::tf2_ros
)

# Install targets
install(TARGETS trajectory_controller
  DESTINATION lib/${PROJECT_NAME}
)
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp"
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME})

ament_package()
