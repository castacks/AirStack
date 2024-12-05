cmake_minimum_required(VERSION 3.5)
project(cost_map_interface)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(airstack_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(trajectory_library REQUIRED)


add_library(cost_map_interface src/cost_map_interface.cpp)
target_include_directories(cost_map_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(cost_map_interface PUBLIC c_std_99 cxx_std_17) # Require C99 and C++17
ament_target_dependencies(
  cost_map_interface
  # Required dependencies
  airstack_msgs
  geometry_msgs
  nav_msgs
  pluginlib
  rclcpp
  rclpy
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  visualization_msgs
  trajectory_library
)
ament_export_targets(cost_map_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp geometry_msgs tf2 tf2_ros)

install(TARGETS cost_map_interface
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS cost_map_interface
  EXPORT cost_map_interface
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_include_directories(
  include
  ${geometry_msgs_INCLUDE_DIRS}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)

  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
