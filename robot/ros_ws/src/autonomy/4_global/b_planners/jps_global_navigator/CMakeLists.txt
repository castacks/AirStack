cmake_minimum_required(VERSION 3.8)
project(jps_global_navigator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(task_msgs REQUIRED)

# Add executable
add_executable(jps_global_navigator_node
  src/jps_global_navigator_node.cpp
  src/jps_global_navigator.cpp)

# Include directories
target_include_directories(jps_global_navigator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Link dependencies
ament_target_dependencies(jps_global_navigator_node
  rclcpp
  rclcpp_action
  nav_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  task_msgs)

# Install targets
install(TARGETS jps_global_navigator_node
  DESTINATION lib/${PROJECT_NAME})

# Install header files
install(DIRECTORY include/
  DESTINATION include/)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch/)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # Unit tests for core functionality
  ament_add_gtest(test_jps_global_navigator_unit
    test/test_jps_global_navigator_unit.cpp
    src/jps_global_navigator.cpp)

  target_include_directories(test_jps_global_navigator_unit PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

  ament_target_dependencies(test_jps_global_navigator_unit
    rclcpp
    rclcpp_action
    nav_msgs
    geometry_msgs
    sensor_msgs
    visualization_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    task_msgs)

  # Integration tests
  ament_add_gtest(test_jps_global_navigator_integration
    test/test_jps_global_navigator_integration.cpp
    src/jps_global_navigator.cpp)

  target_include_directories(test_jps_global_navigator_integration PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

  ament_target_dependencies(test_jps_global_navigator_integration
    rclcpp
    rclcpp_action
    nav_msgs
    geometry_msgs
    sensor_msgs
    visualization_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    task_msgs)

endif()

ament_package()