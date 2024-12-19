cmake_minimum_required(VERSION 3.8)
project(robot_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(mav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(airstack_msgs REQUIRED)
find_package(airstack_common REQUIRED)

add_library(robot_interface src/robot_interface.cpp)
target_include_directories(robot_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(robot_interface PUBLIC c_std_99 cxx_std_17) # Require C99 and C++17
ament_target_dependencies(
  robot_interface

  # Required dependencies
  geometry_msgs
  mav_msgs
  pluginlib
  rclcpp
  tf2
  tf2_ros
)
ament_export_targets(robot_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp mav_msgs geometry_msgs tf2 tf2_ros)

install(TARGETS robot_interface
  DESTINATION lib/${PROJECT_NAME})

add_executable(robot_interface_node src/robot_interface_node.cpp)
target_include_directories(robot_interface_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(robot_interface_node rclcpp pluginlib mav_msgs tf2 tf2_ros airstack_msgs airstack_common)
install(TARGETS robot_interface_node
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS robot_interface
  EXPORT robot_interface
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# odometry conversion
add_executable(odometry_conversion src/odometry_conversion.cpp)

target_include_directories(odometry_conversion PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(odometry_conversion
  rclcpp
  geometry_msgs
  visualization_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  airstack_msgs
  airstack_common
  )
install(TARGETS odometry_conversion
  DESTINATION lib/${PROJECT_NAME})

# stabilized tf node conversion
add_executable(stabilized_tf_node src/stabilized_tf_node.cpp)

target_include_directories(stabilized_tf_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(stabilized_tf_node
  rclcpp
  geometry_msgs
  visualization_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  airstack_msgs
  airstack_common
  )
install(TARGETS stabilized_tf_node
  DESTINATION lib/${PROJECT_NAME})



ament_export_include_directories(
  include
  ${mav_msgs_INCLUDE_DIRS}
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

# Install files.
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})
# install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
# install(DIRECTORY params DESTINATION share/${PROJECT_NAME})
ament_package()
