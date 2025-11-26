cmake_minimum_required(VERSION 3.8)
project(pid_path_tracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mav_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(pid_controller_msgs REQUIRED)
find_package(airstack_msgs REQUIRED)
find_package(airstack_common REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)


add_executable(pid_path_tracker src/pid_path_tracker.cpp)
ament_target_dependencies(pid_path_tracker
  rclcpp
  std_msgs
  mav_msgs
  nav_msgs
  pid_controller_msgs
  trajectory_msgs
  airstack_msgs
  airstack_common
  tf2
  tf2_ros)

add_executable(pid_path_tracker_cmd_vel src/pid_path_tracker_cmd_vel.cpp)
ament_target_dependencies(pid_path_tracker_cmd_vel
  rclcpp
  std_msgs
  mav_msgs
  nav_msgs
  pid_controller_msgs
  tf2_geometry_msgs
  trajectory_msgs
  tf2
  tf2_ros)

install(TARGETS
  pid_path_tracker
  pid_path_tracker_cmd_vel
  DESTINATION lib/${PROJECT_NAME})


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
