cmake_minimum_required(VERSION 3.8)
project(trajectory_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(airstack_msgs REQUIRED)
find_package(airstack_common REQUIRED)
find_package(trajectory_library REQUIRED)
find_package(task_msgs REQUIRED)


add_executable(trajectory_controller src/trajectory_controller.cpp)
target_include_directories(trajectory_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(trajectory_controller
  rclcpp
  rcl_interfaces
  geometry_msgs
  visualization_msgs
  nav_msgs
  std_srvs
  std_msgs
  tf2
  tf2_ros
  airstack_msgs
  airstack_common
  trajectory_library)

add_executable(fixed_trajectory_task src/fixed_trajectory_task.cpp)
target_include_directories(fixed_trajectory_task PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(fixed_trajectory_task
  rclcpp
  rclcpp_action
  nav_msgs
  airstack_msgs
  task_msgs)

install(TARGETS trajectory_controller fixed_trajectory_task
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

#install(
#  DIRECTORY include/
#  DESTINATION include
#)

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME isolated_pause_reference_contract
    COMMAND ${CMAKE_COMMAND} -E env ROS_DOMAIN_ID=196 ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/isolated_pause_reference.py
      --executable $<TARGET_FILE:trajectory_controller>)
  set_tests_properties(isolated_pause_reference_contract
    PROPERTIES TIMEOUT 60 RUN_SERIAL TRUE)
  add_test(NAME isolated_fenced_control_contract
    COMMAND ${CMAKE_COMMAND} -E env ROS_DOMAIN_ID=194 ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/isolated_fenced_control.py
      --executable $<TARGET_FILE:trajectory_controller>)
  set_tests_properties(isolated_fenced_control_contract
    PROPERTIES TIMEOUT 90 RUN_SERIAL TRUE)
  find_package(drone_safety_monitor REQUIRED)
  get_filename_component(monitor_prefix "${drone_safety_monitor_DIR}/../../.." ABSOLUTE)
  add_test(NAME isolated_monitor_fencing_contract
    COMMAND ${CMAKE_COMMAND} -E env ROS_DOMAIN_ID=193 ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/isolated_monitor_fencing.py
      --controller $<TARGET_FILE:trajectory_controller>
      --monitor "${monitor_prefix}/lib/drone_safety_monitor/drone_safety_monitor")
  set_tests_properties(isolated_monitor_fencing_contract
    PROPERTIES TIMEOUT 90 RUN_SERIAL TRUE)
  add_test(NAME isolated_command_lease_contract
    COMMAND ${CMAKE_COMMAND} -E env ROS_DOMAIN_ID=192 ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/isolated_command_lease.py
      --executable $<TARGET_FILE:trajectory_controller>)
  add_test(NAME isolated_paused_clock_lease_contract
    COMMAND ${CMAKE_COMMAND} -E env ROS_DOMAIN_ID=192 ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/isolated_command_lease.py
      --executable $<TARGET_FILE:trajectory_controller> --paused-clock)
  set_tests_properties(isolated_command_lease_contract isolated_paused_clock_lease_contract
    PROPERTIES TIMEOUT 60 RUN_SERIAL TRUE)
  add_test(NAME isolated_invalid_stamp_contract
    COMMAND ${CMAKE_COMMAND} -E env ROS_DOMAIN_ID=190 ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/isolated_invalid_stamps.py
      --executable $<TARGET_FILE:trajectory_controller>)
  set_tests_properties(isolated_invalid_stamp_contract
    PROPERTIES TIMEOUT 120 RUN_SERIAL TRUE)
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
